`default_nettype none

// page_controller.v
// =================
// reference implementation of the external-FPGA side of the page-handshake
// protocol. sits on top of ext/ospi_master.v and a backing program / data
// store (modelled here as two simple memory ports the integrator fills
// with their actual storage backend).
//
// PROTOCOL NOTE: the chip used to expose a `dirty_bits[7:0]` readback at
// 0xFD0000 with one bit per iRAM slot. that register has been removed
// for area; the chip now returns 0xFF (default for unmapped reads) at
// that address. this controller no longer reads 0xFD0000 at all; on
// every page swap it unconditionally writes the full page back.
//
// CHIP GEOMETRY: the iRAM is now 4 slots wide (was 8); each slot stores
// a 16-bit instruction word, addressable as TWO OSPI bytes at addr
// {slot[1:0], byte_select}. this controller treats the program store as
// a byte array with 8 bytes per page, and uses streaming OSPI bursts to
// move those 8 bytes in one transaction per writeback and load.
//
// behaviour
// ---------
//   * watches `page_interrupt` from the chip. on its rising edge the chip
//     has just executed the last slot of `page_current` and is now sitting
//     in ST_PAGE_REQ waiting for a fresh page.
//   * unconditionally reads 8 bytes (lo/hi for each slot) back from iRAM
//     in a single streaming read burst and stores them into the program
//     backing store at offset (page_current * 8 + byte_idx).
//   * asserts `page_loading` on the chip's `ui_in[3]` and issues OSPI
//     transactions to 0x000000..0x000003 with the bytes from the
//     program-store entry for `page_next`.
//   * pulses `page_done` on `ui_in[2]` and returns to idle.
//
// in parallel it services data-memory requests from the chip:
//   * when `data_req` is high it reads 0xFE0000..0xFE0002 to capture
//     {rw, addr_hi, addr_lo}.
//   * for rw=1: reads 0xFE0003 (wdata), writes the data store, OSPI-
//     writes any byte to 0xFE0100 to ack.
//   * for rw=0: reads the data store, OSPI-writes that byte to 0xFE0100.
//
// the page swap takes priority because it unblocks all future fetches;
// a stalled data_req merely blocks one cpu instruction.

module page_controller (
    input  wire        clk,
    input  wire        rst_n,

    // chip status signals (drive these from the chip's uo_out)
    input  wire        page_interrupt,
    input  wire        is_halted,
    input  wire        data_req,

    // chip page-handshake outputs (drive these onto chip's ui_in[2]/[3])
    output reg         page_loading,
    output reg         page_done,

    // ospi_master request port
    output reg         spi_req,
    output reg         spi_rw,         // 0 = read, 1 = write
    output reg  [23:0] spi_addr,
    output reg  [7:0]  spi_wdata,
    output reg  [3:0]  spi_burst_len,
    input  wire [7:0]  spi_rdata,
    input  wire        spi_ack,
    input  wire        spi_data_strobe,
    input  wire [3:0]  spi_burst_idx,

    // program backing store (one byte per slot byte; 256 pages * 2 * slots)
    output reg  [10:0] prog_addr,      // {page[7:0], slot[1:0], byte_sel}
    input  wire [7:0]  prog_rdata,
    output reg  [7:0]  prog_wdata,
    output reg         prog_we,

    // data backing store (linear 64K address space)
    output reg  [15:0] data_addr,
    input  wire [7:0]  data_rdata,
    output reg  [7:0]  data_wdata,
    output reg         data_we,

    // next-page hint. the surrounding glue logic computes this from
    // page_interrupt timing and FARJMP imm decode, then drives it before
    // the chip enters ST_PAGE_REQ.
    input  wire [7:0]  page_current,
    input  wire [7:0]  page_next
);

    // page geometry. must match SLOT_BITS in src/iram_regfile.v.
    localparam integer SLOT_BITS      = 2;
    localparam integer SLOTS_PER_PAGE = 1 << SLOT_BITS;
    localparam integer BYTES_PER_SLOT = 2;
    localparam integer PAGE_BYTES     = SLOTS_PER_PAGE * BYTES_PER_SLOT;
    localparam [3:0]  PAGE_BYTES_LEN  = PAGE_BYTES;

    // -------------------------------------------------------------------
    // states
    // -------------------------------------------------------------------
    localparam [3:0]
        S_IDLE              = 4'd0,
        // page swap - writeback burst
        S_WB_START          = 4'd1,
        S_WB_STREAM         = 4'd2,
        // page swap - load (prefetch + burst)
        S_LD_PREFETCH       = 4'd3,
        S_LD_PREFETCH_WAIT  = 4'd4,
        S_LD_STREAM         = 4'd5,
        S_LD_DONE           = 4'd6,
        // data request
        S_DR_RD_RW          = 4'd7,
        S_DR_RD_HI          = 4'd8,
        S_DR_RD_LO          = 4'd9,
        S_DR_RD_WD          = 4'd10,
        S_DR_SERVE_R        = 4'd11,
        S_DR_SERVE_W        = 4'd12,
        S_DR_ACK            = 4'd13;

    reg [3:0] state;

    // captures used across phases
    reg [2:0]                buf_idx;
    reg [7:0]                page_buf [0:PAGE_BYTES-1];
    reg                      rw_q;
    reg [15:0]               daddr_q;
    reg [7:0]                wdata_q;
    reg [7:0]                rdata_q;
    reg                      spi_data_strobe_q;
    reg [7:0]                spi_rdata_q;

    // rising-edge detector for page_interrupt
    reg page_int_q;
    wire page_int_edge = page_interrupt & ~page_int_q;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) page_int_q <= 1'b0;
        else        page_int_q <= page_interrupt;
    end

    // -------------------------------------------------------------------
    // sequencer
    // -------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= S_IDLE;
            page_loading <= 1'b0;
            page_done    <= 1'b0;
            spi_req      <= 1'b0;
            spi_rw       <= 1'b0;
            spi_addr     <= 24'h0;
            spi_burst_len<= 4'd1;
            prog_addr    <= 11'h0;
            prog_wdata   <= 8'h0;
            prog_we      <= 1'b0;
            data_addr    <= 16'h0;
            data_wdata   <= 8'h0;
            data_we      <= 1'b0;
            buf_idx      <= 3'd0;
            rw_q         <= 1'b0;
            daddr_q      <= 16'h0;
            wdata_q      <= 8'h0;
            rdata_q      <= 8'h0;
            spi_data_strobe_q <= 1'b0;
            spi_rdata_q  <= 8'h0;
        end else begin
            // pulses default low; states set them high for one cycle
            page_done <= 1'b0;
            prog_we   <= 1'b0;
            data_we   <= 1'b0;
            spi_burst_len <= 4'd1;
            spi_data_strobe_q <= spi_data_strobe;
            spi_rdata_q       <= spi_rdata;

            case (state)
                // -----------------------------------------------------------
                S_IDLE: begin
                    spi_req       <= 1'b0;
                    spi_burst_len <= 4'd1;
                    page_loading  <= 1'b0;
                    if (page_int_edge) begin
                        state <= S_WB_START;
                    end else if (data_req) begin
                        spi_req       <= 1'b1;
                        spi_rw        <= 1'b0;
                        spi_addr      <= 24'hFE0000;
                        spi_burst_len <= 4'd1;
                        state         <= S_DR_RD_RW;
                    end
                end

                // -- writeback burst --------------------------------------
                S_WB_START: begin
                    spi_req       <= 1'b1;
                    spi_rw        <= 1'b0;
                    spi_addr      <= 24'h000000;
                    spi_burst_len <= PAGE_BYTES_LEN;
                    buf_idx       <= 3'd0;
                    state         <= S_WB_STREAM;
                end

                S_WB_STREAM: begin
                    spi_burst_len <= PAGE_BYTES_LEN;
                    if (spi_data_strobe_q) begin
                        prog_addr  <= {page_current, buf_idx};
                        prog_wdata <= spi_rdata_q;
                        prog_we    <= 1'b1;
                        if (buf_idx != (PAGE_BYTES - 1))
                            buf_idx <= buf_idx + 1'b1;
                    end
                    if (spi_ack) begin
                        spi_req      <= 1'b0;
                        page_loading <= 1'b1;
                        buf_idx      <= 3'd0;
                        state        <= S_LD_PREFETCH;
                    end
                end

                // -- load new page ----------------------------------------
                S_LD_PREFETCH: begin
                    prog_addr <= {page_next, buf_idx};
                    state     <= S_LD_PREFETCH_WAIT;
                end

                S_LD_PREFETCH_WAIT: begin
                    page_buf[buf_idx] <= prog_rdata;
                    if (buf_idx == (PAGE_BYTES - 1)) begin
                        state <= S_LD_STREAM;
                    end else begin
                        buf_idx <= buf_idx + 1'b1;
                        state   <= S_LD_PREFETCH;
                    end
                end

                S_LD_STREAM: begin
                    spi_req       <= 1'b1;
                    spi_rw        <= 1'b1;
                    spi_addr      <= 24'h000000;
                    spi_burst_len <= PAGE_BYTES_LEN;
                    if (spi_ack) begin
                        spi_req <= 1'b0;
                        state   <= S_LD_DONE;
                    end
                end

                S_LD_DONE: begin
                    page_loading <= 1'b0;
                    page_done    <= 1'b1;     // 1-cycle pulse to the chip
                    state        <= S_IDLE;
                end

                // -- data memory request ----------------------------------
                S_DR_RD_RW: if (spi_ack) begin
                    rw_q     <= spi_rdata[0];
                    spi_req  <= 1'b1;
                    spi_rw   <= 1'b0;
                    spi_addr <= 24'hFE0001;
                    state    <= S_DR_RD_HI;
                end

                S_DR_RD_HI: if (spi_ack) begin
                    daddr_q[15:8] <= spi_rdata;
                    spi_req       <= 1'b1;
                    spi_rw        <= 1'b0;
                    spi_addr      <= 24'hFE0002;
                    state         <= S_DR_RD_LO;
                end

                S_DR_RD_LO: if (spi_ack) begin
                    daddr_q[7:0] <= spi_rdata;
                    if (rw_q) begin
                        spi_req  <= 1'b1;
                        spi_rw   <= 1'b0;
                        spi_addr <= 24'hFE0003;
                        state    <= S_DR_RD_WD;
                    end else begin
                        spi_req   <= 1'b0;
                        data_addr <= {daddr_q[15:8], spi_rdata};
                        state     <= S_DR_SERVE_R;
                    end
                end

                S_DR_RD_WD: if (spi_ack) begin
                    wdata_q   <= spi_rdata;
                    spi_req   <= 1'b0;
                    data_addr <= daddr_q;
                    state     <= S_DR_SERVE_W;
                end

                S_DR_SERVE_W: begin
                    data_wdata <= wdata_q;
                    data_we    <= 1'b1;
                    rdata_q    <= 8'h00;       // chip ignores write rdata
                    spi_req    <= 1'b1;
                    spi_rw     <= 1'b1;
                    spi_addr   <= 24'hFE0100;
                    state      <= S_DR_ACK;
                end

                S_DR_SERVE_R: begin
                    // data_rdata is valid 1 clk after data_addr was set
                    rdata_q   <= data_rdata;
                    spi_req   <= 1'b1;
                    spi_rw    <= 1'b1;
                    spi_addr  <= 24'hFE0100;
                    state     <= S_DR_ACK;
                end

                S_DR_ACK: if (spi_ack) begin
                    spi_req <= 1'b0;
                    state   <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    always @* begin
        spi_wdata = 8'h00;
        if (state == S_LD_STREAM)
            spi_wdata = page_buf[spi_burst_idx[2:0]];
        else if (state == S_DR_SERVE_R)
            spi_wdata = data_rdata;
        else if (state == S_DR_SERVE_W)
            spi_wdata = 8'h00;
    end

endmodule
