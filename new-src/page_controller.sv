`default_nettype none

// Page controller manages paged instruction loading from SPI flash.
//
// SPI flash address layout:
//   [23:16] = page number (8 bits, passed in from cpu page_next)
//   [15:5]  = reserved / zero
//   [4:1]   = slot index (0-15) two bytes per slot
//   [0]     = byte select: 0=high byte {opcode,sub}, 1=low byte {imm8}
//
// So slot N lives at byte addresses:
//   high byte: {page, 11'b0, slot, 1'b0}  = {page, 12'b0, slot[3:0], 1'b0}
//   low byte:  {page, 11'b0, slot, 1'b1}  = {page, 12'b0, slot[3:0], 1'b1}
//
// Writeback: before loading a new page, scan dirty_bits[15:0].
// For each dirty slot, write its two bytes back to the current (old) page address.
// Only then switch page_reg and load new page.
//
// Handshake with cpu:
//   cpu asserts page_req + page_next
//   page_controller asserts page_loading (cpu must be halted by now)
//   page_controller asserts page_done for 1 cycle when iRAM is ready
//   cpu deasserts page_req on seeing page_loading, then waits for page_done

module page_controller (
    input  wire        clk,
    input  wire        rst_n,

    // handshake with cpu
    input  wire        page_req,      // cpu wants page_next loaded
    input  wire [7:0]  page_next,     // page to load
    input  wire [7:0]  page_current,  // current page (for writeback address)
    output reg         page_loading,  // we are busy
    output reg         page_done,     // one-cycle pulse: iRAM is ready

    // dirty bits from iram_regfile
    input  wire [15:0] dirty_bits,

    // iram_regfile read port (for writeback)
    output reg  [3:0]  iram_rd_slot,
    input  wire [15:0] iram_rd_data,  // instruction word to write back

    // iram_regfile write port (for page load)
    output reg         iram_wr_en,
    output reg  [3:0]  iram_wr_slot,
    output reg  [15:0] iram_wr_data,

    // SPI memory bus (arbitrated externally; this module drives it exclusively while page_loading)
    output reg         spi_req,
    output reg         spi_rw,        // 0=read 1=write
    output reg  [23:0] spi_addr,
    output reg  [7:0]  spi_wdata,
    input  wire        spi_ready,
    input  wire [7:0]  spi_rdata
);

    // FSM
    localparam [3:0]
        PC_IDLE      = 4'd0,
        PC_WB_SCAN   = 4'd1,  // scan dirty_bits for next dirty slot
        PC_WB_HI     = 4'd2,  // write back high byte of dirty slot
        PC_WB_LO     = 4'd3,  // write back low  byte of dirty slot
        PC_WB_WAIT_HI= 4'd4,
        PC_WB_WAIT_LO= 4'd5,
        PC_LOAD_HI   = 4'd6,  // read high byte of slot from new page
        PC_LOAD_LO   = 4'd7,  // read low  byte of slot from new page
        PC_WAIT_HI   = 4'd8,
        PC_WAIT_LO   = 4'd9,
        PC_WRITE_IRAM= 4'd10, // commit assembled word to iRAM
        PC_DONE      = 4'd11;

    reg [3:0]  state;
    reg [3:0]  cur_slot;      // which of the 16 slots we are processing
    reg [3:0]  wb_slot;       // which dirty slot we're writing back
    reg [15:0] dirty_snap;    // snapshot of dirty_bits at start of operation
    reg [7:0]  load_hi;       // high byte read from SPI for load
    reg [7:0]  target_page;   // the page we're loading

    // helper: find lowest set bit in dirty_snap (returns 4'hF if none)
    function automatic [3:0] lowest_dirty;
        input [15:0] d;
        integer i;
        begin
            lowest_dirty = 4'hF;
            for (i = 15; i >= 0; i = i - 1)
                if (d[i]) lowest_dirty = i[3:0];
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state        <= PC_IDLE;
            page_loading <= 0;
            page_done    <= 0;
            spi_req      <= 0;
            spi_rw       <= 0;
            spi_addr     <= 24'h0;
            spi_wdata    <= 8'h0;
            iram_wr_en   <= 0;
            iram_wr_slot <= 4'h0;
            iram_wr_data <= 16'h0;
            iram_rd_slot <= 4'h0;
            cur_slot     <= 4'h0;
            wb_slot      <= 4'h0;
            dirty_snap   <= 16'h0;
            load_hi      <= 8'h0;
            target_page  <= 8'h0;
        end else begin
            // default deasserts
            page_done  <= 0;
            iram_wr_en <= 0;
            spi_req    <= spi_req; // held by FSM

            case (state)

                // ----------------------------------------------------------
                PC_IDLE: begin
                    if (page_req) begin
                        page_loading <= 1;
                        target_page  <= page_next;
                        dirty_snap   <= dirty_bits;
                        cur_slot     <= 4'h0;
                        state        <= PC_WB_SCAN;
                    end
                end

                // ----------------------------------------------------------
                // Walk dirty_snap, find next dirty slot to write back
                PC_WB_SCAN: begin
                    if (dirty_snap == 16'h0000) begin
                        // No more dirty slots start loading new page
                        cur_slot  <= 4'h0;
                        state     <= PC_LOAD_HI;
                    end else begin
                        wb_slot      <= lowest_dirty(dirty_snap);
                        iram_rd_slot <= lowest_dirty(dirty_snap);
                        state        <= PC_WB_HI;
                    end
                end

                // ----------------------------------------------------------
                // Write back high byte {opcode[3:0], sub[3:0]} of dirty slot
                PC_WB_HI: begin
                    if (!spi_req && !spi_ready) begin
                        spi_req   <= 1;
                        spi_rw    <= 1;
                        spi_addr  <= {page_current, 4'b0000, wb_slot, 1'b0};
                        spi_wdata <= iram_rd_data[15:8];
                        state     <= PC_WB_WAIT_HI;
                    end
                end

                PC_WB_WAIT_HI: begin
                    if (spi_ready && spi_req) begin
                        spi_req <= 0;
                        state   <= PC_WB_LO;
                    end
                end

                // Write back low byte {imm8} of dirty slot
                PC_WB_LO: begin
                    if (!spi_req && !spi_ready) begin
                        spi_req   <= 1;
                        spi_rw    <= 1;
                        spi_addr  <= {page_current, 4'b0000, wb_slot, 1'b1};
                        spi_wdata <= iram_rd_data[7:0];
                        state     <= PC_WB_WAIT_LO;
                    end
                end

                PC_WB_WAIT_LO: begin
                    if (spi_ready && spi_req) begin
                        spi_req              <= 0;
                        dirty_snap[wb_slot]  <= 0; // clear from snapshot
                        state                <= PC_WB_SCAN;
                    end
                end

                // ----------------------------------------------------------
                // Load high byte of cur_slot from target_page
                PC_LOAD_HI: begin
                    if (!spi_req && !spi_ready) begin
                        spi_req  <= 1;
                        spi_rw   <= 0;
                        spi_addr <= {target_page, 4'b0000, cur_slot, 1'b0};
                        state    <= PC_WAIT_HI;
                    end
                end

                PC_WAIT_HI: begin
                    if (spi_ready && spi_req) begin
                        load_hi <= spi_rdata;
                        spi_req <= 0;
                        state   <= PC_LOAD_LO;
                    end
                end

                // Load low byte (imm8) of cur_slot
                PC_LOAD_LO: begin
                    if (!spi_req && !spi_ready) begin
                        spi_req  <= 1;
                        spi_rw   <= 0;
                        spi_addr <= {target_page, 4'b0000, cur_slot, 1'b1};
                        state    <= PC_WAIT_LO;
                    end
                end

                PC_WAIT_LO: begin
                    if (spi_ready && spi_req) begin
                        spi_req <= 0;
                        // Assemble word and write to iRAM next cycle
                        iram_wr_data <= {load_hi, spi_rdata};
                        state        <= PC_WRITE_IRAM;
                    end
                end

                // ----------------------------------------------------------
                PC_WRITE_IRAM: begin
                    iram_wr_en   <= 1;
                    iram_wr_slot <= cur_slot;
                    // iram_wr_data already latched

                    if (cur_slot == 4'hF) begin
                        state <= PC_DONE;
                    end else begin
                        cur_slot <= cur_slot + 1;
                        state    <= PC_LOAD_HI;
                    end
                end

                // ----------------------------------------------------------
                PC_DONE: begin
                    page_loading <= 0;
                    page_done    <= 1; // one-cycle pulse
                    state        <= PC_IDLE;
                end

                default: state <= PC_IDLE;

            endcase
        end
    end

endmodule
