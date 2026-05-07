`default_nettype none

// OCPU top-level (Tiny Tapeout wrapper)
//
// Pin map:
//   ui_in[0]     = SPI MISO
//   ui_in[7:1]   = unused
//   uo_out[0]    = SPI SCK
//   uo_out[1]    = SPI CS_N
//   uo_out[2]    = SPI MOSI
//   uo_out[7:3]  = unused
//   uio_*        = unused
//
// SPI bus arbitration:
//   The single spi_memory instance is shared between:
//     (a) page_controller for instruction page load/writeback (priority, holds cpu halted)
//     (b) ocpu_core data bus for data reads/writes during normal execution
//   page_controller gets the bus whenever page_loading is asserted.
//   cpu data bus gets the bus when page_loading is deasserted and the cpu has a pending req.

module tt_um_ocpu (
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire       ena,
    input  wire       clk,
    input  wire       rst_n
`ifdef OCPU_SIM
    ,
    output wire [7:0] dbg_a,
    output wire [7:0] dbg_x,
    output wire [7:0] dbg_y,
    output wire [7:0] dbg_sp,
    output wire [7:0] dbg_sr,
    output wire [7:0] dbg_ir,
    output wire [3:0] dbg_pc,
    output wire [7:0] dbg_page
`endif
);

    // -------------------------------------------------------------------------
    // SPI pins
    // -------------------------------------------------------------------------
    wire spi_miso = ui_in[0];
    wire spi_sck, spi_cs_n, spi_mosi;

    assign uo_out[0] = spi_sck;
    assign uo_out[1] = spi_cs_n;
    assign uo_out[2] = spi_mosi;
    assign uo_out[7:3] = 5'b0;

    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

    wire _unused = &{ena, ui_in[7:1], uio_in};

    // -------------------------------------------------------------------------
    // iRAM regfile wires
    // -------------------------------------------------------------------------
    // cpu read port
    wire [3:0]  cpu_iram_rd_slot;
    wire [16:0] cpu_iram_rd_data;

    // cpu write port (SMOD instruction)
    wire        cpu_iram_wr_en;
    wire [3:0]  cpu_iram_wr_slot;
    wire [15:0] cpu_iram_wr_data;

    // page_controller write port
    wire        pg_iram_wr_en;
    wire [3:0]  pg_iram_wr_slot;
    wire [15:0] pg_iram_wr_data;

    // page_controller read port (writeback)
    wire [3:0]  pg_iram_rd_slot;
    wire [15:0] pg_iram_rd_data;

    // dirty bits
    wire [15:0] dirty_bits;

    iram_regfile iram (
        .clk         (clk),
        .rst_n       (rst_n),
        // page_controller write
        .wr_pg_en    (pg_iram_wr_en),
        .wr_pg_slot  (pg_iram_wr_slot),
        .wr_pg_data  (pg_iram_wr_data),
        // cpu write
        .wr_cpu_en   (cpu_iram_wr_en),
        .wr_cpu_slot (cpu_iram_wr_slot),
        .wr_cpu_data (cpu_iram_wr_data),
        // cpu read
        .rd_slot     (cpu_iram_rd_slot),
        .rd_data     (cpu_iram_rd_data),
        // dirty vector
        .dirty_bits  (dirty_bits),
        // page_controller read (writeback)
        .rd_pg_slot  (pg_iram_rd_slot),
        .rd_pg_data  (pg_iram_rd_data)
    );

    // -------------------------------------------------------------------------
    // Page handshake wires
    // -------------------------------------------------------------------------
    wire        page_req;
    wire [7:0]  page_next;
    wire [7:0]  page_reg;
    wire        page_loading;
    wire        page_done;

    // -------------------------------------------------------------------------
    // CPU data memory bus wires
    // -------------------------------------------------------------------------
    wire        cpu_mem_req;
    wire        cpu_mem_rw;
    wire [15:0] cpu_mem_addr;
    wire [7:0]  cpu_mem_wdata;
    reg         cpu_mem_ready;
    reg  [7:0]  cpu_mem_rdata;

    // -------------------------------------------------------------------------
    // SPI controller wires (muxed between page_ctrl and cpu)
    // -------------------------------------------------------------------------
    reg         spi_req;
    reg         spi_rw;
    reg  [23:0] spi_addr;
    reg  [7:0]  spi_wdata;
    wire        spi_ready;
    wire [7:0]  spi_rdata;

    // page_controller's SPI request wires
    wire        pg_spi_req;
    wire        pg_spi_rw;
    wire [23:0] pg_spi_addr;
    wire [7:0]  pg_spi_wdata;

    // -------------------------------------------------------------------------
    // ocpu_core
    // -------------------------------------------------------------------------
    ocpu_core cpu (
        .clk          (clk),
        .rst_n        (rst_n),
        .run_enable   (1'b1),
        .is_halted    (),
        // page handshake
        .page_req     (page_req),
        .page_next    (page_next),
        .page_done    (page_done),
        .page_loading (page_loading),
        // iRAM read
        .iram_rd_slot (cpu_iram_rd_slot),
        .iram_rd_data (cpu_iram_rd_data),
        // iRAM cpu write (SMOD)
        .iram_wr_en   (cpu_iram_wr_en),
        .iram_wr_slot (cpu_iram_wr_slot),
        .iram_wr_data (cpu_iram_wr_data),
        // data memory bus
        .mem_req      (cpu_mem_req),
        .mem_rw       (cpu_mem_rw),
        .mem_addr     (cpu_mem_addr),
        .mem_wdata    (cpu_mem_wdata),
        .mem_ready    (cpu_mem_ready),
        .mem_rdata    (cpu_mem_rdata),
        // page register
        .page_reg     (page_reg),
`ifdef OCPU_SIM
        .dbg_a        (dbg_a),
        .dbg_x        (dbg_x),
        .dbg_y        (dbg_y),
        .dbg_sp       (dbg_sp),
        .dbg_sr       (dbg_sr),
        .dbg_ir       (dbg_ir),
        .dbg_pc       (dbg_pc),
`endif
        .out_pc       ()
    );

`ifdef OCPU_SIM
    assign dbg_page = page_reg;
`endif

    // -------------------------------------------------------------------------
    // page_controller
    // -------------------------------------------------------------------------
    page_controller pgctrl (
        .clk          (clk),
        .rst_n        (rst_n),
        // handshake
        .page_req     (page_req),
        .page_next    (page_next),
        .page_current (page_reg),
        .page_loading (page_loading),
        .page_done    (page_done),
        // dirty bits
        .dirty_bits   (dirty_bits),
        // iRAM read (writeback)
        .iram_rd_slot (pg_iram_rd_slot),
        .iram_rd_data (pg_iram_rd_data),
        // iRAM write (load)
        .iram_wr_en   (pg_iram_wr_en),
        .iram_wr_slot (pg_iram_wr_slot),
        .iram_wr_data (pg_iram_wr_data),
        // SPI bus (raw wires, muxed below)
        .spi_req      (pg_spi_req),
        .spi_rw       (pg_spi_rw),
        .spi_addr     (pg_spi_addr),
        .spi_wdata    (pg_spi_wdata),
        .spi_ready    (spi_ready),
        .spi_rdata    (spi_rdata)
    );

    // -------------------------------------------------------------------------
    // SPI arbitration
    // page_controller has absolute priority while page_loading.
    // cpu data bus gets the bus otherwise (same handshake pattern as old arb).
    // -------------------------------------------------------------------------
    localparam ARB_IDLE    = 2'd0,
               ARB_CPU_REQ = 2'd1,
               ARB_CPU_RSP = 2'd2,
               ARB_CPU_WAIT= 2'd3;

    reg [1:0] arb_state;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            arb_state     <= ARB_IDLE;
            cpu_mem_ready <= 0;
            cpu_mem_rdata <= 0;
            spi_req       <= 0;
            spi_rw        <= 0;
            spi_addr      <= 24'h0;
            spi_wdata     <= 8'h0;
        end else begin
            cpu_mem_ready <= 0; // default deassert

            // Page controller owns the SPI bus when busy
            if (page_loading) begin
                spi_req   <= pg_spi_req;
                spi_rw    <= pg_spi_rw;
                spi_addr  <= pg_spi_addr;
                spi_wdata <= pg_spi_wdata;
                // cpu is halted during page load no cpu responses needed
                arb_state <= ARB_IDLE;
            end else begin
                case (arb_state)
                    ARB_IDLE: begin
                        if (cpu_mem_req) begin
                            spi_req   <= 1;
                            spi_rw    <= cpu_mem_rw;
                            spi_addr  <= {8'h00, cpu_mem_addr}; // data addr (16-bit, no bank)
                            spi_wdata <= cpu_mem_wdata;
                            arb_state <= ARB_CPU_REQ;
                        end
                    end

                    ARB_CPU_REQ: begin
                        if (spi_ready && spi_req) begin
                            spi_req       <= 0;
                            cpu_mem_rdata <= spi_rdata;
                            arb_state     <= ARB_CPU_RSP;
                        end
                    end

                    ARB_CPU_RSP: begin
                        cpu_mem_ready <= 1;
                        arb_state     <= ARB_CPU_WAIT;
                    end

                    ARB_CPU_WAIT: begin
                        if (!cpu_mem_req)
                            arb_state <= ARB_IDLE;
                    end

                    default: arb_state <= ARB_IDLE;
                endcase
            end
        end
    end

    // -------------------------------------------------------------------------
    // spi_memory instance
    // -------------------------------------------------------------------------
    spi_memory spi_ctrl (
        .clk    (clk),
        .rst_n  (rst_n),
        .req    (spi_req),
        .rw     (spi_rw),
        .addr   (spi_addr),
        .wdata  (spi_wdata),
        .ready  (spi_ready),
        .rdata  (spi_rdata),
        .sck    (spi_sck),
        .cs_n   (spi_cs_n),
        .mosi   (spi_mosi),
        .miso   (spi_miso)
    );

endmodule
