`default_nettype none

// OCPU top-level (Tiny Tapeout wrapper) - SPI slave mode
//
// Pin map (SPI slave mode, external master controls):
//   ui_in[0]     = SPI MISO (output from this design)
//   ui_in[7:1]   = unused
//   uo_out[0]    = SPI SCK (input, external master drives)
//   uo_out[1]    = SPI CS_N (input, external master drives)
//   uo_out[2]    = SPI MOSI (input, external master drives)
//   uo_out[7:3]  = unused
//   uio_*        = unused
//
// An external SPI master controls the SPI protocol and reads/writes OCPU memory
// via the slave interface. The OCPU can run with local instruction page 0.

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
    // SPI slave pins
    // slave mode: SCK, CS_N, MOSI are inputs from external master (via uio)
    // MISO is output to external master (via uo_out[0])
    // -------------------------------------------------------------------------
    wire spi_sck_i   = uio_in[0];   // external master drives SCK
    wire spi_cs_n_i  = uio_in[1];   // external master drives CS_N
    wire spi_mosi_i  = uio_in[2];   // external master drives MOSI
    wire spi_miso_o;                 // we drive MISO back to master

    assign uo_out[0] = spi_miso_o;   // MISO output
    assign uo_out[7:1] = 7'b0;       // other outputs unused

    // uio tri-state: SCK, CS_N, MOSI are inputs, so we don't drive them
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;

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

    // page_controller ports (no longer used, tied off)
    wire        pg_iram_wr_en = 1'b0;
    wire [3:0]  pg_iram_wr_slot = 4'h0;
    wire [15:0] pg_iram_wr_data = 16'h0;
    wire [3:0]  pg_iram_rd_slot = 4'h0;
    wire [15:0] pg_iram_rd_data;

    // dirty bits (not used in slave mode)
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
    // Page handshake (minimized for slave mode)
    // -------------------------------------------------------------------------
    wire        page_req;
    wire [7:0]  page_next;
    wire [7:0]  page_reg;
    wire        page_loading;
    wire        page_done;

    // -------------------------------------------------------------------------
    // CPU data memory bus (not connected in slave mode)
    // -------------------------------------------------------------------------
    wire        cpu_mem_req;
    wire        cpu_mem_rw;
    wire [15:0] cpu_mem_addr;
    wire [7:0]  cpu_mem_wdata;
    reg         cpu_mem_ready;
    reg  [7:0]  cpu_mem_rdata;

    // -------------------------------------------------------------------------
    // SPI slave interface (external master controls clock, CS, MOSI)
    // -------------------------------------------------------------------------
    wire [23:0] spi_mem_addr;
    wire [7:0]  spi_mem_wdata;
    wire        spi_mem_write;  // pulse when SPI write command completes
    wire        spi_mem_read;   // pulse when SPI read command completes

    spi_memory spi_slave (
        .clk        (clk),
        .rst_n      (rst_n),
        // slave responds to external master commands
        .sck        (spi_sck_i),    // external master drives this (via uio_in[0])
        .cs_n       (spi_cs_n_i),   // external master drives this (via uio_in[1])
        .mosi       (spi_mosi_i),   // external master drives this (via uio_in[2])
        .miso       (spi_miso_o),   // slave drives this to master (via uo_out[0])
        // internal memory interface
        .mem_addr   (spi_mem_addr),
        .mem_wdata  (spi_mem_wdata),
        .mem_rdata  (8'h00),        // read data (fixed for now, external master provides)
        .mem_write  (spi_mem_write),
        .mem_read   (spi_mem_read)
    );

    wire _unused = &{ena, ui_in};

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
    // page_controller disabled in slave-only mode
    // external SPI master must handle instruction page load via SPI slave
    // -------------------------------------------------------------------------
    // for now, tie off page handshake signals
    assign page_loading = 1'b0;
    assign page_done    = 1'b0;

    // keep page_reg at page 0 for now (cpu can still run with page 0 instructions)
    // external master can later send SPI commands to load different pages via slave
    reg [7:0] page_reg_local;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            page_reg_local <= 8'h00;
        else if (page_req)
            page_reg_local <= page_next;  // latch requested page (ignored for now)
    end

    assign page_reg = page_reg_local;

    // -------------------------------------------------------------------------
    // connect cpu memory bus (not connected to SPI in slave mode)
    // external master communicates via SPI slave, not through cpu mem bus
    // -------------------------------------------------------------------------
    always @(*) begin
        cpu_mem_ready = 0;  // cpu memory requests cannot be serviced
        cpu_mem_rdata = 8'h00;
    end

endmodule
