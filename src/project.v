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
    // SPI/OSPI slave pins
    // slave mode: SCK, CS_N, IO are controlled by external master (FPGA)
    // -------------------------------------------------------------------------
    wire [7:0] ospi_io_i;
    wire [7:0] ospi_io_o;
    wire [7:0] ospi_io_oe;
    wire       ospi_sck_i;
    wire       ospi_cs_n_i;

    // OSPI on uio pins (8-bit parallel)
    assign ospi_sck_i   = uio_in[0];        // SCK input
    assign ospi_cs_n_i  = uio_in[1];        // CS_N input
    assign ospi_io_i    = uio_in[7:0];      // IO input data

    // Status flags output on uo_out[2:0] to external FPGA
    // uo_out[0] = page_interrupt (pulse when page boundary reached)
    // uo_out[1] = page_loading_ack (acknowledge page load request)
    // uo_out[2] = ospi_miso (OSPI output data bit 0) - single bit for slave MISO
    
    wire page_interrupt_flag;
    wire page_loading_flag;
    
    // Tri-state OSPI IO: drive when io_oe=0xFF (slave is not driving), otherwise Hi-Z
    assign uio_out = ospi_io_o;
    assign uio_oe  = ospi_io_oe;

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
        // ospi slave write (external master loads code, 8-bit padded to 16-bit)
        .wr_pg_en    (ospi_mem_write && (ospi_mem_addr[23:20] == 4'h0)),
        .wr_pg_slot  (ospi_mem_addr[3:0]),
        .wr_pg_data  ({ospi_mem_wdata, ospi_mem_wdata}),  // replicate byte to both halves
        // cpu write (SMOD instruction)
        .wr_cpu_en   (cpu_iram_wr_en),
        .wr_cpu_slot (cpu_iram_wr_slot),
        .wr_cpu_data (cpu_iram_wr_data),
        // cpu read
        .rd_slot     (cpu_iram_rd_slot),
        .rd_data     (cpu_iram_rd_data),
        // dirty vector (not used in slave mode)
        .dirty_bits  (dirty_bits),
        // ospi slave read port
        .rd_pg_slot  (ospi_mem_addr[3:0]),
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
    // OSPI slave interface (external FPGA master controls protocol)
    // -------------------------------------------------------------------------
    wire [23:0] ospi_mem_addr;
    wire [7:0]  ospi_mem_wdata;
    wire        ospi_mem_write;  // pulse when OSPI write command completes
    wire        ospi_mem_read;   // pulse when OSPI read command completes

    ospi_memory ospi_slave (
        .clk        (clk),
        .rst_n      (rst_n),
        // slave responds to external master commands
        .sck        (ospi_sck_i),        // external master drives this (via uio_in[0])
        .cs_n       (ospi_cs_n_i),       // external master drives this (via uio_in[1])
        .io_i       (ospi_io_i),         // external master drives this (via uio_in[7:0])
        .io_o       (ospi_io_o),         // slave drives this to master (via uio_out[7:0])
        .io_oe      (ospi_io_oe),        // tri-state control
        // internal memory interface
        .mem_addr   (ospi_mem_addr),
        .mem_wdata  (ospi_mem_wdata),
        .mem_rdata  (ospi_mem_rdata_out),
        .mem_write  (ospi_mem_write),
        .mem_read   (ospi_mem_read)
    );

    wire _unused = &{ena, ui_in};

    // -------------------------------------------------------------------------
    // ocpu_core
    // -------------------------------------------------------------------------
    wire is_halted;
    
    ocpu_core cpu (
        .clk          (clk),
        .rst_n        (rst_n),
        .run_enable   (1'b1),
        .is_halted    (is_halted),
        // page handshake
        .page_req     (page_req),
        .page_next    (page_next),
        .page_done    (page_done),
        .page_loading (page_loading),
        .page_interrupt (page_interrupt),
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
    // OSPI memory interface: handle reads/writes via external master
    // External master can write to iRAM or set page register via OSPI commands
    // -------------------------------------------------------------------------
    reg [7:0] ospi_mem_rdata_out;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ospi_mem_rdata_out <= 8'h00;
        end else if (ospi_mem_read && (ospi_mem_addr[23:20] == 4'h0)) begin
            // read from iRAM slot
            ospi_mem_rdata_out <= pg_iram_rd_data[7:0];
        end else if (ospi_mem_read && (ospi_mem_addr[23:20] == 4'hF)) begin
            // read page register from addr 0xFF00xx
            ospi_mem_rdata_out <= page_reg;
        end
    end

    // -------------------------------------------------------------------------
    // Page handling with paging interrupt
    // CPU can run code from page 0 iRAM (16 instructions).
    // When PC reaches 15, page_interrupt fires and CPU halts.
    // External FPGA reads page_interrupt flag and loads next page via OSPI.
    // When done, FPGA sets page_done, CPU resumes with PC=0.
    // -------------------------------------------------------------------------
    wire page_interrupt;  // from cpu: page boundary reached
    wire page_done_input = uio_in[3];    // from external FPGA: new page loaded
    wire page_loading_input = uio_in[4]; // from external FPGA: currently loading page
    
    assign page_loading = page_loading_input;  // pass through to CPU
    assign page_done = page_done_input;        // pass through to CPU

    reg [7:0] page_reg_local;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            page_reg_local <= 8'h00;
        end else if (ospi_mem_write && (ospi_mem_addr[23:8] == 16'hFF00)) begin
            // external master can write to 0xFF00xx to set page register
            page_reg_local <= ospi_mem_wdata;
        end
    end

    assign page_reg = page_reg_local;
    
    // -------------------------------------------------------------------------
    // Status flags output on uo_out for external FPGA feedback
    // uo_out[0] = page_interrupt (pulse when page boundary reached, PC==15)
    // uo_out[1] = page_loading (acknowledge that we're waiting for page load)
    // uo_out[2] = cpu halted (is_halted from core)
    // uo_out[7:3] = unused
    // -------------------------------------------------------------------------
    assign uo_out[0] = page_interrupt;     // pulse when page boundary hit
    assign uo_out[1] = page_loading;       // asserted while page loading
    assign uo_out[2] = is_halted;          // CPU halted
    assign uo_out[7:3] = 5'b0;

    // -------------------------------------------------------------------------
    // connect cpu memory bus (not connected to SPI in slave mode)
    // external master communicates via SPI slave, not through cpu mem bus
    // -------------------------------------------------------------------------
    always @(*) begin
        cpu_mem_ready = 0;  // cpu memory requests cannot be serviced
        cpu_mem_rdata = 8'h00;
    end

endmodule
