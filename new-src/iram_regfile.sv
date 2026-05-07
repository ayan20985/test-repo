`default_nettype none

// 16-slot instruction RAM.
// Each slot is 17 bits: {dirty, opcode[3:0], sub[3:0], imm8[7:0]}.
//
// Two write clients:
//   - page_controller writes all 16 slots during page-load (wr_pg_en, slot 0-15 sequentially).
//   - cpu core writes individual slots (wr_cpu_en) and always sets dirty=1.
//
// One read client: cpu core reads by slot index every cycle (combinational).
// page_controller also reads dirty bits via dirty_bits[15:0] for writeback scan.
//
// Priority: page_controller write wins over cpu write (cpu is halted during load anyway).

module iram_regfile (
    input  wire        clk,
    input  wire        rst_n,

    // page_controller write port (load from SPI)
    input  wire        wr_pg_en,
    input  wire [3:0]  wr_pg_slot,
    input  wire [15:0] wr_pg_data,   // {opcode[3:0], sub[3:0], imm8[7:0]} dirty cleared

    // cpu write port (self-modifying / patching sets dirty=1)
    input  wire        wr_cpu_en,
    input  wire [3:0]  wr_cpu_slot,
    input  wire [15:0] wr_cpu_data,  // {opcode[3:0], sub[3:0], imm8[7:0]}

    // cpu read port (combinational)
    input  wire [3:0]  rd_slot,
    output wire [16:0] rd_data,      // {dirty, opcode[3:0], sub[3:0], imm8[7:0]}

    // dirty vector for writeback scan
    output wire [15:0] dirty_bits,

    // page_controller read port for writeback
    input  wire [3:0]  rd_pg_slot,
    output wire [15:0] rd_pg_data    // instruction word (no dirty bit) for SPI writeback
);

    reg [16:0] mem [0:15]; // bit16=dirty, bits15:0=instruction

    integer i;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (i = 0; i < 16; i = i + 1)
                mem[i] <= 17'h00000;
        end else begin
            if (wr_pg_en) begin
                // page load: clear dirty, write instruction word
                mem[wr_pg_slot] <= {1'b0, wr_pg_data};
            end else if (wr_cpu_en) begin
                // cpu patch: set dirty
                mem[wr_cpu_slot] <= {1'b1, wr_cpu_data};
            end
        end
    end

    assign rd_data    = mem[rd_slot];
    assign rd_pg_data = mem[rd_pg_slot][15:0];

    genvar g;
    generate
        for (g = 0; g < 16; g = g + 1) begin : gen_dirty
            assign dirty_bits[g] = mem[g][16];
        end
    endgenerate

endmodule
