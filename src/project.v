`default_nettype none

module tt_um_ocpu (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    // ==========================================
    // CPU Registers
    // ==========================================
    reg [7:0] a;      // Accumulator
    reg [7:0] x;      // X index register
    reg [7:0] y;      // Y index register
    reg [7:0] sp;     // Stack pointer
    reg [15:0] pc;    // Program counter
    reg [7:0] sr;     // Status register (NV-BDIZC)
    
    reg [7:0] ir;     // Instruction register
    reg [7:0] mdr;    // Memory data register
    reg [15:0] addr;  // Address bus register
    
    // ==========================================
    // FSM States
    // ==========================================
    localparam MASTER_STATE_INIT = 0,
               MASTER_STATE_RUN = 1,
               MASTER_STATE_HALT = 2,
               MASTER_STATE_SIMD = 3;

    localparam CORE_STATE_RESET = 0,
               CORE_STATE_FETCH = 1,
               CORE_STATE_DECODE = 2,
               CORE_STATE_EXECUTE = 3,
               CORE_STATE_HALTED = 4;
               
    reg [1:0] master_state;
    reg [2:0] core0_state;
    reg [2:0] core1_state;
    
    // Status Register Flags
    wire flag_c = sr[0]; // Carry
    wire flag_z = sr[1]; // Zero
    wire flag_i = sr[2]; // Interrupt Disable
    wire flag_d = sr[3]; // Decimal Mode (ignored)
    wire flag_b = sr[4]; // Break
    // bit 5 unused
    wire flag_v = sr[6]; // Overflow
    wire flag_n = sr[7]; // Negative

    // Placeholder assignments for now (so TinyTapeout builds correctly)
    assign uo_out  = a;  // Map accumulator to output just for visibility right now
    assign uio_out = 8'b0;
    assign uio_oe  = 8'b0;
    wire _unused_ok = &{ena, ui_in, uio_in, R, G, B, hsync, vsync, video_active, pix_x, pix_y};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            master_state <= MASTER_STATE_INIT;
            core0_state <= CORE_STATE_RESET;
            core1_state <= CORE_STATE_RESET;
            
            a  <= 0;
            x  <= 0;
            y  <= 0;
            sp <= 8'hFF;
            pc <= 16'h0000;
            sr <= 8'h20; // Default status state natively
            ir <= 0;
            mdr <= 0;
            addr <= 0;
        end else begin
            // Master FSM
            case (master_state)
                MASTER_STATE_INIT: begin
                    master_state <= MASTER_STATE_RUN;
                end
                MASTER_STATE_RUN: begin
                    // Control core0 and core1 Execution
                end
                MASTER_STATE_HALT: begin
                    // Halt execution
                end
                MASTER_STATE_SIMD: begin
                    // Lock-step execution for both cores
                end
            endcase

            // Core 0 FSM
            case (core0_state)
                CORE_STATE_RESET: begin
                    if (master_state == MASTER_STATE_RUN || master_state == MASTER_STATE_SIMD)
                        core0_state <= CORE_STATE_FETCH;
                end
                
                CORE_STATE_FETCH: begin
                    core0_state <= CORE_STATE_DECODE;
                end
                
                CORE_STATE_DECODE: begin
                    core0_state <= CORE_STATE_EXECUTE;
                end
                
                CORE_STATE_EXECUTE: begin
                    core0_state <= CORE_STATE_FETCH;
                end
                
                CORE_STATE_HALTED: begin
                    // Wait for master FSM
                end
                default: core0_state <= CORE_STATE_RESET;
            endcase

            // Core 1 FSM (placeholder logic, would replicate or use submodule)
            case (core1_state)
                CORE_STATE_RESET: begin
                    if (master_state == MASTER_STATE_RUN || master_state == MASTER_STATE_SIMD)
                        core1_state <= CORE_STATE_FETCH;
                end
                default: core1_state <= CORE_STATE_RESET;
            endcase
        end
    end

endmodule
