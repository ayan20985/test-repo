`default_nettype none

// SPI memory controller SPI mode 0, byte-wide data bus.
// Command frame: 8-bit cmd | 24-bit addr | 8-bit data
// Read  cmd = 0x03, Write cmd = 0x02.
// Total frame = 40 bits = 80 clock edges.

module spi_memory (
    input  wire        clk,
    input  wire        rst_n,

    // cpu/page-controller memory bus
    input  wire        req,
    input  wire        rw,         // 0=read 1=write
    input  wire [23:0] addr,       // 24-bit SPI address
    input  wire [7:0]  wdata,
    output reg         ready,
    output reg  [7:0]  rdata,

    // physical SPI pins
    output reg         sck,
    output reg         cs_n,
    output wire        mosi,
    input  wire        miso
);

    localparam ST_IDLE     = 2'd0,
               ST_TRANSFER = 2'd1,
               ST_DONE     = 2'd2;

    reg [1:0]  state;
    // 32 cmd+addr bits + 8 data bits = 40 bits, 2 edges each = 80 counts
    reg [6:0]  bit_count;
    reg [31:0] shift_out;
    reg [7:0]  shift_in;
    reg        prev_req;

    assign mosi = shift_out[31];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state     <= ST_IDLE;
            sck       <= 0;
            cs_n      <= 1;
            ready     <= 0;
            rdata     <= 0;
            bit_count <= 0;
            shift_out <= 0;
            shift_in  <= 0;
            prev_req  <= 0;
        end else begin
            prev_req <= req;

            case (state)

                ST_IDLE: begin
                    sck       <= 0;
                    cs_n      <= 1;
                    ready     <= 0;
                    bit_count <= 0;
                    if (req && !prev_req) begin
                        cs_n      <= 0;
                        // build 32-bit header: [31:24]=cmd [23:0]=addr
                        shift_out <= {(rw ? 8'h02 : 8'h03), addr};
                        state     <= ST_TRANSFER;
                    end
                end

                ST_TRANSFER: begin
                    if (!sck) begin
                        sck <= 1;
                        // sample MISO on rising edge during data phase (bits 64-78)
                        if (bit_count >= 64 && !rw)
                            shift_in <= {shift_in[6:0], miso};
                    end else begin
                        sck <= 0;
                        // shift MOSI on falling edge
                        if (bit_count < 62) begin
                            shift_out <= {shift_out[30:0], 1'b0};
                        end else if (bit_count == 62) begin
                            if (rw)
                                shift_out <= {wdata, 24'b0};   // pre-load write byte
                            else
                                shift_out <= {shift_out[30:0], 1'b0};
                        end else if (bit_count >= 64 && rw) begin
                            shift_out <= {shift_out[30:0], 1'b0};
                        end

                        bit_count <= bit_count + 2;

                        if (bit_count >= 78)
                            state <= ST_DONE;
                    end
                end

                ST_DONE: begin
                    cs_n  <= 1;
                    ready <= 1;
                    if (!rw)
                        rdata <= shift_in;
                    state <= ST_IDLE;
                end

                default: state <= ST_IDLE;

            endcase
        end
    end

endmodule
