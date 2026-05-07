`default_nettype none

// SPI slave memory interface.
// Acts as a responder to an external SPI master.
// Protocol: master sends [8-bit cmd | 24-bit addr | 8-bit data], slave responds with data on reads.
// Read  cmd = 0x03, Write cmd = 0x02.

module spi_memory (
    input  wire        clk,        // local clock (independent from SPI)
    input  wire        rst_n,

    // internal memory registers (read/written by SPI slave protocol)
    output reg  [23:0] mem_addr,   // address from last SPI transaction
    output reg  [7:0]  mem_wdata,  // write data from last SPI transaction
    input  wire [7:0]  mem_rdata,  // data to return on next SPI read
    output reg         mem_write,  // pulse: SPI write command completed
    output reg         mem_read,   // pulse: SPI read command completed

    // physical SPI slave pins (controlled by external master)
    input  wire        sck,        // serial clock (from master)
    input  wire        cs_n,       // chip select (from master)
    input  wire        mosi,       // master out, slave in (from master)
    output wire        miso        // master in, slave out (to master)
);

    // synchronize external SPI signals to local clock
    reg sck_r1, sck_r2;
    reg cs_r1, cs_r2;
    reg mosi_r1, mosi_r2;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_r1 <= 0;
            sck_r2 <= 0;
            cs_r1 <= 1;
            cs_r2 <= 1;
            mosi_r1 <= 0;
            mosi_r2 <= 0;
        end else begin
            sck_r1  <= sck;
            sck_r2  <= sck_r1;
            cs_r1   <= cs_n;
            cs_r2   <= cs_r1;
            mosi_r1 <= mosi;
            mosi_r2 <= mosi_r1;
        end
    end

    wire sck_sync = sck_r2;
    wire cs_sync = cs_r2;
    wire mosi_sync = mosi_r2;

    // detect SCK rising edge
    reg sck_prev;
    wire sck_rising = sck_sync && !sck_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sck_prev <= 0;
        else
            sck_prev <= sck_sync;
    end

    // shift register for incoming SPI data (40 bits: 8 cmd + 24 addr + 8 data)
    reg [39:0] shift_in;
    reg [6:0]  bit_count;
    reg [7:0]  shift_out;
    reg [7:0]  read_data;
    reg [7:0]  cmd_byte;
    wire [23:0] addr_field = shift_in[31:8];
    wire [7:0]  data_field = shift_in[7:0];

    assign miso = shift_out[7];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            shift_in    <= 40'h0;
            bit_count   <= 0;
            shift_out   <= 8'h0;
            read_data   <= 8'h0;
            mem_addr    <= 24'h0;
            mem_wdata   <= 8'h0;
            mem_write   <= 0;
            mem_read    <= 0;
            cmd_byte    <= 8'h0;
        end else begin
            mem_write <= 0;
            mem_read  <= 0;

            if (cs_sync) begin
                // chip select inactive: reset
                bit_count <= 0;
                shift_in  <= 40'h0;
                shift_out <= 8'h0;
            end else begin
                // chip select active
                if (sck_rising) begin
                    // shift in MOSI on SCK rising edge
                    shift_in <= {shift_in[38:0], mosi_sync};

                    if (bit_count < 40) begin
                        bit_count <= bit_count + 1;

                        // after 8 bits, latch command
                        if (bit_count == 7)
                            cmd_byte <= {shift_in[38:0], mosi_sync}[39:32];

                        // after 32 bits, we have cmd+addr; prepare response
                        if (bit_count == 31) begin
                            mem_addr <= {shift_in[38:0], mosi_sync}[31:8];
                        end

                        // after 40 bits, transaction complete
                        if (bit_count == 39) begin
                            mem_wdata <= {shift_in[38:0], mosi_sync}[7:0];
                            if (cmd_byte == 8'h02) begin
                                // write command
                                mem_write <= 1;
                            end else if (cmd_byte == 8'h03) begin
                                // read command
                                mem_read <= 1;
                                read_data <= mem_rdata;
                            end
                            bit_count <= 0;
                        end
                    end

                    // drive MISO during data phase (bits 32-39)
                    if (bit_count >= 32 && bit_count < 40)
                        shift_out <= {shift_out[6:0], 1'b0};
                    else if (bit_count == 31)
                        shift_out <= read_data;
                end
            end
        end
    end

endmodule
