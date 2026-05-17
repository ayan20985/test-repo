`default_nettype none

// OSPI (8-bit parallel) slave memory interface.
// Acts as a responder to an external OSPI master (FPGA).
// Protocol: master sends [8-bit cmd | 24-bit addr | 8-bit data], slave responds with data on reads.
// Read  cmd = 0x03, Write cmd = 0x02.
// All data is transmitted/received on io[7:0] with io_oe controlling tri-state.

module ospi_memory (
    input  wire        clk,        // local clock (independent from OSPI)
    input  wire        rst_n,

    // internal memory interface
    output reg  [23:0] mem_addr,   // address from last OSPI transaction
    output reg  [7:0]  mem_wdata,  // write data from last OSPI transaction
    input  wire [7:0]  mem_rdata,  // data to return on next OSPI read
    output reg         mem_write,  // pulse: OSPI write command completed
    output reg         mem_read,   // pulse: OSPI read command completed

    // physical OSPI slave pins (controlled by external master)
    input  wire        sck,        // serial clock (from master)
    input  wire        cs_n,       // chip select (from master)
    input  wire [7:0]  io_i,       // input data from master (8 bits parallel)
    output wire [7:0]  io_o,       // output data to master (8 bits parallel)
    output wire [7:0]  io_oe       // tri-state control (1=drive, 0=Hi-Z)
);

    // synchronize external OSPI signals to local clock (single stage)
    reg sck_r1;
    reg cs_r1;
    reg [7:0] io_i_r1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sck_r1 <= 0;
            cs_r1 <= 1;
            io_i_r1 <= 8'h00;
        end else begin
            sck_r1  <= sck;
            cs_r1   <= cs_n;
            io_i_r1 <= io_i;
        end
    end

    wire sck_sync = sck_r1;
    wire cs_sync = cs_r1;
    wire [7:0] io_i_sync = io_i_r1;

    // detect SCK rising edge
    reg sck_prev;
    wire sck_rising = sck_sync && !sck_prev;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            sck_prev <= 0;
        else
            sck_prev <= sck_sync;
    end

    // byte-streamed protocol: incoming bytes are written directly into the
    // address/data output registers driven by byte_count, so no separate
    // 32-bit shift register is needed. cmd_byte is also shrunk to two
    // single-bit flags (read vs. write) latched when byte 0 arrives.
    reg [2:0]  byte_count;  // 0-4 for 5 bytes total
    reg [7:0]  shift_out;
    reg [7:0]  read_data;
    reg        is_read_cmd;
    reg        is_write_cmd;

    assign io_o = shift_out;
    assign io_oe = (cs_sync && byte_count >= 3'd4) ? 8'hFF : 8'h00;  // drive output during data phase

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            byte_count   <= 0;
            shift_out    <= 8'h0;
            read_data    <= 8'h0;
            mem_addr     <= 24'h0;
            mem_wdata    <= 8'h0;
            mem_write    <= 0;
            mem_read     <= 0;
            is_read_cmd  <= 0;
            is_write_cmd <= 0;
        end else begin
            mem_write <= 0;
            mem_read  <= 0;

            if (cs_sync) begin
                // chip select active
                if (sck_rising) begin
                    if (byte_count < 5) begin
                        case (byte_count)
                            // byte 0: command. latch the two cmd-decode flags
                            // instead of keeping the whole 8-bit cmd_byte.
                            3'd0: begin
                                is_write_cmd <= (io_i_sync == 8'h02);
                                is_read_cmd  <= (io_i_sync == 8'h03);
                            end
                            // bytes 1-3: address arrives MSB first. write directly
                            // into the corresponding slice of the output address reg.
                            3'd1: mem_addr[23:16] <= io_i_sync;
                            3'd2: mem_addr[15:8]  <= io_i_sync;
                            3'd3: mem_addr[7:0]   <= io_i_sync;
                            // byte 4: data byte completes the transaction.
                            3'd4: begin
                                mem_wdata <= io_i_sync;
                                if (is_write_cmd)
                                    mem_write <= 1;
                                else if (is_read_cmd) begin
                                    mem_read  <= 1;
                                    read_data <= mem_rdata;
                                end
                            end
                            default: ;
                        endcase

                        if (byte_count == 4)
                            byte_count <= 0;
                        else
                            byte_count <= byte_count + 1;
                    end

                    // drive shift_out for data phase (preserves existing 1-cycle
                    // OSPI read pipeline behaviour expected by the master)
                    if (byte_count == 4)
                        shift_out <= read_data;
                end
            end else begin
                // chip select inactive: reset byte counter and output driver
                byte_count <= 0;
                shift_out  <= 8'h0;
            end
        end
    end

endmodule
