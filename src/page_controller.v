`default_nettype none
module page_controller (
	clk,
	rst_n,
	page_req,
	page_next,
	page_current,
	page_loading,
	page_done,
	dirty_bits,
	iram_rd_slot,
	iram_rd_data,
	iram_wr_en,
	iram_wr_slot,
	iram_wr_data,
	spi_req,
	spi_rw,
	spi_addr,
	spi_wdata,
	spi_ready,
	spi_rdata
);
	input wire clk;
	input wire rst_n;
	input wire page_req;
	input wire [7:0] page_next;
	input wire [7:0] page_current;
	output reg page_loading;
	output reg page_done;
	input wire [15:0] dirty_bits;
	output reg [3:0] iram_rd_slot;
	input wire [15:0] iram_rd_data;
	output reg iram_wr_en;
	output reg [3:0] iram_wr_slot;
	output reg [15:0] iram_wr_data;
	output reg spi_req;
	output reg spi_rw;
	output reg [23:0] spi_addr;
	output reg [7:0] spi_wdata;
	input wire spi_ready;
	input wire [7:0] spi_rdata;
	localparam [3:0] PC_IDLE = 4'd0;
	localparam [3:0] PC_WB_SCAN = 4'd1;
	localparam [3:0] PC_WB_HI = 4'd2;
	localparam [3:0] PC_WB_LO = 4'd3;
	localparam [3:0] PC_WB_WAIT_HI = 4'd4;
	localparam [3:0] PC_WB_WAIT_LO = 4'd5;
	localparam [3:0] PC_LOAD_HI = 4'd6;
	localparam [3:0] PC_LOAD_LO = 4'd7;
	localparam [3:0] PC_WAIT_HI = 4'd8;
	localparam [3:0] PC_WAIT_LO = 4'd9;
	localparam [3:0] PC_WRITE_IRAM = 4'd10;
	localparam [3:0] PC_DONE = 4'd11;
	reg [3:0] state;
	reg [3:0] cur_slot;
	reg [3:0] wb_slot;
	reg [15:0] dirty_snap;
	reg [7:0] load_hi;
	reg [7:0] target_page;
	function automatic [3:0] lowest_dirty;
		input [15:0] d;
		integer i;
		begin
			lowest_dirty = 4'hf;
			for (i = 15; i >= 0; i = i - 1)
				if (d[i])
					lowest_dirty = i[3:0];
		end
	endfunction
	always @(posedge clk or negedge rst_n)
		if (!rst_n) begin
			state <= PC_IDLE;
			page_loading <= 0;
			page_done <= 0;
			spi_req <= 0;
			spi_rw <= 0;
			spi_addr <= 24'h000000;
			spi_wdata <= 8'h00;
			iram_wr_en <= 0;
			iram_wr_slot <= 4'h0;
			iram_wr_data <= 16'h0000;
			iram_rd_slot <= 4'h0;
			cur_slot <= 4'h0;
			wb_slot <= 4'h0;
			dirty_snap <= 16'h0000;
			load_hi <= 8'h00;
			target_page <= 8'h00;
		end
		else begin
			page_done <= 0;
			iram_wr_en <= 0;
			spi_req <= spi_req;
			case (state)
				PC_IDLE:
					if (page_req) begin
						page_loading <= 1;
						target_page <= page_next;
						dirty_snap <= dirty_bits;
						cur_slot <= 4'h0;
						state <= PC_WB_SCAN;
					end
				PC_WB_SCAN:
					if (dirty_snap == 16'h0000) begin
						cur_slot <= 4'h0;
						state <= PC_LOAD_HI;
					end
					else begin
						wb_slot <= lowest_dirty(dirty_snap);
						iram_rd_slot <= lowest_dirty(dirty_snap);
						state <= PC_WB_HI;
					end
				PC_WB_HI:
					if (!spi_req && !spi_ready) begin
						spi_req <= 1;
						spi_rw <= 1;
						spi_addr <= {page_current, 4'b0000, wb_slot, 1'b0};
						spi_wdata <= iram_rd_data[15:8];
						state <= PC_WB_WAIT_HI;
					end
				PC_WB_WAIT_HI:
					if (spi_ready && spi_req) begin
						spi_req <= 0;
						state <= PC_WB_LO;
					end
				PC_WB_LO:
					if (!spi_req && !spi_ready) begin
						spi_req <= 1;
						spi_rw <= 1;
						spi_addr <= {page_current, 4'b0000, wb_slot, 1'b1};
						spi_wdata <= iram_rd_data[7:0];
						state <= PC_WB_WAIT_LO;
					end
				PC_WB_WAIT_LO:
					if (spi_ready && spi_req) begin
						spi_req <= 0;
						dirty_snap[wb_slot] <= 0;
						state <= PC_WB_SCAN;
					end
				PC_LOAD_HI:
					if (!spi_req && !spi_ready) begin
						spi_req <= 1;
						spi_rw <= 0;
						spi_addr <= {target_page, 4'b0000, cur_slot, 1'b0};
						state <= PC_WAIT_HI;
					end
				PC_WAIT_HI:
					if (spi_ready && spi_req) begin
						load_hi <= spi_rdata;
						spi_req <= 0;
						state <= PC_LOAD_LO;
					end
				PC_LOAD_LO:
					if (!spi_req && !spi_ready) begin
						spi_req <= 1;
						spi_rw <= 0;
						spi_addr <= {target_page, 4'b0000, cur_slot, 1'b1};
						state <= PC_WAIT_LO;
					end
				PC_WAIT_LO:
					if (spi_ready && spi_req) begin
						spi_req <= 0;
						iram_wr_data <= {load_hi, spi_rdata};
						state <= PC_WRITE_IRAM;
					end
				PC_WRITE_IRAM: begin
					iram_wr_en <= 1;
					iram_wr_slot <= cur_slot;
					if (cur_slot == 4'hf)
						state <= PC_DONE;
					else begin
						cur_slot <= cur_slot + 1;
						state <= PC_LOAD_HI;
					end
				end
				PC_DONE: begin
					page_loading <= 0;
					page_done <= 1;
					state <= PC_IDLE;
				end
				default: state <= PC_IDLE;
			endcase
		end
endmodule
