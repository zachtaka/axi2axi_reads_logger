module axi2axi_logger #(
	parameter AHB_DATA_WIDTH=64,
	parameter AHB_ADDRESS_WIDTH=32,
	parameter int TIDW = 1,
	parameter int AW  = 32,
	parameter int DW  = 64,
	parameter int USERW  = 1
	)(
	input clk,    // Clock
	input rst_n,  // Asynchronous reset active low

	// Master side axi2axi
		// AR (Read Address) channel (NI -> Target)
		input  logic[TIDW-1:0]                  axi_ar_id_m,    // ARID
		input  logic[AW-1:0]                     axi_ar_addr_m,  // ARADDR
		input  logic[7:0]                            axi_ar_len_m,   // ARLEN
		input  logic[2:0]                            axi_ar_size_m,  // ARSIZE
		input  logic[1:0]                            axi_ar_burst_m, // ARBURST
		input  logic[1:0]                            axi_ar_lock_m,  // ARLOCK / 2-bit always for AMBA==3 compliance, but MSB is always tied to zero (no locked support)
		input  logic[3:0]                            axi_ar_cache_m, // ARCACHE
		input  logic[2:0]                            axi_ar_prot_m,  // ARPROT
		input  logic[3:0]                            axi_ar_qos_m,   // ARQOS
		input  logic[3:0]                            axi_ar_region_m,// ARREGION
		input  logic[USERW-1:0]              axi_ar_user_m,  // ARUSER
		input  logic                                  axi_ar_valid_m, // ARVALID
		input logic                                 axi_ar_ready_m, // ARREADY
		// R (Read Data) channel (Target -> NI)
		input logic[TIDW-1:0]                  axi_r_id_m,     // RID
		input logic[DW-1:0]                     axi_r_data_m,   // RDATA
		input logic[1:0]                            axi_r_resp_m,   // RRESP
		input logic                                  axi_r_last_m,   // RLAST
		input logic[USERW-1:0]              axi_r_user_m,   // RUSER
		input logic                                  axi_r_valid_m,  // RVALID
		input  logic                                   axi_r_ready_m,   // RREADY

	// Slave side axi2axi
		// AR (Read Address) 
		input logic[TIDW-1:0]                     axi_ar_id_s,    // ARID
		input logic[AW-1:0]                        axi_ar_addr_s,  // ARADDR
		input logic[7:0]                               axi_ar_len_s,   // ARLEN
		input logic[2:0]                               axi_ar_size_s,  // ARSIZE
		input logic[1:0]                               axi_ar_burst_s, // ARBURST
		input logic[1:0]                               axi_ar_lock_s,  // ARLOCK / 2-bit always for AMBA==3 compliance, but MSB is always tied to zero (no locked support)
		input logic[3:0]                               axi_ar_cache_s, // ARCACHE
		input logic[2:0]                               axi_ar_prot_s,  // ARPROT
		input logic[3:0]                               axi_ar_qos_s,   // ARQOS
		input logic[3:0]                               axi_ar_region_s,// ARREGION
		input logic[USERW-1:0]                 axi_ar_user_s,  // ARUSER
		input logic                                     axi_ar_valid_s, // ARVALID
		input logic                                       axi_ar_ready_s, // ARREADY
		// R (Read Data) 
		input logic[TIDW-1:0]                      axi_r_id_s,     // RID
		input logic[DW-1:0]                         axi_r_data_s,   // RDATA
		input logic[1:0]                                axi_r_resp_s,   // RRESP
		input logic                                      axi_r_last_s,   // RLAST
		input logic[USERW-1:0]                  axi_r_user_s,   // RUSER
		input logic                                      axi_r_valid_s,  // RVALID
		input  logic                                   axi_r_ready_s   // RREADY
);

mailbox id_addr_data_source = new();
mailbox id_addr_data_dest = new();

/*------------------------------------------------------------------------------
--  Master side
------------------------------------------------------------------------------*/
logic valid_m,ready_m,push_m,pop_m;
logic [TIDW+AW+3-1:0] push_data_m,pop_data_m;
logic [AHB_ADDRESS_WIDTH-1:0] ar_address_buffer_m,address_out_source;
logic [2:0] ar_size_buffer_m,ar_size_m;
logic [TIDW-1:0]   id_buffer_m,ar_id_m,id_out_source;
logic [AW-1:0]ar_addr_m;
logic [7:0] data_out_source;
int log_file;
logic [TIDW-1:0] fifo_id;
logic [AW-1:0] fifo_addr;
logic [2:0] fifo_size;
initial begin 
	log_file = $fopen("axi_bytes_log_master_side.txt", "w") ;
end

assign ar_ack_m = axi_ar_valid_m && axi_ar_ready_m;
assign r_ack_m = axi_r_valid_m && axi_r_ready_m;

logic first_read_data_beat_m;
logic [TIDW+AHB_ADDRESS_WIDTH+8-1:0] mail_input_source,mail_output_source;
always_ff @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		ar_address_buffer_m <= 0;
		ar_size_buffer_m <= 0;
		id_buffer_m <= 0;
		first_read_data_beat_m <=1'b1;
	end else begin


		if(first_read_data_beat_m) begin
			if(push_m && ~valid_m) begin
				ar_address_buffer_m <= axi_ar_addr_m & ~((1<<axi_ar_size_m)-1); // CHANGE THIS TO: aligned address
				ar_size_buffer_m <= axi_ar_size_m;
				id_buffer_m <= axi_ar_id_m;
			end else if(valid_m) begin
				{ar_id_m,ar_addr_m,ar_size_m} = pop_data_m;
				ar_address_buffer_m <= ar_addr_m & ~((1<<ar_size_m)-1); // CHANGE THIS TO: aligned address
				ar_size_buffer_m <= ar_size_m;
				id_buffer_m <= ar_id_m;
			end
		
			
		end 

	
		if(r_ack_m) begin
			automatic int j=0;
			first_read_data_beat_m<=0;
			ar_address_buffer_m<=ar_address_buffer_m+2**ar_size_buffer_m;
			

			for (int i = ar_address_buffer_m%(DW/8); i < ar_address_buffer_m%(DW/8)+2**ar_size_buffer_m; i++) begin
				$fwrite(log_file,"ID=%0d address=%0h byte=%h\n", id_buffer_m, ar_address_buffer_m + j, axi_r_data_m[i*8+:8]);
				mail_input_source = {ar_address_buffer_m + j, axi_r_data_m[i*8+:8]};
				id_addr_data_source.try_put(mail_input_source);
				j++;
			end


			if(axi_r_last_m) begin
				first_read_data_beat_m<=1'b1;
			end else begin 
				first_read_data_beat_m<=0;
			end

		end

		/*------------------------------------------------------------------------------
		--  ID CHECK
		------------------------------------------------------------------------------*/

		if(r_ack_m) begin
			if(axi_r_id_m==fifo_id) begin
				// $fatal(1,"no worries");
				$display("ID is okay");
			end else begin 
				$fatal(1,"ID is wrong? ar_id=%0d and r_id=%0d",fifo_id,axi_r_id_m);
			end
		end

	end
end

// initial begin 
// 	while (1) begin 
// 		if(id_addr_data_source.num()>0) begin
// 			id_addr_data_source.try_get(mail_output_source);
// 			{address_out_source,data_out_source} = mail_output_source;
// 			$display("Master_side Mailbox got address=%0h data=%0h",address_out_source,data_out_source);
// 		end else begin 
// 			#5 ;
// 		end

// 	end

// end


fifo_duth #(
		.DATA_WIDTH(TIDW+AHB_ADDRESS_WIDTH+3),
		.RAM_DEPTH(10)
	) master_side_fifo (
		.clk       (clk),
		.rst       (~rst_n),
		.push_data (push_data_m),
		.push      (push_m),
		.ready     (ready_m),
		.pop_data  (pop_data_m),
		.valid     (valid_m),
		.pop       (pop_m)
	);

always_comb begin 
	if(ar_ack_m) begin
		push_m =1'b1;
		push_data_m = {axi_ar_id_m,axi_ar_addr_m & ~((1<<axi_ar_size_m)-1),axi_ar_size_m};
	end else begin 
		push_m =0;
	end

	if(axi_r_last_m & r_ack_m) begin
		pop_m = 1'b1;
	end else begin 
		pop_m = 0;
	end

	{fifo_id,fifo_addr,fifo_size}=pop_data_m;
	

end

/*------------------------------------------------------------------------------
--  Slave side
------------------------------------------------------------------------------*/
logic valid_s,ready_s,push_s,pop_s;
logic [TIDW+AW+3-1:0] push_data_s,pop_data_s;
logic [AHB_ADDRESS_WIDTH-1:0] ar_address_buffer_s,address_out_dest;
logic [2:0] ar_size_buffer_s,ar_size_s;
logic [TIDW-1:0]   id_buffer_s,ar_id_s,id_out_dest;
logic [AW-1:0]ar_addr_s;
logic [7:0] data_out_dest;
int log_file2;
initial begin 
	log_file2 = $fopen("axi_bytes_log_slave_side.txt", "w") ;
end

assign ar_ack_s = axi_ar_valid_s && axi_ar_ready_s;
assign r_ack_s = axi_r_valid_s && axi_r_ready_s;

logic first_read_data_beat_s;
logic [TIDW+AHB_ADDRESS_WIDTH+8-1:0] mail_input_dest,mail_output_dest;
always_ff @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		ar_address_buffer_s <= 0;
		ar_size_buffer_s <= 0;
		id_buffer_s <= 0;
		first_read_data_beat_s <=1'b1;
	end else begin


		if(first_read_data_beat_s) begin
			if(push_s && ~valid_s) begin
				ar_address_buffer_s <= axi_ar_addr_s & ~((1<<axi_ar_size_s)-1); // CHANGE THIS TO: aligned address
				ar_size_buffer_s <= axi_ar_size_s;
				id_buffer_s <= axi_ar_id_s;
			end else if(valid_s) begin
				{ar_id_s,ar_addr_s,ar_size_s} = pop_data_s;
				ar_address_buffer_s <= ar_addr_s & ~((1<<ar_size_s)-1); // CHANGE THIS TO: aligned address
				ar_size_buffer_s <= ar_size_s;
				id_buffer_s <= ar_id_s;
			end
		end 


		if(r_ack_s) begin
			automatic int j=0;
			first_read_data_beat_s<=0;
			ar_address_buffer_s<=ar_address_buffer_s+2**ar_size_buffer_s;
			

			for (int i = ar_address_buffer_s%(DW/8); i < ar_address_buffer_s%(DW/8)+2**ar_size_buffer_s; i++) begin
				$fwrite(log_file2,"ID=%0d address=%0h byte=%h\n", id_buffer_s, ar_address_buffer_s + j, axi_r_data_s[i*8+:8]);
				mail_input_dest = {ar_address_buffer_s + j, axi_r_data_s[i*8+:8]};
				id_addr_data_dest.try_put(mail_input_dest);
				j++;
			end


			if(axi_r_last_s) begin
				first_read_data_beat_s<=1'b1;
			end else begin 
				first_read_data_beat_s<=0;
			end

		end




	end
end

// initial begin 
// 	while (1) begin 
// 		if(id_addr_data_dest.num()>0) begin
// 			id_addr_data_dest.try_get(mail_output_dest);
// 			{address_out_dest,data_out_dest} = mail_output_dest;
// 			$display("Slave_side Mailbox got address=%0h data=%0h",address_out_dest,data_out_dest);
// 		end else begin 
// 			#5 ;
// 		end

// 	end

// end


fifo_duth #(
		.DATA_WIDTH(TIDW+AHB_ADDRESS_WIDTH+3),
		.RAM_DEPTH(10)
	) slave_side_fifo (
		.clk       (clk),
		.rst       (~rst_n),
		.push_data (push_data_s),
		.push      (push_s),
		.ready     (ready_s),
		.pop_data  (pop_data_s),
		.valid     (valid_s),
		.pop       (pop_s)
	);

always_comb begin 
	if(ar_ack_s) begin
		push_s =1'b1;
		push_data_s = {axi_ar_id_s,axi_ar_addr_s & ~((1<<axi_ar_size_s)-1),axi_ar_size_s};
	end else begin 
		push_s =0;
	end

	if(axi_r_last_s & r_ack_s) begin
		pop_s = 1'b1;
	end else begin 
		pop_s = 0;
	end

end




initial begin 
	while (1) begin 
		if(id_addr_data_dest.num()>0 && id_addr_data_source.num()>0) begin
			id_addr_data_dest.try_get(mail_output_dest);
			{address_out_dest,data_out_dest} = mail_output_dest;
			// $display("Slave_side Mailbox got address=%0h data=%0h",address_out_dest,data_out_dest);

			id_addr_data_source.try_get(mail_output_source);
			{address_out_source,data_out_source} = mail_output_source;
			// $display("Master_side Mailbox got address=%0h data=%0h",address_out_source,data_out_source);

			if(address_out_source==address_out_dest && data_out_source==data_out_dest) begin
				$display("everything okay");
			end else begin 
				$fatal(1,"Slave_side Mailbox got address=%0h data=%0h \nMaster_side Mailbox got address=%0h data=%0h",address_out_dest,data_out_dest,address_out_source,data_out_source);
			end


		end else begin 
			#5 ;
		end

	end

end




endmodule