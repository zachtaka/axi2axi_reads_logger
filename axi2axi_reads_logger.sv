
module axi2axi_reads_logger  #(
	parameter int MTIDW = 1,
	parameter int STIDW = 1,
	parameter int AW  = 32,
	parameter int DW  = 64,
	parameter int USERW  = 1,
	parameter int TB_SLAVES = 1,
	parameter logic[TB_SLAVES*AW-1:0] ADDR_BASE,
	parameter logic[AW-1:0] ADDR_MAX
	)(
	input clk,    // Clock
	input rst_n,  // Asynchronous reset active low

	// Master side axi2axi
		// AR (Read Address) channel (NI -> Target)
		input  logic[MTIDW-1:0]                  axi_ar_id_m,    // ARID
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
		input logic[MTIDW-1:0]                  axi_r_id_m,     // RID
		input logic[DW-1:0]                     axi_r_data_m,   // RDATA
		input logic[1:0]                            axi_r_resp_m,   // RRESP
		input logic                                  axi_r_last_m,   // RLAST
		input logic[USERW-1:0]              axi_r_user_m,   // RUSER
		input logic                                  axi_r_valid_m,  // RVALID
		input  logic                                   axi_r_ready_m,   // RREADY

	// Slave side axi2axi
		// AR (Read Address) 
		input logic [TB_SLAVES-1:0] [STIDW-1:0]                     axi_ar_id_s,    // ARID
		input logic [TB_SLAVES-1:0] [AW-1:0]                        axi_ar_addr_s,  // ARADDR
		input logic [TB_SLAVES-1:0] [7:0]                               axi_ar_len_s,   // ARLEN
		input logic [TB_SLAVES-1:0] [2:0]                               axi_ar_size_s,  // ARSIZE
		input logic [TB_SLAVES-1:0] [1:0]                               axi_ar_burst_s, // ARBURST
		input logic [TB_SLAVES-1:0] [1:0]                               axi_ar_lock_s,  // ARLOCK / 2-bit always for AMBA==3 compliance, but MSB is always tied to zero (no locked support)
		input logic [TB_SLAVES-1:0] [3:0]                               axi_ar_cache_s, // ARCACHE
		input logic [TB_SLAVES-1:0] [2:0]                               axi_ar_prot_s,  // ARPROT
		input logic [TB_SLAVES-1:0] [3:0]                               axi_ar_qos_s,   // ARQOS
		input logic [TB_SLAVES-1:0] [3:0]                               axi_ar_region_s,// ARREGION
		input logic [TB_SLAVES-1:0] [USERW-1:0]                 axi_ar_user_s,  // ARUSER
		input logic [TB_SLAVES-1:0]                                      axi_ar_valid_s, // ARVALID
		input logic [TB_SLAVES-1:0]                                        axi_ar_ready_s, // ARREADY
		// R (Read Data) 
		input logic [TB_SLAVES-1:0] [STIDW-1:0]                      axi_r_id_s,     // RID
		input logic [TB_SLAVES-1:0] [DW-1:0]                         axi_r_data_s,   // RDATA
		input logic [TB_SLAVES-1:0] [1:0]                                axi_r_resp_s,   // RRESP
		input logic [TB_SLAVES-1:0]                                       axi_r_last_s,   // RLAST
		input logic [TB_SLAVES-1:0] [USERW-1:0]                  axi_r_user_s,   // RUSER
		input logic [TB_SLAVES-1:0]                                       axi_r_valid_s,  // RVALID
		input logic [TB_SLAVES-1:0]                                     axi_r_ready_s,   // RREADY

		output logic both_mailbox_empty
);
/*------------------------------------------------------------------------------
-- Master Side  
------------------------------------------------------------------------------*/
// signal declarations
// ~ master side FIFO
logic valid_m,ready_m,push_m,pop_m;
logic [$clog2(TB_SLAVES)+MTIDW+AW+3-1:0] push_data_m,pop_data_m;
// ~ AR signal buffers
logic [AW-1:0] ar_address_buffer_m;
logic [2:0] ar_size_buffer_m,ar_size_m;
logic [MTIDW-1:0]   id_buffer_m,fifo_id,ar_id_m;
logic first_read_data_beat_m;
logic [AW-1:0] fifo_addr,ar_addr_m;
logic [2:0] fifo_size;
logic ar_ack_m,r_ack_m;
// ~ R data
logic [AW+8-1:0] input_m;

// Master side mailbox
mailbox #(logic [AW+8-1:0]) master_side_mail = new();

// Master side fifo, to store aligned starting address, size, id
fifo_duth #(
		.DATA_WIDTH($clog2(TB_SLAVES)+MTIDW+AW+3),
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

logic [$clog2(TB_SLAVES)-1:0] expected_slave,fifo_expected_slave;
logic [TB_SLAVES-1:0] slave_en; 
always_comb begin 
	expected_slave = axi_ar_addr_m [4*3 +: 8];
	if(ar_ack_m && ready_m) begin
		push_m =1'b1;
		push_data_m = {expected_slave,axi_ar_id_m,axi_ar_addr_m & ~((1<<axi_ar_size_m)-1),axi_ar_size_m};
	end else begin 
		push_m =0;
	end

	if(axi_r_last_m & r_ack_m && valid_m) begin
		pop_m = 1'b1;
	end else begin 
		pop_m = 0;
	end

	
	{fifo_expected_slave,fifo_id,fifo_addr,fifo_size}=pop_data_m;

	// slave_en is onehot version of expected slave
	for (int i = 0; i < TB_SLAVES; i++) begin
		if(i==fifo_expected_slave) begin
			slave_en[i]=1'b1;
		end else begin 
			slave_en[i]=0;
		end
	end

	

end

assign ar_ack_m = axi_ar_valid_m && axi_ar_ready_m;
assign r_ack_m = axi_r_valid_m && axi_r_ready_m;



always_ff @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		ar_address_buffer_m <= 0;
		ar_size_buffer_m <= 0;
		id_buffer_m <= 0;
		first_read_data_beat_m <=1'b1;
	end else begin
		// first data beat
		if(r_ack_m && axi_r_last_m) begin
			first_read_data_beat_m<=1'b1;
		end else if (r_ack_m) begin 
			first_read_data_beat_m<=0;
		end

		// store AR info to buffers
		if(first_read_data_beat_m) begin
			// if(push_m && ~valid_m) begin
			// 	ar_address_buffer_m <= axi_ar_addr_m & ~((1<<axi_ar_size_m)-1); // CHANGE THIS TO: aligned address
			// 	ar_size_buffer_m <= axi_ar_size_m;
			// 	id_buffer_m <= axi_ar_id_m;
			// end else 
			if(valid_m) begin
				{ar_id_m,ar_addr_m,ar_size_m} = pop_data_m;
				ar_address_buffer_m <= ar_addr_m & ~((1<<ar_size_m)-1); // CHANGE THIS TO: aligned address
				ar_size_buffer_m <= ar_size_m;
				id_buffer_m <= ar_id_m;
			end
		end 

		// Log R data beats
		if(r_ack_m) begin
			automatic int j=0;
			// increment address
			ar_address_buffer_m<=ar_address_buffer_m+2**ar_size_buffer_m;
			
			for (int i = ar_address_buffer_m%(DW/8); i < ar_address_buffer_m%(DW/8)+2**ar_size_buffer_m; i++) begin
				$display("Master Mailbox :: Put @%t \t Address=%0h \t Data=%0h",$time(),ar_address_buffer_m + j,axi_r_data_m[i*8+:8]);
				input_m = {ar_address_buffer_m + j, axi_r_data_m[i*8+:8]};
				assert (master_side_mail.try_put(input_m)) else $fatal("couldn't put data to master side mailbox!");
				j++;
			end
			$display("/*******************************************************************/");
		end

		/*------------------------------------------------------------------------------
		--  ID CHECK
		------------------------------------------------------------------------------*/
		if(r_ack_m) begin
			if(axi_r_id_m==fifo_id) begin
				$display("ID is okay");
			end else begin 
				$fatal(1,"ID is wrong? ar_id=%0d and r_id=%0d",fifo_id,axi_r_id_m);
			end
		end


	end
end

/*------------------------------------------------------------------------------
--  Slave side
------------------------------------------------------------------------------*/
// signal declarations
// ~ master side FIFO
logic [TB_SLAVES-1:0] valid_s,ready_s,push_s,pop_s;
logic [TB_SLAVES-1:0][STIDW+AW+3-1:0] push_data_s,pop_data_s;
logic [TB_SLAVES-1:0] ar_ack_s, r_ack_s;

logic [TB_SLAVES-1:0][AW-1:0]ar_address_buffer_s,ar_addr_s;
logic [TB_SLAVES-1:0][2:0] ar_size_buffer_s,ar_size_s;
logic [TB_SLAVES-1:0][STIDW-1:0] id_buffer_s,ar_id_s;
logic [TB_SLAVES-1:0] first_read_data_beat_s;


logic [AW+8-1:0]  input_s;

// Slave side mailbox
mailbox #(logic [AW+8-1:0]) slave_side_mail = new();

for (genvar s = 0; s < TB_SLAVES; s++) begin
	
	// Slave side fifo, to store aligned starting address, size, id
	fifo_duth #(
			.DATA_WIDTH(STIDW+AW+3),
			.RAM_DEPTH(10)
		) slave_side_fifo (
			.clk       (clk),
			.rst       (~rst_n),
			.push_data (push_data_s[s]),
			.push      (push_s[s]),
			.ready     (ready_s[s]),
			.pop_data  (pop_data_s[s]),
			.valid     (valid_s[s]),
			.pop       (pop_s[s])
		);
end

always_comb begin 
	for (int s = 0; s < TB_SLAVES; s++) begin
		ar_ack_s[s] = axi_ar_valid_s[s] && axi_ar_ready_s[s];
		r_ack_s[s] = axi_r_valid_s[s] && axi_r_ready_s[s];
		{ar_id_s[s],ar_addr_s[s],ar_size_s[s]} = pop_data_s[s];

		if(ar_ack_s[s]) begin
			push_s[s] =1'b1;
			push_data_s[s] = {axi_ar_id_s[s],axi_ar_addr_s[s] & ~((1<<axi_ar_size_s[s])-1),axi_ar_size_s[s]};
		end else begin 
			push_s[s] =0;
		end

		if(axi_r_last_s[s] & r_ack_s[s]) begin
			pop_s[s] = 1'b1;
		end else begin 
			pop_s[s] = 0;
		end
	end
end


for (genvar s = 0; s < TB_SLAVES; s++) begin
	always_ff @(posedge clk or negedge rst_n) begin
		if(~rst_n) begin
			ar_address_buffer_s[s] <= 0;
			ar_size_buffer_s[s] <= 0;
			id_buffer_s[s] <= 0;
			first_read_data_beat_s[s] <= 1'b1;
		end else begin
			// first data beat
			if(r_ack_s[s] && axi_r_last_s[s]) begin
				first_read_data_beat_s[s]<=1'b1;
			end else if (r_ack_s[s]) begin 
				first_read_data_beat_s[s]<=0;
			end

			// store AR info to buffers
			if(first_read_data_beat_s[s]) begin
				if(push_s[s] && ~valid_s[s]) begin
					ar_address_buffer_s[s] <= axi_ar_addr_s[s] & ~((1<<axi_ar_size_s[s])-1); // CHANGE THIS TO: aligned address
					ar_size_buffer_s[s] <= axi_ar_size_s[s];
					id_buffer_s[s] <= axi_ar_id_s[s];
				end else if(valid_s[s]) begin
					ar_address_buffer_s[s] <= ar_addr_s[s] & ~((1<<ar_size_s[s])-1); // CHANGE THIS TO: aligned address
					ar_size_buffer_s[s] <= ar_size_s[s];
					id_buffer_s[s] <= ar_id_s[s];
				end
			end 

			if(r_ack_s[s] && ~slave_en[s]) begin
				$fatal("Data comes from slave %0d but he is not enabled");
			end
			if(r_ack_s[s] && slave_en[s]) begin
				automatic int j=0;
				first_read_data_beat_s[s]<=0;
				ar_address_buffer_s[s]<=ar_address_buffer_s[s]+2**ar_size_buffer_s[s];
				
				
				$display("/*******************************************************************/");
				for (int i = ar_address_buffer_s[s]%(DW/8); i < ar_address_buffer_s[s]%(DW/8)+2**ar_size_buffer_s[s]; i++) begin
					$display("Slave Mailbox :: Put @%t \t Address=%0h \t Data=%0h",$time(),ar_address_buffer_s[s] + j,axi_r_data_s[s][i*8+:8]);
					input_s = {ar_address_buffer_s[s] + j, axi_r_data_s[s][i*8+:8]};
					assert (slave_side_mail.try_put(input_s)) else $fatal("couldn't put data to slave side mailbox!");
					j++;
				end


				if(axi_r_last_s[s]) begin
					first_read_data_beat_s[s]<=1'b1;
				end else begin 
					first_read_data_beat_s[s]<=0;
				end

			end

		
		end
	end
end


/*------------------------------------------------------------------------------
--  Comparing Master side Mailbox & Slave side Mailbox
------------------------------------------------------------------------------*/
logic [AW-1:0] address_1,address_2;
logic [7:0] data_1,data_2;
logic [AW+8-1:0] pkt_1,pkt_2;

initial begin 
	while (1) begin 
		if(master_side_mail.num()>0 && slave_side_mail.num()>0) begin
			// Master side mailbox unboxing
			master_side_mail.try_get(pkt_1);
			{address_1,data_1} = pkt_1;

			// Slave side mailbox unboxing
			slave_side_mail.try_get(pkt_2);
			{address_2,data_2} = pkt_2;

			if(address_1==address_2 && data_1==data_2) begin
				$display("everything okay");
			end else begin 
				$fatal(1,"Master side Mailbox got address=%0h data=%0h \nSlave side Mailbox got address=%0h data=%0h",address_1,data_1,address_2,data_2);
			end
		end else begin 
			#5;
		end
		

	end
end
assign both_mailbox_empty = ((master_side_mail.num()==0)&&(slave_side_mail.num()==0));




for (genvar i = 0; i < TB_SLAVES; i++) begin
	mailbox slave_mailbox = new();
end


endmodule