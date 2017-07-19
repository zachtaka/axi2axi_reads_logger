
module axi2axi_reads_logger  #(
	parameter int MTIDW = 1,
	parameter int STIDW = 1,
	parameter int AW  = 32,
	parameter int DW  = 64,
	parameter int USERW  = 1,
	parameter int TB_MASTERS = 1,
	parameter int TB_SLAVES = 1
	)(
	input clk,    // Clock
	input rst_n,  // Asynchronous reset active low

	// Master side axi2axi
		// AR (Read Address) channel (NI -> Target)
		input  logic[TB_MASTERS-1:0][MTIDW-1:0]                  axi_ar_id_m,    // ARID
		input  logic[TB_MASTERS-1:0][AW-1:0]                     axi_ar_addr_m,  // ARADDR
		input  logic[TB_MASTERS-1:0][7:0]                            axi_ar_len_m,   // ARLEN
		input  logic[TB_MASTERS-1:0][2:0]                            axi_ar_size_m,  // ARSIZE
		input  logic[TB_MASTERS-1:0][1:0]                            axi_ar_burst_m, // ARBURST
		input  logic[TB_MASTERS-1:0][1:0]                            axi_ar_lock_m,  // ARLOCK / 2-bit always for AMBA==3 compliance, but MSB is always tied to zero (no locked support)
		input  logic[TB_MASTERS-1:0][3:0]                            axi_ar_cache_m, // ARCACHE
		input  logic[TB_MASTERS-1:0][2:0]                            axi_ar_prot_m,  // ARPROT
		input  logic[TB_MASTERS-1:0][3:0]                            axi_ar_qos_m,   // ARQOS
		input  logic[TB_MASTERS-1:0][3:0]                            axi_ar_region_m,// ARREGION
		input  logic[TB_MASTERS-1:0][USERW-1:0]              axi_ar_user_m,  // ARUSER
		input  logic[TB_MASTERS-1:0]                                  axi_ar_valid_m, // ARVALID
		input logic[TB_MASTERS-1:0]                                 axi_ar_ready_m, // ARREADY
		// R (Read Data) channel (Target -> NI)
		input logic[TB_MASTERS-1:0][MTIDW-1:0]                  axi_r_id_m,     // RID
		input logic[TB_MASTERS-1:0][DW-1:0]                     axi_r_data_m,   // RDATA
		input logic[TB_MASTERS-1:0][1:0]                            axi_r_resp_m,   // RRESP
		input logic[TB_MASTERS-1:0]                                  axi_r_last_m,   // RLAST
		input logic[TB_MASTERS-1:0][USERW-1:0]              axi_r_user_m,   // RUSER
		input logic[TB_MASTERS-1:0]                                  axi_r_valid_m,  // RVALID
		input  logic[TB_MASTERS-1:0]                                   axi_r_ready_m,   // RREADY

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
logic [TB_MASTERS-1:0]valid_m,ready_m,push_m,pop_m;
logic [TB_MASTERS-1:0][8+MTIDW+AW+3-1:0] push_data_m,pop_data_m;
// ~ AR signal buffers
logic [TB_MASTERS-1:0][AW-1:0] ar_address_buffer_m;
logic [TB_MASTERS-1:0][2:0] ar_size_buffer_m,ar_size_m;
logic [TB_MASTERS-1:0][MTIDW-1:0]   id_buffer_m,fifo_id,ar_id_m;
logic [TB_MASTERS-1:0]first_read_data_beat_m;
logic [TB_MASTERS-1:0][AW-1:0] fifo_addr,ar_addr_m;
logic [TB_MASTERS-1:0][2:0] fifo_size;
logic [TB_MASTERS-1:0]ar_ack_m,r_ack_m;
// ~ R data
logic [TB_MASTERS-1:0][1+AW+8-1:0] input_m;


// Master side mailbox
mailbox #(logic [1+AW+8-1:0]) master_side_mail [TB_MASTERS];
initial begin 
	for (int i = 0; i < TB_MASTERS; i++) begin
		master_side_mail[i] = new();
	end
end

for (genvar m = 0; m < TB_MASTERS; m++) begin
	// Master side fifo, to store aligned starting address, size, id
	fifo_duth #(
			.DATA_WIDTH(8+MTIDW+AW+3),
			.RAM_DEPTH(10)
		) master_side_fifo (
			.clk       (clk),
			.rst       (~rst_n),
			.push_data (push_data_m[m]),
			.push      (push_m[m]),
			.ready     (ready_m[m]),
			.pop_data  (pop_data_m[m]),
			.valid     (valid_m[m]),
			.pop       (pop_m[m])
		);
end


logic [TB_MASTERS-1:0][8-1:0] expected_slave;
logic [TB_MASTERS-1:0][TB_SLAVES-1:0] slave_en;
logic [TB_MASTERS-1:0][8-1:0] fifo_expected_slave; 
always_comb begin 
	for (int m = 0; m < TB_MASTERS; m++) begin
		ar_ack_m[m] = axi_ar_valid_m[m] && axi_ar_ready_m[m];
		r_ack_m[m] = axi_r_valid_m[m] && axi_r_ready_m[m];


		expected_slave[m] = axi_ar_addr_m[m][4*3 +: 8];
		// slave_en is onehot version of expected slave
		for (int s = 0; s < TB_SLAVES; s++) begin
			if(s==expected_slave[m]) begin
				slave_en[m][s]=1'b1;
			end else begin 
				slave_en[m][s]=0;
			end
		end

		if(ar_ack_m[m] && ready_m[m]) begin
			push_m[m] =1'b1;
			push_data_m[m] = {expected_slave[m],axi_ar_id_m[m],axi_ar_addr_m[m] & ~((1<<axi_ar_size_m[m])-1),axi_ar_size_m[m]};
		end else begin 
			push_m[m] =0;
		end

		if(axi_r_last_m[m] & r_ack_m[m] && valid_m[m]) begin
			pop_m[m] = 1'b1;
		end else begin 
			pop_m[m] = 0;
		end

		{fifo_expected_slave[m],fifo_id[m],fifo_addr[m],fifo_size[m]}=pop_data_m[m];

		
		
	end
	


	
end




always_ff @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		for (int m = 0; m < TB_MASTERS; m++) begin
			ar_address_buffer_m[m] <= 0;
			ar_size_buffer_m[m] <= 0;
			id_buffer_m[m] <= 0;
			first_read_data_beat_m[m] <=1'b1;
		end
	end else begin
		for (int m = 0; m < TB_MASTERS; m++) begin
			// first data beat
			if(r_ack_m[m] && axi_r_last_m[m]) begin
				first_read_data_beat_m[m]<=1'b1;
			end else if (r_ack_m[m]) begin 
				first_read_data_beat_m[m]<=0;
			end
			// store AR info to buffers
			if(first_read_data_beat_m[m]) begin
				if(valid_m[m]) begin
					{ar_id_m[m],ar_addr_m[m],ar_size_m[m]} = pop_data_m[m];
					ar_address_buffer_m[m] <= ar_addr_m[m] & ~((1<<ar_size_m[m])-1); // CHANGE THIS TO: aligned address
					ar_size_buffer_m[m] <= ar_size_m[m];
					id_buffer_m[m] <= ar_id_m[m];
				end
			end 


			// Log R data beats
			if(r_ack_m[m]) begin
				automatic int j=0;
				// increment address
				ar_address_buffer_m[m]<=ar_address_buffer_m[m]+2**ar_size_buffer_m[m];
				
				for (int i = ar_address_buffer_m[m]%(DW/8); i < ar_address_buffer_m[m]%(DW/8)+2**ar_size_buffer_m[m]; i++) begin
					$display("Master Mailbox[%0d] :: Put @%t \t Last=%0h Address=%0h \t Data=%0h",m,$time(),(i==(ar_address_buffer_m[m]%(DW/8)+2**ar_size_buffer_m[m]-1))&&axi_r_last_m[m],ar_address_buffer_m[m] + j,axi_r_data_m[m][i*8+:8]);
					input_m[m] = {(i==(ar_address_buffer_m[m]%(DW/8)+2**ar_size_buffer_m[m]-1))&&axi_r_last_m[m],ar_address_buffer_m[m] + j, axi_r_data_m[m][i*8+:8]};
					assert (master_side_mail[m].try_put(input_m[m])) else $fatal("couldn't put data to master side mailbox!");
					j++;
				end
				// compare_mailboxes(m,fifo_expected_slave[m],master_side_mail[m],slave_side_mail[fifo_expected_slave[m]]);
				// count_mailboxes(master_side_mail,slave_side_mail);
				if(axi_r_last_m[m]) begin
					
					// count_mailboxes(master_side_mail,slave_side_mail);
				end
				
				
				
				$display("/*******************************************************************/");
			end


			/*------------------------------------------------------------------------------
			--  ID CHECK
			------------------------------------------------------------------------------*/
			if(r_ack_m[m]) begin
				if(axi_r_id_m[m]==fifo_id[m]) begin
					// $display("ID is okay");
				end else begin 
					$fatal(1,"ID is wrong? ar_id=%0d and r_id=%0d",fifo_id[m],axi_r_id_m[m]);
				end
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


logic [TB_SLAVES-1:0][AW+8-1:0]  input_s;

// Slave side mailbox
mailbox #(logic [AW+8-1:0]) slave_side_mail [TB_SLAVES];
initial begin 
	for (int i = 0; i < TB_SLAVES; i++) begin
		slave_side_mail[i] = new();
	end
end
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

			// if(r_ack_s[s] && ~slave_en[0][s]) begin
			// 	$fatal("Data comes from slave %0d but he is not enabled");
			// end
			if(r_ack_s[s]) begin
				automatic int j=0;
				first_read_data_beat_s[s]<=0;
				ar_address_buffer_s[s]<=ar_address_buffer_s[s]+2**ar_size_buffer_s[s];
				
				
				$display("/*******************************************************************/");
				for (int i = ar_address_buffer_s[s]%(DW/8); i < ar_address_buffer_s[s]%(DW/8)+2**ar_size_buffer_s[s]; i++) begin
					$display("Slave Mailbox[%0d] :: Put @%t \t Address=%0h \t Data=%0h",s,$time(),ar_address_buffer_s[s] + j,axi_r_data_s[s][i*8+:8]);
					input_s[s] = {ar_address_buffer_s[s] + j, axi_r_data_s[s][i*8+:8]};
					assert (slave_side_mail[s].try_put(input_s[s])) else $fatal("couldn't put data to slave side mailbox!");
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
logic [TB_MASTERS-1:0] push_expected_slave,pop_expected_slave,valid_expected_slave,ready_expected_slave;
logic [TB_MASTERS-1:0][8-1:0] push_data_expected_slave,pop_data_expected_slave;
logic [TB_MASTERS-1:0] lut_done;

initial begin 
	forever begin 
		for (int m = 0; m < TB_MASTERS; m++) begin
			for (int s = 0; s < TB_SLAVES; s++) begin
				if(s==pop_data_expected_slave[m]) begin
					compare_mailboxes(m,s,master_side_mail[m],slave_side_mail[s],lut_done[m]);
				end
			end
		end
		count_mailboxes(master_side_mail,slave_side_mail);
		 @(posedge clk);
	end
end

// expected_slave fifo for every master
// a master pops an expected_slave when he loots his bytes from him
for (genvar m = 0; m < TB_MASTERS; m++) begin
	fifo_duth #(
			.DATA_WIDTH(8),
			.RAM_DEPTH(1000)
		) expected_slave_fifo (
			.clk       (clk),
			.rst       (~rst_n),
			.push_data (push_data_expected_slave[m]),
			.push      (push_expected_slave[m]),
			.ready     (ready_expected_slave[m]),
			.pop_data  (pop_data_expected_slave[m]),
			.valid     (valid_expected_slave[m]),
			.pop       (pop_expected_slave[m])
		);
end
always_comb begin
	for (int m = 0; m < TB_MASTERS; m++) begin
		if(ar_ack_m[m] && ready_expected_slave[m]) begin
			push_expected_slave[m]=1'b1;
			push_data_expected_slave[m]=expected_slave[m];
		end else begin 
			push_expected_slave[m]=0;
			push_data_expected_slave[m]=0;
		end

		if(lut_done[m]) begin
			pop_expected_slave[m] = 1'b1;
		end else begin 
			pop_expected_slave[m] = 0;
		end
	end

end












task compare_mailboxes(
	input int m,
	input int s,
	input mailbox #(logic [1+AW+8-1:0]) mailbox_1,
	input mailbox #(logic [AW+8-1:0]) mailbox_2,
	output logic lut_done
	);

logic [AW-1:0] address_1,address_1_test;
logic [7:0] data_1,data_1_test;
logic [1+AW+8-1:0] pkt_1,pkt_1_test;

logic [AW-1:0] address_2,address_2_test;
logic [7:0] data_2,data_2_test;
logic [AW+8-1:0]pkt_2,pkt_2_test;
lut_done=0;
while (mailbox_1.num()>0 && mailbox_2.num()>0) begin 
	// before unboxing, trying to peek if master and slave address is the same
	// because maybe some data travelled faster to their master
	assert (mailbox_1.try_peek(pkt_1_test)) else $fatal(".get method failed at mailbox_1");
	{address_1_test,data_1_test} = pkt_1_test; // lut_done = last_byte
	assert (mailbox_2.try_peek(pkt_2_test)) else $fatal(".get method failed at mailbox_2");
	{address_2_test,data_2_test} = pkt_2_test;
	
	if(address_1_test!==address_2_test) begin
		// lut_done=0;
		break; // wait for other master to loot their bytes
	end else begin 
		// lut_done=1'b1;
	end
	// while (address_1_test!==address_2_test) begin 
	// 	#5;
	// end

	// mailbox_1 unboxing
	assert (mailbox_1.try_get(pkt_1)) else $fatal(".get method failed at mailbox_1");
	{lut_done,address_1,data_1} = pkt_1;
	$display("Unboxing master side mailbox[%0d] \t address=%0h \t data=%0h",m,address_1,data_1);

	// mailbox_2 unboxing
	assert (mailbox_2.try_get(pkt_2)) else $fatal(".get method failed at slave mailbox_2");
	{address_2,data_2} = pkt_2;
	$display("Unboxing slave side mailbox[%0d] \t address=%0h \t data=%0h",s,address_2,data_2);

	if(address_1==address_2 && data_1==data_2) begin
		$display("everything okay");
		
	end else begin 
		$fatal(1,"\nmaster side mailbox[%0d] got address=%0h data=%0h \nslave side mailbox[%0d] got address=%0h data=%0h",m,address_1,data_1,s,address_2,data_2);
		
	end
end





endtask : compare_mailboxes

task count_mailboxes(
	input mailbox #(logic [1+AW+8-1:0]) mailbox_m [TB_MASTERS],
	input mailbox #(logic [AW+8-1:0]) mailbox_s [TB_SLAVES]
	);


$display("~~~~~~~MASTER MAILBOX ITEMS COUNT~~~~~~~",);
for (int m = 0; m < TB_MASTERS; m++) begin
	$display("Master Mailbox[%0d].num=%0d",m,mailbox_m[m].num());
end
$display("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~",);


$display("~~~~~~~SLAVE MAILBOX ITEMS COUNT~~~~~~~",);
for (int s = 0; s < TB_SLAVES; s++) begin
	$display("Slave Mailbox[%0d].num=%0d",s,mailbox_s[s].num());
end
$display("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~",);

endtask : count_mailboxes


int debug_file;
initial begin 
	debug_file = $fopen("AR_requests.txt", "w") ;
end
always_ff @(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		
	end else begin
		for (int m = 0; m < TB_MASTERS; m++) begin
			if(ar_ack_m[m]) begin
				$fwrite(debug_file,"@%t \t Master[%0d] \t Address=%0h \t len=%0d \t size=%0d\n",$time(),m,axi_ar_addr_m[m],axi_ar_len_m[m],axi_ar_size_m[m]);
			end
		end
		
		for (int s = 0; s < TB_SLAVES; s++) begin
			if(ar_ack_s[s]) begin
				$fwrite(debug_file,"@%t \t Slave[%0d] \t Address=%0h \t len=%0d \t size=%0d\n",$time(),s,axi_ar_addr_s[s],axi_ar_len_s[s],axi_ar_size_s[s]);
			end
		end
	end
end

endmodule