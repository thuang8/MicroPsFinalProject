
//**********************************************************************************************************************************************************//
//																RAM / Memory related modules																//
//**********************************************************************************************************************************************************//


//////////////////////////////////////////////////////////////////////
//					EBR CONTROLLER MODULE							//
//																	//
// The memory of the EBR will be wiped when reset is asserted.		//
// This means it will take two clock cycles to actually write the   //
// display data: one to clear, one to write new data			    //
//																	//
//////////////////////////////////////////////////////////////////////

module EBRController(input logic clk,
					 input logic reset,
					 input logic w_enable,
					 input logic r_enable,
					 input logic get_buffer,
					 input logic [5:0] y_in [63:0],
					 input logic [5:0] row_0_sel ,
					 input logic [5:0] row_1_sel ,
					 output logic [63:0] row_0 ,
					 output logic [63:0]row_1 );
					 
					 
	logic [63:0] out_write_buffer [63:0];
	EBRWriteControl WriteController(clk,reset,w_enable, get_buffer, y_in,out_write_buffer);
	EBRReadControl ReadController(clk, reset, r_enable, get_buffer, row_0_sel, row_1_sel,out_write_buffer, row_0, row_1 );

endmodule

typedef enum logic [3:0] {default_WC, clear_WC, write_WC} WriteState;
module EBRWriteControl(input logic clk,
					   input logic reset,
					   input logic w_en,
					   input logic get_buffer,
					   input logic[5:0] y_in[63:0],
					   output logic [63:0] out_write_buffer [63:0] );
	logic [63:0] write_buffer [63:0];
	WriteState current_state, next_state;
	
	//state register
	always_ff @(posedge clk) begin
		if(reset) current_state<=default_WC;
		else current_state<=next_state;
	end
	
	//next state logic
	always_comb begin
		case(current_state)
			default_WC: 	if(w_en) next_state = clear_WC;
							else next_state = default_WC;
			clear_WC: 	next_state = write_WC;
			write_WC: 	next_state = default_WC;
			default:	next_state = default_WC;
		endcase
	end
	
	//output logic
	
	always_ff @(posedge clk) begin
		//clears the write buffer
		if(current_state==clear_WC)begin
			write_buffer<= '{default:63'b0000000000000000000000000000000000000000000000000000000000000000};
		end
		//sets the write buffer
		if(current_state==write_WC)begin
			for (int i=0; i<64;i=i+1) begin
				write_buffer[i][y_in[i]] = 1'b1;
			end
		end
		//update the buffer stuff
		if((current_state==default_WC) && get_buffer) out_write_buffer<=write_buffer;
		
	end				   
endmodule


module EBRReadControl  (input logic clk,
						input logic reset,
						input logic r_en,
						input logic get_buffer,
						input logic [5:0] row_0_sel,
						input logic [5:0] row_1_sel,
						input logic [63:0] write_buffer [63:0], 
						output logic [63:0] row_0,
						output logic [63:0]row_1 );
	logic [63:0] read_buffer [63:0];
	always_ff @(posedge clk) begin
		if(r_en)begin
			row_0 <= read_buffer[row_0_sel];
			row_1 <= read_buffer[row_1_sel];
		end
		else if(get_buffer)read_buffer<=write_buffer;
	end	
endmodule







//////////////////////////////////////////////////////////////////////
//					EBR CONTROLLER MODULE							//
//																	//
// The memory of the EBR will be wiped when reset is asserted.		//
// This means it will take two clock cycles to actually write the   //
// display data: one to clear, one to write new data			    //
//																	//
//////////////////////////////////////////////////////////////////////
/*typedef enum logic [3:0] {default_EBR, clear_EBR, write_EBR} EBRState;
module EBRController(input logic clk,
					 input logic reset,
					 input logic w_enable,
					 input logic r_enable,
					 input logic [5:0] y_in [63:0],
					 input logic [5:0] row_0_sel ,
					 input logic [5:0] row_1_sel ,
					 output logic [63:0] row_0 ,
					 output logic [63:0]row_1 );
	logic [63:0] mem_map [63:0];
	EBRState current_state, next_state;
	//state register
	always_ff @(posedge clk) begin
		//EBR is dual port
		if(reset)begin
			//clear the memory map
			mem_map<= '{default:63'b0000000000000000000000000000000000000000000000000000000000000000};
			current_state<=default_EBR;
		end
		else current_state<=next_state;
	end
	//next state logic
	always_comb begin
		case(current_state)
			default_EBR: 	if(w_enable) next_state = clear_EBR;
							else next_state = default_EBR;
			clear_EBR: 	next_state = write_EBR;
			write_EBR: 	next_state = default_EBR;
			default:		next_state = default_EBR;
		endcase
	end
	
	//output logic (basically flop)
	always_ff @(posedge clk) begin
		if(current_state==clear_EBR)begin
			//clear the memory map
			mem_map<= '{default:63'b0000000000000000000000000000000000000000000000000000000000000000};
		end
		if(current_state==write_EBR)begin
			for (int i=0; i<64;i=i+1) begin
				mem_map[i][y_in[i]] = 1'b1;
			end
		end
		if(current_state==default_EBR && r_enable) begin
			//stuff for reading data
			row_0 <= mem_map[row_0_sel];
			row_1 <= mem_map[row_1_sel];
		end
			
	end
endmodule
*/