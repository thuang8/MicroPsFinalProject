




module top(	input logic r,
				output logic r0_out,
				output logic r1_out,
				output logic g0_out,
				output logic g1_out,
				output logic b0_out,
				output logic b1_out,  
				output logic latch,
				output logic OE,
				output logic sck,
				output logic [4:0] addr);
	logic clk;
	logic int_osc;
	logic reset;
	logic read_en;
	
	logic [63:0] row_0_in;
	logic [63:0] row_1_in;
	
	assign row_0_in = 64'hFFFFFF00FFFFFFFF;
	assign row_1_in = 64'hFFFFFF00FFFFFFFF;
	
	
	assign reset = ~r;
	

	DisplayController #(.WIDTH(4)) DisplayControllerTest(int_osc, reset, row_0_in, row_1_in, addr, OE, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck,read_en);
	
	// Internal high-speed oscillator
	HSOSC #(.CLKHF_DIV(2'b11))
	hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));	
	

endmodule



/***************************************************************************************************************************/
/*															MAIN.SV														   */
/***************************************************************************************************************************/

/*

module top(	input logic r,
				output logic r0_out,
				output logic r1_out,
				output logic g0_out,
				output logic g1_out,
				output logic b0_out,
				output logic b1_out,  
				output logic latch,
				output logic OE,
				output logic sck,
				output logic [4:0] addr);
	logic clk;
	logic int_osc;
	logic reset;
	
	logic [3:0] r0_map_cm [63:0];
	logic [3:0] r1_map_cm [63:0];
	logic [3:0] g0_map_cm [63:0];
	logic [3:0] g1_map_cm [63:0];
	logic [3:0] b0_map_cm [63:0];
	logic [3:0] b1_map_cm [63:0];	
	

	
	//logic [3:0] vals [5:0][63:0];
	
	//initial $readmemh("IntegratedDisplay.txt",vals); //load test vectors
	
	assign r0_map_cm = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,  	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15};
	assign g0_map_cm = {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15, 		0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	assign b0_map_cm = {15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,  15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0, 		15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0,  0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	
	assign r1_map_cm = {0,0,3,3,7,7,7,15,15,7,7,7,3,3,0,0,  				15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0, 				15,14,13,12,11,10,9,8,7,8,9,10,11,12,13,14,			15,15,15,7,7,3,3,0,0,3,3,7,7,15,15,15};
	assign g1_map_cm = {15,14,13,12,11,10,9,8,7,8,9,10,11,12,13,14,  		15,15,15,7,7,3,3,0,0,3,3,7,7,15,15,15,			 	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  	 		0,0,3,3,7,7,7,15,15,7,7,7,3,3,0,0};
	assign b1_map_cm = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,  	 		15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15, 	15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,  	0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15};
	
	
	assign reset = ~r;
	
	//slowspeed
	//DisplayControllerColorInput #(.WIDTH(4)) DisplayControllerTest(clk, reset, r0_map_cm,r1_map_cm, g0_map_cm, g1_map_cm,b0_map_cm,b1_map_cm,  addr, OE, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck);
	//fast speed
	DisplayControllerColorInput #(.WIDTH(4)) DisplayControllerTest(int_osc, reset, r0_map_cm,r1_map_cm, g0_map_cm, g1_map_cm,b0_map_cm,b1_map_cm,  addr, OE, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck);
	
	// Internal high-speed oscillator
	HSOSC #(.CLKHF_DIV(2'b11))
	hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));	
	Incrementer #(.WIDTH(15), .INCREMENT(16000)) clkDivide(int_osc, clk);
endmodule
*/



//**********************************************************************************************************************************************************//
//																RAM AND DISPLAY COORDINATION																//
//**********************************************************************************************************************************************************//


//////////////////////////////////////////////////////////////////////
//						RAMDISPLAY	MODULE			 				//
//	Controls memory reading and displaying in a wrapper				//
//////////////////////////////////////////////////////////////////////

/*module RamDisplay #(WIDTH=4)   (input logic clk,
								input logic reset,
								
						);
	
endmodule*/



//**********************************************************************************************************************************************************//
//																	DISPLAY CONTROL SECTION																	//
//**********************************************************************************************************************************************************//

//////////////////////////////////////////////////////////////////////
//						DISPLAYCONTROL	MODULE						//
//				Handles all the display stuff						//
//////////////////////////////////////////////////////////////////////
module DisplayController #(WIDTH=4)(input logic clk,
									input logic reset,
									input logic [63:0 ]row_0_in,
									input logic [63:0 ]row_1_in,
									output logic [4:0] current_addr,
									output logic blank,
									output logic latch,
									output logic r0_out,
									output logic r1_out,
									output logic g0_out,
									output logic g1_out,
									output logic b0_out,
									output logic b1_out,
									output logic out_clk,
									output logic read_en);
	//colshift
	logic pwm_en;
	logic [63:0] r0_cm_cs;
	logic [63:0] r1_cm_cs;
	logic [63:0] g0_cm_cs;
	logic [63:0] g1_cm_cs;
	logic [63:0] b0_cm_cs;
	logic [63:0] b1_cm_cs;		
	logic col_cnt_overflow;	
	logic col_cnt_en;
	

	
	//combcolormod
    logic [WIDTH-1:0] pwm_cnt;
	logic [WIDTH-1:0] r0_map_cm [63:0];
	logic [WIDTH-1:0] r1_map_cm [63:0];
	logic [WIDTH-1:0] g0_map_cm [63:0];
	logic [WIDTH-1:0] g1_map_cm [63:0];
	logic [WIDTH-1:0] b0_map_cm [63:0];
	logic [WIDTH-1:0] b1_map_cm [63:0];
	logic col_sck;
	//maprgb
	logic[5:0] ro1_0_sel;
	logic[5:0] ro1_1_sel;
	
	
	//ColorModFSM
	logic [4:0] next_addr;
	
	
	logic pwm_overflow;
	
	
	//////// MainDisplayFSM	////////
	MainDisplayFSM MainDisplayFSMModule(clk, reset, pwm_overflow,current_addr, pwm_en, next_addr,read_en);
	//////// ColorModFSM	////////
	ColorModFSM #(.WIDTH(WIDTH)) ColorModFSMModule(clk, reset, col_cnt_overflow, pwm_en, next_addr, col_sck,blank, pwm_cnt, latch, current_addr, ro1_0_sel, ro1_1_sel, pwm_overflow,out_clk,col_cnt_en);
	//////// MapRGB	////////
	MapRGB #(.WIDTH(WIDTH)) MapRGBModule(ro1_0_sel, ro1_1_sel, row_0_in, row_1_in, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm);
	//////// CombColorMod	////////
	CombColorMod #(.WIDTH(WIDTH)) CombColorModulationModule(pwm_cnt, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm, r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs);
	//////// ColShift	////////
	ColShift ColShiftModule (clk, reset, col_cnt_en,r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out, col_cnt_overflow,col_sck);
	
endmodule

//////////////////////////////////////////////////////////////////////
//							MemReadDisplay	MODULE					//
//							A TESTING MODULE						//
//////////////////////////////////////////////////////////////////////
module MemReadDisplay(  input logic clk,
						input logic reset,
						output logic r0_out,
						output logic r1_out,
						output logic g0_out,
						output logic g1_out,
						output logic b0_out,
						output logic b1_out,  
						output logic latch,
						output logic OE,
						output logic sck,
						output logic [4:0] addr);
	
	 
	 
	  logic [63:0] row_0_in;
	  logic [63:0] row_1_in;
	
	  logic read_en;

	//DisplayController #(.WIDTH(4)) DisplayControllerTest(int_osc, reset, row_0_in, row_1_in, addr, OE, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck);
	 
									/*input logic [63:0 ]row_1_in,
									output logic [4:0] current_addr,
									output logic blank,
									output logic latch,
									output logic r0_out,
									output logic r1_out,
									output logic g0_out,
									output logic g1_out,
									output logic b0_out,
									output logic b1_out,
									output logic out_clk*/

	/*TESTEBRReadControl  (clk,
						reset,
						input logic r_en,
						input logic [5:0] row_0_sel,
						input logic [5:0] row_1_sel,
						input logic [63:0] write_buffer [63:0], 
						output logic [63:0] row_0,
						output logic [63:0]row_1 );*/
	
	
endmodule





//////////////////////////////////////////////////////////////////////
//						DISPLAYCONTROLCOLORINPUT	MODULE			//
//							A TESTING MODULE						//
//////////////////////////////////////////////////////////////////////
module DisplayControllerColorInput #(WIDTH=4)(input logic clk,
												input logic reset,
												input logic [WIDTH-1:0] r0_map_cm [63:0],
												input logic [WIDTH-1:0] r1_map_cm [63:0],
												input logic [WIDTH-1:0] g0_map_cm [63:0],
												input logic [WIDTH-1:0] g1_map_cm [63:0],
												input logic [WIDTH-1:0] b0_map_cm [63:0],
												input logic [WIDTH-1:0] b1_map_cm [63:0],
												output logic [4:0] current_addr,
												output logic blank,
												output logic latch,
												output logic r0_out,
												output logic r1_out,
												output logic g0_out,
												output logic g1_out,
												output logic b0_out,
												output logic b1_out,
												output logic out_clk,
												output logic read_en);
	//colshift
	logic pwm_en;
	logic [63:0] r0_cm_cs;
	logic [63:0] r1_cm_cs;
	logic [63:0] g0_cm_cs;
	logic [63:0] g1_cm_cs;
	logic [63:0] b0_cm_cs;
	logic [63:0] b1_cm_cs;		
	logic col_cnt_overflow;	
	logic col_sck;
	logic col_cnt_en;
	
	//combcolormod
    logic [WIDTH-1:0] pwm_cnt;

	
	//maprgb
	logic[5:0] ro1_0_sel;
	logic[5:0] ro1_1_sel;
	
	
	//ColorModFSM
	logic [4:0] next_addr;
	
	logic pwm_overflow;
	
	
	//////// MainDisplayFSM	////////
	MainDisplayFSM MainDisplayFSMModule(clk, reset, pwm_overflow,current_addr, pwm_en, next_addr,read_en);
	//////// ColorModFSM	////////
	ColorModFSM #(.WIDTH(WIDTH)) ColorModFSMModule(clk, reset, col_cnt_overflow, pwm_en, next_addr,col_sck, blank, pwm_cnt, latch, current_addr, ro1_0_sel, ro1_1_sel, pwm_overflow,out_clk,col_cnt_en);
	//////// CombColorMod	////////
	CombColorMod #(.WIDTH(WIDTH)) CombColorModulationModule(pwm_cnt, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm, r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs);
	//////// ColShift	////////
	ColShift ColShiftModule (clk, reset, col_cnt_en,r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out, col_cnt_overflow,col_sck);
	
endmodule




//**********************************************************************************************************************************************************//
//																	OTHER CONTROL SECTION																	//
//**********************************************************************************************************************************************************//
//////////////////////////////////////////////////////////////////////
//							SPI MODULE								//
//////////////////////////////////////////////////////////////////////
module SPIIn #(WIDTH=512)(input  logic sck, 
               input  logic sdi,
               output logic sdo,
               input  logic done,
               output logic [WIDTH-1:0] key, plaintext,
               input  logic [WIDTH-1:0] cyphertext);
    logic         sdodelayed, wasdone;
    logic [511:0] cyphertextcaptured;     
    // assert load
    // apply 256 sclks to shift in key and plaintext, starting with plaintext[127]
    // then deassert load, wait until done
    // then apply 128 sclks to shift out cyphertext, starting with cyphertext[127]
    // SPI mode is equivalent to cpol = 0, cpha = 0 since data is sampled on first edge and the first
    // edge is a rising edge (clock going from low in the idle state to high).
    always_ff @(posedge sck)
        if (!wasdone)  {cyphertextcaptured, plaintext, key} = {cyphertext, plaintext[WIDTH-2:0], key, sdi};
        else           {cyphertextcaptured, plaintext, key} = {cyphertextcaptured[WIDTH-2:0], plaintext, key, sdi}; 
    
    // sdo should change on the negative edge of sck
    always_ff @(negedge sck) begin
        wasdone = done;
        sdodelayed = cyphertextcaptured[WIDTH-2];
    end
    // when done is first asserted, shift out msb before clock edge
    assign sdo = (done & !wasdone) ? cyphertext[WIDTH-1] : sdodelayed;
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
		//if((current_state==default_WC) && get_buffer) out_write_buffer<=write_buffer;
		if((current_state==default_WC)) out_write_buffer<=write_buffer;
		
		
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



module TESTEBRReadControl  (input logic clk,
						input logic reset,
						input logic r_en,
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
		read_buffer<=write_buffer;
	end	
endmodule


//**********************************************************************************************************************************************************//
//																	COUNTERS AND CLOCKS																		//
//**********************************************************************************************************************************************************//


//////////////////////////////////////////////////////////////////////
//						COUNTER MODULE								//
//				Counts up and outputs the current count				//
//				Allows for a variable increment	(0 or 1)			//
//////////////////////////////////////////////////////////////////////
module Counter #(WIDTH=6)(	input logic clk, reset,
							input logic increment,
							output logic [WIDTH-1:0] out_count);


	logic [WIDTH-1:0] counter = 0;
	always_ff @(posedge clk)
		begin
			if(reset) counter<=0;
			else counter <= counter + increment;
		end	
	assign out_count = counter;
endmodule




//////////////////////////////////////////////////////////////////////
//						FIXED COUNTER MODULE						//
//			Counts up and outputs an overflow signal				//
//////////////////////////////////////////////////////////////////////
module FixedCounter #(WIDTH=6)(	input logic clk, reset,
								output logic [WIDTH-1:0] out_count);
	logic [WIDTH-1:0] counter = 0;
	always_ff @(posedge clk)
		begin
			if(reset) counter<=0;
			else counter <= counter + 1;
		end	
	assign overflow = counter==(1<<WIDTH)-1;
	assign out_count = counter;
endmodule

//////////////////////////////////////////////////////////////////////
//						INCREMENTER MODULE							//
//			Counts by a set increment and overflows					//
//////////////////////////////////////////////////////////////////////
module Incrementer #(WIDTH=25, INCREMENT=783)(	input logic clk_SwitchingSpeed,
												output logic sClk_SwitchingSpeed);
	logic [WIDTH-1:0] counter = 0;
	//Divides the clock by 2^WIDTH-1 and adds by INCREMENT. 
	always_ff @(posedge clk_SwitchingSpeed)
		begin
			counter <= counter + INCREMENT;
		end	
	//assigns the led to the counter
	assign sClk_SwitchingSpeed = counter[WIDTH-1];
endmodule


//////////////////////////////////////////////////////////////////////
//						ROWCLK MODULE								//
//			Generates the clock for the row	module					//
//////////////////////////////////////////////////////////////////////
module RowClk (	input logic clk,
				input logic reset,
				output logic row_clk);
	logic [4:0] clk_count;
	always_ff @(posedge clk)
	begin
		if (reset)
			clk_count<=0;
		else begin
			clk_count<=clk_count+1;
			row_clk<= clk_count[4];
			//row_clk<= (clk_count==69);
		end
	end
endmodule

