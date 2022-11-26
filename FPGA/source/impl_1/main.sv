/***************************************************************************************************************************/
/*															MAIN.SV														   */
/***************************************************************************************************************************/
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
	

	
	
	assign reset = ~r;
	
	MemReadDisplay #(.WIDTH(4)) MemReadDisplayTest(int_osc,reset, r0_out,  r1_out,  g0_out,  g1_out, b0_out, b1_out,  latch,  OE,  sck, addr);

	//DisplayController #(.WIDTH(4)) DisplayControllerTest(int_osc, reset, row_0_in, row_1_in, addr, OE, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck,read_en);
	
	// Internal high-speed oscillator
	HSOSC #(.CLKHF_DIV(2'b01))
	hf_osc (.CLKHFPU(1'b1), .CLKHFEN(1'b1), .CLKHF(int_osc));	
	

endmodule



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
	logic [63:0] r0_cm_cs, r1_cm_cs,g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs;		
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
	logic[5:0] row_0_sel;
	logic[5:0] row_1_sel;	
	//ColorModFSM
	logic [4:0] next_addr;
	logic pwm_overflow;
	//////// MainDisplayFSM	////////
	MainDisplayFSM MainDisplayFSMModule(clk, reset, pwm_overflow,current_addr, pwm_en, next_addr,read_en);
	//////// ColorModFSM	////////
	ColorModFSM #(.WIDTH(WIDTH)) ColorModFSMModule(clk, reset, col_cnt_overflow, pwm_en, next_addr, col_sck,blank, pwm_cnt, latch, current_addr, row_0_sel, row_1_sel, pwm_overflow,out_clk,col_cnt_en);
	//////// MapRGB	////////
	MapRGB #(.WIDTH(WIDTH)) MapRGBModule(row_0_sel, row_1_sel, row_0_in, row_1_in, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm);
	//////// CombColorMod	////////
	CombColorMod #(.WIDTH(WIDTH)) CombColorModulationModule(pwm_cnt, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm, r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs);
	//////// ColShift	////////
	ColShift ColShiftModule (clk, reset, col_cnt_en,r0_cm_cs, r1_cm_cs, g0_cm_cs, g1_cm_cs, b0_cm_cs, b1_cm_cs, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out, col_cnt_overflow,col_sck);
	
endmodule




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

