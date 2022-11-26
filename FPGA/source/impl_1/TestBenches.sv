//******************************************************************//
//						Project Test Benches 						//
//******************************************************************//



//////////////////////////////////////////////////////////////////////
//					EBRController TESTBENCH							//
//////////////////////////////////////////////////////////////////////

module EBRController_tb();
	logic clk;
	logic reset;
	logic w_enable;
	logic r_enable;
	logic get_buffer;
	logic [5:0] y_in [63:0];
	logic [5:0] row_0_sel;
	logic [5:0] row_1_sel;
	logic [63:0] row_0;
	logic [63:0]row_1;
	Clk TestClk(clk);
		
	initial begin
		#5;
		reset<=1'b0;		//set reset to 0
		w_enable<=1'b0;		//set w_enable to 0
		r_enable<=1'b0;		//set r_enable to 0
		get_buffer<=1'b0;	//set get_buffer to 0
		$readmemb("EBR.tv",y_in);
		//y_in<= '{default:5'b00001};		//set y_in to all 1's
		row_0_sel<=0;		//set row_0_sel to 0
		row_1_sel<=32;		//set row_1_sel to 0
		
		
		#10;
		w_enable <=1;
		#10;
		w_enable<=0;
		
		#10;
		#10;
		#10;
		get_buffer<=1'b1;	
		#10;
		get_buffer<=1'b0;	
		#10;

		//#10;
		r_enable<=1;
	end
	
	//the DUT
	EBRController DUT(clk,reset,w_enable,r_enable,get_buffer, y_in,row_0_sel,row_1_sel,row_0,row_1);
	
	

	
	//main test loop
	always @(posedge clk) begin
		//increment
		row_0_sel = row_0_sel+1;
		row_1_sel = row_1_sel+1;
		if (row_0_sel==10)begin
			r_enable<=0;
			$readmemb("EBR1.tv",y_in);
			w_enable<=1;
			#10;
			w_enable<=0;
			#10;
			//#10;
			get_buffer<=1'b1;	
			#10;
			//#10;
			get_buffer<=1'b0;
			#10;
			r_enable<=1;
		end
	end
		
endmodule



//////////////////////////////////////////////////////////////////////
//							ColShift TESTBENCH						//
//////////////////////////////////////////////////////////////////////

module MainDisplayFSM_tb();
	
	
	logic clk;
	logic reset;
	logic pwm_overflow;
	logic pwm_en;
	logic [4:0]current_addr;
	logic [4:0]next_addr;
	Clk TestClk(clk);
	
	
	
	
	initial begin
		#5;
		reset<=1'b1;
		#10;
		reset<=1'b0;
		#10;
		#10;
		#10;
		#10;
		#10;
		pwm_overflow<=1;
		#10;
		pwm_overflow<=0;
	end
	
	
	//The DUT
	MainDisplayFSM DUT(clk,reset, pwm_overflow, current_addr, pwm_en, next_addr);
	
	
endmodule









//////////////////////////////////////////////////////////////////////
//							ColShift TESTBENCH						//
//////////////////////////////////////////////////////////////////////
module ColShift_tb();
	logic clk;
	logic reset;
	logic pwm_en;
	logic col_cnt_overflow;
	logic [63:0] r0_in;
	logic [63:0] r1_in;
	logic [63:0] g0_in;
	logic [63:0] g1_in;
	logic [63:0] b0_in;
	logic [63:0] b1_in;
	logic r0_out;
	logic r1_out;
	logic g0_out;
	logic g1_out;
	logic b0_out;
	logic b1_out;
	logic col_sck;
	logic[63:0] tv[5:0];
			   
	//initialization
	initial begin
		reset<=1'b0;			//set reset to 0
		pwm_en<=1'b0;		//set data ready to false
		$readmemb("ColSelect.tv",tv);	//load test vectors
		assign r0_in = tv[0];	//set test vectors to channels
		assign g0_in = tv[1];
		assign b0_in = tv[2];
		assign r1_in = tv[3];
		assign g1_in = tv[4];
		assign b1_in = tv[5];
	end
	
	Clk TestClk(clk);
	//the DUT
	ColShift DUT(clk,reset,pwm_en,r0_in,r1_in,g0_in,g1_in,b0_in,b1_in,r0_out,r1_out,g0_out,g1_out,b0_out,b1_out,col_cnt_overflow,col_sck);
		
	initial begin
		#5;
		reset<=1'b1;
		#10;
		reset<=1'b0;
		pwm_en<=1;
		#10;
		pwm_en<=0;
	end
	//main loop
	always @(posedge clk) begin
		
	end
endmodule


//////////////////////////////////////////////////////////////////////
//							ColorModFSM TESTBENCH					//
//////////////////////////////////////////////////////////////////////
module ColorModFSM_tb();
	parameter COLORWID = 4;
    logic clk;
	logic reset;
	logic col_cnt_overflow;
	logic pwm_en;
	logic [4:0] next_addr;
	logic blank;
	logic [COLORWID-1:0] pwm_cnt;
	logic latch;
	logic [4:0]current_addr;
	logic [5:0]row_0_sel;
	logic [5:0]row_1_sel;	
	
	logic [63:0] r0_in;
	logic [63:0] r1_in;
	logic [63:0] g0_in;
	logic [63:0] g1_in;
	logic [63:0] b0_in;
	logic [63:0] b1_in;
		
	logic r0_out;
	logic r1_out;
	logic g0_out;
	logic g1_out;
	logic b0_out;
	logic b1_out;
	logic sck;
	logic col_sck;
	logic pwm_overflow;

	logic[63:0] tv[5:0];
	
	//initialization
	initial begin
		$readmemb("ColSelect.tv",tv);	//load test vectors
		assign r0_in = tv[0];
		assign g0_in = tv[1];
		assign b0_in = tv[2];
		assign r1_in = tv[3];
		assign g1_in = tv[4];
		assign b1_in = tv[5];
		
		#5;
		reset<=1;
		next_addr<=5'b00010;
		#10;
		reset<=0;
		pwm_en<=1;
		#10;
		pwm_en<=0;
	end
	Clk TestClk(clk);	//Clock
	//DUT
	ColorModFSM #(.WIDTH(COLORWID)) DUT(clk, reset, col_cnt_overflow, pwm_en, next_addr,col_sck,blank, pwm_cnt,latch, current_addr, row_0_sel, row_1_sel,pwm_overflow,sck);
	//Colshift tester
	ColShift COLTESTER(clk,reset,pwm_en,r0_in,r1_in,g0_in,g1_in,b0_in,b1_in,r0_out,r1_out,g0_out,g1_out,b0_out,b1_out,col_cnt_overflow,col_sck);

	
endmodule




//////////////////////////////////////////////////////////////////////
//						Integrated TESTBENCH	 					//
//////////////////////////////////////////////////////////////////////
module Integrated_tb();
	parameter COLORWID = 4;
    logic clk;
	logic reset;
	logic [63:0 ]row_0_in;
	logic [63:0 ]row_1_in;
	logic [4:0] current_addr;
	logic blank;
	logic latch;
	logic r0_out;
	logic r1_out;
	logic g0_out;
	logic g1_out;
	logic b0_out;
	logic b1_out;
	logic sck;
	logic[63:0] tv[1:0];
	logic read_en;
	
	//initialization
	initial begin
		$readmemb("IntegratedTB.tv",tv);	//load test vectors
		assign row_0_in = tv[0];
		assign row_1_in = tv[1];	
		//#5;
		reset<=1;
		#10;
		reset<=0;

	end
	
	//DUT
	DisplayController #(.WIDTH(COLORWID)) DUT (clk, reset, row_0_in, row_1_in, current_addr, blank, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck,read_en);
	
	Clk TestClk(clk);	//Clock


endmodule



//////////////////////////////////////////////////////////////////////
//						IntegratedMEM TESTBENCH	 					//
//////////////////////////////////////////////////////////////////////
module IntegratedMEM_tb();
	parameter COLORWID = 4;
    logic clk;
	logic reset;
	logic [63:0 ]row_0_in;
	logic [63:0 ]row_1_in;
	logic [4:0] current_addr;
	logic blank;
	logic latch;
	logic r0_out;
	logic r1_out;
	logic g0_out;
	logic g1_out;
	logic b0_out;
	logic b1_out;
	logic sck;
	logic[63:0] tv[1:0];
	
	
	//initialization
	initial begin
		//#5;
		reset<=1;
		#10;
		reset<=0;

	end
	
	//DUT
	MemReadDisplay #(.WIDTH(COLORWID)) DUT(clk,reset, r0_out, r1_out,g0_out,g1_out,b0_out,b1_out,  latch, OE,sck,current_addr);
	//DUT
	//DisplayController #(.WIDTH(COLORWID)) DUT (clk, reset, row_0_in, row_1_in, current_addr, blank, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck);
	
	Clk TestClk(clk);	//Clock


endmodule



//////////////////////////////////////////////////////////////////////
//					IntegratedDispay TESTBENCH	 					//
//////////////////////////////////////////////////////////////////////
module IntegratedDispay_tb();
	parameter COLORWID = 4;
    logic clk;
	logic reset;
	logic [COLORWID-1:0] r0_map_cm [63:0];
	logic [COLORWID-1:0] r1_map_cm [63:0];
	logic [COLORWID-1:0] g0_map_cm [63:0];
	logic [COLORWID-1:0] g1_map_cm [63:0];
	logic [COLORWID-1:0] b0_map_cm [63:0];
	logic [COLORWID-1:0] b1_map_cm [63:0];
	
	logic [4:0] current_addr;
	logic blank;
	logic latch;
	logic r0_out;
	logic r1_out;
	logic g0_out;
	logic g1_out;
	logic b0_out;
	logic b1_out;
	logic sck;
	
	
	logic r0_DIS;
	logic r1_DIS;
	logic g0_DIS;
	logic g1_DIS;
	logic b0_DIS;
	logic b1_DIS;
	
	logic read_en;
	
	logic  [COLORWID-1:0] tv [5:0][63:0];
	
	
	//initialization
	initial begin
		$readmemb("IntegratedDisplay.tv",tv);	//load test vectors
		r0_map_cm <= tv[0];
		r1_map_cm <= tv[1];
		g0_map_cm <= tv[2];
		g1_map_cm <= tv[3];
		b0_map_cm <= tv[4];
		b1_map_cm <= tv[5];

		#5;
		reset<=1;
		#10;
		reset<=0;

	end
	
	//DUT
	DisplayControllerColorInput #(.WIDTH(COLORWID)) DUT (clk, reset, r0_map_cm, r1_map_cm, g0_map_cm, g1_map_cm, b0_map_cm, b1_map_cm, current_addr, blank, latch, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out,sck,read_en);
	
	Clk TestClk(clk);	//Clock
	
	DisplayValues DisplayValueData(sck, r0_out, r1_out, g0_out, g1_out, b0_out, b1_out, r0_DIS, r1_DIS, g0_DIS, g1_DIS, b0_DIS, b1_DIS);


endmodule

