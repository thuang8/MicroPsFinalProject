//******************************************************************//
//					Test Bench Utility Functions					//
//******************************************************************//
module Clk(output logic clk);
	initial 	//generate clock
        forever begin
            clk = 1'b0; #5;
            clk = 1'b1; #5;
        end
endmodule


module DisplayValues(input logic clk,
					 input logic r0_in,
					 input logic r1_in,
					 input logic g0_in,
					 input logic g1_in,
					 input logic b0_in,
					 input logic b1_in,
					 output logic r0_out,
					 output logic r1_out,
					 output logic g0_out,
					 output logic g1_out,
					 output logic b0_out,
					 output logic b1_out );
	always_ff @(posedge clk) begin
		r0_out<=r0_in;
		r1_out<=r1_in;
		g0_out<=g0_in;
		g1_out<=g1_in;
		b0_out<=b0_in;
		b1_out<=b1_in;
	end
endmodule