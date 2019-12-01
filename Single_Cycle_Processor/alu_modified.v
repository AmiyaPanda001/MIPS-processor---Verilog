`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/12/2019 01:01:19 PM
// Design Name: 
// Module Name: alu
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

//ALU does not provide support to arithmatic exception ----> will be fixed in the later labs

//Defining control signals

`define AND 4'b0000
`define OR 4'b0001
`define ADD 4'b0010
`define SLL 4'b0011
`define SRL 4'b0100
`define SUB 4'b0110
`define SLT 4'b0111
`define ADDU 4'b1000
`define SUBU 4'b1001
`define XOR 4'b1010
`define SLTU 4'b1011
`define NOR 4'b1100
`define SRA 4'b1101
`define LUI 4'b1110

//ALU module defination

 module ALU(input [31:0]BusA, 	//32 bit input operand A
 			input [31:0]BusB,	//32 bit input operand B
			input [3:0]ALUCtrl, //4 bit control signal
			input [4:0]shmt,      //for shift operations
			output reg [31:0]BusW,  //32 bit output 
			output reg Zero);		//Zero signal if the output is 0
			
	wire less, sra, temp, less1;
	wire signed [31:0] signed_BusA, signed_BusB;
	
	assign signed_BusA = BusA;
	assign signed_BusB = BusB;
	
	assign less = ( signed_BusA <  signed_BusB  ? 1'b1 : 1'b0);
	assign less1 = ({1'b0,BusA} < {1'b0,BusB}  ? 1'b1 : 1'b0);
	//assign temp = 32'hffffffff << (32'd32 - BusB);
	//assign sra =  ((BusA & 32'h80000000) == 32'h80000000 ? (BusA | temp) : (BusA >> BusB));
	//assign sra = (BusA >> BusB);
	 		
	always @(ALUCtrl, BusA, BusB, less, shmt, less1) begin
		case(ALUCtrl)
			`AND : BusW <= BusA & BusB;
			`OR  : BusW <= BusA | BusB;
			`ADD : BusW <= BusA + BusB;
			`SLL : BusW <= BusB << shmt;
			`SUB : BusW <= BusA - BusB;
			`SRL : BusW <= BusB >> shmt;
			`SLT : BusW <= less;
			`ADDU: BusW <= BusA + BusB;
			`SUBU: BusW <= BusA - BusB;
			`XOR : BusW <= BusA ^ BusB;
			`SLTU: BusW <= less1;
			`NOR : BusW <= ~(BusA | BusB);
			`SRA : BusW <= ((BusB & 32'h80000000) == 32'h80000000 ? ( (BusB >> shmt) | (32'hffffffff << (32'd32 - shmt))) : (BusB >> shmt));
			`LUI : BusW <= {BusB,16'b0};
			default : BusW <= 0;
		endcase
	end
	
	always @(BusW) begin
		if(BusW == 0) Zero <= 1;
		else Zero <= 0 ;
	end

endmodule