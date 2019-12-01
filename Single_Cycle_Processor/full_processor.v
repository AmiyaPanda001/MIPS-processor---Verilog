`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/15/2019 03:52:37 PM
// Design Name: 
// Module Name: full_processor
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
//Defining 2:1 muxes
module mux21(input [31:0]zero , input [31:0]one, input select, output [31:0]out);
	assign out = ((select == 0) ? zero : one);
endmodule

module mux21_5bit(input [4:0]zero , input [4:0]one, input select, output [4:0]out);
	assign out = ((select == 0) ? zero : one);
endmodule

//Defining sign-extendeing hardware
module sign_extend(input [15:0]in, input signExtend, output reg [31:0]out);
	always @(*) begin
	   if (signExtend == 0) begin
	       out <= 32'h0 + in;
	   end
	   else begin
	       out <= (in[15] ? (32'hffff0000 + in) : (32'h0 + in));
	   end
	end
endmodule

//Defining the 32-bit single cycle mips machine 
module SingleCycleProc(input CLK, Reset_L, 
						input [31:0]startPC, 
						output [31:0]dMemOut
						//output [31:0] instruction, output [31:0]register_output, output [31:0]ALU_input1, output [31:0]ALU_output, 
						//output [31:0]signExtend_output, output [31:0]mux_datamemory_output, output [31:0]program_counter, 
						//output [31:0]interim_counter, output [31:0]next_counter
						);
	
	//Instantiating registers and wires
	reg [31:0]program_counter; //program_counter
	wire [31:0]interim_counter; // program_counter + 4
	wire [31:0]instruction;     // output of instruction memory
	wire [27:0]interim_jump;   //instruction[25:0]<<2
	wire [31:0]jump_address;   //final jump_address 
	wire [31:0]next_counter;   //next program_counter
	wire [31:0]signExtend_output; //output of sign-extendeing unit
	wire [31:0]interim_branch; //signExtend_output << 2
	wire [31:0]branch_address; //final branch_address
	wire [31:0]branch_mux_output; //output of mux_pc_branch
	wire [4:0]registerfile_input; //output of mux_registerfile
	wire [31:0]ALU_input1;
	wire [31:0]ALU_input2;
	wire [31:0]ALU_output;
	wire ALU_output_zero;
	wire [31:0]register_output;
	wire [31:0]ram_output;
	wire [31:0]mux_datamemory_output;
	wire branch_mux_select;
	
	//Instantiating control signals
	wire RegDst, Jump , Branch , MemRead , MemToReg , MemWrite , ALUSrc , RegWrite;
	wire [3:0]ALUCtrl; wire [3:0]ALUOp ;
	
	//PC logic	
	always @(negedge CLK, negedge Reset_L) begin
	                         if (Reset_L == 0) program_counter <= startPC;
                             else program_counter <= next_counter; // update program_counter
 					
							
						  end
						  
//						  	always @(negedge CLK) program_counter <= next_counter; // update program_counter
//                            always @(posedge Reset_L) program_counter <= startPC;           
                                                  
                                                
    assign interim_counter = program_counter + 4 ; //program_counter adder
    //assign next_counter = interim_counter;
    
	//instantiating the instruction memory
    InstructionMemory InstructionMemory(.Data(instruction), .Address(program_counter)); 
		  
	
	//JUMP address calculation
	assign interim_jump = instruction[25:0] << 2;
	assign jump_address[31:28] = interim_counter[31:28];
	assign jump_address[27:0] = interim_jump;
	
	//calculating branch_address
	sign_extend sign_extend (.in(instruction[15:0]), .signExtend(SignExtend), .out(signExtend_output));
	assign interim_branch = signExtend_output << 2;
	assign branch_address = interim_branch + interim_counter;
	
	assign branch_mux_select = Branch & ALU_output_zero;
	
	mux21 mux_pc_branch (.zero(interim_counter),.one(branch_address),.select(branch_mux_select),.out(branch_mux_output)) ;// selects between (pc + 4) and branch
	//mux21 mux_pc_branch (.zero(interim_counter),.one(branch_address),.select(branch_mux_select),.out(branch_mux_output)) ;
	
	mux21 mux_jump_branch (.zero(branch_mux_output),.one(jump_address),.select(Jump),.out(next_counter)); // selects between branch mux output and jump
	//mux21 mux_jump_branch (.zero(branch_mux_output),.one(jump_address),.select(jump),.out(next_counter)); 
	                                             
	
	mux21_5bit mux_registerfile (.zero(instruction[20:16]), .one(instruction[15:11]), .select(RegDst), .out(registerfile_input));
	
	//Instantiating registerfile
	registerfile registerfile (.Ra(instruction[25:21]), .Rb(instruction[20:16]), .Rw(registerfile_input), .Regwr(RegWrite), .clk(CLK), .Bw(mux_datamemory_output),
							   .Ba(ALU_input1), .Bb(register_output), .reset(Reset_L)); 
	
	//Instantiating control unit
	SingleCycleControl SingleCycleControl (.RegDst(RegDst),
					   .ALUSrc(ALUSrc), 
					   .MemToReg(MemToReg), 
					   .RegWrite(RegWrite), 
					   .MemRead(MemRead), 
					   .MemWrite(MemWrite), 
					   .Branch(Branch), 
					   .Jump(Jump), 
					   .SignExtend(SignExtend), 
					   .ALUOp(ALUOp), 
					   .Opcode(instruction[31:26])
								);

    //Instantiating ALU control
	ALUControl ALUControl(.ALUCtrl(ALUCtrl), .ALUop(ALUOp), .FuncCode(instruction[5:0]));
								
	mux21 mux_alu_input (.zero(register_output), .one(signExtend_output), .select(ALUSrc), .out(ALU_input2));
			
	//Instantiating ALU
	ALU ALU (.BusA(ALU_input1), .BusB(ALU_input2), .ALUCtrl(ALUCtrl), .BusW(ALU_output), .Zero(ALU_output_zero), .shmt(instruction[10:6]));
	
	
	//Instantiating random access memory
    DataMemory DataMemory ( .Clock(CLK), //input clock
                    .MemoryRead(MemRead),  //Read enable pin
                    .MemoryWrite(MemWrite), //Write enable pin
                    .Address(ALU_output[5:0]), // 6 bit address to access 64 bit registers
                    .ReadData(ram_output), //read out bus
                    .WriteData(register_output) ); //input write data
        
    mux21 mux_datamemory (.zero(ALU_output), .one(ram_output), .select(MemToReg), .out(mux_datamemory_output));//datamemory mux
        
	assign dMemOut = ram_output;
	
endmodule