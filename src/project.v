/*
 * Copyright (c) 2024 Your Name
 * SPDX-License-Identifier: Apache-2.0
 */

`default_nettype none

module tt_um_example (
    input  wire [7:0] ui_in,    // Dedicated inputs
    output wire [7:0] uo_out,   // Dedicated outputs
    input  wire [7:0] uio_in,   // IOs: Input path
    output wire [7:0] uio_out,  // IOs: Output path
    output wire [7:0] uio_oe,   // IOs: Enable path (active high: 0=input, 1=output)
    input  wire       ena,      // always 1 when the design is powered, so you can ignore it
    input  wire       clk,      // clock
    input  wire       rst_n     // reset_n - low to reset
);

    wire _unused, rst;
  // All output pins must be assigned. If not used, assign to 0.
    assign uo_out[7:5]  = {2'b00, _unused};
    assign uio_out = 0;
    assign uio_oe  = 0;

  // List all unused inputs to prevent warnings
    assign _unused = &{ena, clk, rst_n, 1'b0, ui_in, uio_in};
    assign rst = ~rst_n;


    CPU_top CPU_TOP (
        .clk(clk),
        .rst(rst),
        .COUNT(uo_out[4:0])
    );  
endmodule


module CPU_top (
	//Inputs for the RISC V CPU
	input wire clk,
	input wire rst,

	//Output of the RISC V CPU
    output wire [4:0] COUNT
    //output wire [31:0] alu_out
);

	//Wires for integration
	wire halt_wire;
	wire wen_wire;
	wire [2:0] operation_wire;
	wire [4:0] count_wire;
	wire [4:0] Rs1_wire;
	wire [4:0] Rs2_wire;
	wire [4:0] Rd_wire;
	wire [31:0] valueIn_wire;
	wire [31:0] instruction_wire;
	wire [31:0] op1_wire;
	wire [31:0] op2_wire;
	wire [31:0] alu_out_wire;
	
	//assign alu_out = alu_out_wire;
	assign COUNT = count_wire;
	//Instantiating the program counter
	programCounter program_counter (.clk(clk),
									.rst(rst),
									.halt(halt_wire),
									.count(count_wire)
	
	);
	
	
	//Instantiating the program memory
	programMemory program_memory (.clk(clk),
								  .rst(rst),
								  .addr(count_wire),
								  .instruction(instruction_wire)
	);
	
	
	//Instantiating the instruction decoder
	instructionDecoder instruction_decoder (.clk(clk),
										   .instruction(instruction_wire),
										   .halt(halt_wire),
										   .wen(wen_wire),
										   .Rs1(Rs1_wire),
										   .Rs2(Rs2_wire),
										   .Rd(Rd_wire),
										   .operation(operation_wire),
										   .valueIn(valueIn_wire)
	);
	
	
	//Instantiating the register file
	registerFile register_file (.clk(clk),
								.rst(rst),
								.Rs1(Rs1_wire),
								.Rs2(Rs2_wire),
								.Rd(Rd_wire),
								.wen(wen_wire),
								.op1(op1_wire),
								.op2(op2_wire),
								.dataIn(alu_out_wire)
	);
	
	
	//Instantiating the ALU unit
	alu alu_unit (.rst(rst),
				  .clk(clk),
				  .op1(op1_wire),
				  .op2(op2_wire),
				  .valueIn(valueIn_wire),
				  .operation(operation_wire),
				  .out(alu_out_wire)
	);

endmodule



module alu #(
	parameter WIDTH = 32
)(
	input wire clk,
	input wire rst,
	input wire[WIDTH-1:0] op1,
	input wire [WIDTH-1:0] op2,
	input wire [WIDTH-1:0] valueIn,
	input wire [2:0] operation,
	output reg [WIDTH-1:0] out
);

	reg [WIDTH-1:0] alu_out;
	
	//Evaluates the results based on the 3-bit signal "operation"
	always @ (*) begin
		if(rst) begin
			alu_out = {WIDTH{1'b0}};
		end
		
		else begin 
			case (operation) 
				3'b000: alu_out = out;					//NOP
				3'b001: alu_out = op1 + op2;       //ADD
				3'b010: alu_out = op1 - op2;			//SUB
				3'b011: alu_out = op1 & op2;       //AND
				3'b100: alu_out = op1 | op2;			//OR
				3'b101: alu_out = op1 ^ op2;			//XOR
				3'b110: alu_out = valueIn;			//LOAD
				default: alu_out = {WIDTH{1'b0}};
			endcase
		end
	end
	
	
	//Assigns the value to the output "out"
	always @ (posedge clk) begin
		out <= alu_out;
	end
	
endmodule



module registerFile #(
	parameter WIDTH = 32,
	parameter ADDR_WIDTH = 5,
	parameter DEPTH = 2 ** ADDR_WIDTH
)(
	input wire clk,
	input wire rst,
	input wire wen,
	input wire [ADDR_WIDTH-1:0] Rs1,
	input wire [ADDR_WIDTH-1:0] Rs2,
	input wire [ADDR_WIDTH-1:0] Rd,
	input wire [WIDTH-1:0] dataIn,
	output reg [WIDTH-1:0] op1,
	output reg [WIDTH-1:0] op2
);

	reg [DEPTH-1:0] registerSet [WIDTH-1:0];
	integer i;
	
	//write block
	always @ (posedge clk) begin
		if(rst) begin
			for (i=0; i<DEPTH; i=i+1) begin
				registerSet[i] <= {WIDTH{1'b0}}; 
			end
		end
		
		else if (wen) begin
			registerSet[Rd] <= dataIn;
		end
		
		else begin
			registerSet[Rd] <= registerSet[Rd];
		end
	end
	
	//read block
	always @ (posedge clk) begin
		if(rst) begin
			op1 <= {WIDTH{1'b0}};
			op2 <= {WIDTH{1'b0}};
		end
		
		else begin
			op1 <= registerSet[Rs1];
			op2 <= registerSet[Rs2];
		end
	end	
	
endmodule


module instructionDecoder #(
	parameter WIDTH = 32
)(
	input wire clk,
	input wire [WIDTH-1:0] instruction,
	output reg wen,
	output wire halt,
	output reg [4:0] Rs1,
	output reg [4:0] Rs2,
	output reg [4:0] Rd,
	output reg [2:0] operation,
	output reg [31:0] valueIn
);
	reg [31:0] valueIn_reg;
	reg [4:0] Rd_reg;
	reg [2:0] operation_reg;
	reg wen_reg;
	
	wire [2:0] func3;
	wire [6:0] opcode;
	wire [6:0] func7;
	wire [31:0] valueIn_wire;
	
	assign opcode = instruction[6:0];
	assign func3 = instruction[14:12];
	assign func7 = instruction[31:25];
	
	//Assigns the immediate value to the signal "valueIn" and halt flag to the signal "halt"
	assign valueIn_wire = (opcode == 7'b1111111) ? ({12'b0,instruction[31:12]}) : (32'b0);
	assign halt = (opcode == 7'b0000001) ? 1'b1 : 1'b0;					//HALT

	always @ (posedge clk) begin
		Rs1 <= instruction[19:15];
		Rs2 <= instruction[24:20];
		Rd_reg <= instruction[11:7];
	end
	
	/*
	consider the signal "operation" as follows:
	NOP = 000
	ADD = 001
	SUB = 010
	AND = 011
	OR = 100
	XOR = 101
	LOAD = 110  -> opcode = 1111111
	DEFAULT = 000
	
	also opcode = 0000001 -> HALT (PC <= PC)
	*/
	
	always @ (posedge clk) begin
		if (opcode == 7'b1111111) begin	//LOAD
			operation_reg <= 3'b110;
			wen_reg <= 1'b1;
		end
		
		else if (opcode == 7'b0000000) begin
			operation_reg <= 3'b000;			//NOP
			wen_reg <= 1'b0;
		end
		
		else if (opcode == 7'b0110011) begin
			case (func3)
				3'b000: begin
							if(func7 == 7'b0) begin				//ADD
								operation_reg <= 3'b001;
								wen_reg <= 1'b1;
							end
							
							else if (func7 == 7'b0100000) begin	//SUB
								operation_reg <= 3'b010;
								wen_reg <= 1'b1;
							end
						end
					
				3'b110: begin
							operation_reg <= 3'b011;				//AND
							wen_reg <= 1'b1;
						end
			
				3'b111: begin
							operation_reg <= 3'b100;				//OR		
							wen_reg <= 1'b1;
						end
					
				3'b100: begin
							operation_reg <= 3'b101;				//XOR
							wen_reg <= 1'b1;
						end
					
				default: begin								//NOP
							operation_reg <= 3'b000;
							wen_reg <= 1'b0;
						end
			endcase
		end
		
		else begin											//NOP
			operation_reg <= 3'b000;
			wen_reg <= 1'b0;
		end
	end
	
	
	//Pipelining register for "operation" signal
	always @ (posedge clk) begin
		operation <= operation_reg;
	end
	
	//Pipelining register for "wen" signal
	always @ (posedge clk) begin
		wen <= wen_reg;
	end
	
	//Pipelining register for "Rd" signal
	always @ (posedge clk) begin
		Rd <= Rd_reg;
	end
	
	//Pipelining register for "valueIn" signal (immediate value for LOAD)
	always @ (posedge clk) begin
		valueIn_reg <= valueIn_wire;
		valueIn <= valueIn_reg;
	end

endmodule



module programCounter #(
	parameter ADDR_WIDTH = 5
)(
	input wire clk,
	input wire rst,
	input wire halt,
	output reg [ADDR_WIDTH-1:0] count
);

	reg [ADDR_WIDTH-1:0] counter;
	
	//Counter block
	always @ (posedge clk) begin
		if(rst) begin
			counter <= {ADDR_WIDTH{1'b0}};
		end
		
		else begin
			if (halt)
				counter <= counter;
			else
				counter <= counter + 1'b1;
		end
	end
	
	//Assigns the value to the output count
	always @ (counter) begin
		count = counter;
	end
	
endmodule


/*
Two instruction formats used are:- 
1. R-format:
	31-------25 24-----20 19-----15 14---12 11-----7 6-------0
	|  func7   |   Rs2   |   Rs1   | func3 |   Rd   | OPCODE |
    	 ---------- --------- --------- ------- -------- --------

2. U-format:

	31-----------------------------------12 11-----7 6-------0
	|					IMMEDIATE               |   Rd   | OPCODE |
	 -------------------------------------- -------- --------
*/

/*
The instructions are of the form:-

*Arithmetic/Logical operations: 
1. ADD -> 0000000_XXXXX_XXXXX_000_XXXXX_0110011
2. SUB -> 0100000_XXXXX_XXXXX_000_XXXXX_0110011
3. AND -> 0000000_XXXXX_XXXXX_110_XXXXX_0110011
4.  OR -> 0000000_XXXXX_XXXXX_111_XXXXX_0110011
5. XOR -> 0000000_XXXXX_XXXXX_100_XXXXX_0110011

*Additional operations:
1. NOP -> 0000000_00000_00000_000_00000_0000000
2. LOAD-> xxxxxxx_xxxxx_xxxxx_xxx_XXXXX_1111111
3. HALT-> 0000000_00000_00000_000_00000_0000001
*/



module programMemory #(
	parameter ADDR_WIDTH = 5,
	parameter WIDTH = 32,
	parameter DEPTH = 2 ** ADDR_WIDTH
)(
	input wire clk,
	input wire rst,
	input wire [ADDR_WIDTH-1:0] addr,
	output reg [WIDTH-1:0] instruction
);
	
	//Initializing the memory block
	//reg [DEPTH-1:0] memoryBlock [WIDTH-1:0];



	/*
	//Read block - read at the positive edge of the clock
	always @ (posedge clk) begin
		if (rst) begin
			instruction <= {WIDTH{1'b0}};
		end
		
		else begin
			instruction <= memoryBlock [addr];
		end
	end
	
	
	//Program is written at the negetive egde of the reset signal
	always @ (posedge clk) begin
		//Program is written below:-
		memoryBlock[0] <= 32'b0000000_00000_00000_111_00001_1111111;  //LOAD R1, 7
		memoryBlock[1] <= 32'b0000000_00000_00000_011_00010_1111111;  //LOAD R2, 7
		memoryBlock[2] <= 32'b0000000_00000_00000_000_00000_0000000;  //NOP
		memoryBlock[3] <= 32'b0000000_00010_00001_000_00011_0110011;  //ADD R3, R1, R2 
		memoryBlock[4] <= 32'b0100000_00010_00001_000_00100_0110011;  //SUB R4, R1, R2 
		memoryBlock[5] <= 32'b0000000_00010_00001_110_00101_0110011;  //AND R5, R1, R2
		memoryBlock[6] <= 32'b0000000_00010_00001_111_00110_0110011;  //OR R6, R1, R2
		memoryBlock[7] <= 32'b0000000_00000_00000_000_00000_0000000;  //NOP
		memoryBlock[8] <= 32'b0000000_00011_00110_110_00111_0110011;  //AND R7, R3, R6
		memoryBlock[9] <= 32'b0000000_00100_00011_100_01000_0110011;  //XOR R8, R3, R4
    		memoryBlock[10] <= 32'b0000000_00000_00000_000_00000_0000001; //HALT
		memoryBlock[11] <= 32'b0000000_00000_00000_000_00000_0000001; //HALT
	end
	*/


	//Instruction is written at the posedge of the clk depending on the addr value
	always @ (posedge clk) begin
		if (rst)
			instruction <= {WIDTH{1'b0}};

		else begin
			case(addr)
				'd0: instruction <= 32'b0000000_00000_00000_111_00001_1111111;  //LOAD R1, 7
				'd1: instruction <= 32'b0000000_00000_00000_011_00010_1111111;  //LOAD R2, 7
				'd2: instruction <= 32'b0000000_00000_00000_000_00000_0000000;  //NOP
				'd3: instruction <= 32'b0000000_00010_00001_000_00011_0110011;  //ADD R3, R1, R2 
				'd4: instruction <= 32'b0100000_00010_00001_000_00100_0110011;  //SUB R4, R1, R2
				'd5: instruction <= 32'b0000000_00010_00001_110_00101_0110011;  //AND R5, R1, R2
				'd6: instruction <= 32'b0000000_00010_00001_111_00110_0110011;  //OR R6, R1, R2
				'd7: instruction <= 32'b0000000_00000_00000_000_00000_0000000;  //NOP
				'd8: instruction <= 32'b0000000_00011_00110_110_00111_0110011;  //AND R7, R3, R6
				'd9: instruction <= 32'b0000000_00100_00011_100_01000_0110011;  //XOR R8, R3, R4
				'd10: instruction <= 32'b0000000_00000_00000_000_00000_0000001; //HALT
				'd11: instruction <= 32'b0000000_00000_00000_000_00000_0000001; //HALT
				default: instruction <= 32'b0000000_00000_00000_000_00000_0000000;  //NOP
			endcase
		end
	end
	
endmodule
