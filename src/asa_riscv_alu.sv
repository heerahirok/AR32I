/************************************************************************
* ASA Microsystems, Inc.
*
* Copyright (c) 2017 ASA Microsystems, Inc.
* All Rights Reserved.
*
* All information contained herein is, and remains the property of 
* ASA Microsystems, Inc. The intellectual and technical concepts contained 
* herein are proprietary to ASA Microsystems, Inc. and may be covered by U.S.
* and Foreign Patents, patents in process, and are protected by trade secret
* or copyright law. Dissemination of this information or reproduction of this 
* material is strictly forbidden unless prior written permission is obtained
* from ASA Microsystems, Inc.
*
* file: [asa_riscv_alu.sv]
* version: 0.1
* date: 2017/06/10
* author: Ashraful
* brief:
* note:
***************************************************************************/

import riscv_defines::*;

module riscv_alu
(
	input  logic                     clk,
	input  logic                     rst_n,

	input  logic [ALU_OP_WIDTH-1:0]  operator_i, // What operation to be done
	input  logic [XLEN-1:0]          operand_a_i, // Contents of Rs1
	input  logic [XLEN-1:0]          operand_b_i, // Contents of Rs2 or other
//	input  logic [31:0]              operand_c_i,

	output logic [XLEN -1:0]         result_o,
	output logic                     comparison_result_o,

	output logic                     ready_o,
	input  logic                     ex_ready_i
);

  logic [XLEN:0]    operand_b_neg;
  logic [XLEN -1:0] shift_result; //Result of left/right shift
  logic [XLEN:0]    adds_result;  //Result of ADD/SUB
  logic             comparison_result_o; //Binary comparison of Less Than

  logic [XLEN -1:0] operand_a_left; //Left shifted (internal)
  logic [XLEN -1:0] operand_a_right; //Right shifted (internal)
  logic [4:0]       shamt; //Shift amount
  logic             shift_right_arith; //Does the right shift is arithmatic?
  logic             shift_left; //Is the instruction is Left shift?

  logic             cmp_signed; //Is the comparison is signed?

//////////////////////////////////////
//  _______  ____________________   //
// /  ___| | | |_   _|  ___|_   _|  //
// \ `--.| |_| | | | | |_    | |    //
//  `--. \  _  | | | |  _|   | |    //
// /\__/ / | | |_| |_| |     | |    //
// \____/\_| |_/\___/\_|     \_/    //
//                                  //
//////////////////////////////////////

  assign shamt = operand_b_i[4:0];
  assign shift_right_arith = (operator_i == ALU_SRA) ? 1'b1 : 1'b0;
  assign operand_a_right  = shift_right_arith ? ($signed(operand_a_i) >>> shamt) : (operand_a_i >> shamt);
  assign shift_left = (operator_i == ALU_SLL) ? 1'b1 : 1'b0;
  assign operand_a_left = operand_a_i << shamt;
  assign shift_result = shift_left ? operand_a_left : operand_a_right;

///////////////////////////////////////////////
//   ___ ____________   _______ _   _______  // 
//  / _ \|  _  \  _  \ / /  ___| | | | ___ \ //
// / /_\ \ | | | | | |/ /\ `--.| | | | |_/ / //
// |  _  | | | | | | / /  `--. \ | | | ___ \ //
// | | | | |/ /| |/ / /  /\__/ / |_| | |_/ / //
// \_| |_/___/ |___/_/   \____/ \___/\____/  //
//                                           //
///////////////////////////////////////////////

/*  assign adds_result = (operator_i == ALU_ADD) ? ($signed(operand_a_i) + $signed(operand_b_i)):
		(operator_i == ALU_ADDU) ? (operand_a_i + operand_b_i):
		(operator_i == ALU_SUB) ? ($signed(operand_a_i) - $signed(operand_b_i)) : 
		(operator_i == ALU_SUBU) ? (operand_a_i - operand_b_i) ; 
*/
  // ALU_SUB denotes Signed subtraction
  assign operand_b_neg = (operator_i == ALU_SUB) ? ~({operand_b_i[XLEN-1],operand_b_i[XLEN-1:0]}) : ~({1'b0,operand_b_i[XLEN-1:0]});

  assign adds_result = (operator_i == ALU_SUB) ? ({operand_a_i[XLEN-1], operand_a_i[XLEN-1:0]} + operand_b_neg + 1'b1) :
	  (operator_i == ALU_SUBU) ? ({1'b0,operand_a_i[XLEN-1:0]} + operand_b_neg + 1'b1) : 
	  ({operand_a_i[XLEN-1],operand_a_i[XLEN-1:0]} + {operand_b_i[XLEN-1],operand_b_i[XLEN-1:0]);

////////////////////////////
//  _____ ___  _________  //
// /  __ \|  \/  || ___ \ //
// | /  \/| .  . || |_/ / //
// | |    | |\/| ||  __/  //
// | \__/\| |  | || |     //
//  \____/\_|  |_/\_|     //
//                        //
////////////////////////////

//assign cmp_signed = (operator_i == ALU_SLTS) ? 1'b1 : 1'b0; 

assign comparison_result_o = (operator_i == ALU_SLTS) ? $signed( {operand_a_i[XLEN -1], operand_a_i[XLEN -1:0]} ) < $signed( {operand_b_i[XLEN -1], operand_b_i[XLEN -1:0]} ) : 
	(operator_i == ALU_CMP_GES) ? $signed( {operand_a_i[XLEN -1], operand_a_i[XLEN -1:0]} ) >= $signed( {operand_b_i[XLEN -1], operand_b_i[XLEN -1:0]} ) :
	(operator_i == ALU_SLTU) ? operand_a_i < operand_b_i : 
	(operator_i == ALU_CMP_GEU) ? operand_a_i >= operand_b_i :
	(operator_i == ALU_CMP_NEQ) : operand_a_i != operand_b_i : 
	operand_a_i == operand_b_i; //ALU_CMP_EQ

/////////////////////////
//  __  __ _   ___  __ //
// |  \/  | | | \ \/ / //
// | |\/| | | | |\  /  //
// | |  | | |_| |/  \  //
// |_|  |_|\___//_/\_\ //
/////////////////////////

  always_comb
  begin
    result_o   = '0;

    unique case (operator_i)
      // Standard Operations
      ALU_AND:  result_o = operand_a_i & operand_b_i;
      ALU_OR:   result_o = operand_a_i | operand_b_i;
      ALU_XOR:  result_o = operand_a_i ^ operand_b_i;

      // Addition/Subtraction Operations
      ALU_ADD, ALU_ADDU, ALU_SUB, ALU_SUBU:  result_o = adds_result[XLEN-1:0];

      // Shift Operations
      ALU_SLL, ALU_SRL, ALU_SRA:  result_o = shift_result;

//      ALU_SLETS, ALU_SLETU
      ALU_SLTS,  ALU_SLTU: result_o = {XLEN-1'b0, comparison_result_o};

      default: result_o   = '0; // default case to suppress unique warning
    endcase
  end

  assign ready_o = 1'b1;
  
endmodule
