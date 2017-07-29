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
* file: [asa_ex_stage.sv]
* version: 0.1
* date: 2017/06/10
* author: Ashraful
* brief:
* note:
***************************************************************************/

import asa_riscv_defines::*;


module asa_ex_stage
(
	input  logic        clk,
	input  logic        rst_n,

	// ALU signals from ID stage
	input  logic [ALU_OP_WIDTH-1:0] operator_i,
	input  logic [XLEN-1:0] operand_a_i,
	input  logic [XLEN-1:0] operand_b_i,

	// directly passed through to WB stage from id stage, not used in EX
	input  logic        regfile_wr_en_i,
	input  logic [4:0]  regfile_wr_addr_i,
	// Output of EX stage pipeline
	output logic        regfile_wr_en_o,
	output logic [4:0]  regfile_wr_addr_o,

	// CSR access
	//input  logic        csr_access_i,
	//input  logic [31:0] csr_rdata_i,

	// ALU/MUL/DIV result
	output logic [XLEN-1:0] regfile_wr_data,    // forward to RF 

	// To IF: Jump and branch target address and branching decision
	output logic [PCLEN-1:0] jmp_addr,
	output logic             br_taken,

	// Stall Control
	//input  logic        lsu_ready_ex_i, // EX part of LSU is done

	output logic        ex_ready_o, // EX stage ready for new data
	input  logic        ex_valid_i, // EX stage gets new data
	//input  logic        wb_ready_i  // WB stage ready for new data
);

  logic [31:0] alu_result;
//  logic [31:0] alu_csr_result;
  logic [31:0] mult_result;
  logic        alu_cmp_result;

  logic        alu_ready;
  logic        is_it_mul;
  logic        is_it_div;


  //assign regfile_alu_we_fw_o    = regfile_alu_we_i;
  //assign regfile_alu_waddr_fw_o = regfile_alu_waddr_i;


  // branch handling
  assign br_taken = alu_cmp_result;
  assign jmp_addr = alu_result [PCLEN-1:0];


  ////////////////////////////
  //     _    _    _   _    //
  //    / \  | |  | | | |   //
  //   / _ \ | |  | | | |   //
  //  / ___ \| |__| |_| |   //
  // /_/   \_\_____\___/    //
  //                        //
  ////////////////////////////

    asa_riscv_alu alu_int
  (
    .clk                 ( clk             ),
    .rst_n               ( rst_n           ),

    .operator_i          ( operator_i  ),
    .operand_a_i         ( operand_a_i ),
    .operand_b_i         ( operand_b_i ),
//    .operand_c_i         ( alu_operand_c_i ),

    .result_o            ( alu_result      ),
    .comparison_result_o ( alu_cmp_result  ),

    .ready_o             ( alu_ready       ),
    .ex_ready_i          ( ex_ready_o      )
  );


/////////////////////////
// ______ _____ _   _  //
// |  _  \_   _| | | | //
// | | | | | | | | | | //
// | | | | | | | | | | //
// | |/ / _| |_\ \_/ / //
// |___/  \___/ \___/  //
//                     //
////////////////////////

asa_riscv_div (
	.clk(clk),
	.rstn(rstn),

	//Instruction
	.Vld(is_it_div),	
	.op_instr(operator_i),
	.div_busy(div_busy),
	.
	//from ID
	.opA(operand_a_i),
	.opB(operand_b_i),
	
	//to WB
	.div_bubble(div_bubble),
	.div_r(result_div)
);


//////////////////////////
// ___  ____   _ _      //
// |  \/  | | | | |     //
// | .  . | | | | |     //
// | |\/| | | | | |     //
// | |  | | |_| | |____ //
// \_|  |_/\___/\_____/ //
//                      //
//////////////////////////

asa_riscv_mul (
	.clk(clk),
	.rstn(rstn),

	.Vld(is_it_mul),
	.op_instr(operator_i),
	.mul_busy(mul_busy),

	.opA(operand_a_i),
	.opB(operand_b_i),

	.mul_bubble(mul_bubble),
	.mul_r(result_mul)
);


  always_comb
    unique case (operator_i )
      ALU_MUL    : is_it_mul = 1'b1;
      ALU_MULH   : is_it_mul = 1'b1;
      //MULW   : is_it_mul = 1'b1;
      ALU_MULHSU : is_it_mul = 1'b1;
      ALU_MULHU  : is_it_mul = 1'b1;
      default: is_it_mul = 1'b0;
    endcase

  always_comb
    unique case (operator_i )
      ALU_DIV   : is_it_div = 1'b1;
      ALU_DIVU  : is_it_div = 1'b1;
      ALU_REM   : is_it_div = 1'b1;
      ALU_REMU  : is_it_div = 1'b1;
      default: is_it_div = 1'b0;
    endcase


// EX Stgae output (multiplexed from different unit)
//  assign ex_int_alu_o = mult_en_i ? mult_result : alu_result; 
 
  always_comb
    unique case ({is_it_mul, is_it_div} )
      2'b10: ex_int_alu_o = !mul_bubble ? result_mul : alu_result;
      2'b01: ex_int_alu_o = !div_bubble ? result_div : alu_result;
      default: ex_int_alu_o = alu_result;
    endcase

    ///////////////////////////////////////
  // EX/WB Pipeline Register           //
  ///////////////////////////////////////
  always_ff @(posedge clk, negedge rst_n)
  begin : EX_WB_Pipeline_Register
    if (~rst_n)
    begin
      regfile_waddr_wb_o   <= '0;
      regfile_we_wb_o      <= 1'b0;
    end
    else
    begin
      if (ex_valid_o) // wb_ready_i is implied
      begin
        regfile_we_wb_o    <= regfile_we_i;
        if (regfile_we_i) begin
          regfile_waddr_wb_o <= regfile_waddr_i;
        end
      end else if (wb_ready_i) begin
        // we are ready for a new instruction, but there is none available,
        // so we just flush the current one out of the pipe
        regfile_we_wb_o    <= 1'b0;
      end
    end
  end

   // branch handling
  assign branch_decision_o = alu_cmp_result;
  assign jump_target_o     = alu_result;

  // As valid always goes to the right and ready to the left, and we are able
  // to finish branches without going to the WB stage, ex_valid does not
  // depend on ex_ready.
  //assign ex_ready_o = (alu_ready & mult_ready & lsu_ready_ex_i & wb_ready_i);// | branch_in_ex_i;
  assign ex_ready_o = (alu_ready & !mul_busy & !div_busy & lsu_ready_ex_i & wb_ready_i);
  assign ex_valid_o = (alu_ready & !mul_busy & !div_busy & lsu_ready_ex_i & wb_ready_i);



endmodule
