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
* date: 2017/06/11
* author: Ashraful
* brief:
* note:
***************************************************************************/

import riscv_defines::*;

module asa_riscv_mul
(
	input  logic                    clk,
	input  logic                    rstn,

	//Instruction
	input  logic                    Vld,
	input  logic [ALU_OP_WIDTH-1:0] op_instr,
	output logic                    mul_busy,

	//from ID
	input  logic [XLEN-1:0]         opA,
	input  logic [XLEN-1:0]         opB,
	//to WB
	output logic                    mul_bubble,
	output logic [XLEN-1:0]         mul_r
);

  ////////////////////////////////////////////////////////////////
  //
  // Constants
  //
  localparam DXLEN       = 2*XLEN;

  localparam MAX_LATENCY = 3;
  localparam LATENCY     = 0; //MULT_LATENCY > MAX_LATENCY ? MAX_LATENCY : MULT_LATENCY;


  ////////////////////////////////////////////////////////////////
  //
  // Checks (assertions)
  //
/*  initial
  begin
      a1: assert (MULT_LATENCY <= MAX_LATENCY)
          else $warning("MULT_LATENCY=%0d larger than allowed. Changed to %0d", MULT_LATENCY, MAX_LATENCY);
  end

*/
  ////////////////////////////////////////////////////////////////
  //
  // functions
  //
  function [XLEN-1:0] sext32;
    input [31:0] operand;
    logic sign;

    sign   = operand[31];
    sext32 = { {XLEN-32{sign}}, operand};
  endfunction


  function [XLEN-1:0] twos;
    input [XLEN-1:0] a;

    twos = ~a +'h1;
  endfunction

  function [DXLEN-1:0] twos_dxlen;
    input [DXLEN-1:0] a;

    twos_dxlen = ~a +'h1;
  endfunction


  function [XLEN-1:0] abs;
    input [XLEN-1:0] a;

    abs = a[XLEN-1] ? twos(a) : a;
  endfunction



  ////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic [ALU_OP_WIDTH-1:0] mul_instr;

//  logic [            6:2] opcode, mul_opcode;
//  logic [            2:0] func3,  mul_func3;
//  logic [            6:0] func7,  mul_func7;


  //Operand generation
  logic [31:0]             opA32;
  logic [31:0]             opB32;


  logic              mult_neg,      mult_neg_reg;
  logic [XLEN  -1:0] mult_opA,      mult_opA_reg,
                     mult_opB,      mult_opB_reg;
  logic [DXLEN -1:0] mult_r,        mult_r_reg,
                     mult_r_signed, mult_r_signed_reg;

  //FSM (bubble, stall generation)
  logic       is_mul;
  logic [1:0] cnt;
  enum logic {ST_IDLE=1'b0, ST_WAIT=1'b1} state;


  ////////////////////////////////////////////////////////////////
  //
  // Module Body
  //
//  import riscv_pkg::*;


  /*
   * Instruction
   */
  //assign func7  = op_instr[31:25];
  //assign func3  = op_instr[14:12];
  //assign opcode = op_instr[ 6: 2];

  //assign mul_func7  = mul_instr[31:25];
  //assign mul_func3  = mul_instr[14:12];
  //assign mul_opcode = mul_instr[ 6: 2];

  assign mul_instr = op_instr;

  /*
   * 32bit operands
   */
  assign opA32   = opA[31:0];
  assign opB32   = opB[31:0];


  /*
   *  Multiply operations
   *
   * Transform all multiplications into 1 unsigned multiplication
   * This avoids building multiple multipliers (signed x signed, signed x unsigned, unsigned x unsigned)
   *   at the expense of potentially making the path slower
   */

  //multiplier operand-A
  always_comb 
    unique casex ( mul_instr )
      //ALU_MULW   : mult_opA = abs( sext32(opA32) ); //RV64
      ALU_MULHU  : mult_opA =             opA     ;
      default: mult_opA = abs(        opA    );
    endcase

  //multiplier operand-B
  always_comb 
    unique casex ( mul_instr )
      //ALU_MULW   : mult_opB = abs( sext32(opB32) ); //RV64
      ALU_MULHSU : mult_opB =             opB     ;
      ALU_MULHU  : mult_opB =             opB     ;
      default: mult_opB = abs(        opB    );
    endcase

  //negate multiplier output?
  always_comb 
    unique casex ( mul_instr )
      ALU_MUL    : mult_neg = opA[XLEN-1] ^ opB[XLEN-1];
      ALU_MULH   : mult_neg = opA[XLEN-1] ^ opB[XLEN-1];
      ALU_MULHSU : mult_neg = opA[XLEN-1];
      ALU_MULHU  : mult_neg = 1'b0;
      //ALU_MULW   : mult_neg = opA32[31] ^ opB32[31];  //RV64
      default: mult_neg = 'h0;
    endcase


  //Actual multiplier
  assign mult_r        = $unsigned(mult_opA_reg) * $unsigned(mult_opB_reg);

  //Correct sign
  assign mult_r_signed = mult_neg_reg ? twos_dxlen(mult_r_reg) : mult_r_reg;


  /*
   *
   */
generate
  if (LATENCY == 0)
  begin
      /*
       * Single cycle multiplier
       *
       * Registers at: - output
       */
      //Register holding instruction for multiplier-output-selector
      //assign mul_instr = op_instr;

      //Registers holding multiplier operands
      assign mult_opA_reg = mult_opA;
      assign mult_opB_reg = mult_opB;
      assign mult_neg_reg = mult_neg;

      //Register holding multiplier output
      assign mult_r_reg = mult_r;

      //Register holding sign correction
      assign mult_r_signed_reg = mult_r_signed;
  end
  else
  begin
      /*
       * Multi cycle multiplier
       *
       * Registers at: - input
       *               - output
       */
      //Register holding instruction for multiplier-output-selector
//      always @(posedge clk)
//        if (Vld) mul_instr <= op_instr; 

      //Registers holding multiplier operands
      always @(posedge clk)
        if (Vld)
        begin
            mult_opA_reg <= mult_opA;
            mult_opB_reg <= mult_opB;
            mult_neg_reg <= mult_neg;
        end

      if (LATENCY == 1)
      begin
          //Register holding multiplier output
          assign mult_r_reg = mult_r;

          //Register holding sign correction
          assign mult_r_signed_reg = mult_r_signed;
      end
      else if (LATENCY == 2)
      begin
          //Register holding multiplier output
          always @(posedge clk)
            mult_r_reg <= mult_r;

          //Register holding sign correction
          assign mult_r_signed_reg = mult_r_signed;
      end
      else //LATENCY=3
      begin
          //Register holding multiplier output
          always @(posedge clk)
            mult_r_reg <= mult_r;

          //Register holding sign correction
          always @(posedge clk)
            mult_r_signed_reg <= mult_r_signed;
      end
  end
endgenerate



  /*
   * Final output register
   */
  always @(posedge clk)
    unique casex ( mul_instr )
      ALU_MUL    : mul_r <= mult_r_signed_reg[XLEN -1:   0];
      //ALU_MULW   : mul_r <= sext32( mult_r_signed_reg[31:0] );  //RV64
      default: mul_r <= mult_r_signed_reg[DXLEN-1:XLEN];
    endcase


  /*
   * Stall / Bubble generation
   */
/*  always_comb
    unique casex (mul_instr )
      ALU_MUL    : is_mul = 1'b1;
      ALU_MULH   : is_mul = 1'b1;
      ALU_MULW   : is_mul = 1'b1;
      ALU_MULHSU : is_mul = 1'b1;
      ALU_MULHU  : is_mul = 1'b1;
      default: is_mul = 1'b0;
    endcase
*/

  always @(posedge clk,negedge rstn)
    if (!rstn)
    begin
        state      <= ST_IDLE;
        cnt        <= LATENCY;

        mul_bubble <= 1'b1;
        mul_busy  <= 1'b0;
    end
    else
    begin
        mul_bubble <= 1'b1;

        case (state)
          ST_IDLE: if (Vld)
                     //if (is_mul)
                     begin
                         if (LATENCY == 0)
                         begin
                             mul_bubble <= 1'b0;
                             mul_busy  <= 1'b0;
                         end
                         else
                         begin
                             state      <= ST_WAIT;
                             cnt        <= cnt -1;

                             mul_bubble <= 1'b1;
                             mul_busy  <= 1'b1;
                          end
                       end

          ST_WAIT: if (|cnt)
                     cnt <= cnt -1;
                   else
                   begin
                       state <= ST_IDLE;
                       cnt   <= LATENCY;

                       mul_bubble <= 1'b0;
                       mul_busy  <= 1'b0;
                   end
        endcase
    end

endmodule 
