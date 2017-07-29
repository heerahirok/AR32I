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

module asa_riscv_div 
(
	input  logic                     clk,
	input  logic                     rstn,

	//Instruction
	input  logic                     Vld,	
	input  logic [ALU_OP_WIDTH-1:0]  op_instr,
	output logic                     div_busy,

	//from ID
	input  logic [XLEN-1:0]          opA,
	input  logic [XLEN-1:0]          opB,

	//to WB
	output logic                     div_bubble,
	output logic [XLEN-1:0]          div_r
);

//////////////////////////////////////////////
//   __                  _   _              //
//  / _|                | | (_)             //
// | |_ _   _ _ __   ___| |_ _  ___  _ __   //
// |  _| | | | '_ \ / __| __| |/ _ \| '_ \  //
// | | | |_| | | | | (__| |_| | (_) | | | | //
// |_|  \__,_|_| |_|\___|\__|_|\___/|_| |_| //
//                                          //
//////////////////////////////////////////////

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


  function [XLEN-1:0] abs;
    input [XLEN-1:0] a;

    abs = a[XLEN-1] ? twos(a) : a;
  endfunction

  ////////////////////////////////////////////////////////////////
  //
  // Variables
  //
  logic [ALU_OP_WIDTH-1:0] div_instr;

//  logic [           6:2] opcode, div_opcode;
//  logic [           2:0] func3,  div_func3;
//  logic [           6:0] func7,  div_func7;
//  logic                  is_rv64;

  //Operand generation
  logic [31:0]             opA32;
  logic [31:0]             opB32;

  logic [$clog2(XLEN)-1:0] cnt; //for 32-bit operation it will [4:0]
  logic                    neg_q; //negative quotient
  logic                    neg_s; //negative remainder

  //divider internals
  typedef struct packed {
    logic [XLEN-1:0] p, a;
  } pa_struct;

  pa_struct                pa;
  pa_struct                pa_shifted;
  logic [XLEN:0]           p_minus_b;
  logic [XLEN-1:0]         b;


  //FSM
  enum logic [1:0] {ST_CHK=2'b00, ST_DIV=2'b01,ST_RES=2'b10} state;

  ////////////////////////////////////////////////////////////////
  //
  // Module Body
  //
  //  import riscv_pkg::*;


  /*
   * Instruction
   */
  //assign func7      = op_instr[31:25];
  //assign func3      = op_instr[14:12];
  //assign opcode     = op_instr[ 6: 2];

  //assign div_func7  = div_instr[31:25];
  //assign div_func3  = div_instr[14:12];
  //assign div_opcode = div_instr[ 6: 2];


  //assign is_rv64 = (XLEN == 64);

  assign div_instr = op_instr;

  /*
   * 32bit operands
   */
  assign opA32   = opA[31:0];
  assign opB32   = opB[31:0];


  /*
   *  Divide operations
   *
   */
  assign pa_shifted = pa << 1;
  assign p_minus_b  = pa_shifted.p - b;

  //Division: bit-serial. Max XLEN cycles
  // q = z/d + s
  // z: Dividend
  // d: Divisor
  // q: Quotient
  // s: Remainder
  always @(posedge clk,negedge rstn)
    if (!rstn)
    begin
        state      <= ST_CHK;
        div_bubble <= 1'b1;
        //div_busy  <= 1'b0;

        div_r      <=  'h0;

        pa         <=  'h0;
        b          <=  'h0;
        neg_q      <= 1'b0;
        neg_s      <= 1'b0;
    end
    else
    begin
        div_bubble <= 1'b1;

        case (state)

          /*
           * Check for exceptions (divide by zero, signed overflow)
           * Setup dividor registers
           */
          ST_CHK: 
	  begin  //:ST_CHK
		  if (Vld)
		  begin //:Vld
                    unique case ( div_instr )
                       ALU_DIV : if (~|opB)
                                begin //signed divide by zero
                                    div_r      <= {XLEN{1'b1}}; //=-1
                                    div_bubble <= 1'b0;
                                end
                                else
                                if (opA == {1'b1,{XLEN-1{1'b0}}} && &opB) // signed overflow (Dividend=-2^(XLEN-1), Divisor=-1)
                                begin
                                    div_r      <= {1'b1,{XLEN-1{1'b0}}};
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {$bits(cnt){1'b1}}; //for 32-bit it will 5'd31
                                    state     <= ST_DIV; //Go for division operation
                                    //div_busy <= 1'b1; //We are computing ...

                                    neg_q     <= opA[XLEN-1] ^ opB[XLEN-1]; //either opA or opB is negative and quotient will be negative
                                    neg_s     <= opA[XLEN-1]; //Dividend is negative so the reminder will be negative

                                    pa.p      <= 'h0;
                                    pa.a      <= abs(opA); 
                                    b         <= abs(opB);
                                 end

/*                       DIVW   : if (~|opB32)
                                begin //signed divide by zero
                                    div_r      <= {XLEN{1'b1}}; //=-1
                                    div_bubble <= 1'b0;
                                end
                                else
                                if (opA32 == {1'b1,{31{1'b0}}} && &opB32) // signed overflow (Dividend=-2^(XLEN-1), Divisor=-1)
                                begin
                                    div_r      <= sext32( {1'b1,{31{1'b0}}} );
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {1'b0, {$bits(cnt)-1{1'b1}} };
                                    state     <= ST_DIV;
                                    //div_busy <= 1'b1;

                                    neg_q     <= opA32[31] ^ opB32[31];
                                    neg_s     <= opA32[31];

                                    pa.p      <= 'h0;
                                    pa.a      <= { abs( sext32(opA32) ), {XLEN-32{1'b0}}      };
                                    b         <= abs( sext32(opB32) );
                                end
*/
                       ALU_DIVU : if (~|opB)
                                begin //unsigned divide by zero
                                    div_r      <= {XLEN{1'b1}}; //= 2^XLEN -1
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {$bits(cnt){1'b1}}; //for 32-bit it will 5'd31
                                    state     <= ST_DIV; //Go for division operation
                                    //div_busy <= 1'b1; //We are computing ...

                                    neg_q     <= 1'b0; //unsigned division so quotient will be positive
                                    neg_s     <= 1'b0; //unsigned division so reminder will be positive

                                    pa.p      <= 'h0;
                                    pa.a      <= opA;
                                    b         <= opB;
                                end

/*                       DIVUW  : if (~|opB32)
                                begin //unsigned divide by zero
                                    div_r      <= {XLEN{1'b1}}; //= 2^XLEN -1
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {1'b0, {$bits(cnt)-1{1'b1}} };
                                    state     <= ST_DIV;
                                    //div_busy <= 1'b1;

                                    neg_q     <= 1'b0;
                                    neg_s     <= 1'b0;

                                    pa.p      <= 'h0;
                                    pa.a      <= { opA32, {XLEN-32{1'b0}} };
                                    b         <= { {XLEN-32{1'b0}}, opB32 };
                                end
*/
                       ALU_REM  : if (~|opB)
                                begin //signed divide by zero
                                    div_r      <= opA;
                                    div_bubble <= 1'b0;
                                end
                                else
                                if (opA == {1'b1,{XLEN-1{1'b0}}} && &opB) // signed overflow (Dividend=-2^(XLEN-1), Divisor=-1)
                                begin
                                    div_r      <=  'h0;
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {$bits(cnt){1'b1}}; //for 32-bit it will 5'd31
                                    state     <= ST_DIV; //Go for division operation
                                    //div_busy <= 1'b1; //We are computing ...

                                    neg_q     <= opA[XLEN-1] ^ opB[XLEN-1]; //either opA or opB is negative and quotient will be negative
                                    neg_s     <= opA[XLEN-1];  //Dividend is negative so the reminder will be negative

                                    pa.p      <= 'h0;
                                    pa.a      <= abs(opA);
                                    b         <= abs(opB);
                                end

/*                       REMW   : if (~|opB32)
                                begin //signed divide by zero
                                    div_r      <= sext32(opA32);
                                    div_bubble <= 1'b0;
                                end
                                else
                                if (opA32 == {1'b1,{31{1'b0}}} && &opB32) // signed overflow (Dividend=-2^(XLEN-1), Divisor=-1)
                                begin
                                    div_r      <=  'h0;
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {1'b0, {$bits(cnt)-1{1'b1}} };
                                    state     <= ST_DIV;
                                    //div_busy <= 1'b1;

                                    neg_q     <= opA32[31] ^ opB32[31];
                                    neg_s     <= opA32[31];

                                    pa.p      <= 'h0;
                                    pa.a      <= { abs( sext32(opA32) ), {XLEN-32{1'b0}}      };
                                    b         <= abs( sext32(opB32) );
                                end
*/
                       ALU_REMU : if (~|opB)
                                begin //unsigned divide by zero
                                    div_r      <= opA;
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {$bits(cnt){1'b1}}; //for 32-bit it will 5'd31
                                    state     <= ST_DIV; //Go for division operation
                                    //div_busy <= 1'b1; //We are computing ...

                                    neg_q     <= 1'b0; //unsigned division so quotient will be positive
                                    neg_s     <= 1'b0; //unsigned division so reminder will be positive

                                    pa.p      <= 'h0;
                                    pa.a      <= opA;
                                    b         <= opB;
                                end

/*                       REMUW  : if (~|opB32)
                                begin
                                    div_r      <= sext32(opA32);
                                    div_bubble <= 1'b0;
                                end
                                else
                                begin
                                    cnt       <= {1'b0, {$bits(cnt)-1{1'b1}} };
                                    state     <= ST_DIV;
                                    //div_busy <= 1'b1;

                                    neg_q     <= 1'b0;
                                    neg_s     <= 1'b0;

                                    pa.p      <= 'h0;
                                    pa.a      <= { opA32, {XLEN-32{1'b0}} };
                                    b         <= { {XLEN-32{1'b0}}, opB32 };
                                end
*/
                       default: ;
                    endcase
	    end //:Vld
	    end //:ST_CHK

          /*
           * actual division loop
           */
          ST_DIV:
	  begin //:ST_DIV
                      cnt <= cnt -1;
                      if (~| cnt) state <= ST_RES; //If we have completed 31 iteration then go for result

                      //restoring divider section
                      if (p_minus_b[XLEN])
                      begin //sub gave negative result
                          pa.p <=  pa_shifted.p;                   //restore
                          pa.a <= {pa_shifted.a[XLEN-1:1], 1'b0};  //shift in '0' for Q
                      end
                      else
                      begin //sub gave positive result
                          pa.p <=  p_minus_b[XLEN-1:0];            //store sub result
                          pa.a <= {pa_shifted.a[XLEN-1:1], 1'b1};  //shift in '1' for Q
                      end
	  end //:ST_DIV

          /*
           * Result
           */
          ST_RES:
	  begin //:ST_RES
                      state      <= ST_CHK;
                      div_bubble <= 1'b0;
                      //div_busy  <= 1'b0;

                      unique case ( div_instr )
                         ALU_DIV    : div_r <=         neg_q ? twos(pa.a) : pa.a; 
                         //DIVW   : div_r <= sext32( neg_q ? twos(pa.a) : pa.a );
                         ALU_DIVU   : div_r <=                              pa.a;
                         //DIVUW  : div_r <= sext32(                      pa.a );
                         ALU_REM    : div_r <=         neg_s ? twos(pa.p) : pa.p;
                         //REMW   : div_r <= sext32( neg_s ? twos(pa.p) : pa.p );
                         ALU_REMU   : div_r <=                              pa.p;
                         //REMUW  : div_r <= sext32(                      pa.p );
                         default: div_r <= 'hx;
                      endcase
           end //:ST_RES
        endcase // case (state)
    end // else (!rstn)

  always_comb// @(posedge clk,negedge rstn)
  begin
    if(!rstn)
      div_busy  = 1'b0;
    else
    begin
        case(state)
        ST_CHK: 
	  begin //: ST_CHK
		  if (Vld)
		  begin //:if_Vld
                    case ( div_instr )
                       ALU_DIV, ALU_REM : if (~|opB)
                                begin //signed divide by zero
                                    div_busy  = 1'b0;
                                end
                                else if (opA == {1'b1,{XLEN-1{1'b0}}} && &opB) // signed overflow (Dividend=-2^(XLEN-1), Divisor=-1)
                                begin
                                    div_busy  = 1'b0;
                                end
                                else
                                begin
                                    div_busy = 1'b1; //We are computing ...
                                end
                       ALU_DIVU, ALU_REMU : if (~|opB)
                                begin //unsigned divide by zero
                                    div_busy  = 1'b0;
                                end
                                else
                                begin
                                    div_busy <= 1'b1; //We are computing ...
                                end
                       default: div_busy  = 1'b0;
                    endcase
 		  end //:if_vld
		  else
		  begin
			  div_busy  = 1'b0;
		  end
	  end //:ST_CHK
        ST_DIV: div_busy = 1'b1; //We are computing ...
        ST_RES: div_busy  = 1'b0;
        endcase
   end
  end


endmodule 
  
