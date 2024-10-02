`timescale 1ns / 1ps
// =============================================================================
//  Program : clint.v
//  Author  : Jin-you Wu
//  Date    : Feb/28/2019
// -----------------------------------------------------------------------------
//  Description:
//  This module implements the RISC-V Core Local Interrupt (CLINT) Controller.
//  The ticking signal is currently fixed to the CPU clock instead of an external
//  RTC clock. The OS (e.g., FreeRTOS) must set the frequency of clk_i to the
//  OS timer parameter properly (e.g., configCPU_CLOCK_HZ).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/24/2022, by Chun-Jen Tsai:
//    Remove the TIMER parameter and use the CPU clock to drive CLINT.
//    Previous code assumes that the interrupt generator is driven by a
//    1000 Hz clock (i.e. 1 msec ticks) and the TIMER parameter is
//    used to set the number of CPU ticks within 1 msec.  Unfortunately,
//    this design does not match the popular SiFive CLINT behavior.
//
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================
`include "aquila_config.vh"

module float_clint
#( parameter XLEN = 32 )
(
    input                   clk_i,
    input                   rst_i,

    (* mark_debug = "true" *) input en_i,
    (* mark_debug = "true" *) input we_i,
    input [XLEN-1 : 0]      addr_i,
    input [XLEN-1 : 0]      data_i,
    output reg [XLEN-1 : 0] data_o,
    (* mark_debug = "true" *) output data_ready_o,

    output                  tmr_irq_o,
    output                  sft_irq_o
);

reg  [XLEN-1 : 0] clint_mem[0:2];
(* mark_debug = "true" *)reg out_ready_flag;
(* mark_debug = "true" *)reg out_ready;
(* mark_debug = "true" *)reg inA_ready;
(* mark_debug = "true" *)reg inB_ready;
(* mark_debug = "true" *)reg inC_ready;

assign data_ready_o = inA_ready | inB_ready | inC_ready | out_ready;

(* mark_debug = "true" *) wire valid;
(* mark_debug = "true" *) wire [XLEN-1 : 0] dataA;
(* mark_debug = "true" *) wire [XLEN-1 : 0] dataB;
(* mark_debug = "true" *) wire [XLEN-1 : 0] dataC;
(* mark_debug = "true" *) wire result_valid;
(* mark_debug = "true" *) wire [XLEN-1 : 0] result_data;

reg valid_r;
reg [XLEN-1 : 0] dataA_r;
reg [XLEN-1 : 0] dataB_r;
reg [XLEN-1 : 0] dataC_r;

(* mark_debug = "true" *) reg [32-1 : 0] data_ABC_cycle;
(* mark_debug = "true" *) reg [32-1 : 0] data_result_cycle;
(* mark_debug = "true" *) reg [32-1 : 0] data_cal_cycle;
(* mark_debug = "true" *) reg calculating;

assign valid = valid_r;
assign dataA = dataA_r;
assign dataB = dataB_r;
assign dataC = dataC_r;

always @(posedge clk_i) 
begin
    if (rst_i) begin
        out_ready <= 0;
        out_ready_flag <= 0;
        
        calculating <= 0;
        data_result_cycle <= 32'b0;
        data_cal_cycle <= 32'b0;
    end
    else if(out_ready) begin
        out_ready <= 0;
        out_ready_flag <= 0;
    end
    else if((result_valid || out_ready_flag)) begin
        out_ready_flag <= 1;
        if((addr_i == 32'hC4300000)) begin
            out_ready <= 1;
        end

        calculating <= 0;
        data_result_cycle <= data_result_cycle + 1;
    end
    else if(valid_r || calculating)begin
        calculating <= 1;
        data_cal_cycle <= data_cal_cycle + 1;
    end
    
    data_o <= result_data;
end

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        dataA_r <= 32'b0;
        dataB_r <= 32'b0;
        dataC_r <= 32'b0;
        valid_r <= 1'b0;
        inA_ready <= 1'b0;
        inB_ready <= 1'b0;
        inC_ready <= 1'b0;

        data_ABC_cycle <= 32'b0;
    end
    else if (valid_r) begin
        valid_r <= 1'b0;
        inA_ready <= 1'b0;
        inB_ready <= 1'b0;
        inC_ready <= 1'b0;
    end
    else if (en_i && we_i)
    begin

        // clint_mem[addr_i[21:20]] <= data_i;
        if(addr_i == 32'hC400_0000) begin
            dataA_r <= data_i;
            inA_ready <= 1;
            data_ABC_cycle <= data_ABC_cycle + 1;
        end
        else if(addr_i == 32'hC410_0000) begin
            dataB_r <= data_i;
            inB_ready <= 1;
            data_ABC_cycle <= data_ABC_cycle + 1;
        end
        else if(addr_i == 32'hC420_0000) begin
            dataC_r <= data_i;
            inC_ready <= 1;
            valid_r <= 1;
            data_ABC_cycle <= data_ABC_cycle + 1;
        end
    end
    else begin
        inA_ready <= 0;
        inB_ready <= 0;
        inC_ready <= 0;
    end
end

floating_point_0 floating_point_0(
    .aclk(clk_i),
    .s_axis_a_tvalid(valid),
    .s_axis_a_tdata(dataA),

    .s_axis_b_tvalid(valid),
    .s_axis_b_tdata(dataB),

    .s_axis_c_tvalid(valid),
    .s_axis_c_tdata(dataC),

    .s_axis_operation_tvalid(1'b1),
    .s_axis_operation_tdata(6'b0),

    .m_axis_result_tvalid(result_valid),
    .m_axis_result_tdata(result_data)
);

assign tmr_irq_o = 0;
assign sft_irq_o = 0;


(* mark_debug = "true" *) wire [31:0] check_data0;
(* mark_debug = "true" *) wire [31:0] check_addr;
assign check_data0 = data_o;
assign check_addr = addr_i;


endmodule // clint
