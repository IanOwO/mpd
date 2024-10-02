`timescale 1ns / 1ps
// =============================================================================
//  Program : bpu.v
//  Author  : Jin-you Wu
//  Date    : Jan/19/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the Branch Prediction Unit (BPU) of the Aquila core (A RISC-V core).
// -----------------------------------------------------------------------------
//  Revision information:
//
//  Aug/15/2020, by Chun-Jen Tsai:
//    Hanlding of JAL in this BPU. In the original code, an additional
//    Unconditional Branch Prediction Unit (UC-BPU) was used to handle
//    the JAL instruction, which seemed redundant.
//
// Aug/16/2023, by Chun-Jen Tsai:
//    Replace the fully associative BHT by the standard Bimodal BHT table.
//    The performance drops a little (1.0 DMIPS -> 0.97 DMIPS), but the resource 
//    usage drops significantly.
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

module bpu #( parameter ENTRY_NUM = 32, parameter XLEN = 32 )
(
    // System signals
    input               clk_i,
    input               rst_i,
    input               stall_i,

    // from Program_Counter
    input  [XLEN-1 : 0] pc_i, // Addr of the next instruction to be fetched.

    // from Decode
    input               is_jal_i,
    input               is_cond_branch_i,
    input  [XLEN-1 : 0] dec_pc_i, // Addr of the instr. just processed by decoder.

    // from Execute
    input               exe_is_branch_i,
    input               branch_taken_i,
    input               branch_misprediction_i,
    input  [XLEN-1 : 0] branch_target_addr_i,

    // to Program_Counter
    output              branch_hit_o,
    output              branch_decision_o,
    output [XLEN-1 : 0] branch_target_addr_o
);

localparam NBITS = $clog2(ENTRY_NUM);

// localparam GBHT_SIZE = ($clog2(ENTRY_NUM)-1)/2; // for concate
localparam GBHT_SIZE = $clog2(ENTRY_NUM); // for one-level & gshare

wire                    we;
wire [XLEN-1 : 0]       branch_inst_tag;
wire [NBITS-1 : 0]      read_addr; // for one-level & gshare
wire [NBITS-1 : 0]      write_addr; // for one-level & gshare
// wire [GBHT_SIZE : 0]      read_addr; // for concate
// wire [GBHT_SIZE : 0]      write_addr; // for concate

// new added reg/wire
reg  [XLEN-1 : 0]       branch_inst_tag_fec2dec;
reg  [XLEN-1 : 0]       branch_inst_tag_dec2exe;
wire                    we_glob_BHT;

// global BHT variable
reg  [GBHT_SIZE-1 : 0]  global_BHT;
reg  [GBHT_SIZE-1 : 0]  restore_global_BHT;
reg  [GBHT_SIZE-1 : 0]  global_BHT_fec2dec;
reg  [GBHT_SIZE-1 : 0]  global_BHT_dec2exe;

wire [NBITS-1 : 0]      write_PHT;
wire [NBITS-1 : 0]      read_PHT;


// two-bit saturating counter
reg  [1 : 0]            branch_likelihood[ENTRY_NUM-1 : 0];

// "we" is enabled to add a new entry to the BHT table when
// the decoder sees a branch instruction for the first time.
// CY Hsiang 0220_2020: added "~stall_i" to "we ="
assign we = ~stall_i & (is_cond_branch_i | is_jal_i) & (branch_inst_tag_dec2exe != dec_pc_i);

assign read_addr = pc_i[NBITS+1 : 2]; // for one-level & gshare
assign write_addr = dec_pc_i[NBITS+1 : 2]; // for one-level & gshare
// assign read_addr = pc_i[NBITS+1-GBHT_SIZE : 2]; // for concate
// assign write_addr = dec_pc_i[NBITS+1-GBHT_SIZE : 2]; // for concate

integer idx;

// branch_likelihood
// // original one
// always @(posedge clk_i)
// begin
//     if (rst_i)
//     begin
//         for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
//             branch_likelihood[idx] <= 2'b0;
//     end
//     else if (stall_i)
//     begin
//         for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
//             branch_likelihood[idx] <= branch_likelihood[idx];
//     end
//     else
//     begin
//         if (we) // Execute the branch instruction for the first time.
//         begin
//             branch_likelihood[write_addr] <= {branch_taken_i, branch_taken_i};
//         end
//         else if (exe_is_branch_i)
//         begin
//             case (branch_likelihood[write_addr])
//                 2'b00:  // strongly not taken
//                     if (branch_taken_i)
//                         branch_likelihood[write_addr] <= 2'b01;
//                     else
//                         branch_likelihood[write_addr] <= 2'b00;
//                 2'b01:  // weakly not taken
//                     if (branch_taken_i)
//                         branch_likelihood[write_addr] <= 2'b11;
//                     else
//                         branch_likelihood[write_addr] <= 2'b00;
//                 2'b10:  // weakly taken
//                     if (branch_taken_i)
//                         branch_likelihood[write_addr] <= 2'b11;
//                     else
//                         branch_likelihood[write_addr] <= 2'b00;
//                 2'b11:  // strongly taken
//                     if (branch_taken_i)
//                         branch_likelihood[write_addr] <= 2'b11;
//                     else
//                         branch_likelihood[write_addr] <= 2'b10;
//             endcase
//         end
//     end
// end

// for gshare & concate
always @(posedge clk_i)
begin
    if (rst_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            branch_likelihood[idx] <= 2'b0;
    end
    else if (stall_i)
    begin
        for (idx = 0; idx < ENTRY_NUM; idx = idx + 1)
            branch_likelihood[idx] <= branch_likelihood[idx];
    end
    else
    begin
        if (we) // Execute the branch instruction for the first time.
        begin
            branch_likelihood[write_PHT] <= {branch_taken_i, branch_taken_i};
        end
        else if (exe_is_branch_i)
        begin
            case (branch_likelihood[write_PHT])
                2'b00:  // strongly not taken
                    if (branch_taken_i)
                        branch_likelihood[write_PHT] <= 2'b01;
                    else
                        branch_likelihood[write_PHT] <= 2'b00;
                2'b01:  // weakly not taken
                    if (branch_taken_i)
                        branch_likelihood[write_PHT] <= 2'b11;
                    else
                        branch_likelihood[write_PHT] <= 2'b00;
                2'b10:  // weakly taken
                    if (branch_taken_i)
                        branch_likelihood[write_PHT] <= 2'b11;
                    else
                        branch_likelihood[write_PHT] <= 2'b00;
                2'b11:  // strongly taken
                    if (branch_taken_i)
                        branch_likelihood[write_PHT] <= 2'b11;
                    else
                        branch_likelihood[write_PHT] <= 2'b10;
            endcase
        end
    end
end

// ===========================================================================
//  Branch History Table (BHT). Here, we use a direct-mapping cache table to
//  store branch history. Each entry of the table contains two fields:
//  the branch_target_addr and the PC of the branch instruction (as the tag).
//

// // for normal BPU
// distri_ram #(.ENTRY_NUM(ENTRY_NUM), .XLEN(XLEN*2))
// BPU_BHT(
//     .clk_i(clk_i),
//     .we_i(we),                  // Write-enabled when the instruction at the Decode
//                                 //   is a branch and has never been executed before.
//     .write_addr_i(write_addr),  // Direct-mapping index for the branch at Decode.
//     .read_addr_i(read_addr),    // Direct-mapping Index for the next PC to be fetched.

//     .data_i({branch_target_addr_i, dec_pc_i}), // Input is not used when 'we' is 0.
//     .data_o({branch_target_addr_o, branch_inst_tag})
// );

// for gshare & concate
distri_ram #(.ENTRY_NUM(ENTRY_NUM), .XLEN(XLEN*2))
BPU_BHT(
    .clk_i(clk_i),
    .we_i(we),                  // Write-enabled when the instruction at the Decode
                                //   is a branch and has never been executed before.
    .write_addr_i(write_PHT),  // Direct-mapping index for the branch at Decode.
    .read_addr_i(read_PHT),    // Direct-mapping Index for the next PC to be fetched.

    .data_i({branch_target_addr_i, dec_pc_i}), // Input is not used when 'we' is 0.
    .data_o({branch_target_addr_o, branch_inst_tag})
);

// ===========================================================================
//  Outputs signals
//
// // for normal bpu
// assign branch_hit_o = (branch_inst_tag == pc_i);
// assign branch_decision_o = branch_likelihood[read_addr][1];

// for concate & gshare
assign branch_hit_o = (branch_inst_tag == pc_i);
assign branch_decision_o = branch_likelihood[read_PHT][1];
// ===========================================================================
// pipeline tag
// 

always @(posedge clk_i)
begin
    if (rst_i)
    begin
        branch_inst_tag_fec2dec <= 0;
        branch_inst_tag_dec2exe <= 0;

        global_BHT_fec2dec <= 0;
        global_BHT_dec2exe <= 0;
    end
    else if (stall_i)
    begin
        branch_inst_tag_fec2dec <= branch_inst_tag_fec2dec;
        branch_inst_tag_dec2exe <= branch_inst_tag_dec2exe;

        global_BHT_fec2dec <= global_BHT_fec2dec;
        global_BHT_dec2exe <= global_BHT_dec2exe;
    end
    else
    begin
        branch_inst_tag_fec2dec <= branch_inst_tag;
        branch_inst_tag_dec2exe <= branch_inst_tag_fec2dec;

        global_BHT_fec2dec <= global_BHT;
        global_BHT_dec2exe <= global_BHT_fec2dec;
    end
end

reg  [GBHT_SIZE-1 : 0]  global_BHT_fec2dec;
reg  [GBHT_SIZE-1 : 0]  global_BHT_dec2exe;

// ===========================================================================
// Global BHT 
//
assign we_glob_BHT = ~stall_i & (is_cond_branch_i | is_jal_i);
assign read_PHT = global_BHT ^ read_addr; // for gshare
// assign write_PHT = global_BHT ^ write_addr; // for gshare
assign write_PHT = global_BHT_dec2exe ^ write_addr; // for gshare

// assign read_PHT = {global_BHT, read_addr}; // for concate
// // assign write_PHT = {global_BHT, write_addr}; // for concate
// assign write_PHT = {global_BHT_dec2exe, write_addr}; // for concate


always @(posedge clk_i)
begin
    if (rst_i)
    begin
        global_BHT <= 0;
        restore_global_BHT <= 0;
    end
    else if (stall_i)
    begin
        global_BHT <= global_BHT;
        restore_global_BHT <= restore_global_BHT;
    end
    else
    begin
        if (branch_misprediction_i)
        begin
            global_BHT <= restore_global_BHT;
        end
        else
        begin
            if (is_jal_i || is_cond_branch_i)
            begin
                global_BHT <= {global_BHT[GBHT_SIZE - 1 - 1:0],branch_taken_i};
                restore_global_BHT <= {global_BHT[GBHT_SIZE - 1 - 1:0], ~branch_taken_i};
            end
            else
            begin
                global_BHT <= global_BHT;
                restore_global_BHT <= restore_global_BHT;
            end
        end
    end
end

// ===========================================================================
//  cycle counter part
//
reg start_count_area;
reg end_count_area;
(* mark_debug = "true" *) wire total_cycle_flag;

assign total_cycle_flag = (dec_pc_i >= 32'h00001000 && dec_pc_i <= 32'h6f2c) && ~stall_i;

// assign total_cycle_flag = (start_count_area && ~end_count_area) && ~stall_i;
// always @(posedge clk_i)begin
//     if (rst_i) begin
//         start_count_area <= 0;
//         end_count_area <= 0;
//     end 
//     else begin
//         if (dec_pc_i == 32'h0000_1000) begin
//             start_count_area <= 1;
//         end
//         else if ((dec_pc_i < 32'h00001000 || dec_pc_i > 32'h6f2c) && start_count_area == 1) begin
//             end_count_area <= 1;
//         end
//     end
// end

(* mark_debug = "true" *) wire [32-1:0] dec_pc_ila;
assign dec_pc_ila = dec_pc_i;
(* mark_debug = "true" *) wire [32-1:0] branch_inst_tag_ila;
assign branch_inst_tag_ila = branch_inst_tag_dec2exe;
(* mark_debug = "true" *) wire [32-1:0] branch_target_addr_ila;
assign branch_target_addr_ila = branch_target_addr_i;

wire check_equal;
assign check_equal = (branch_inst_tag_dec2exe == dec_pc_i);

// counter for branches
(* mark_debug = "true" *) reg [32-1:0] forward_jump_counter;
(* mark_debug = "true" *) reg [32-1:0] backward_jump_counter;
(* mark_debug = "true" *) reg [32-1:0] unconditional_jump_counter;

(* mark_debug = "true" *) reg [32-1:0] forward_jump_miss_counter;
(* mark_debug = "true" *) reg [32-1:0] backward_jump_miss_counter;
(* mark_debug = "true" *) reg [32-1:0] unconditional_jump_miss_counter;

// counter flag
wire forward_jump_counter_flag = is_cond_branch_i && total_cycle_flag && (dec_pc_i < branch_target_addr_i);
wire backward_jump_counter_flag = is_cond_branch_i && total_cycle_flag && (dec_pc_i > branch_target_addr_i);
wire unconditional_jump_counter_flag = is_jal_i && total_cycle_flag && (dec_pc_i != branch_target_addr_i);

wire forward_jump_miss_flag = forward_jump_counter_flag && (branch_misprediction_i || (branch_taken_i && (branch_inst_tag_dec2exe != dec_pc_i)));
wire backward_jump_miss_flag =backward_jump_counter_flag && (branch_misprediction_i || (branch_taken_i && (branch_inst_tag_dec2exe != dec_pc_i)));
wire unconditional_jump_miss_flag = unconditional_jump_counter_flag && (branch_misprediction_i || (branch_taken_i && (branch_inst_tag_dec2exe != dec_pc_i)));

always @(posedge clk_i)begin
    if (rst_i) begin
        forward_jump_counter <= 0;
        backward_jump_counter <= 0;
        unconditional_jump_counter <= 0;
    end 
    else begin
        if (forward_jump_counter_flag) forward_jump_counter <= forward_jump_counter + 1;
        else if (backward_jump_counter_flag) backward_jump_counter <= backward_jump_counter + 1;
        else if (unconditional_jump_counter_flag) unconditional_jump_counter <= unconditional_jump_counter + 1;
        else begin
            forward_jump_counter <= forward_jump_counter;
            backward_jump_counter <= backward_jump_counter;
            unconditional_jump_counter <= unconditional_jump_counter;
        end
    end
end

// for jump miss counter
always @(posedge clk_i)begin
    if (rst_i) begin
        forward_jump_miss_counter <= 0;
        backward_jump_miss_counter <= 0;
        unconditional_jump_miss_counter <= 0;
    end 
    else begin
        if (forward_jump_miss_flag) forward_jump_miss_counter <= forward_jump_miss_counter + 1;
        else if (backward_jump_miss_flag) backward_jump_miss_counter <= backward_jump_miss_counter + 1;
        else if (unconditional_jump_miss_flag) unconditional_jump_miss_counter <= unconditional_jump_miss_counter + 1;
        else begin
            forward_jump_miss_counter <= forward_jump_miss_counter;
            backward_jump_miss_counter <= backward_jump_miss_counter;
            unconditional_jump_miss_counter <= unconditional_jump_miss_counter;
        end
    end
end

endmodule
