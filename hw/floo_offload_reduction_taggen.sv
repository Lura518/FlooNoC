// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Raphael Roth <raroth@student.ethz.ch>

// This module generates an TAG for each reduction element (short element) that enters the reduction. If elements needs to 
// be reduced together then they have an equal tag. The main goal behind the idea of tag is that this allows to separate the
// complexity we want to support from the rest of the problem. The rest of the system is dumb as it just combines all elements
// with the same tag. Depending on the system restriction we can have a more sophiticated tag generator or not. In the most general
// case we woould support reduction of out-of-order arriving elements (NOT SUPPORTED!)

// Current Implementation:
// We want to support the most general input pattern without the overhead of out-of-order tracking.
// The main problem is that the tag can never be out of sync in respect for all inputs. If one element is excpected from a certain input
// direction then it is only allowed to increment the Tag if the element actually arrives (and not sooner), therefor we have to count the
// pending elements on each input. However, if no element is excpected from this direction then the Tag should be incremented immidiatly.

// Restriction:
// - With the current implementation it is impossible to handle two different incoming reduction request in the same cycle.
//   It should work if a pending elements incomes together with an new reduction request.
// - All inputs needs to be strictly in order.

// TODO: Open Issues:
// - The taggen modul can not handle backpressure well. The problem is that if one input is backpressured then it is still
//   possible for another to increment the tag. This leads to an AXI violation and should be avoided.
//   Solution: Introduce both ready & valid signal into the module and add FF @ The End of the modul.
//   --> Differentiat between only valid asserted - e.g. locked in and valid handshake

`include "common_cells/registers.svh"

module floo_offload_reduction_taggen #(
    parameter int unsigned NumRoutes                    = 1,
    parameter type TAG_T                                = logic,
    parameter int unsigned RdTagBits                    = 1
) (
    input  logic                                clk_i,
    input  logic                                rst_ni,
    input  logic                                flush_i,
    
    /// All Input directions
    input logic [NumRoutes-1:0][NumRoutes-1:0]  mask_i,
    input logic [NumRoutes-1:0]                 valid_i,
    input logic [NumRoutes-1:0]                 ready_i,

    /// Generated Tag for each output
    output TAG_T [NumRoutes-1:0]                tag_o
);

/* All local parameter */
localparam int unsigned  MaxNumberofOutstandingRed = 1 << RdTagBits;

/* All Typedef Vars */

/* Variable declaration */
logic [NumRoutes-1:0] inc_pending;
logic [NumRoutes-1:0] dec_pending;
logic [NumRoutes-1:0] outstanding_pending;

logic [NumRoutes-1:0][NumRoutes-1:0] gen_mask_with_pending;

logic [NumRoutes-1:0] inc_tag;
logic [NumRoutes-1:0] inc_tag_pending_src;
logic [NumRoutes-1:0] general_mask;

TAG_T [NumRoutes-1:0] tag_q, tag_d;

logic [NumRoutes-1:0] handshake;

logic new_reduction_incoming;

/* Module Declaration */

assign handshake = valid_i & ready_i;

for (genvar i = 0; i < NumRoutes; i++) begin : gen_pending_tracker

    // Generate Credit Counter once per input
    credit_counter #(
        .NumCredits         (MaxNumberofOutstandingRed),
        .InitCreditEmpty    (1'b1)
    ) i_credit_counter (
        .clk_i              (clk_i),
        .rst_ni             (rst_ni),
        .credit_o           (),
        .credit_give_i      (inc_pending[i]),
        .credit_take_i      (dec_pending[i]),
        .credit_init_i      (1'b0),
        .credit_left_o      (outstanding_pending[i]),   // == 1'b1 if credits are available
        .credit_crit_o      (),  // Giving one more credit will fill the credits
        .credit_full_o      ()
    );
end

// Generat the mask - if no pending incoming req then forward the mask, otherwise set to 0!
for (genvar i = 0; i < NumRoutes; i++) begin
    for (genvar j = 0; j < NumRoutes; j++) begin
        assign gen_mask_with_pending[j][i] = ((outstanding_pending[i] == 1'b0) && (handshake[i] == 1'b1)) ? mask_i[i][j] : 1'b0;
    end
end

// The general mask indicates if the router excpect an element on this input. The or-connection between all inputs is to receive the
// first handshake on any interface (Here is also the problem when two different reduction request arrive at the same time: the generated
// mask would be the combination of the two and the tag would be mixed up). The mask is only included in the general mask if no pending
// element is on this input as all pending elements are from the earlier request and the tag was already incremented for all other inputs.

// Generate the General Mask (OR-Connect all 1 bit / 2 bit etc.)
for (genvar i = 0; i < NumRoutes; i++) begin : gen_reduce_bitwise_outer
    assign general_mask[i] = |gen_mask_with_pending[i];
end

// Generate the Signal where we indicate if a new reduction is incoming
// (the handshake OR should be redundant as the general mask is always 0 if the corr. handshake[i] is not set)
assign new_reduction_incoming = (|handshake) & (|general_mask);

always_comb begin
    // Init all Vars
    inc_pending = '0;
    dec_pending = '0;
    inc_tag = '0;
    inc_tag_pending_src = '0;

    // Iterate over all inputs
    for (int i = 0; i < NumRoutes;i++) begin
        // Increment the Tag if we have a valid handshake and the bit in the general mask is set
        // (Element expected from this input and element is actually there)
        if((general_mask[i] == 1'b1) && (handshake[i] == 1'b1) && (new_reduction_incoming == 1'b1)) begin
            // Edge case: On another input we have new incoming request but we have also a pending one with the same maskon this input
            // therefore the received entry is the pending one (handled further down) and not the "new" one - so increment the pending one
            if(outstanding_pending[i] == 1'b1) begin
                inc_pending[i] = 1'b1;
            end else begin
                inc_tag[i] = 1'b1;
            end
        end

        // Increment the Pending for this Input when the general mask bit is set but we do not have a hs
        // (Element expected from this input but element is not there)
        if((general_mask[i] == 1'b1) && (handshake[i] == 1'b0) && (new_reduction_incoming == 1'b1)) begin
            inc_pending[i] = 1'b1;
        end

        // Increment the Tag if the general mask bit is clear but somewhere exists a hs
        // (No Element expected from this input - make sure to only increment by 1)
        // However if this entry is backpressured then add a pending
        if((general_mask[i] == 1'b0) && (new_reduction_incoming == 1'b1)) begin
            if(valid_i[i] == 1'b0) begin
                inc_tag[i] = 1'b1;
            end else begin
                inc_pending[i] = 1'b1;
            end
        end

        // Decrement the Pending for this Input if we have a pending incoming element and a valid hs
        // (Element arrives from a erlier handled request but was pending)
        if((outstanding_pending[i] == 1'b1) && (handshake[i] == 1'b1)) begin
            dec_pending[i] = 1'b1;
            inc_tag_pending_src[i] = 1'b1;
        end
    end
end

// Generate the Tag's here!
always_comb begin
    // Init all Vars
    tag_d = tag_q;

    // Iterate over all inputs
    for (int i = 0; i < NumRoutes;i++) begin

        // Increment the Tag
        if(inc_tag[i] == 1'b1) begin
            tag_d[i] = tag_d[i] + 1;
        end

        // Increment the Tag again if we have a second HS
        if(inc_tag_pending_src[i] == 1'b1) begin
            tag_d[i] = tag_d[i] + 1;
        end
    end
end


// Assign the output tag
assign tag_o = tag_q;

// buffer the tag
`FF(tag_q, tag_d, '0, clk_i, rst_ni)

/* ASSERTION Checks */
endmodule


/*
// Small Testbench to verify the TAG Generator
module tb_fp_reduction_taggen #();

/* All local parameter * /
localparam int unsigned  NumberInputs = 5;
localparam int unsigned  CycleSim = 6;
time ApplDelay = 100ps;
time AcqDelay = 500ps;

/* All Typedef Vars * /
typedef logic [3:0] test_tag_t;          // Test tag generation
typedef logic [NumberInputs-1:0] mask_t;

/* Variable declaration * /

// Control Var
logic clk;
logic rst_n;
clk_rst_gen #(.ClkPeriod(10ns), .RstClkCycles(1)) i_clk_rst_gen (.clk_o(clk), .rst_no(rst_n));

// Input Variable
mask_t [NumberInputs-1:0] in_mask;
logic [NumberInputs-1:0] in_handshake;

// Output Variable
test_tag_t [NumberInputs-1:0] out_tag;

mask_t [5][6] input_data;
logic [5][6] input_hs;

/* Module Declaration * /

floo_fp_reduction_taggen #(
    .NumRoutes                  (NumberInputs),
    .TAG_T                      (test_tag_t)
) i_dut (
    .clk_i                      (clk),
    .rst_ni                     (rst_n),
    .flush_i                    (1'b0),
    .mask_i                     (in_mask),
    .valid_i                    (valid_i),
    .ready_i                    (ready_i),
    .tag_o                      (out_tag)
);

int cnt_cycle;

/* Describe the Testbench Here * /

    // Feed the input with the 
    initial begin
        /*
        // Define the input data
        input_data[0] = {5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011};
        input_data[1] = {5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011};
        input_data[2] = {5'b00000, 5'b00000, 5'b00000, 5'b00000, 5'b00000, 5'b00000};
        input_data[3] = {5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011};
        input_data[4] = {5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011, 5'b11011};

        /*
        // Fully Pipeline
        input_hs[0] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        input_hs[1] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        input_hs[2] = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
        input_hs[3] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        input_hs[4] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        * /

        // Fully Pipeline But [1]&[3] Element shifted
        input_hs[0] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        input_hs[1] = {1'b0, 1'b1, 1'b1, 1'b0, 1'b0, 1'b1};
        input_hs[2] = {1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0};
        input_hs[3] = {1'b0, 1'b0, 1'b1, 1'b1, 1'b0, 1'b1};
        input_hs[4] = {1'b1, 1'b1, 1'b0, 1'b0, 1'b1, 1'b0};
        * /

        input_data[0] = {5'b11011, 5'b11011, 5'b00000, 5'b10101, 5'b10101, 5'b00000};
        input_data[1] = {5'b00000, 5'b11011, 5'b11011, 5'b00000, 5'b00000, 5'b00000};
        input_data[2] = {5'b00000, 5'b00000, 5'b00000, 5'b00000, 5'b10101, 5'b10101};
        input_data[3] = {5'b11011, 5'b00000, 5'b11011, 5'b00000, 5'b00000, 5'b00000};
        input_data[4] = {5'b11011, 5'b11011, 5'b00000, 5'b10101, 5'b00000, 5'b10101};

        input_hs[0] = 6'b110110;
        input_hs[1] = 6'b011000;
        input_hs[2] = 6'b000011;
        input_hs[3] = 6'b101000;
        input_hs[4] = 6'b110101;

        // Init all Data here
        in_mask = '0;
        in_handshake = '0;
        cnt_cycle = 0;

        // Wait for 5 cycle
        repeat (5) @(posedge clk);

        // Provide Data here
        while(1) begin
            @(posedge clk);
            #(ApplDelay);

            for(int i = 0; i < NumberInputs;i++) begin
                in_mask[i] = input_data[i][cnt_cycle];
                in_handshake[i] = input_hs[i][cnt_cycle];
            end
            cnt_cycle = cnt_cycle + 1;

            if(cnt_cycle > CycleSim) begin
                $stop();
            end
        end
    end

    // Plot the output tag generated
    initial begin
        while(1) begin
            @(posedge clk);

            for(int i = 0; i < NumberInputs; i++) begin
                // Generate the binary rep of the input mask
                if(in_handshake[i] == 1'b1) begin
                    $display($time, " HS on IF %d: Gen Mask %s Tag %d", i, genBitRep(in_mask[i]), out_tag[i]);
                end
            end
        end
    end

    function string genBitRep (logic [NumberInputs-1:0] in);
		string retVal;
        retVal = "B";
        for(int i = 0; i < NumberInputs; i++) begin
            if(in[NumberInputs-1-i] == 1'b1) begin
                retVal = {retVal, "1"};
            end else begin
                retVal = {retVal, "0"};
            end
        end
        return retVal;
	endfunction

endmodule
*/