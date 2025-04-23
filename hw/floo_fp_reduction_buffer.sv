// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Raphael Roth <raroth@student.ethz.ch>

// This module creates a buffer in which all elements can be accessed from the outside.
// With an Index the currently output element can be changed.
// To reduce the overall number of bit required a tag is used.

// The selection for each output port is designed as AXI Handshaking port. However if the
// selector is changed while the valid is asserted (By itself AXI violation) then the
// output data will change too.

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

module floo_fp_reduction_buffer #(
    parameter type TAG_T = logic,
    parameter type DATA_T = logic,  // Vivado requires a default value for type parameters.
    parameter integer N_ELEMENTS = 0,
    parameter integer N_OUT_PORTS = 1,
    /// Dependent parameters, DO NOT OVERRIDE!
    parameter integer LOG_N_ELEMENTS = (N_ELEMENTS > 32'd1) ? unsigned'($clog2(N_ELEMENTS)) : 1'b1
) (
    input  logic                        clk_i,
    input  logic                        rst_ni,
    input  logic                        flush_i,
    
    /// All Input Connections
    input  TAG_T                        inp_tag_i,
    input  DATA_T                       inp_data_i,
    input  logic                        inp_valid_i,
    output logic                        inp_ready_o,

    /// All Output Connections
    output DATA_T [N_OUT_PORTS-1:0]     oup_data_o,
    output logic  [N_OUT_PORTS-1:0]     oup_valid_o,
    input  logic  [N_OUT_PORTS-1:0]     oup_ready_i,

    /// Selections
    input  logic [N_OUT_PORTS-1:0][LOG_N_ELEMENTS-1:0]   inp_sel_i,

    /// Spyglass to all entries of the Buffer
    output logic [N_ELEMENTS-1:0]       spyglass_valid_o, 
    output TAG_T [N_ELEMENTS-1:0]       spyglass_tag_o
);

/* All Typedef Vars */

typedef struct packed {
    TAG_T tag;
    DATA_T data;
    logic f_valid;
} buff_entry_t;

/* Variable declaration */


/* Description */

if(N_ELEMENTS == 0) begin : gen_no_partial_res_buffer
    // Tie everything to ground
    assign inp_ready_o = '0;
    assign oup_data_o = '0;
    assign oup_valid_o = '0;
    assign spyglass_valid_o = '0;
    assign spyglass_tag_o = '0;

end else begin : gen_partial_res_buffer

    buff_entry_t [N_ELEMENTS-1:0] buffer_d, buffer_q; // Buffer to hold the data
    logic empty_field_found;
    logic [N_ELEMENTS-1:0] ready_sig_buf;

    
    always_comb begin : insertData
        buffer_d = buffer_q;    // Init the buffer with the old data

        // All Output signal
        inp_ready_o = 1'b0;
        oup_data_o = '0;
        oup_valid_o = '0;
        spyglass_tag_o = '0;
        spyglass_valid_o = '0;

        // Intermidiate signal
        empty_field_found = 1'b0;
        ready_sig_buf = '0;

        // Store the Data into the Buffer
        for(int i = 0; i < N_ELEMENTS; i++) begin
            if((buffer_d[i].f_valid == 1'b0) && (empty_field_found == 1'b0) && (inp_valid_i == 1'b1)) begin
                // Lock the Entry
                empty_field_found = 1'b1;

                // Ack the handshake
                inp_ready_o = 1'b1;

                // Copy the actual data
                buffer_d[i].data = inp_data_i;
                buffer_d[i].tag = inp_tag_i;
                buffer_d[i].f_valid = 1'b1;
            end
        end

        // Implement the Spyglass here!
        for(int i = 0; i < N_ELEMENTS; i++) begin
            spyglass_tag_o[i] = buffer_d[i].tag;
            spyglass_valid_o[i] = buffer_d[i].f_valid;
        end

        // Assign the output for each defined port
        for(int i = 0; i < N_OUT_PORTS;i++) begin
            oup_data_o[i] = buffer_d[inp_sel_i[i]].data;
            oup_valid_o[i] = buffer_d[inp_sel_i[i]].f_valid;
            ready_sig_buf[inp_sel_i[i]] = oup_ready_i[i];
        end

        // If we receive any valid handshake on any IF then reset the valid flag
        for(int i = 0; i < N_ELEMENTS; i++) begin
            if((ready_sig_buf[i] == 1'b1) && (buffer_d[i].f_valid == 1'b1)) begin
                buffer_d[i].f_valid = 1'b0;
            end
        end

        // Reset Buffer if flush is asserted & avoid piping data to the output
        if(flush_i == 1'b1) begin
            buffer_d = '0;
            oup_valid_o = '0;
        end
    end

    // Store the Buffer
    `FF(buffer_q, buffer_d, '0, clk_i, rst_ni)

end

endmodule