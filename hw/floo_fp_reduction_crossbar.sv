// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Raphael Roth <raroth@student.ethz.ch>

// This module creates a reduced crossbar. The difference between the module and the common cell
// stream_xbar is the output stage. In the CC the output is an rr_arbiter which merges all 
// incoming request. In this module it is however the output is explicitly managed by the outside!

module floo_fp_reduction_crossbar #(
    parameter type DATA_T = logic,  // Vivado requires a default value for type parameters.
    parameter integer N_INP = 0,    // Synopsys DC requires a default value for value parameters.
    parameter integer N_OUP = 0,
    /// Dependent parameters, DO NOT OVERRIDE!
    parameter integer LOG_N_INP = (N_INP > 32'd1) ? unsigned'($clog2(N_INP)) : 1'b1,
    parameter integer LOG_N_OUP = (N_OUP > 32'd1) ? unsigned'($clog2(N_OUP)) : 1'b1
) (
    /// All Input Connections
    input  DATA_T [N_INP-1:0]     inp_data_i,
    input  logic  [N_INP-1:0]     inp_valid_i,
    output logic  [N_INP-1:0]     inp_ready_o,

    /// All Output Connections
    output DATA_T [N_OUP-1:0]     oup_data_o,
    output logic  [N_OUP-1:0]     oup_valid_o,
    input  logic  [N_OUP-1:0]     oup_ready_i,

    /// Selections
    input  logic  [N_INP-1:0][LOG_N_OUP-1:0] inp_sel_i,
    input  logic  [N_OUP-1:0][LOG_N_INP-1:0] oup_sel_i
);
  
    // HS Signal for the Output of the DeMux Stage
    logic     [N_INP-1:0][N_OUP-1:0] inp_valid;
    logic     [N_INP-1:0][N_OUP-1:0] inp_ready;

    // HS Signals for the Input of the Mux Stage
    logic     [N_OUP-1:0][N_INP-1:0] out_valid;
    logic     [N_OUP-1:0][N_INP-1:0] out_ready;

    // Generate the input selection
    for (genvar i = 0; unsigned'(i) < N_INP; i++) begin : gen_inp_demux
        stream_demux #(
            .N_OUP            (N_OUP)
        ) i_stream_demux (
            .inp_valid_i      (inp_valid_i[i]),
            .inp_ready_o      (inp_ready_o[i]),
            .oup_sel_i        (inp_sel_i[i]),
            .oup_valid_o      (inp_valid[i]),
            .oup_ready_i      (inp_ready[i])
        );
    end

    // Crosswire all HS signals here (Only for a better readability)
    for (genvar i = 0; i < N_INP; i++) begin : gen_crossbar_inp
        for (genvar j = 0; j < N_OUP; j++) begin : gen_crossbar_oup
            // Switch the handshaking
            assign out_valid[j][i] = inp_valid[i][j];
            assign inp_ready[i][j] = out_ready[j][i];
        end
    end

    // Generate the output selection
    for (genvar i = 0; i < N_OUP; i++) begin : gen_oup_mux
        stream_mux #(
            .DATA_T             (DATA_T),
            .N_INP              (N_INP)
        ) i_stream_mux (
            .inp_data_i         (inp_data_i),
            .inp_valid_i        (out_valid[i]),
            .inp_ready_o        (out_ready[i]),
            .inp_sel_i          (oup_sel_i[i]),
            .oup_data_o         (oup_data_o[i]),
            .oup_valid_o        (oup_valid_o[i]),
            .oup_ready_i        (oup_ready_i[i])
        );
    end

endmodule
