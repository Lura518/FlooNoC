// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Michael Rogenmoser <michaero@iis.ee.ethz.ch>

`include "common_cells/assertions.svh"
`include "common_cells/registers.svh"

/// A simple router with configurable number of ports, physical and virtual channels, and input/output buffers
module floo_router
  import floo_pkg::*;
#(
  /// Number of ports
  parameter int unsigned NumRoutes            = 0,
  /// More fine-grained control over number of input ports
  parameter int unsigned NumInput             = NumRoutes,
  /// More fine-grained control over number of output ports
  parameter int unsigned NumOutput            = NumRoutes,
  /// Number of virtual channels
  parameter int unsigned NumVirtChannels      = 0,
  /// Number of physical channels
  parameter int unsigned NumPhysChannels      = 1,
  /// Depth of input FIFOs
  parameter int unsigned InFifoDepth          = 0,
  /// Depth of output FIFOs
  parameter int unsigned OutFifoDepth         = 0,
  /// Routing algorithm
  parameter route_algo_e RouteAlgo            = IdTable,
  /// Parameters, only used for ID-based and XY routing
  parameter int unsigned IdWidth              = 0,
  parameter type         id_t                 = logic[IdWidth-1:0],
  /// Used for ID-based routing
  parameter int unsigned NumAddrRules         = 1,
  /// Configuration parameters for special network topologies
  /// Disables Y->X connections in XYRouting
  parameter bit          XYRouteOpt           = 1'b1,
  /// Disables loopback connections
  parameter bit          NoLoopback           = 1'b1,
  /// Enable Multicast feature
  parameter bit          EnMultiCast          = 1'b0,
  /// Enable reduction feature
  parameter bit          EnReduction          = 1'b0,
  /// Enable offload reduction feature
  parameter bit          EnOffloadReduction   = 1'b0,
  /// Various types
  parameter type         addr_rule_t          = logic,
  parameter type         flit_t               = logic,
  parameter type         hdr_t                = logic,
  parameter type         payload_t            = logic,
  parameter payload_t    NarrowRspMask        = '0,
  parameter payload_t    WideRspMask          = '0,
  /// Offload reduction parameter
  /// Possible operation for offloading (must match type in header)
  parameter type         RdOperation_t        = logic,
  /// Data type of the offload reduction
  parameter type         RdData_t             = logic,
  /// Depth and Fallthrough Configuration of the input Fifo
  parameter bit          RdFifoFallThrough    = 1'b0,
  parameter int unsigned RdFifoDepth          = 2,
  /// Depth of the offload reduction pipeline
  parameter int unsigned RdPipelineDepth      = 3,
  /// Reduction controller configuration options
  parameter int unsigned RdControllerComplex  = 2,
  parameter int unsigned RdPartialBufferSize  = 2,
  parameter int unsigned RdTagBits            = 4
) (
  input  logic                                       clk_i,
  input  logic                                       rst_ni,
  input  logic                                       test_enable_i,
  /// Only used for `XYRouting`, tie to '0 otherwise
  input  id_t                                        xy_id_i,
  /// Only used for `IdTable` routing, tie to '0 otherwise
  input  addr_rule_t [NumAddrRules-1:0]              id_route_map_i,
  /// Input channels
  input  logic  [NumInput-1:0][NumVirtChannels-1:0]  valid_i,
  output logic  [NumInput-1:0][NumVirtChannels-1:0]  ready_o,
  input  flit_t [NumInput-1:0][NumPhysChannels-1:0]  data_i,
  /// Output channels
  output logic  [NumOutput-1:0][NumVirtChannels-1:0] valid_o,
  input  logic  [NumOutput-1:0][NumVirtChannels-1:0] ready_i,
  output flit_t [NumOutput-1:0][NumPhysChannels-1:0] data_o,
  /// IF towards the offload logic
  output RdOperation_t                               offload_req_op_o,
  output RdData_t                                    offload_req_operand1_o,
  output RdData_t                                    offload_req_operand2_o,
  output logic                                       offload_req_valid_o,
  input logic                                        offload_req_ready_i,
  /// IF from external FPU
  input RdData_t                                     offload_resp_result_i,
  input logic                                        offload_resp_valid_i,
  output logic                                       offload_resp_ready_o
);

  // TODO MICHAERO: assert NumPhysChannels <= NumVirtChannels

  // When a offloadreduction is dedected then the data will be split off infront of the crossbar.
  // The completly reduced result will be inserted in the outpur arbiter for each output port therefor
  // requiring an extra port.

  // TODO RAROTH: when a reduction only involve one member then we could bypass the hole reduction part
  //              required changes:
  //              - move the block "floo_route_xymask" outside and do the $onehot() on the result!
  //              - change the condition of the stream_mux accordingly

  // Generate local Number of routes
  localparam int unsigned localNumInputs = (EnOffloadReduction == 1'b1) ? (NumInput + 1) : (NumInput);

  // Generate the vars to handle the input of the router
  flit_t [NumInput-1:0][NumVirtChannels-1:0] in_data, in_routed_data;
  logic  [NumInput-1:0][NumVirtChannels-1:0] in_valid, in_ready;
  logic  [NumInput-1:0][NumVirtChannels-1:0][NumOutput-1:0] route_mask;

  // Router input part
  for (genvar in = 0; in < NumInput; in++) begin : gen_input
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt_input

      logic [cf_math_pkg::idx_width(NumPhysChannels)-1:0] in_p;
      if (NumPhysChannels == 1) begin : gen_single_phys
        assign in_p = '0;
      end else if (NumPhysChannels == NumVirtChannels) begin : gen_virt_eq_phys
        assign in_p = v;
      end else begin : gen_odd_phys
        $fatal(1, "unimplemented");
      end

      (* ungroup *)
      stream_fifo_optimal_wrap #(
        .Depth  ( InFifoDepth ),
        .type_t ( flit_t      )
      ) i_stream_fifo (
        .clk_i      ( clk_i         ),
        .rst_ni     ( rst_ni        ),
        .testmode_i ( test_enable_i ),
        .flush_i    ( 1'b0  ),
        .usage_o    (       ),
        .data_i     ( data_i  [in][in_p] ),
        .valid_i    ( valid_i [in][v]    ),
        .ready_o    ( ready_o [in][v]    ),
        .data_o     ( in_data [in][v]    ),
        .valid_o    ( in_valid[in][v]    ),
        .ready_i    ( in_ready[in][v]    )
      );

      floo_route_select #(
        .NumRoutes        ( NumOutput        ),
        .flit_t           ( flit_t           ),
        .RouteAlgo        ( RouteAlgo        ),
        .IdWidth          ( IdWidth          ),
        .id_t             ( id_t             ),
        .NumAddrRules     ( NumAddrRules     ),
        .addr_rule_t      ( addr_rule_t      ),
        .EnMultiCast      ( EnMultiCast      )
      ) i_route_select (
        .clk_i,
        .rst_ni,
        .test_enable_i,

        .xy_id_i        ( xy_id_i               ),
        .id_route_map_i ( id_route_map_i        ),
        .channel_i      ( in_data       [in][v] ),
        .valid_i        ( in_valid      [in][v] ),
        .ready_i        ( in_ready      [in][v] ),
        .channel_o      ( in_routed_data[in][v] ),
        .route_sel_o    ( route_mask    [in][v] ),
        .route_sel_id_o (                       )
      );

    end
  end

  // Var for the "normal" dataflow without any reduction
  logic  [NumInput-1:0][NumVirtChannels-1:0] cross_valid, cross_ready;

  // Vars to branch the reduction off the main path (No virtual channel support for reduction)
  logic  [NumInput-1:0] red_valid_in, red_ready_in;
  logic  [NumInput-1:0][NumOutput-1:0] red_route_selected;
  flit_t [NumInput-1:0] red_data_in;

  // Vars for the data comming from the reduction
  logic  [NumInput-1:0] red_valid_out, red_ready_out;
  flit_t [NumOutput-1:0] red_data_out;


  // If we support offload reduction and a reduction is dedected then we split the signal and forward it to the reduction
  if(EnOffloadReduction == 1'b1) begin : gen_offload_reduction_demux
    for (genvar in = 0; in < NumInput; in++) begin : gen_input
      for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt_input
        // Generate the handshaking
        // TODO - Incorperate a reduction with only 1 master here and forward it directly
        stream_demux #(
          .N_OUP              (2)
        ) i_stream_demux (
          .inp_valid_i        (in_valid[in][v]),
          .inp_ready_o        (in_ready[in][v]),
          .oup_sel_i          (in_routed_data[in][v].hdr.commtype == OffloadReduction),
          .oup_valid_o        ({red_valid_in, cross_valid}),
          .oup_ready_i        ({red_ready_in, cross_ready})
        );
        // Assign the data
        assign red_data_in[in] = in_routed_data[in][v];
        assign red_route_selected[in] = route_mask[in][v];
      end
    end
  end else begin
    assign cross_valid = in_valid;
    assign in_ready = cross_ready;
    assign red_valid_in = '0;
    assign red_data_in = '0;
    assign red_route_selected = '0;
  end

  // Reduction logic
  if(EnOffloadReduction == 1'b1) begin : gen_reduction_logic
    floo_fp_reduction_arbiter #(
      .NumRoutes                (NumInput),
      .flit_t                   (flit_t),
      .hdr_t                    (hdr_t),
      .id_t                     (id_t),
      .RdData_t                 (RdData_t),
      .RdOperation_t            (RdOperation_t),
      .RdFifoDepth              (RdFifoDepth),
      .RdFifoFallThrough        (RdFifoFallThrough),
      .RdPipelineDepth          (RdPipelineDepth),
      .RdPartialBufferSize      (RdPartialBufferSize),
      .RdTagBits                (RdTagBits),
      .RdContollerComplexity    (RdControllerComplex)
    ) i_reduction_logic (
      .clk_i                    (clk_i),
      .rst_ni                   (rst_ni),
      .flush_i                  (1'b0),
      .valid_i                  (red_valid_in),
      .ready_o                  (red_ready_in),
      .data_i                   (red_data_in),
      .output_route_i           (red_route_selected),
      .node_id_i                (xy_id_i),
      .valid_o                  (red_valid_out),
      .ready_i                  (red_ready_out),
      .data_o                   (red_data_out),
      .reduction_req_op1_o      (offload_req_operand1_o),
      .reduction_req_op2_o      (offload_req_operand2_o),
      .reduction_req_type_o     (offload_req_op_o),
      .reduction_req_valid_o    (offload_req_valid_o),
      .reduction_req_ready_i    (offload_req_ready_i),
      .reduction_resp_data_i    (offload_resp_result_i),
      .reduction_resp_valid_i   (offload_resp_valid_i),
      .reduction_resp_ready_o   (offload_resp_ready_o)
    );    
  end else begin
    assign red_data_out = '0;
    assign red_valid_out = '0;
  end

  // Normal crossbar between all in / out routes
  logic [NumOutput-1:0][NumVirtChannels-1:0][NumInput-1:0] masked_valid, masked_ready;
  logic [NumInput-1:0][NumVirtChannels-1:0][NumOutput-1:0] masked_all_ready;
  logic [NumInput-1:0][NumVirtChannels-1:0][NumOutput-1:0] acc_masked_ready_q, acc_masked_ready_d;
  logic [NumInput-1:0][NumVirtChannels-1:0][NumOutput-1:0] current_accumulated;

  flit_t [NumOutput-1:0][NumVirtChannels-1:0][NumInput-1:0] masked_data;

  for (genvar in = 0; in < NumInput; in++) begin : gen_hs_input
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_hs_virt
      for (genvar out = 0; out < NumOutput; out++) begin : gen_hs_output
        // In case of loopback connections (to itself) and Y->X connections in XYRouting,
        // we tie the handshake & data signals to 0, to optimize them away during synthesis
        if((NoLoopback && (in == out)) ||
           ((RouteAlgo == XYRouting) && XYRouteOpt &&
            (in == South || in == North) && (out == East || out == West)))
        begin : gen_no_conn
          assign masked_all_ready[in][v][out] = '0;
          assign masked_valid[out][v][in]     = '0;
          assign masked_data[out][v][in]      = '0;
        end else begin : gen_conn
          assign masked_all_ready[in][v][out] = masked_ready[out][v][in];
          assign masked_valid[out][v][in]     = cross_valid[in][v] & route_mask[in][v][out] &
                                                (!EnMultiCast || ~acc_masked_ready_q[in][v][out]);
          assign masked_data[out][v][in]      = in_routed_data[in][v];
        end
      end
      if (!EnMultiCast) begin : gen_unicast
        assign cross_ready[in][v] = |(masked_all_ready[in][v] & route_mask[in][v]);
      end else begin : gen_multicast
        // TODO(fischeti): Clarify with Chen
        // Logic to fork the multicast packet into multiple ports
        assign acc_masked_ready_d[in][v] = (in_ready[in][v]) ? '0 :
                                            (acc_masked_ready_q[in][v] | masked_all_ready[in][v]);
        assign current_accumulated[in][v] = acc_masked_ready_q[in][v] | masked_all_ready[in][v];
        assign cross_ready[in][v] = &(current_accumulated[in][v] | ~(NoLoopback? (route_mask[in][v] & ~(1 << in)) : route_mask[in][v]));
      end
    end
  end

  `FF(acc_masked_ready_q, acc_masked_ready_d, '0)

  // We merge the data from the reduction as an additional input of our output arbiter.
  logic [NumOutput-1:0][NumVirtChannels-1:0][localNumInputs-1:0] merged_valid, merged_ready;
  flit_t [NumOutput-1:0][NumVirtChannels-1:0][localNumInputs-1:0] merged_data;

  if(EnOffloadReduction == 1'b1) begin : gen_assign_data_output
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_con_virt
      for (genvar out = 0; out < NumOutput; out++) begin : gen_con_output
        assign merged_data[out][v] = {red_data_out, masked_data[out][v]};
        assign merged_valid[out][v] = {red_valid_out, masked_valid[out][v]};
        assign masked_ready[out][v] = merged_ready[out][v][localNumInputs-2:0];
        assign red_ready_out = merged_ready[out][v][localNumInputs-1];
      end
    end
  end else begin
    assign merged_data = masked_data;
    assign merged_valid = masked_valid;
    assign masked_ready = merged_ready;
  end

  // Vars to handle the output of the arbiter and the optinal fifos
  flit_t [NumOutput-1:0][NumVirtChannels-1:0] out_data, out_buffered_data;
  logic  [NumOutput-1:0][NumVirtChannels-1:0] out_valid, out_ready;
  logic  [NumOutput-1:0][NumVirtChannels-1:0] out_buffered_valid_in, out_buffered_ready_in;

  for (genvar out = 0; out < NumOutput; out++) begin : gen_output

    // arbitrate input fifos per virtual channel
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt_output
      if(!EnReduction) begin : gen_wh_arb
        floo_wormhole_arbiter #(
          .NumRoutes  ( localNumInputs ),
          .flit_t     ( flit_t   )
        ) i_wormhole_arbiter (
          .clk_i,
          .rst_ni,

          .valid_i ( merged_valid[out][v] ),
          .ready_o ( merged_ready[out][v] ),
          .data_i  ( merged_data [out][v] ),

          .valid_o ( out_valid[out][v] ),
          .ready_i ( out_ready[out][v] ),
          .data_o  ( out_data [out][v] )
        );
      end else begin : gen_red_arb
        // Arbiter to be instantiated for reduction operations.
        // Repsonses from a multicast request are also treated as reductions.
        floo_output_arbiter #(
          .NumRoutes     ( localNumInputs      ),
          .flit_t        ( flit_t        ),
          .payload_t     ( payload_t     ),
          .NarrowRspMask ( NarrowRspMask ),
          .WideRspMask   ( WideRspMask   ),
          .id_t          ( id_t          )
        ) i_output_arbiter (
          .clk_i,
          .rst_ni,

          .valid_i  ( merged_valid[out][v] ),
          .ready_o  ( merged_ready[out][v] ),
          .data_i   ( merged_data [out][v] ),
          .xy_id_i  ( xy_id_i              ),

          .valid_o ( out_valid[out][v] ),
          .ready_i ( out_ready[out][v] ),
          .data_o  ( out_data [out][v] )
        );
      end

      if (OutFifoDepth > 0) begin : gen_out_fifo
        (* ungroup *)
        stream_fifo_optimal_wrap #(
          .Depth  ( OutFifoDepth  ),
          .type_t ( flit_t        )
        ) i_stream_fifo (
          .clk_i      ( clk_i         ),
          .rst_ni     ( rst_ni        ),
          .testmode_i ( test_enable_i ),
          .flush_i    ( 1'b0          ),
          .usage_o    (               ),
          .data_i     ( out_data          [out][v] ),
          .valid_i    ( out_valid         [out][v] ),
          .ready_o    ( out_ready         [out][v] ),
          .data_o     ( out_buffered_data [out][v] ),
          .valid_o    ( out_buffered_valid_in[out][v] ),
          .ready_i    ( out_buffered_ready_in[out][v] )
        );
      end else begin : gen_no_out_fifo
        assign out_buffered_data [out][v] = out_data          [out][v];
        assign out_buffered_valid_in[out][v] = out_valid         [out][v];
        assign out_ready         [out][v] = out_buffered_ready_in[out][v];
      end
    end

    // Arbitrate virtual channels onto the physical channel
    floo_vc_arbiter #(
      .NumVirtChannels ( NumVirtChannels ),
      .flit_t          ( flit_t          ),
      .NumPhysChannels ( NumPhysChannels )
    ) i_vc_arbiter (
      .clk_i,
      .rst_ni,

      .valid_i ( out_buffered_valid_in[out] ),
      .ready_o ( out_buffered_ready_in[out] ),
      .data_i  ( out_buffered_data [out] ),

      .ready_i ( ready_i  [out] ),
      .valid_o ( valid_o  [out] ),
      .data_o  ( data_o   [out] )
    );
  end

  for (genvar i = 0; i < NumInput; i++) begin : gen_input_assert
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt_assert
      // Assert that the input data is stable when valid is asserted
      // `ASSERT(StableDataIn, valid_i[i][v] && !ready_o[i][v] |=> $stable(data_i[i][v]))
      // Assert that valid is stable when ready is not asserted
      `ASSERT(StableValidIn, valid_i[i][v] && !ready_o[i][v] |=> $stable(valid_i[i][v]))
    end
  end

  for (genvar o = 0; o < NumOutput; o++) begin : gen_output_assert
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt_assert
      // Assert that the input data is stable when valid is asserted
      // `ASSERT(StableDataOut, valid_o[o][v] && !ready_i[o][v] |=> $stable(data_o[o][v]))
      // Assert that valid is stable when ready is not asserted
      `ASSERT(StableValidOut, valid_o[o][v] && !ready_i[o][v] |=> $stable(valid_o[o][v]))
    end
  end

  // If XYRouting optimization is enabled, assert that not Y->X routing occurs
  if ((RouteAlgo == XYRouting) && XYRouteOpt) begin : gen_xy_opt_assert
    for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt
      `ASSERT(XYDirectionNotAllowed,
          !(in_valid[South][v] && route_mask[South][v][East]) &&
          !(in_valid[South][v] && route_mask[South][v][West]) &&
          !(in_valid[North][v] && route_mask[North][v][East]) &&
          !(in_valid[North][v] && route_mask[North][v][West]))
    end
  end

  // If `NoLoopback` is enabled, assert that no loopback occurs
  if (NoLoopback) begin: gen_no_loopback_assert
    for (genvar in = 0; in < NumInput; in++) begin : gen_input
      for (genvar v = 0; v < NumVirtChannels; v++) begin : gen_virt
        `ASSERT(NoLoopback, !(in_valid[in][v] && route_mask[in][v][in] &&
                            (in_data[in][v].hdr.commtype == Unicast)))
      end
    end
  end

  // Multicast is currently only supported for `XYRouting`
  `ASSERT_INIT(NoMultiCastSupport, !(EnMultiCast && RouteAlgo != XYRouting))
  // Assertian check that when we use the FP reduction no virtual channel are init
  `ASSERT_INIT(NoVirtChanSupport, (EnOffloadReduction && (NumVirtChannels != 1)))

endmodule
