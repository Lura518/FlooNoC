// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Chen Wu
// Raphael Roth <raroth@student.ethz.ch>

// This module handles any offloaded reduction. It encompass only the control logic therefor allowing a selection of reduction logic.
// For an overview of the generated Hardware see the (personal) documentation of raroth (@pisoc3: /scratch2/msc25f10/documentation)
// As it is indepent of the reduction operation the type needs to be provided from the outside by RdOperation_t. The selected operation we want
// to support for now are defined under floo_pkg.sv @ floo_pkg::reduction_op_e!

// The main design goal was to allow a fully pipelined operation e.g. if out inputs provide each cycle a new elements to reduce
// then the underlying reduction utilization should be 100% during the reduction.

// One of the main design consideration was to use a tag based system. All elements which hold the same tag needs to be reduced together.
// This allows to separate the tag generation and the reduction logic. The reduction logic only needs to compare tag and if they
// match then reduce them. The tag generation is done on the input and is system independent. To reduce the size of the crossbar / multiplexer
// the tag is provided directly to the output by the controller.

// Additionally ever element gets an mask which indicates which elements are already reduced in the red_data. This allows for an easy
// comparison for the final result e.g. if it is equal to the input mask then all required inputs are reduced together.

// Currently we support three different complexity of reduction, see the controller for more dcumentation.

// Restriction:
// - Currently it is only supported that we reduce only one transmission at the time.
// - Either the same src issues multiple reduction to the same subset of dst (pipelined) or the phyisical links 
//   from two different reductions are not allowed to cross (sw restriction).
// - The max number of input is currently fixed to 6. This can be extended but then the bitwidth of the tag_t needs to be extended too.
//   The tag must be unique e.g. every data piece equipped with this tag needs to be reduced in the same result!
// - We only support symmetric configurations e.g. NumInput and NumOutput needs to be equal

// Open Points:
// - Support unsymmetric configurations!
// - The status of the reduction logic (e.g. FPU) is currently not evaluated / ignored by the contoller!

`include "common_cells/assertions.svh"

module floo_offload_reduction import floo_pkg::*; #(
  /// Number of Routes (Currently support only symmetric configurations)
  parameter int unsigned NumRoutes              = 1,
  /// Various types used by floonoc / routing
  parameter type         flit_t                 = logic,
  parameter type         hdr_t                  = logic,
  parameter type         id_t                   = logic,
  /// Data payload size to extract from the floo flit
  parameter type         RdData_t               = logic,
  /// Possible reduction operation(s)
  parameter type         RdOperation_t          = logic,
  /// Depth / Falltrough of the internal input FIFO
  parameter int unsigned RdFifoDepth            = 2,
  parameter bit          RdFifoFallThrough      = 1'b1,
  /// Pipeline depth of the external reduction logic (Needs to be at least 1)
  parameter int unsigned RdPipelineDepth        = 2,
  /// Partial buffer size for partial results (used in Generic / Stalling config)
  parameter int unsigned RdPartialBufferSize    = 2,
  /// Number of bits used for the reduction Bits (used in the Generic config)
  parameter int unsigned RdTagBits              = 4,
  /// Defines the controller complexity (0 = Simple / 1 = Stalling / 2 = Generic)
  parameter int unsigned RdContollerComplexity  = 2,
  /// Defines if the underlying protocol is AXI
  parameter bit          RdSupportAxi           = 1'b1,
  /// Axi Configuration
  parameter axi_cfg_t    AxiCfg                 = '0,
  /// Define if we support a bypass or not (for AXI AW header)
  parameter bit          RdEnableBypass         = 1'b1
) (
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  flush_i,
  input  id_t                                   node_id_i,
  /// Ports towards the input routes
  input  logic  [NumRoutes-1:0]                 valid_i,
  output logic  [NumRoutes-1:0]                 ready_o,
  input  flit_t [NumRoutes-1:0]                 data_i,
  input  logic  [NumRoutes-1:0][NumRoutes-1:0]  output_route_i,
  input  logic  [NumRoutes-1:0][NumRoutes-1:0]  expected_input_i,
  /// Ports towards the output routes
  output logic  [NumRoutes-1:0]                 valid_o,
  input  logic  [NumRoutes-1:0]                 ready_i,
  output flit_t [NumRoutes-1:0]                 data_o,
  /// IF towards external reduction
  output RdOperation_t                          reduction_req_type_o,
  output RdData_t                               reduction_req_op1_o,
  output RdData_t                               reduction_req_op2_o,
  output logic                                  reduction_req_valid_o,
  input  logic                                  reduction_req_ready_i,
  /// IF from external reduction
  input  RdData_t                               reduction_resp_data_i,
  input  logic                                  reduction_resp_valid_i,
  output logic                                  reduction_resp_ready_o
);

/* All local parameter */

// Set the complexity of the Controller
localparam bit GENERIC  = (RdContollerComplexity == 2) ? 1'b1 : 1'b0;
localparam bit SIMPLE   = (RdContollerComplexity == 0) ? 1'b1 : 1'b0;
localparam bit STALLING = (RdContollerComplexity == 1) ? 1'b1 : 1'b0;

/* All Typedef Vars */
// Index Variable to control the crossbar and the partial buffer
typedef logic [cf_math_pkg::idx_width(RdPartialBufferSize)-1:0] part_res_idx_t;

// Generate the types for the mask, the tag and the red_data
typedef logic [RdTagBits-1:0] tag_t;
typedef logic [NumRoutes-1:0] mask_t;

// dfferent combination between flit / data / tag / mask for the main data path
typedef struct packed {
  flit_t flit;
  mask_t input_exp;
  mask_t output_dir;
  tag_t tag;
} flit_in_out_dir_tag_t;

typedef struct packed {
  flit_t flit;
  mask_t mask;
  tag_t tag;
} flit_mask_tag_t;

typedef struct packed {
  RdData_t data;
  mask_t mask;
  tag_t tag;
} red_data_mask_tag_t;

typedef struct packed {
  RdData_t data;
  mask_t mask;
} red_data_mask_t;

typedef struct packed {
  RdData_t data;
  tag_t tag;
} red_data_tag_t;

/* Variable declaration */

// Variable for the tag generation
tag_t  [NumRoutes-1:0] fifo_tag;

// Output signals from the input FIFO's
flit_in_out_dir_tag_t [NumRoutes-1:0] fifo_out_data;
logic                 [NumRoutes-1:0] fifo_out_valid;
logic                 [NumRoutes-1:0] fifo_out_ready;

// Input signals which are already mapped to the corresponding operand
red_data_mask_tag_t [1:0] input_mapped_operands_data;
logic               [1:0] input_mapped_operands_valid;
logic               [1:0] input_mapped_operands_ready;

// Signal from the partial result buffer
red_data_mask_tag_t [1:0] partial_result_buffer_data;
logic               [1:0] partial_result_buffer_valid;
logic               [1:0] partial_result_buffer_ready;

// Signal after the merge between the partial result buffer and the inputs
red_data_mask_tag_t [1:0] merged_data;
logic               [1:0] merged_valid;
logic               [1:0] merged_ready;

// Signal after joining the handsake
logic join_operands_valid;
logic join_operands_ready;

// Var to determint the executed operation
RdOperation_t reduction_scheduled_operation;

// Signal for the FPU response
red_data_mask_tag_t reduction_resp_data;

// Singal to the partial result buffer
red_data_mask_tag_t input_partial_result_buf_data;
logic               input_partial_result_buf_valid;
logic               input_partial_result_buf_ready;

// Signal for the fully reduced result
red_data_tag_t  fully_reduced_data;
logic           fully_reduced_valid;
logic           fully_reduced_ready;

// Signal toward the output of the reduction logic
flit_mask_tag_t final_flit;
logic           final_valid;
logic           final_ready;

// Control Signal to either merge the partial buffer or the inputs
logic [1:0] ctrl_sel_part_res;

// Control Signal for the output demultiplexer
logic ctrl_demux;

// Selector for the partial result buffer
part_res_idx_t [1:0] ctrl_sel_buffer_idx;

// Spyglass signals from the partial result buffer
tag_t [RdPartialBufferSize-1:0] spyglass_tag;
logic [RdPartialBufferSize-1:0] spyglass_valid;

/* Module Declaration */

// Only generate the Tag if we generate the most generic hardware.
if(GENERIC == 1'b1) begin : gen_tag_generation
  // For each incoming element generate the corresponding tag.
  floo_offload_reduction_taggen #(
      .NumRoutes        (NumRoutes),
      .TAG_T            (tag_t)
  ) i_gen_tag (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      .flush_i          (flush_i),
      .mask_i           (expected_input_i),
      .valid_i          (valid_i),
      .ready_i          (ready_o),
      .tag_o            (fifo_tag)
  );
end else begin : gen_bypass_tag_generation
  assign fifo_tag = '0;
end

// Fifo's for all inputs to ack the incoming data and reduce unnecessary backpressure in the system
for (genvar i = 0; i < NumRoutes; i++) begin : gen_optinal_fifo
    // Buffer the input inside a (very) small fifo
    stream_fifo #(
      .FALL_THROUGH           (RdFifoFallThrough),
      .DEPTH                  (RdFifoDepth),
      .T                      (flit_in_out_dir_tag_t)
    ) i_in_fifo_generic (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .flush_i                (flush_i),
      .testmode_i             (1'b0),
      .usage_o                (),
      .data_i                 ({data_i[i], expected_input_i[i], output_route_i[i], fifo_tag[i]}),
      .valid_i                (valid_i[i]),
      .ready_o                (ready_o[i]),
      .data_o                 (fifo_out_data[i]),
      .valid_o                (fifo_out_valid[i]),
      .ready_i                (fifo_out_ready[i])
    );
end

// Controller which runs the hole reduction
floo_offload_reduction_controller #(
  .NumRoutes                    (NumRoutes),
  .RdPartialBufferSize          (RdPartialBufferSize),
  .RdPipelineDepth              (RdPipelineDepth),
  .RdData_t                     (RdData_t),
  .RdOperation_t                (RdOperation_t),
  .tag_t                        (tag_t),
  .mask_t                       (mask_t),
  .flit_t                       (flit_t),
  .hdr_t                        (hdr_t),
  .data_tag_t                   (red_data_tag_t),
  .data_mask_tag_t              (red_data_mask_tag_t),
  .flit_mask_tag_t              (flit_mask_tag_t),
  .flit_in_out_dir_tag_t        (flit_in_out_dir_tag_t),
  .idx_part_res_t               (part_res_idx_t),
  .GENERIC                      (GENERIC),
  .SIMPLE                       (SIMPLE),
  .STALLING                     (STALLING),
  .RdSupportAxi                 (RdSupportAxi),
  .AxiCfg                       (AxiCfg),
  .RdEnableBypass               (RdEnableBypass)
) i_reduction_controller (
  .clk_i                        (clk_i),
  .rst_ni                       (rst_ni),
  .flush_i                      (flush_i),
  .head_fifo_flit_i             (fifo_out_data),
  .head_fifo_valid_i            (fifo_out_valid),
  .head_fifo_ready_o            (fifo_out_ready),
  .operand_data_o               (input_mapped_operands_data),
  .operand_valid_o              (input_mapped_operands_valid),
  .operand_ready_i              (input_mapped_operands_ready),
  .reduction_req_operation_o    (reduction_scheduled_operation),
  .reduction_req_valid_i        (reduction_req_valid_o),
  .reduction_req_ready_i        (reduction_req_ready_i),
  .fully_red_data_i             (fully_reduced_data),
  .fully_red_valid_i            (fully_reduced_valid),
  .fully_red_ready_o            (fully_reduced_ready),
  .reduction_resp_mask_i        (reduction_resp_data.mask),
  .reduction_resp_tag_i         (reduction_resp_data.tag),
  .reduction_resp_valid_i       (reduction_resp_valid_i),
  .reduction_resp_ready_i       (reduction_resp_ready_o),
  .final_flit_o                 (final_flit),
  .final_valid_o                (final_valid),
  .final_ready_i                (final_ready),
  .buf_spyglass_tag_i           (spyglass_tag),
  .buf_spyglass_valid_i         (spyglass_valid),
  .select_partial_result_idx_o  (ctrl_sel_buffer_idx),
  .ctrl_part_res_mux_o          (ctrl_sel_part_res),
  .ctrl_output_demux_o          (ctrl_demux)
);

// Generate the MUX to include the partial buffer only if we either use the GENERIC case or the stalling case
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_mux_partial_result
  for (genvar i = 0; i < 2; i++) begin : gen_mux_partial_result_loop
      stream_mux #(
          .DATA_T             (red_data_mask_tag_t),
          .N_INP              (2)
      ) i_merge_part_res_and_input (
          .inp_data_i         ({partial_result_buffer_data[i], input_mapped_operands_data[i]}),
          .inp_valid_i        ({partial_result_buffer_valid[i], input_mapped_operands_valid[i]}),
          .inp_ready_o        ({partial_result_buffer_ready[i], input_mapped_operands_ready[i]}),
          .inp_sel_i          (ctrl_sel_part_res[i]),
          .oup_data_o         (merged_data[i]),
          .oup_valid_o        (merged_valid[i]),
          .oup_ready_i        (merged_ready[i])
      );
  end
end else begin : gen_bypass_mux_partial_result
  assign merged_data = input_mapped_operands_data;
  assign merged_valid = input_mapped_operands_valid;
  assign input_mapped_operands_ready = merged_ready;
  assign partial_result_buffer_ready = '0;
end

// Join the Handshake for the operands controll path's
stream_join #(
    .N_INP                  (2)
) i_join_controlpath_operands (
    .inp_valid_i            (merged_valid),
    .inp_ready_o            (merged_ready),
    .oup_valid_o            (join_operands_valid),
    .oup_ready_i            (join_operands_ready)
);

// Connect the HS for the output request to the FPU
assign reduction_req_valid_o = join_operands_valid;
assign join_operands_ready = reduction_req_ready_i;

// Output the operands here
assign reduction_req_op1_o = merged_data[0].data;
assign reduction_req_op2_o = merged_data[1].data;
assign reduction_req_type_o = reduction_scheduled_operation;


// Note: At this position in the dataflow of this file lies the external reduction hardware (mostly FPU)!
// After some (3) cycles the request turns comes back as respons!
// The external Reduction alg needs at least 1 cycle (to avoid loops)!

// We have fifo's for the tag as the FPU tag is otherwise used
if(GENERIC == 1'b1) begin : gen_fifo_for_tag
  fifo_v3 #(
      .FALL_THROUGH     (1'b0),
      .dtype            (tag_t),
      .DEPTH            (RdPipelineDepth+1)
  ) i_fifo_mask_parallel_fpu (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      .flush_i          (flush_i),
      .testmode_i       (1'b0),
      .full_o           (),
      .empty_o          (),
      .usage_o          (),
      .data_i           (merged_data[0].tag), 
      .push_i           (reduction_req_ready_i & reduction_req_valid_o),  // push mask on active fpu req hs
      .data_o           (reduction_resp_data.tag),
      .pop_i            (reduction_resp_valid_i & reduction_resp_ready_o) // pop mask on active fpu resp hs
  );
end else begin
  assign reduction_resp_data.tag = '0;
end

// We have fifo's for the mask as the FPU tag is otherwise used
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_fifo_for_mask
  fifo_v3 #(
      .FALL_THROUGH     (1'b0),
      .dtype            (mask_t),
      .DEPTH            (RdPipelineDepth+1)
  ) i_fifo_mask_parallel_fpu (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      .flush_i          (flush_i),
      .testmode_i       (1'b0),
      .full_o           (),
      .empty_o          (),
      .usage_o          (),
      .data_i           (merged_data[0].mask | merged_data[1].mask), // Or Connect both involved Mask
      .push_i           (reduction_req_ready_i & reduction_req_valid_o),  // push mask on active fpu req hs
      .data_o           (reduction_resp_data.mask),
      .pop_i            (reduction_resp_valid_i & reduction_resp_ready_o) // pop mask on active fpu resp hs
  );
end else begin
  assign reduction_resp_data.mask = '0;
end

// Merge the response from the reduction with the interal tag / mask storage
assign reduction_resp_data.data = reduction_resp_data_i;

// Demux the output of the fpu
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_demux_partial_result
  stream_demux #(
    .N_OUP              (2)
  ) i_stream_demux_output_fpu (
    .inp_valid_i        (reduction_resp_valid_i),
    .inp_ready_o        (reduction_resp_ready_o),
    .oup_sel_i          (ctrl_demux),
    .oup_valid_o        ({fully_reduced_valid, input_partial_result_buf_valid}),
    .oup_ready_i        ({fully_reduced_ready, input_partial_result_buf_ready})
  );
end else begin
  assign fully_reduced_valid = reduction_resp_valid_i;
  assign reduction_resp_ready_o = fully_reduced_ready;
  assign input_partial_result_buf_valid = 1'b0;
end

// Assign the data beloning to the mux
assign input_partial_result_buf_data = reduction_resp_data;
assign fully_reduced_data.data = reduction_resp_data.data;
assign fully_reduced_data.tag = reduction_resp_data.tag;

// Dynammically fork the data into the coorect output direction
// (Currently only 1 output direction is set by the dyn fork. 
// However in the future we want to have more than 1 output dir and therefor this is already supported)
stream_fork_dynamic #(
  .N_OUP          (NumRoutes)
) i_dynamic_fork (
  .clk_i          (clk_i),
  .rst_ni         (rst_ni),
  .valid_i        (final_valid),
  .ready_o        (final_ready),
  .sel_i          (final_flit.mask),
  .sel_valid_i    (final_valid),
  .sel_ready_o    (),
  .valid_o        (valid_o),
  .ready_i        (ready_i)
);

// Dublicate the output data for all output IF
always_comb begin : gen_dublicate_output_data
  data_o = '0;
  for(int i = 0; i < NumRoutes;i++) begin
    data_o[i] = final_flit.flit;
  end
end

// Generate the partial result buffer only if we are in the GENERIC or the STALLING case
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_partial_result_buffer
  floo_offload_reduction_buffer #(
      .data_mask_tag_t    (red_data_mask_tag_t),
      .tag_t              (tag_t),
      .NElements          (RdPartialBufferSize),
      .NOutPorts          (2)
  ) i_buf_part_result (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .flush_i            (flush_i),
      .inp_data_i         (input_partial_result_buf_data),
      .inp_valid_i        (input_partial_result_buf_valid),
      .inp_ready_o        (input_partial_result_buf_ready),
      .oup_data_o         (partial_result_buffer_data),
      .oup_valid_o        (partial_result_buffer_valid),
      .oup_ready_i        (partial_result_buffer_ready),
      .inp_sel_valid_i    (ctrl_sel_part_res),
      .inp_sel_i          (ctrl_sel_buffer_idx),
      .spyglass_valid_o   (spyglass_valid), 
      .spyglass_tag_o     (spyglass_tag)
  );
end else begin
  assign input_partial_result_buf_ready = 1'b0;
  assign partial_result_buffer_data = '0;
  assign partial_result_buffer_valid = '0;
  assign spyglass_valid = '0;
  assign spyglass_tag = '0;
end

/* ASSERTION Checks */
// The fp reduction supports up to 6 operands
`ASSERT_INIT(Number_Input_Route_Invalid, !(NumRoutes > 6))
// Currently we only support reduction extension with an pipeline depth of at least 1 cycle as otherwise loops could be generated!
`ASSERT_INIT(ReductionPipelineDepth, !(RdPipelineDepth == 0))
// The size needs to be at least 2 for the partial buffer for the generic / stalling proceessor
`ASSERT_INIT(PartialBufferSize, !((GENERIC | STALLING) && (RdPartialBufferSize < 2)))
// We can only run GENERIC or SIMPLE or STALLING
`ASSERT_INIT(Invalid_Configuration_1, !(GENERIC & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_2, !(STALLING & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_3, !(GENERIC & STALLING))
`ASSERT_INIT(Invalid_Configuration_4, (GENERIC | STALLING | SIMPLE))

endmodule
