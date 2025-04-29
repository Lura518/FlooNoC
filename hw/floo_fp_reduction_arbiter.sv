// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Chen Wu
// Raphael Roth <raroth@student.ethz.ch>

// This module handle the floating point reduction.
// For an overview of the generated Hardware see the figure ....
// Currently we allow the following reduction: FADD / FMUL / FMin / FMax
// These are defined in the coressponding CommType from the floo_pkg::commtype_e
// The main design goal was to allow a fully pipelined operation e.g. If out inputs provide each cycle a new elements to reduce
// then the underlying FPU utilization should be 100% during the reduction.

// One of the main design consideration was to use a tag based system. All elements which hold the same tag needs to be reduced together.
// This allows to separate the tag generation and the reduction logic. The reduction logic only needs to compare tag and if they
// match then reduce them. The tag generation is done on the input and is system independent. To reduce the size of the crossbar / multiplexer
// the tag is provided directly to the output by the controller.

// Additionally ever element gets an mask which indicates which elements are already reduced in the red_data. This allows for an easy
// Comparison for the final result e.g. if it is equal to the input mask then all required inputs are reduced together.

// Restriction:
// - Currently it is only supported that we reduce only one transmission at the time.
// - Either the same src issues multiple reduction to the same subset of dst (pipelined) or the phyisical links 
//   from two different reductions are not allowed to cross (sw restriction).
// - The max number of input is currently fixed to 6. This can be extended but then the bitwidth of the tag_t needs to be extended too.
//   The tag must be unique e.g. every data piece equipped with this tag needs to be reduced in the same result!

// Open Points:
// - The status of the FPU resp is currently not evaluated by the contoller and is ignored!

`include "common_cells/assertions.svh"

module floo_fp_reduction_arbiter import floo_pkg::*; #(
  parameter int unsigned NumRoutes              = 1,
  parameter type         flit_t                 = logic,
  parameter type         hdr_t                  = logic,
  parameter type         id_t                   = logic,
  parameter type         RdData_t               = logic,
  parameter type         RdOperation_t          = logic,
  parameter int unsigned RdFifoDepth            = 2,
  parameter bit          RdFifoFallThrough      = 1'b1,
  parameter int unsigned RdPipelineDepth        = 2,
  parameter int unsigned RdPartialBufferSize    = 2,
  parameter int unsigned RdTagBits              = 4,
  parameter int unsigned RdContollerComplexity  = 2
) (
  input  logic                                  clk_i,
  input  logic                                  rst_ni,
  input  logic                                  flush_i,
  /// Ports towards the input routes
  input  logic  [NumRoutes-1:0]                 valid_i,
  output logic  [NumRoutes-1:0]                 ready_o,
  input  flit_t [NumRoutes-1:0]                 data_i,
  input  logic  [NumRoutes-1:0][NumRoutes-1:0]  output_route_i,
  input  id_t                                   node_id_i,
  /// Ports towards the output routes
  output logic  [NumRoutes-1:0]                 valid_o,
  input  logic  [NumRoutes-1:0]                 ready_i,
  output flit_t [NumRoutes-1:0]                 data_o,
  /// IF towards external FPU
  output RdData_t                               reduction_req_op1_o,
  output RdData_t                               reduction_req_op2_o,
  output RdOperation_t                          reduction_req_type_o,
  output logic                                  reduction_req_valid_o,
  input logic                                   reduction_req_ready_i,
  /// IF from external FPU
  input RdData_t                                reduction_resp_data_i,
  input logic                                   reduction_resp_valid_i,
  output logic                                  reduction_resp_ready_o
);

/* All local parameter */

// Set the complexity of the Controller
localparam bit GENERIC  = (RdContollerComplexity == 2) ? 1'b1 : 1'b0;
localparam bit SIMPLE   = (RdContollerComplexity == 0) ? 1'b1 : 1'b0;
localparam bit STALLING = (RdContollerComplexity == 1) ? 1'b1 : 1'b0;

// Determint intermidiate parameters
localparam int unsigned  NumOutCrossbar = 4;    // Two operands, one bypass to the output and one dummy output
localparam int unsigned  NumInCrossbar = NumRoutes;

// Dummyparam to allow shifting
localparam bit [NumRoutes-1:0] ONES = 1;

/* All Typedef Vars */
// Index Variable to control the crossbar and the partial buffer
typedef logic [cf_math_pkg::idx_width(NumInCrossbar)-1:0] in_cross_idx_t;
typedef logic [cf_math_pkg::idx_width(NumOutCrossbar)-1:0] out_cross_idx_t;
typedef logic [cf_math_pkg::idx_width(RdPartialBufferSize)-1:0] part_res_idx_t;
typedef logic [cf_math_pkg::idx_width(NumRoutes)-1:0] route_idx_t;

// Generate the types for the mask, the tag and the red_data
typedef logic [RdTagBits-1:0] tag_t;
typedef logic [NumRoutes-1:0] mask_t;

// Typedef for the main datapath
typedef struct packed {
  flit_t flit;
  tag_t tag;
  mask_t mask;
  mask_t output_dir;
} flit_mask_tag_t;

typedef struct packed {
  RdData_t data;
  tag_t tag;
  mask_t mask;
  mask_t output_dir;
} red_data_mask_tag_t;

typedef struct packed {
  RdData_t data;
  mask_t mask;
} red_data_mask_t;

/* Variable declaration */

// Variable for the tag generation
mask_t [NumRoutes-1:0] incoming_mask;
tag_t  [NumRoutes-1:0] fifo_tag;

// Output signals for the input FIFO's
logic           [NumRoutes-1:0] fifo_out_valid_q;
logic           [NumRoutes-1:0] fifo_out_ready_q;
flit_mask_tag_t [NumRoutes-1:0] fifo_out_data_q;

// Temp Signals for conversation
logic           [NumRoutes-1:0] temp_crossbar_valid;
logic           [NumRoutes-1:0] temp_crossbar_ready;
red_data_mask_t [NumRoutes-1:0] temp_crossbar_in;

// Output signal of the crossbar - the MSB signal will not be used!
logic [NumOutCrossbar-1:0] crossbar_out_valid;
logic [NumOutCrossbar-1:0] crossbar_out_ready;
red_data_mask_t [NumOutCrossbar-1:0] crossbar_out_data;

// Signal from the partial result
logic [1:0] part_res_valid;
logic [1:0] part_res_ready;
red_data_mask_t [1:0] part_res_data;

// Signal after the merge between the partial result and the input
logic [1:0] merge_part_res_input_valid;
logic [1:0] merge_part_res_input_ready;
red_data_mask_t [1:0] merge_part_res_input_data;

// Signal after joining the handsake
logic join_operands_valid;
logic join_operands_ready;

// Provide metadata for the fpu resp to the FPU
tag_t reduction_tag;
RdOperation_t reduction_type;

// Signal for the FPU response
red_data_mask_tag_t reduction_resp_data;

// Signal for the partial result buffer input
logic in_buf_part_res_valid;
logic in_buf_part_res_ready;
red_data_mask_tag_t in_buf_part_res_data;

// Output Signal before merging the bypass signal
logic output_befor_bypass_valid;
logic output_befor_bypass_ready;
RdData_t output_befor_bypass_data;

// Output Signal
logic out_valid;
logic out_ready;
RdData_t out_data;
flit_t out_header;
flit_t out_data_merged;

// Output Route Direction
logic [NumRoutes-1:0] output_route_onehot;
route_idx_t output_route_binary;

// Control Signal for the Crossbar
out_cross_idx_t [NumInCrossbar-1:0] ctrl_sel_input;     // For each input select to which output it should connect to
in_cross_idx_t [NumOutCrossbar-1:0] ctrl_sel_output;    // For each output select to which input it should connect to

// Control Signal to either merge the partial buffer or the inputs
logic [1:0] ctrl_sel_part_res;

// Control Signal for the output demultiplexer
logic ctrl_demux;

// Control Signal which allows to bypass the reduction for one signa
logic ctrl_bypass_reduction;

// Selector for the partial result buffer
part_res_idx_t [1:0] ctrl_sel_buffer_idx;

// Spyglass signals from the partial result buffer
tag_t [RdPartialBufferSize-1:0] spyglass_tag;
logic [RdPartialBufferSize-1:0] spyglass_valid;

/* Module Declaration */

// With pure logic generate a Mask which indicates from which input we expect a element for the reduction
for (genvar i = 0; i < NumRoutes; i++) begin : gen_input_mask
  floo_route_xymask #(
    .NumRoutes (NumRoutes),
    .flit_t    (flit_t),
    .id_t      (id_t),
    .FwdMode      (0)
  ) i_gen_route_xymask (
    .channel_i (data_i[i]),
    .xy_id_i   (node_id_i),
    .route_sel_o(incoming_mask[i])
  );
end

// Only generate the Tag if we generate the most generic hardware.
if(GENERIC == 1'b1) begin : gen_tag_generation
  // For each incoming element generate the corresponding tag.
  floo_fp_reduction_taggen #(
      .NumRoutes        (NumRoutes),
      .TAG_T            (tag_t)
  ) i_gen_tag (
      .clk_i            (clk_i),
      .rst_ni           (rst_ni),
      .flush_i          (flush_i),
      .mask_i           (incoming_mask),
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
      .T                      (flit_mask_tag_t)
    ) i_in_fifo_generic (
      .clk_i                  (clk_i),
      .rst_ni                 (rst_ni),
      .flush_i                (flush_i),
      .testmode_i             (1'b0),
      .usage_o                (),
      .data_i                 ({data_i[i], fifo_tag[i], incoming_mask[i], output_route_i[i]}),    // Correct ordering?
      .valid_i                (valid_i[i]),
      .ready_o                (ready_o[i]),
      .data_o                 (fifo_out_data_q[i]),
      .valid_o                (fifo_out_valid_q[i]),
      .ready_i                (fifo_out_ready_q[i])
    );
end

if((GENERIC == 1'b1) || (SIMPLE == 1'b1)) begin : gen_bypass_stalling
  // When we are in the generic case / simple case then we can directly bypass the stalling
  for (genvar i = 0; i < NumRoutes; i++) begin : gen_bypass_stalling_loop
    // Connect the handshake
    assign temp_crossbar_valid[i] = fifo_out_valid_q[i];
    assign fifo_out_ready_q[i] = temp_crossbar_ready[i];

    // Assign the data Signals here
    assign temp_crossbar_in[i].data = fifo_out_data_q[i].flit.payload[$bits(RdData_t)-1:0]; // TODO: Verify if this statement works properly
    assign temp_crossbar_in[i].mask = ONES << i;                   // Indicate the input (TODO: Verify Correctness)
  end
end else begin : gen_stalling
  for (genvar i = 0; i < NumRoutes; i++) begin : gen_stalling_loop
    // Generate the stalling modul which backpressures any signal until the output of the arbiter handsakes
    floo_fp_reduction_stalling i_stalling (
      .clk_i           (clk_i),
      .rst_ni          (rst_ni),
      .flush_i         (flush_i),
      .src_valid_i     (fifo_out_valid_q[i]),
      .src_ready_o     (fifo_out_ready_q[i]),
      .stalling_i      (valid_o & ready_i),
      .dst_valid_o     (temp_crossbar_valid[i]),
      .dst_ready_i     (temp_crossbar_ready[i])
    );

    // Assign the data Signals here
    assign temp_crossbar_in[i].data = fifo_out_data_q[i].flit.payload[$bits(RdData_t)-1:0]; // TODO: Verify if this statement works properly
    assign temp_crossbar_in[i].mask = ONES << i;                   // Indicate the input (TODO: Verify Correctness)
  end
end

// Instanciate the crossbar
floo_fp_reduction_crossbar #(
  .DATA_T           (red_data_mask_t),
  .N_INP            (NumInCrossbar),
  .N_OUP            (NumOutCrossbar)
) i_reduced_crossbar (
  .inp_data_i       (temp_crossbar_in),
  .inp_valid_i      (temp_crossbar_valid),
  .inp_ready_o      (temp_crossbar_ready),

  /// All Output Connections
  .oup_data_o       (crossbar_out_data),
  .oup_valid_o      (crossbar_out_valid),
  .oup_ready_i      (crossbar_out_ready),

  /// Selections
  .inp_sel_i        (ctrl_sel_input),  // For each input select to which output it should connect to
  .oup_sel_i        (ctrl_sel_output)  // For each output select to which input it should connect to
);

// Tie down the dummy crossbar output
assign crossbar_out_ready[NumOutCrossbar-1] = 1'b0;

// Generate the MUX to include the partial buffer only if we either use the GENERIC case or the stalling case
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_mux_partial_result
  for (genvar i = 0; i < 2; i++) begin : gen_mux_partial_result_loop
      stream_mux #(
          .DATA_T             (red_data_mask_t),
          .N_INP              (2)
      ) i_merge_part_res_and_input (
          .inp_data_i         ({part_res_data[i], crossbar_out_data[i]}),
          .inp_valid_i        ({part_res_valid[i], crossbar_out_valid[i]}),
          .inp_ready_o        ({part_res_ready[i], crossbar_out_ready[i]}),
          .inp_sel_i          (ctrl_sel_part_res[i]),
          .oup_data_o         (merge_part_res_input_data[i]),
          .oup_valid_o        (merge_part_res_input_valid[i]),
          .oup_ready_i        (merge_part_res_input_ready[i])
      );
  end
end else begin : gen_bypass_mux_partial_result
  for (genvar i = 0; i < 2; i++) begin : gen_bypass_mux_partial_result_loop
    assign merge_part_res_input_data[i] = crossbar_out_data[i];
    assign merge_part_res_input_valid[i] = crossbar_out_valid[i];
    assign crossbar_out_ready[i] = merge_part_res_input_ready[i];
    assign part_res_ready[i] = 1'b0;
  end
end

// Join the Handshake for the operands controll path's
stream_join #(
    .N_INP                  (2)
) i_join_controlpath_operands (
    .inp_valid_i            (merge_part_res_input_valid),
    .inp_ready_o            (merge_part_res_input_ready),
    .oup_valid_o            (join_operands_valid),
    .oup_ready_i            (join_operands_ready)
);

// Connect the HS for the output request to the FPU
assign reduction_req_valid_o = join_operands_valid;
assign join_operands_ready = reduction_req_ready_i;

// Output the operands here
assign reduction_req_op1_o = merge_part_res_input_data[0].data;
assign reduction_req_op2_o = merge_part_res_input_data[1].data;
assign reduction_req_type_o = reduction_type;

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
      .data_i           (reduction_tag), // Or Connect both involved Mask
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
      .data_i           (merge_part_res_input_data[0].mask | merge_part_res_input_data[1].mask), // Or Connect both involved Mask
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
    .oup_valid_o        ({output_befor_bypass_valid, in_buf_part_res_valid}),
    .oup_ready_i        ({output_befor_bypass_ready, in_buf_part_res_ready})
  );
end else begin
  assign output_befor_bypass_valid = reduction_resp_valid_i;
  assign reduction_resp_ready_o = output_befor_bypass_ready;
  assign in_buf_part_res_valid = 1'b0;
end

// Assign the data beloning to the mux
assign output_befor_bypass_data = reduction_resp_data.data;
assign in_buf_part_res_data.data = reduction_resp_data.data;
assign in_buf_part_res_data.mask = reduction_resp_data.mask;
assign in_buf_part_res_data.tag = reduction_resp_data.tag;

// This Mux allows to directly connect an input to the output to bypass the reduction if only one element is involved.
// Mux only the payload to the output (all metadata provided by the controller!)
stream_mux #(
  .DATA_T             (RdData_t),
  .N_INP              (2)
) i_merge_part_res_and_input (
  .inp_data_i         ({crossbar_out_data[NumOutCrossbar-2].data, output_befor_bypass_data}),
  .inp_valid_i        ({crossbar_out_valid[NumOutCrossbar-2], output_befor_bypass_valid}),
  .inp_ready_o        ({crossbar_out_ready[NumOutCrossbar-2], output_befor_bypass_ready}),
  .inp_sel_i          (ctrl_bypass_reduction),
  .oup_data_o         (out_data),
  .oup_valid_o        (out_valid),
  .oup_ready_i        (out_ready)
);

// Generate the Output data from the provided data from the controller and the result from the Mux
always_comb begin
  out_data_merged = out_header;
  out_data_merged.payload[$bits(RdData_t)-1:0] = out_data;
end

// Decoed the output route from the controller
onehot_to_bin #(
    .ONEHOT_WIDTH   (NumRoutes)
) i_onehot_decoding  (
    .onehot         (output_route_onehot),
    .bin            (output_route_binary)
);

// Split data up according to the input output map
stream_demux #(
  .N_OUP          (NumRoutes)
) i_demux_different_output_dir (
  .inp_valid_i    (out_valid),
  .inp_ready_o    (out_ready),
  .oup_sel_i      (output_route_binary),
  .oup_valid_o    (valid_o),
  .oup_ready_i    (ready_i)
);

// Dublicate the output data for all output IF
for (genvar i = 0; i < NumRoutes; i++) begin : gen_dublicate_output_data
  assign data_o[i] = out_data_merged;
end

// Generate the partial result buffer only if we are in the GENERIC or the STALLING case
if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_partial_result_buffer
  floo_fp_reduction_buffer #(
      .TAG_T              (tag_t),
      .DATA_T             (red_data_mask_t),
      .N_ELEMENTS         (RdPartialBufferSize),
      .N_OUT_PORTS        (2)
  ) i_buf_part_result (
      .clk_i              (clk_i),
      .rst_ni             (rst_ni),
      .flush_i            (flush_i),
      .inp_tag_i          (in_buf_part_res_data.tag),
      .inp_data_i         ({in_buf_part_res_data.data, in_buf_part_res_data.mask}), // TODO: Is this correct the assignment
      .inp_valid_i        (in_buf_part_res_valid),
      .inp_ready_o        (in_buf_part_res_ready),
      .oup_data_o         (part_res_data),
      .oup_valid_o        (part_res_valid),
      .oup_ready_i        (part_res_ready),
      .inp_sel_i          (ctrl_sel_buffer_idx),
      .spyglass_valid_o   (spyglass_valid), 
      .spyglass_tag_o     (spyglass_tag)
  );
end else begin
  assign in_buf_part_res_ready = 1'b0;
  assign part_res_data = '0;
  assign part_res_valid = '0;
  assign spyglass_valid = '0;
  assign spyglass_tag = '0;
end


// Main Controller which controls all crossbars / mux / etc.
floo_fp_reduction_controller #(
    .NumRoutes                  (NumRoutes),
    .NumInCrossbar              (NumInCrossbar),
    .NumOutCrossbar             (NumOutCrossbar),
    .RdPartialBufferSize        (RdPartialBufferSize),
    .RdPipelineDepth            (RdPipelineDepth),
    .RdOperation_t              (RdOperation_t),
    .tag_t                      (tag_t),
    .mask_t                     (mask_t),
    .flit_t                     (flit_t),
    .flit_mask_tag_t            (flit_mask_tag_t),
    .idx_out_cross_t            (out_cross_idx_t),
    .idx_in_cross_t             (in_cross_idx_t),
    .idx_part_res_t             (part_res_idx_t),
    .GENERIC                    (GENERIC),
    .SIMPLE                     (SIMPLE),
    .STALLING                   (STALLING)
) i_reduction_controller (
    .clk_i                      (clk_i),
    .rst_ni                     (rst_ni),
    .flush_i                    (flush_i),
    .head_fifo_data_i           (fifo_out_data_q),  // Provide data from the head of the fifo
    .head_fifo_valid_i          (fifo_out_valid_q),
    .buf_spyglass_tag_i         (spyglass_tag),     // Provide data from the partial result buffer
    .buf_spyglass_valid_i       (spyglass_valid),
    .reduction_req_tag_o        (reduction_tag),
    .reduction_type_o           (reduction_type),
    .reduction_req_valid_i      (reduction_req_valid_o),
    .reduction_req_ready_i      (reduction_req_ready_i),
    .reduction_resp_valid_i     (reduction_resp_valid_i),
    .reduction_resp_ready_i     (reduction_resp_ready_o),
    .output_valid_i             (out_valid),
    .output_ready_i             (out_ready),
    .reduction_resp_tag_i       (reduction_resp_data.tag),
    .reduction_resp_mask_i      (reduction_resp_data.mask),
    .header_output_o            (out_header),
    .output_dir_o               (output_route_onehot),
    .ctrl_sel_in_cross_o        (ctrl_sel_input),     // Output selectors for the crossbar
    .ctrl_sel_out_cross_o       (ctrl_sel_output),
    .ctrl_sel_part_res_buf_o    (ctrl_sel_buffer_idx),  // Both output selector(s) for the partial result buffer
    .ctrl_part_res_mux_o        (ctrl_sel_part_res),
    .ctrl_output_demux_o        (ctrl_demux),
    .ctrl_bypass_reduction_o    (ctrl_bypass_reduction)
);


/* ASSERTION Checks */
// The fp reduction supports up to 6 operands
`ASSERT_INIT(Number_Input_Route_Invalid, !(NumRoutes > 6))
// Currently we only support reduction extension with an pipeline depth of at least 1 cycle as otherwise loops could be generated!
`ASSERT_INIT(ReductionPipelineDepth, !(RdPipelineDepth == 0))
// We can only run GENERIC or SIMPLE or STALLING
`ASSERT_INIT(Invalid_Configuration_1, !(GENERIC & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_2, !(STALLING & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_3, !(GENERIC & STALLING))
`ASSERT_INIT(Invalid_Configuration_4, (GENERIC | STALLING | SIMPLE))

endmodule
