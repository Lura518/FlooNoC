// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Raphael Roth <raroth@student.ethz.ch>

// This module allows to implement a reduction HW to simulate a reduction operation.
// Simple Testbench implementation!

// Open Points:

`include "common_cells/assertions.svh"

// This Wrapper allows to wrap 8x 64 Bit (512 Bits) in parallel
module floo_reduction_wrapper import floo_pkg::*; #(
  parameter type         RdData_t               = logic,
  parameter int unsigned RdElements             = 8,
  parameter bit          FPU_ACTIVE             = 1'b0,
  parameter bit          ALU_ACTIVE             = 1'b0,
  parameter bit          DEBUG_PRINT_TRACE      = 1'b0
) (
  input  logic                          clk_i,
  input  logic                          rst_ni,
  input  logic                          flush_i,
  /// IF towards external FPU
  input  RdData_t                       reduction_req_op1_i,
  input  RdData_t                       reduction_req_op2_i,
  input  reduction_op_e                 reduction_req_type_i,
  input  logic                          reduction_req_valid_i,
  output logic                          reduction_req_ready_o,
  /// IF from external FPU
  output RdData_t                       reduction_resp_data_o,
  output logic                          reduction_resp_valid_o,
  input  logic                          reduction_resp_ready_i
);

  // Parameter
  localparam int unsigned FLEN = 64;

  // Variable
  logic comp_req_valid[RdElements];
  logic comp_req_ready[RdElements];
  logic comp_resp_valid[RdElements];
  logic comp_resp_ready[RdElements];

  // Fork the hadshaking
  stream_fork #(
    .N_OUP         (RdElements)
  ) i_dca_fork_fpu (
    .clk_i         (clk_i),
    .rst_ni        (rst_ni),
    .valid_i       (reduction_req_valid_i),
    .ready_o       (reduction_req_ready_o),
    .valid_o       (comp_req_valid),
    .ready_i       (comp_req_ready)
  );

  // Implement FPU(s)
  for (genvar i = 0; i < RdElements; i++) begin : gen_fpu_metadata

    // Generate the FPU
    if(FPU_ACTIVE == 1'b1) begin
      floo_reduction_fpu #(
        .ID                   (i),
        .DEBUG_PRINT_TRACE    (DEBUG_PRINT_TRACE)
      ) i_fpu (
        .clk_i                (clk_i),
        .rst_ni               (rst_ni),
        .flush_i              (flush_i),
        .fpu_req_op1_i        (reduction_req_op1_i[(FLEN*(i+1))-1:FLEN*i]),
        .fpu_req_op2_i        (reduction_req_op2_i[(FLEN*(i+1))-1:FLEN*i]),
        .fpu_req_type_i       (reduction_req_type_i),
        .fpu_req_valid_i      (comp_req_valid[i]),
        .fpu_req_ready_o      (comp_req_ready[i]),
        .fpu_resp_data_o      (reduction_resp_data_o[(FLEN*(i+1)-1):FLEN*i]),
        .fpu_resp_valid_o     (comp_resp_valid[i]),
        .fpu_resp_ready_i     (comp_resp_ready[i])
      );
    end

    // Generate the ALU
    if(ALU_ACTIVE == 1'b1) begin
      floo_reduction_alu #(
        .ID                   (i),
        .DEBUG_PRINT_TRACE    (DEBUG_PRINT_TRACE)
      ) (
        .clk_i                (clk_i),
        .rst_ni               (rst_ni),
        .flush_i              (flush_i),
        .alu_req_op1_i        (reduction_req_op1_i[(FLEN*(i+1))-1:FLEN*i]),
        .alu_req_op2_i        (reduction_req_op2_i[(FLEN*(i+1))-1:FLEN*i]),
        .alu_req_type_i       (reduction_req_type_i),
        .alu_req_valid_i      (comp_req_valid[i]),
        .alu_req_ready_o      (comp_req_ready[i]),
        .alu_resp_data_o      (reduction_resp_data_o[(FLEN*(i+1)-1):FLEN*i]),
        .alu_resp_valid_o     (comp_resp_valid[i]),
        .alu_resp_ready_i     (comp_resp_ready[i])
      );
    end

  end

  // Join all the signal together
  stream_join #(
    .N_INP           (RdElements)
  ) i_dca_join_fpu (
    .inp_valid_i     (comp_resp_valid),
    .inp_ready_o     (comp_resp_ready),
    .oup_valid_o     (reduction_resp_valid_o),
    .oup_ready_i     (reduction_resp_ready_i)
  );

  // Sanity Check
  `ASSERT_INIT(Invalid_ALU_or_FPU, ((FPU_ACTIVE ^ ALU_ACTIVE) == 1'b0))
  `ASSERT_INIT(Invalid_Config, ($bits(RdData_t) != (RdElements*FLEN)))

endmodule

package alu_pkg;
  // STRONGLY Inspired by the fpnew from openhw group!

  // ---------
  // INT TYPES
  // ---------
  // | Enumerator | Width  |
  // |:----------:|-------:|
  // | INT8       |  8 bit |
  // | UINT8      |  8 bit |
  // | INT16      | 16 bit |
  // | UINT16     | 16 bit |
  // | INT32      | 32 bit |
  // | UINT32     | 32 bit |
  // | INT64      | 64 bit |
  // | UINT64     | 64 bit |
  // *NOTE:* Add new formats only at the end of the enumeration for backwards compatibilty!
  localparam int unsigned NUM_INT_FORMATS = 8;
  localparam int unsigned INT_FORMAT_BITS = $clog2(NUM_INT_FORMATS);

  // Int formats (Uint required for differentation between signed / unsigned min max)
  typedef enum logic [INT_FORMAT_BITS-1:0] {
    INT8,
    UINT8,
    INT16,
    UINT16,
    INT32,
    UINT32,
    INT64,
    UINT64
    // add new formats here
  } alu_int_format_e;

    // Returns the width of an INT format by index
  function automatic int unsigned int_width(alu_int_format_e ifmt);
    unique case (ifmt)
      INT8:  return 8;
      UINT8:  return 8;
      INT16: return 16;
      UINT16: return 16;
      INT32: return 32;
      UINT32: return 32;
      INT64: return 64;
      UINT64: return 64;
      default: begin
        // pragma translate_off
        $fatal(1, "Invalid INT format supplied");
        // pragma translate_on
        // just return any integer to avoid any latches
        // hopefully this error is caught by simulation
        return INT8;
      end
    endcase
  endfunction

  // --------------
  // ALU OPERATIONS
  // --------------
  localparam int unsigned NUM_INT_OPERATION = 4;
  localparam int unsigned INT_OPERATION_BITS = $clog2(NUM_INT_OPERATION);

  // Int Operation
  typedef enum logic [INT_OPERATION_BITS-1:0] { 
    ADD,
    MUL,
    MIN,
    MAX
  } alu_operation_e;

  // --------------
  // STATUS
  // --------------
  typedef struct packed {
    logic is_zero;
  } alu_status_t;

endpackage

module floo_reduction_alu import floo_pkg::*; #(
  parameter int unsigned ID = 0,
  parameter bit          DEBUG_PRINT_TRACE      = 1'b0
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              flush_i,
  /// IF towards external FPU
  input  logic[63:0]        alu_req_op1_i,
  input  logic[63:0]        alu_req_op2_i,
  input  reduction_op_e     alu_req_type_i,
  input  logic              alu_req_valid_i,
  output logic              alu_req_ready_o,
  /// IF from external ALU
  output logic[63:0]        alu_resp_data_o,
  output logic              alu_resp_valid_o,
  input  logic              alu_resp_ready_i
);

  /* All local parameter */

  /* All Typedef Vars */
  typedef struct packed {
    logic [1:0][63:0]         operands;
    alu_pkg::alu_operation_e  op;
    alu_pkg::alu_int_format_e fmt;
    logic                     vectorial_op;
  } alu_in_t;

  typedef struct packed {
    logic [63:0] result;
  } alu_out_t;

  /* Variable declaration */
  alu_in_t alu_in;
  alu_out_t alu_out;

  /* Module Declaration */

  // Parse the ALU request
  always_comb begin
    // Init default values
    alu_in = '0;

    // Set default Values
    alu_in.vectorial_op = 1'b0;
    alu_in.operands[0] = alu_req_op1_i;
    alu_in.operands[1] = alu_req_op2_i;

    // Define the operation we want to execute on the FPU
    unique casez (alu_req_type_i)
      (floo_pkg::A_Add) : begin
        alu_in.op = alu_pkg::ADD;
        alu_in.fmt = alu_pkg::INT32;
      end
      (floo_pkg::A_Mul) : begin
        alu_in.op = alu_pkg::MUL;
        alu_in.fmt = alu_pkg::INT32;
      end                
      (floo_pkg::A_Min_S) : begin
        alu_in.op = alu_pkg::MIN;
        alu_in.fmt = alu_pkg::INT32;
      end
      (floo_pkg::A_Min_U) : begin
        alu_in.op = alu_pkg::MIN;
        alu_in.fmt = alu_pkg::UINT32;
      end
      (floo_pkg::A_Max_S) : begin
        alu_in.op = alu_pkg::MAX;
        alu_in.fmt = alu_pkg::INT32;
      end
      (floo_pkg::A_Max_U) : begin
        alu_in.op = alu_pkg::MAX;
        alu_in.fmt = alu_pkg::UINT32;
      end
      default : begin
        alu_in.op = alu_pkg::ADD;
        alu_in.fmt = alu_pkg::INT32;
      end
    endcase
  end

  // Instanciate the ALU
  floo_alu_top #(
    .tag_t                (logic),
    .CutOutput            (1'b1),
    .CutInput             (1'b0)
  ) i_alu (
    .clk_i                (clk_i),
    .rst_ni               (rst_ni),
    .flush_i              (flush_i),
    .operands_i           (alu_in.operands),
    .op_i                 (alu_in.op),
    .fmt_i                (alu_in.fmt),
    .vector_mode_i        (alu_in.vectorial_op),
    .tag_i                (1'b0),
    .in_valid_i           (alu_req_valid_i),
    .in_ready_o           (alu_req_ready_o),
    .result_o             (alu_out.result),
    .status_o             (),
    .tag_o                (),
    .out_valid_o          (alu_resp_valid_o),
    .out_ready_i          (alu_resp_ready_i)
  );

  // Print the Status info
  if(DEBUG_PRINT_TRACE) begin
    int cnt_in;
    int cnt_out;
    initial begin
      cnt_in = 0;
      cnt_out = 0;
      while(1) begin
        @(posedge clk_i);
        // Print the incoming operation
        if((alu_req_valid_i == 1'b1) && (alu_req_ready_o == 1'b1)) begin
          $display($time, "[tb %d - %d] ALU Ops: [%d, %d] ALU Op: %s", ID, cnt_in, alu_req_op1_i, alu_req_op2_i, genOpAlu(alu_req_type_i));
          cnt_in = cnt_in + 1;
        end

        // Print Result / Status of alu
        if((alu_resp_valid_o == 1'b1) && (alu_resp_ready_i == 1'b1)) begin
          $display($time, "[tb %d - %d] ALU Result: %d", ID, cnt_out, alu_out.result);
          cnt_out = cnt_out + 1;
        end
      end
    end

    function string genOpAlu (reduction_op_e type_reduction);
      string retVal;
      retVal = "";
      unique casez (type_reduction)
        (floo_pkg::A_Add) : begin
          retVal = "Atomic Add";
        end
        (floo_pkg::A_Mul) : begin
          retVal = "Atmoic Mul";
        end                
        (floo_pkg::A_Max_S) : begin
          retVal = "Atomic Max S";
        end
        (floo_pkg::A_Max_U) : begin
          retVal = "Atomic Max U";
        end
        (floo_pkg::A_Min_S) : begin
          retVal = "Atomic Min S";
        end
        (floo_pkg::A_Min_U) : begin
          retVal = "Atomic Min U";
        end
      endcase
      return retVal;
    endfunction
  end

endmodule



// Floating Point Reduction
module floo_reduction_fpu import floo_pkg::*; #(
  parameter int unsigned ID = 0,
  parameter bit          DEBUG_PRINT_TRACE      = 1'b0
) (
  input  logic              clk_i,
  input  logic              rst_ni,
  input  logic              flush_i,
  /// IF towards external FPU
  input  logic[63:0]        fpu_req_op1_i,
  input  logic[63:0]        fpu_req_op2_i,
  input  reduction_op_e     fpu_req_type_i,
  input  logic              fpu_req_valid_i,
  output logic              fpu_req_ready_o,
  /// IF from external FPU
  output logic[63:0]        fpu_resp_data_o,
  output logic              fpu_resp_valid_o,
  input  logic              fpu_resp_ready_i
);

  /* All local parameter */

  // FPU Configuration
  localparam fpnew_pkg::fpu_features_t FPUFeatures = '{
    Width:             64,
    EnableVectors:     1'b1,
    EnableNanBox:      1'b1,
    FpFmtMask:         {1'b1, 1'b1, 1'b1, 1'b1, 1'b1, 1'b1}, //{RVF, RVD, XF16, XF8, XF16ALT, XF8ALT},
    IntFmtMask:        {1'b1, 1'b1, 1'b1, 1'b1} //{XFVEC && (XF8 || XF8ALT), XFVEC && (XF16 || XF16ALT), 1'b1, 1'b0}
  };

  // FPU Implementation copied from the generated code (messy as fuck)
  localparam fpnew_pkg::fpu_implementation_t FPUImplementation [1] = '{
      '{
          PipeRegs: 
                    '{'{2, 3, 1, 1, 1, 1},   // FMA Block
                      '{1, 1, 1, 1, 1, 1},   // DIVSQRT
                      '{1, 1, 1, 1, 1, 1},   // NONCOMP
                      '{2, 2, 2, 2, 2, 2},   // CONV
                      '{3, 3, 3, 3, 3, 3}    // DOTP
                      },
          UnitTypes: '{'{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED},  // FMA
                      '{fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED, fpnew_pkg::DISABLED}, // DIVSQRT
                      '{fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL, fpnew_pkg::PARALLEL}, // NONCOMP
                      '{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED},   // CONV
                      '{fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED, fpnew_pkg::MERGED}},  // DOTP
          PipeConfig: fpnew_pkg::BEFORE
      }
    };

  /* All Typedef Vars */
  typedef struct packed {
    logic [2:0][63:0]        operands;
    fpnew_pkg::roundmode_e   rnd_mode;
    fpnew_pkg::operation_e   op;
    logic                    op_mod;
    fpnew_pkg::fp_format_e   src_fmt;
    fpnew_pkg::fp_format_e   dst_fmt;
    fpnew_pkg::int_format_e  int_fmt;
    logic                    vectorial_op;
  } fpu_in_t;

  typedef struct packed {
    logic [63:0] result;
    logic [4:0]      status;
  } fpu_out_t;

  /* Variable declaration */
  fpu_in_t fpu_in;
  fpu_out_t fpu_out;

  /* Module Declaration */

  // Parse the FPU Request
  always_comb begin
    // Init default values
    fpu_in = '0;

    // Set default Values
    fpu_in.src_fmt = fpnew_pkg::FP64;
    fpu_in.dst_fmt = fpnew_pkg::FP64;
    fpu_in.int_fmt = fpnew_pkg::INT64;
    fpu_in.vectorial_op = 1'b0;
    fpu_in.op_mod = 1'b0;
    fpu_in.rnd_mode = fpnew_pkg::RNE;
    fpu_in.op = fpnew_pkg::ADD;

    // Define the operation we want to execute on the FPU
    unique casez (fpu_req_type_i)
      (floo_pkg::F_Add) : begin
        fpu_in.op = fpnew_pkg::ADD;
        fpu_in.operands[0] = '0;
        fpu_in.operands[1] = fpu_req_op1_i;
        fpu_in.operands[2] = fpu_req_op2_i;
      end
      (floo_pkg::F_Mul) : begin
        fpu_in.op = fpnew_pkg::MUL;
        fpu_in.operands[0] = fpu_req_op1_i;
        fpu_in.operands[1] = fpu_req_op2_i;
        fpu_in.operands[2] = '0;
      end                
      (floo_pkg::F_Max) : begin
        fpu_in.op = fpnew_pkg::MINMAX;
        fpu_in.rnd_mode = fpnew_pkg::RNE;
        fpu_in.operands[0] = fpu_req_op1_i;
        fpu_in.operands[1] = fpu_req_op2_i;
        fpu_in.operands[2] = '0;
      end
      (floo_pkg::F_Min) : begin
        fpu_in.op = fpnew_pkg::MINMAX;
        fpu_in.rnd_mode = fpnew_pkg::RTZ;
        fpu_in.operands[0] = fpu_req_op1_i;
        fpu_in.operands[1] = fpu_req_op2_i;
        fpu_in.operands[2] = '0;
      end
      default : begin
        fpu_in.op = fpnew_pkg::ADD;
        fpu_in.operands[0] = '0;
        fpu_in.operands[1] = '0;
        fpu_in.operands[2] = '0;
      end
    endcase
  end

  // Instanciate the FPU as single element
  fpnew_top #(
    // FPU configuration
    .Features                    (FPUFeatures),
    .Implementation              (FPUImplementation[0]),
    .TagType                     (logic),
    .CompressedVecCmpResult      (1),
    .StochasticRndImplementation (fpnew_pkg::DEFAULT_RSR)
  ) i_fpu (
    .clk_i            (clk_i),
    .rst_ni           (rst_ni),
    .hart_id_i        ('0),
    .operands_i       (fpu_in.operands),
    .rnd_mode_i       (fpu_in.rnd_mode),
    .op_i             (fpu_in.op),
    .op_mod_i         (fpu_in.op_mod),
    .src_fmt_i        (fpu_in.src_fmt),
    .dst_fmt_i        (fpu_in.dst_fmt),
    .int_fmt_i        (fpu_in.int_fmt),
    .vectorial_op_i   (fpu_in.vectorial_op),
    .tag_i            ('0),
    .simd_mask_i      ('1),
    .in_valid_i       (fpu_req_valid_i),
    .in_ready_o       (fpu_req_ready_o),
    .flush_i          (flush_i),
    .result_o         (fpu_out.result),
    .status_o         (fpu_out.status),
    .tag_o            (),
    .out_valid_o      (fpu_resp_valid_o),
    .out_ready_i      (fpu_resp_ready_i),
    .busy_o           ()
  );

  // Provide the data to the output
  assign fpu_resp_data_o = fpu_out.result;

  // Print the Status info
  if(DEBUG_PRINT_TRACE) begin
    int cnt_in;
    int cnt_out;
    initial begin
      cnt_in = 0;
      cnt_out = 0;
      while(1) begin
        @(posedge clk_i);
        // Print the incoming operation
        if((fpu_req_valid_i == 1'b1) && (fpu_req_ready_o == 1'b1)) begin
          $display($time, "[tb %d - %d] FPU Ops: [%f, %f] FPU Op: %s", ID, cnt_in, fpu_req_op1_i, fpu_req_op2_i, genOp(fpu_req_type_i));
          cnt_in = cnt_in + 1;
        end

        // Print Result / Status of FPU
        if((fpu_resp_valid_o == 1'b1) && (fpu_resp_ready_i == 1'b1)) begin
          $display($time, "[tb %d - %d] FPU Result: %f FPU Status: %s", ID, cnt_out, fpu_out.result, genBitRep(fpu_out.status));
          cnt_out = cnt_out + 1;
        end
      end
    end

    // Helper Function to generate Bitstring
    function string genBitRep (logic [4:0] in);
      string retVal;
      retVal = "B";
      for(int i = 0; i < 5; i++) begin
          if(in[4-i] == 1'b1) begin
              retVal = {retVal, "1"};
          end else begin
              retVal = {retVal, "0"};
          end
      end
      return retVal;
    endfunction

    function string genOp (reduction_op_e type_reduction);
      string retVal;
      retVal = "";
      unique casez (type_reduction)
        (floo_pkg::F_Add) : begin
          retVal = "FAdd";
        end
        (floo_pkg::F_Mul) : begin
          retVal = "FMul";
        end                
        (floo_pkg::F_Max) : begin
          retVal = "FMax";
        end
        (floo_pkg::F_Min) : begin
          retVal = "FMin";
        end
      endcase
      return retVal;
    endfunction
  end

endmodule

module floo_alu_top #(
  parameter type          tag_t = logic,
  parameter bit           CutOutput = 1'b1,
  parameter bit           CutInput = 1'b1,
  // Do not change
  localparam int unsigned WIDTH = 64,
  localparam int unsigned NUM_OPERANDS = 2
) (
  input logic                                 clk_i,
  input logic                                 rst_ni,
  input logic                                 flush_i,
  /// Input Signal
  input logic [NUM_OPERANDS-1:0][WIDTH-1:0]   operands_i,
  input alu_pkg::alu_operation_e              op_i,
  input alu_pkg::alu_int_format_e             fmt_i,
  input logic                                 vector_mode_i,
  input tag_t                                 tag_i,
  input logic                                 in_valid_i,
  output logic                                in_ready_o,
  /// Output Signal
  output logic [WIDTH-1:0]                    result_o,
  output alu_pkg::alu_status_t                status_o,
  output tag_t                                tag_o,
  output logic                                out_valid_o,
  input  logic                                out_ready_i
);

// Implement a simple ALU
// Open Points: Vector Mode is currently not supported!

/* All local parameter */
typedef struct packed {
  logic [NUM_OPERANDS-1:0][WIDTH-1:0] operands;
  alu_pkg::alu_operation_e op;
  alu_pkg::alu_int_format_e fmt;
  logic vector_mode;
  tag_t tag;
} cut_input_t;

typedef struct packed {
  logic [WIDTH-1:0] result;
  alu_pkg::alu_status_t status;
  tag_t tag;
} cut_output_t;

/* All Typedef Vars */

/* Variable declaration */

// Delayed input vars
logic [NUM_OPERANDS-1:0][WIDTH-1:0]   operands_q;
alu_pkg::alu_operation_e op_q;
alu_pkg::alu_int_format_e fmt_q;
logic vector_mode_q;
tag_t tag_q;
logic in_valid_q;
logic in_ready_q;

// output var infront of cut
logic [WIDTH-1:0] result_d;
alu_pkg::alu_status_t status_d;
tag_t tag_d;
logic out_valid_d;
logic out_ready_d;

// trunc'ed signal to support only 32 Bit signal
logic [NUM_OPERANDS-1:0][31:0]        operands_32;
logic [31:0] res_32;
logic [31:0] adder_res_32;
logic [31:0] mul_res_32;
logic [31:0] min_res_32;
logic [31:0] max_res_32;

/* Module Declaration */

// Input Cut to split the ALU from the rest of the system
if (CutInput == 1'b1) begin
  // introduce cut at input of ALU
  spill_register_flushable #(
    .T                  (cut_input_t),
    .Bypass             (1'b0)
  ) i_output_cut (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .valid_i            (in_valid_i),
    .flush_i            (flush_i),
    .ready_o            (in_ready_o),
    .data_i             ({operands_i, op_i, fmt_i, vector_mode_i, tag_i}),
    .valid_o            (in_valid_q),
    .ready_i            (in_ready_q),
    .data_o             ({operands_q, op_q, fmt_q, vector_mode_q, tag_q})
  );
end else begin
  assign operands_q = operands_i;
  assign op_q = op_i;
  assign fmt_q = fmt_i;
  assign vector_mode_q = vector_mode_i;
  assign tag_q = tag_i;
end

// Implement ALU here
// Parse both operands to 32 Bit
for (genvar i = 0; i < NUM_OPERANDS;i++) begin
  assign operands_32[i] = operands_q[i][31:0];
end

// Adder Path
assign adder_res_32 = operands_32[1] + operands_32[0];

// Multiplier Path
always_comb begin
  mul_res_32 = '0;
  for (int i = 0; i < 32; i++) begin
    mul_res_32 = (|((operands_32[0] >> i) & 1)) ? mul_res_32 ^ (operands_32[1] << i) : mul_res_32;
  end
end

// Min / Max Path
always_comb begin : gen_minmax
  logic sign;

  max_res_32 = '0;
  min_res_32 = '0;
  sign = 1'b0;

  // Determint if we require sign > When we extend the signal by 1 bit then we can use the signed hw
  // for both the signed and unsigned case.
  if(fmt_q == alu_pkg::INT32) begin
    sign = 1'b1;
  end

  // Calc the min / max signal in the same case
  if($signed({sgn & operands_32[0][31], operands_32[0]}) > $signed({sgn & operands_32[1][31], operands_32[1]})) begin
    max_res_32 = operands_32[0];
    min_res_32 = operands_32[1];
  end else begin
    max_res_32 = operands_32[1];
    min_res_32 = operands_32[0];
  end
end

// Mux the result together
always_comb begin : result_mux
  res_32 = '0;
  unique case (op_i)
    alu_pkg::ADD:   res_32 = adder_res_32;
    alu_pkg::MUL:   res_32 = mul_res_32;
    alu_pkg::MIN:   res_32 = min_res_32;
    alu_pkg::MAX:   res_32 = max_res_32;
    default:        res_32 = '0;
  endcase
end

// Sign extend the 32 Bit result
assign result_d = {{32{res_32[31]}},res_32};

// Bypass tag & handshake
assign tag_d = tag_q;
assign out_valid_d = in_valid_q;
assign in_ready_q = out_ready_d;
assign status_d.is_zero = ~ (|res_32); // Or Connect all signal and invert to determin if we have a 0 signal

if (CutOutput == 1'b1) begin
  // introduce cut at input of ALU
  spill_register_flushable #(
    .T                  (cut_output_t),
    .Bypass             (1'b0)
  ) i_output_cut (
    .clk_i              (clk_i),
    .rst_ni             (rst_ni),
    .valid_i            (out_valid_d),
    .flush_i            (flush_i),
    .ready_o            (out_ready_d),
    .data_i             ({result_d, status_d, tag_d}),
    .valid_o            (out_valid_o),
    .ready_i            (out_ready_i),
    .data_o             ({result_o, status_o, tag_o})
  );
end else begin
  assign result_o = result_d;
  assign status_o = status_d;
  assign tag_o = tag_d;
end

/* Assertions for the module */

// Currently we only support 32Bit operations! Could be extended in the future
`ASSERT(Invalid_Input, (fmt_i != alu_pkg::INT32) && (fmt_i != alu_pkg::UINT32))
`ASSERT(Invalid_Vector_Ops, (vector_mode_i != 1'b0))

endmodule




