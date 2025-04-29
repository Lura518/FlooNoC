// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Raphael Roth <raroth@student.ethz.ch>

// This modul controls all the Mux / DeMux / Buffer Selectors.
// The input signals are equipped with an tag e.g. all tags needs to be reduced
// together. The Reduction can at most work on three different tags (Pipeline Depth)
// of FPU therefor the buffer depth of the control logic is 3.
// To garantee the ordering this modul implements priority scheme e.g. the first
// buffer entry has the most priority, then the second, then the third.
// It works only on elements from the lower priority if the higher ones can not schedule
// any operation.

// Limits:
// - We can not handle out-of-order
// - We can not handle multiple reduction when they do not belong to the same subset of addresses

// Possible Improvments:
// - Handle the status return of the FPU response

// Open Points:
// - Potentially there could be a AXI locked in violation when determine the current fpu req as it is not locked in - could be the case or not?
// - Possible AXI Violation on the output of the reduction as we do not lock it in! However the system design (no out-of-order / no mixed reduction) should prevent these cases!

// Disclaimer:
// Sorry for the mess in the code ;) I had to add too much configuration option(s)!

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

module floo_fp_reduction_controller #(
    parameter int unsigned  NumRoutes           = 1,
    parameter int unsigned  NumInCrossbar       = (NumRoutes),
    parameter int unsigned  NumOutCrossbar      = 4,
    parameter int unsigned  RdPartialBufferSize = 3,
    parameter int unsigned  RdPipelineDepth     = 3,
    parameter type          RdOperation_t       = logic,
    parameter type          tag_t               = logic,
    parameter type          mask_t              = logic,
    parameter type          flit_t              = logic,
    parameter type          flit_mask_tag_t     = logic,
    parameter type          idx_out_cross_t     = logic,
    parameter type          idx_in_cross_t      = logic,
    parameter type          idx_part_res_t      = logic,
    parameter bit           GENERIC             = 1'b1,
    parameter bit           SIMPLE              = 1'b0,
    parameter bit           STALLING            = 1'b0
) (
    /// Control Signals
    input  logic                                clk_i,
    input  logic                                rst_ni,
    input  logic                                flush_i,
    
    /// First Element from Fifo's
    input flit_mask_tag_t [NumRoutes-1:0]       head_fifo_data_i,
    input logic [NumRoutes-1:0]                 head_fifo_valid_i,

    /// Spyglass from the partial result buffer
    input tag_t [RdPartialBufferSize-1:0]       buf_spyglass_tag_i,
    input logic [RdPartialBufferSize-1:0]       buf_spyglass_valid_i,

    /// Provide the Metadata to the FPU request
    output tag_t                                reduction_req_tag_o,
    output RdOperation_t                        reduction_type_o,

    /// Monitor the Handshaking of the FPU to dedect backpressure
    input logic                                 reduction_req_valid_i,
    input logic                                 reduction_req_ready_i,
    input logic                                 reduction_resp_valid_i,
    input logic                                 reduction_resp_ready_i,

    /// Monitor the Handshaking on the Output of the Arbiter
    input logic                                 output_valid_i,
    input logic                                 output_ready_i,

    /// Receive the Metadata about the current FPU response
    input tag_t                                 reduction_resp_tag_i,
    input mask_t                                reduction_resp_mask_i,
    
    // Provide the header to the output of the reduction
    output flit_t                               header_output_o,
    output logic [NumRoutes-1:0]                output_dir_o,

    /// Control Output's for the Crossbar
    output idx_out_cross_t [NumInCrossbar-1:0]  ctrl_sel_in_cross_o,     // For each input select to which output it should connect to
    output idx_in_cross_t [NumOutCrossbar-1:0]  ctrl_sel_out_cross_o,    // For each output select to which input it should connect to

    /// Contol Output for the index of the partial result buffer
    output idx_part_res_t [1:0]                 ctrl_sel_part_res_buf_o, // Select output from partial result buffer for each operand

    /// Control Signal for the Muxes / DeMuxes
    output logic [1:0]                          ctrl_part_res_mux_o,
    output logic                                ctrl_output_demux_o,
    output logic                                ctrl_bypass_reduction_o
);

/* All local parameter */
localparam bit [cf_math_pkg::idx_width(NumOutCrossbar)-1:0] THREE = 3;

/* All Typedef Vars */
typedef logic [cf_math_pkg::idx_width(NumRoutes)-1:0] idx_input_t;

typedef struct packed {
    flit_t                      header;             // Why flit_t and not hdr_t? -> We also need the AXI Metadata and not only the floonoc header
    logic [NumRoutes-1:0]       final_mask;
    tag_t                       tag;
    logic [NumRoutes-1:0]       output_dir;
    logic                       f_valid;
    logic                       f_bypass;
} buffer_t;

/* Variable declaration */
buffer_t [RdPipelineDepth-1:0] buffer_q, buffer_d;

// Signal to determine if one input is not in the buffer
flit_mask_tag_t insert_data;
logic insert_valid;

// Sigal to indicate backpressure on the fpu_resp
logic backpressure_fpu_resp;

// Temp Signal to make a final assignmenton all Mux
idx_out_cross_t [NumInCrossbar-1:0]  tmp_sel_in_cross;
idx_in_cross_t [NumOutCrossbar-1:0]  tmp_sel_out_cross;
idx_part_res_t [1:0]                 tmp_sel_part_res_buf;
logic [1:0]                          tmp_part_res_mux;

// Temp Flags to indicate certain condition
logic f_ops_scheduled;
logic f_op1_found;
logic f_op2_found;
logic f_insert_data_in_buffer;

// Singal to mask all tag's
flit_mask_tag_t                     mask_insert_data;
tag_t [RdPartialBufferSize-1:0]     mask_spyglass_tag;
tag_t                               mask_reduction_resp_tag;
flit_mask_tag_t [NumRoutes-1:0]     mask_head_fifo_data;


// Variable only for the Simple Case
logic       f_simple_op1_found;
logic       f_simple_op2_found;
logic       f_simple_bypass;
idx_input_t simple_op1_idx;
idx_input_t simple_op2_idx;

flit_t simple_fifo_data_in;
flit_t simple_fifo_data_out;
logic [NumRoutes-1:0] simple_fifo_outdir_in;
logic [NumRoutes-1:0] simple_fifo_outdir_out;

/* Module Declaration */

// Generate the input data if the generic controller is enabled. The buffer contrains X amoun of entries which we have to compare the
// new data against
if(GENERIC == 1'b1) begin : gen_input_data_generic_controller

    // Search if any element on the input can be inserted into the buffer
    always_comb begin
        // Init all Vars
        insert_data = '0;
        insert_valid = 1'b0;

        // Loop over all inputs
        for(int k = 0; k < NumRoutes; k++) begin
            // Search for a ptential match in the buffer only if the incoming element is valid (And we have not yet scheduled any insertion)
            if((head_fifo_valid_i[k] == 1'b1) && (insert_valid == 1'b0)) begin
                // This input can be inserted if it is not already in the buffer
                insert_valid = 1'b1;
                insert_data = head_fifo_data_i[k];
                // Go through the hole buffer and check if the element is already inside or not
                for(int j = 0; j < RdPipelineDepth; j++) begin
                    if((head_fifo_data_i[k].tag == buffer_q[j].tag) && (buffer_q[j].f_valid == 1'b1)) begin
                        insert_valid = 1'b0;
                    end
                end
            end
        end
    end
end

// Generate the input data if the stalling controller is enabled. The buffer size is fixed to 1.
// From the concept we know that we can only one reduction therefor if the buffer entry is set to valid we know
// that the reduction is already ongoing.
if(STALLING == 1'b1) begin : gen_input_data_stalling_controller

    // Search if any element on the input can be inserted into the buffer
    always_comb begin
        // Init all Vars
        insert_data = '0;
        insert_valid = 1'b0;

        // Loop over all inputs
        for(int k = 0; k < NumRoutes; k++) begin
            // Search for a ptential match in the buffer only if the incoming element is valid (And we have not yet scheduled any insertion)
            if((head_fifo_valid_i[k] == 1'b1) && (insert_valid == 1'b0) && (buffer_q[0].f_valid == 1'b0)) begin
                // This input can be inserted if it is not already in the buffer
                insert_valid = 1'b1;
                insert_data = head_fifo_data_i[k];
            end
        end
    end
end

// Dedect if the system does apply backpressure
// The problem is that we want to fill the FPU pipeline however to prevent deadlocks we want to lock in only if the data are consumed in the next cycle
assign backpressure_fpu_resp = reduction_resp_valid_i & (~reduction_resp_ready_i);

// The entries of the buffer are prioritized by the index. If possible an operation of this entry is scheduled,
// then from the second entry etc.

// The control part can be split into 4 distinctive stages (with additional substages)
// 1. Stage: Populate the buffer with new data
// 2. Stage: Schedule when possible an reduction
// 3. Stage: Handle the FPU response accordingly
// 4. Stage: If one higher prio buffer entry is free then push the buffer by one position

if((GENERIC == 1'b1) || (STALLING == 1'b1)) begin : gen_mask_all_tag_if_necessary
    // Disable all input TAG into the modul (theo. should be removed either way, but its here for visibility / clearification)!
    always_comb begin
        mask_insert_data = insert_data;
        mask_spyglass_tag = buf_spyglass_tag_i;
        mask_reduction_resp_tag = reduction_resp_tag_i;
        mask_head_fifo_data = head_fifo_data_i;

        if(STALLING == 1'b1) begin
            mask_insert_data.tag = '0;
            mask_spyglass_tag = '0;
            mask_reduction_resp_tag = '0;
            for(int i = 0; i < RdPartialBufferSize;i++) begin
                mask_head_fifo_data[i].tag = '0;
            end
        end
    end


    always_comb begin
        // Init all Vars
        buffer_d = buffer_q;
        header_output_o = '0;
        reduction_type_o = floo_pkg::F_Add;
        output_dir_o = '0;
        reduction_req_tag_o = '0;

        // Init all default state of the muxes
        ctrl_sel_part_res_buf_o = '0;
        ctrl_part_res_mux_o = '0;

        // Set the output of all inputs of the crossbar to the dummy one
        ctrl_sel_in_cross_o = '0;
        for(int i = 0; i < NumInCrossbar;i++) begin
            ctrl_sel_in_cross_o[i] = THREE;
        end

        // Set the input for all outputs to the first one (easiest!)
        ctrl_sel_out_cross_o = '0;

        // Forward the FU Resp normally to the partial result buffer
        ctrl_output_demux_o = 1'b0;     

        // Init the bypass if the element does not need to be reduced
        ctrl_bypass_reduction_o = 1'b0;

        // Temporary Helper signal to tidy the code up
        f_ops_scheduled = 1'b0;
        f_op1_found = 1'b0;
        f_op2_found = 1'b0;

        f_insert_data_in_buffer = 1'b0;

        tmp_sel_in_cross = '0;
        tmp_sel_out_cross = '0;
        tmp_sel_part_res_buf = '0;
        tmp_part_res_mux = '0;

        // Iterate over all buffer entries - handle by prio
        for(int i = 0; i < RdPipelineDepth; i++) begin
            // Reset Var for the loop
            f_op1_found = 1'b0;
            f_op2_found = 1'b0;
            tmp_sel_in_cross = '0;
            tmp_sel_out_cross = '0;
            tmp_sel_part_res_buf = '0;
            tmp_part_res_mux = '0;

            
            // 1. Stage: Accept new Data into the Buffer if we have free space and a valid entry
            if(buffer_d[i].f_valid == 1'b0) begin
                // Check if we can insert a new element
                if((insert_valid == 1'b1) && (f_insert_data_in_buffer == 1'b0)) begin
                    // Lock in such a way that only one buffer entry can accept the data
                    f_insert_data_in_buffer = 1'b1;

                    // Insert the data into the selected entry
                    buffer_d[i].header = mask_insert_data.flit;
                    buffer_d[i].final_mask = mask_insert_data.mask;
                    buffer_d[i].output_dir = mask_insert_data.output_dir;
                    buffer_d[i].f_valid = 1'b1;
                    buffer_d[i].f_bypass = $onehot(mask_insert_data.mask); // Dedect if no reduction is necessary
                    buffer_d[i].tag = mask_insert_data.tag;
                end
            end

            // 2.1 Stage: Try to schedula an operation from the partial result buffer When:
            //      - Entry in Buffer is valid
            //      - No higher prioritized buffer already has scheduled an operation
            //      - The reduction contains more than one element
            if( (buffer_d[i].f_valid == 1'b1) && 
                (f_ops_scheduled == 1'b0) && 
                (buffer_d[i].f_bypass == 0)) begin

                // First iterate over the partial result buffer
                for(int j = 0; j < RdPartialBufferSize;j++) begin
                    if((mask_spyglass_tag[j] == buffer_d[i].tag) && (buf_spyglass_valid_i[j] == 1'b1)) begin
                        if(f_op1_found == 1'b0) begin
                            tmp_sel_part_res_buf[0] = j;
                            tmp_part_res_mux[0] = 1'b1;   // Switch the Mux0 from the input to the partial result
                            f_op1_found = 1'b1;
                        end else if(f_op2_found == 1'b0) begin
                            tmp_sel_part_res_buf[1] = j;
                            tmp_part_res_mux[1] = 1'b1;   // Switch the Mux0 from the input to the partial result
                            f_op2_found = 1'b1;
                        end
                    end
                end
            end

            // 2.2 Stage: Try to schedula an operation from the inputs when:
            //      - Entry in Buffer is valid
            //      - No higher prioritized buffer already has scheduled an operation
            //      - The reduction contains more than one element
            //      - No backpressure is applied to the FPU response or f_op1_found is 1 and f_op2_found is 0 (otherwise deadlock potential!)
            if( (buffer_d[i].f_valid == 1'b1) && 
                (f_ops_scheduled == 1'b0) && 
                (buffer_d[i].f_bypass == 0) && 
                ((backpressure_fpu_resp == 1'b0) || ((f_op1_found == 1'b1) && (f_op2_found == 1'b0)))) begin
                    
                // Iterate over all inputs
                for(int j = 0; j < NumRoutes;j++) begin
                    if((mask_head_fifo_data[j].tag == buffer_d[i].tag) && (head_fifo_valid_i[j] == 1'b1)) begin
                        if(f_op1_found == 1'b0) begin
                            tmp_sel_in_cross[j] = 0;
                            tmp_sel_out_cross[0] = j;
                            f_op1_found = 1'b1;
                        end else if(f_op2_found == 1'b0) begin
                            tmp_sel_in_cross[j] = 1;
                            tmp_sel_out_cross[1] = j;
                            f_op2_found = 1'b1;
                        end
                    end
                end
            end

            // 2.3 Stage: If we have two possible operands then schedule the operation
            if((f_op1_found == 1'b1) && (f_op2_found == 1'b1)) begin : fix_ops_sched    // Fix the found schedulable operation
                f_ops_scheduled = 1'b1;

                // Set all Control Signal to generate the data path
                ctrl_sel_in_cross_o = tmp_sel_in_cross;
                ctrl_sel_out_cross_o = tmp_sel_out_cross;
                ctrl_part_res_mux_o = tmp_part_res_mux;
                ctrl_sel_part_res_buf_o = tmp_sel_part_res_buf;

                // Set the Metadata for the request here
                reduction_req_tag_o = buffer_d[i].tag;

                // Generate the Metadata
                reduction_type_o = buffer_d[i].header.hdr.reduction_op;
            end

            // 3.1 Stage: retire an element from the input when:
            // - Entry in Buffer is valid
            // - No reduction is necessary
            // - Only for the most prio. buffer entry
            if( (buffer_d[i].f_valid == 1'b1) && 
                (buffer_d[i].f_bypass == 1'b1) &&
                (i == 0)) begin

                // Iterate over all fifo heads to find the corresponding element
                for(int j = 0; j < NumRoutes;j++) begin
                    if((mask_head_fifo_data[j].tag == buffer_d[i].tag) && (head_fifo_valid_i[j] == 1'b1)) begin
                        ctrl_sel_in_cross_o[j] = 2;
                        ctrl_sel_out_cross_o[2] = j;
                        ctrl_bypass_reduction_o = 1'b1;
                    end
                end
            end

            // 3.2 Stage: retire an element from the fpu resp when:
            // - Entry in Buffer is valid
            // - Final Mask matches with the given one
            // - The Tag matches
            // - The FPU response is valid
            // - Only for the most prio. buffer entry
            if( (buffer_d[i].f_valid == 1'b1) && 
                (buffer_d[i].final_mask == reduction_resp_mask_i) &&
                (buffer_d[i].tag == mask_reduction_resp_tag) &&
                (reduction_resp_valid_i == 1'b1) && 
                (i == 0)) begin

                // Set the ouput DeMux to the output
                ctrl_output_demux_o = 1'b1;

                // Set the header & the output direction to the Output
                header_output_o = buffer_d[i].header;
                output_dir_o = buffer_d[i].output_dir;
            end

            // 3.4 Stage: Reset the buffer entry if we have a valid handshake on the output
            if((output_valid_i == 1'b1) && (output_ready_i == 1'b1)) begin
                buffer_d[i].f_valid = 1'b0;
            end

            // 4 Stage: Copy the data to a higher prio slot if it is free and we are valid
            // (Generic case already handled by having i only 0!)
            if(i != 0)begin
                if((buffer_d[i-1].f_valid == 1'b0) && (buffer_d[i].f_valid == 1'b1)) begin
                    buffer_d[i-1] = buffer_d[i];
                    buffer_d[i-1].f_valid = 1'b1;
                    buffer_d[i].f_valid = 1'b0;
                end
            end
        end
    end
end

// Simple controller which can either reduce two operands or forward a single one.
// Any other operation is invalid and leads to undefined behaviour
if(SIMPLE == 1'b1) begin : gen_simple_controller

    always_comb begin
        // Init all Vars here
        buffer_d = '0;
        reduction_req_tag_o = '0;
        header_output_o = '0;
        output_dir_o = '0;

        // Init all default state of the muxes
        ctrl_sel_part_res_buf_o = '0;
        ctrl_part_res_mux_o = '0;

        // Set the output of all inputs of the crossbar to the dummy one
        ctrl_sel_in_cross_o = '0;
        for(int i = 0; i < NumInCrossbar;i++) begin
            ctrl_sel_in_cross_o[i] = THREE;
        end

        // Set the input for all outputs to the first one (easiest!)
        ctrl_sel_out_cross_o = '0;

        // Forward the FU Resp normally to the partial result buffer
        ctrl_output_demux_o = 1'b0;     

        // Init the bypass if the element does not need to be reduced
        ctrl_bypass_reduction_o = 1'b0;

        // Default values for the fifo
        simple_fifo_data_in = '0;
        simple_fifo_outdir_in = '0;

        // Temp signal
        f_simple_op1_found = 1'b0;
        f_simple_op2_found = 1'b0;
        simple_op1_idx = '0;
        simple_op2_idx = '0;

        // Iterate over all inputs to found two operands
        for(int i = 0; i < NumRoutes; i++) begin
            // Find the first operand
            if((head_fifo_valid_i[i] == 1'b1) && (f_simple_op1_found == 1'b0)) begin
                f_simple_op1_found = 1'b1;
                f_simple_bypass = $onehot(head_fifo_data_i[i].mask);
                simple_op1_idx = i;
            end
            // Find the second operand
            if((head_fifo_valid_i[i] == 1'b1) && (f_simple_op1_found == 1'b1) && (f_simple_op2_found == 1'b0)) begin
                f_simple_op2_found = 1'b1;
                simple_op2_idx = i;
            end
        end

        // Evaluate if we found a direct bypass
        if((f_simple_op1_found == 1'b1) && (f_simple_bypass == 1'b1) && (f_simple_op2_found == 1'b0)) begin
            ctrl_sel_in_cross_o[simple_op1_idx] = 2;
            ctrl_sel_out_cross_o[2] = simple_op1_idx;
            ctrl_bypass_reduction_o = 1'b1;
            header_output_o = head_fifo_data_i[simple_op1_idx].flit;
            output_dir_o = head_fifo_data_i[simple_op1_idx].output_dir;
        end

        // Evaluate if we found a reduction
        if((f_simple_op1_found == 1'b1) && (f_simple_bypass == 1'b0) && (f_simple_op2_found == 1'b1)) begin
            ctrl_sel_in_cross_o[simple_op1_idx] = 0;
            ctrl_sel_out_cross_o[0] = simple_op1_idx;
            ctrl_sel_in_cross_o[simple_op2_idx] = 1;
            ctrl_sel_out_cross_o[1] = simple_op2_idx;

            // Push the header to the FIFO
            simple_fifo_data_in = head_fifo_data_i[simple_op1_idx].flit;
            simple_fifo_outdir_in = head_fifo_data_i[simple_op1_idx].output_dir;

            // Generate the Metadata
            reduction_type_o = buffer_d[simple_op1_idx].header.hdr.reduction_op;
        end

        // Set the header for the output of the reduction here
        if(reduction_resp_valid_i == 1'b1) begin
            ctrl_bypass_reduction_o = 1'b0;
            header_output_o = simple_fifo_data_out;
            output_dir_o = simple_fifo_outdir_out;
        end
    end

    // Fifo to store the header of the element during the FPU reduction
    fifo_v3 #(
        .FALL_THROUGH     (1'b0),
        .dtype            (flit_t),
        .DEPTH            (RdPipelineDepth+1)
    ) i_fifo_mask_parallel_fpu (
        .clk_i            (clk_i),
        .rst_ni           (rst_ni),
        .flush_i          (flush_i),
        .testmode_i       (1'b0),
        .full_o           (),
        .empty_o          (),
        .usage_o          (),
        .data_i           (simple_fifo_data_in), // Or Connect both involved Mask
        .push_i           (reduction_req_valid_i & reduction_req_ready_i),  // push mask on active fpu req hs
        .data_o           (simple_fifo_data_out),
        .pop_i            (reduction_resp_valid_i & reduction_resp_ready_i) // pop mask on active fpu resp hs
    );

    // Fifo to store the output direction of the element during the FPU reduction
    fifo_v3 #(
        .FALL_THROUGH     (1'b0),
        .DATA_WIDTH       (NumRoutes),
        .DEPTH            (RdPipelineDepth+1)
    ) i_fifo_outdir_parallel_fpu (
        .clk_i            (clk_i),
        .rst_ni           (rst_ni),
        .flush_i          (flush_i),
        .testmode_i       (1'b0),
        .full_o           (),
        .empty_o          (),
        .usage_o          (),
        .data_i           (simple_fifo_outdir_in), // Or Connect both involved Mask
        .push_i           (reduction_req_valid_i & reduction_req_ready_i),  // push mask on active fpu req hs
        .data_o           (simple_fifo_outdir_out),
        .pop_i            (reduction_resp_valid_i & reduction_resp_ready_i) // pop mask on active fpu resp hs
    );

end

// Store the data in the buffer
`FF(buffer_q, buffer_d, '0, clk_i, rst_ni)

/* ASSERTION Checks */
// We can only run GENERIC or SIMPLE or STALLING
`ASSERT_INIT(Invalid_Configuration_1, !(GENERIC & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_2, !(STALLING & SIMPLE))
`ASSERT_INIT(Invalid_Configuration_3, !(GENERIC & STALLING))
`ASSERT_INIT(Invalid_Configuration_4, (GENERIC | STALLING | SIMPLE))
//`ASSERT_INIT(Invalid_Configuration_5, !(STALLING && (RdPipelineDepth != 1))) // Why do we need this assertion?

endmodule