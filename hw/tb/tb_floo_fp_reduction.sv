// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Tim Fischer <fischeti@iis.ee.ethz.ch>
// Raphael Roth  <raroth@student.ethz.ch>

// Due to the route selection when generating the reduction we get some combination of master and
// slave address spaces that are not allowed to be generated!

`include "axi/typedef.svh"
`include "axi/assign.svh"
`include "floo_noc/typedef.svh"

`define TARGET_SIMULATION

module tb_floo_fp_reduction;

  import floo_pkg::*;

  /* Functions */
  // Function to generate a chimney config with a RoB
  function automatic chimney_cfg_t gen_rob_chimney_cfg();
    chimney_cfg_t cfg = ChimneyDefaultCfg;
    cfg.BRoBType = SimpleRoB;
    cfg.BRoBSize = 64;
    cfg.RRoBType = NoRoB;
    cfg.RRoBSize = 64;
    return cfg;
  endfunction

  /* All local parameter */
  localparam time CyclTime = 10ns;
  localparam time ApplTime = 2ns;
  localparam time TestTime = 8ns;

  localparam int unsigned NumReductions = 2;

  localparam chimney_cfg_t RoBChimneyCfg = gen_rob_chimney_cfg();

  localparam floo_pkg::axi_cfg_t AxiNarrow = '{
    AddrWidth: 32,
    DataWidth: 64,
    UserWidth: 38,
    InIdWidth: 4,
    OutIdWidth: 4
  };

  // AXI nw_chimney parameters
  localparam floo_pkg::axi_cfg_t AxiWide = '{
    AddrWidth: 32,
    DataWidth: 512,
    UserWidth: 1,
    InIdWidth: 3,
    OutIdWidth: 1
  };

  // Generate the user fields
  typedef struct packed {
    logic [31:0] mask;
    floo_pkg::collect_comm_e coll_operation_type;
    floo_pkg::reduction_op_e coll_offload_ops;
  } axi_subfield_user_t;

  // TODO: Change @ Chimney too because Questa pisses itself if we use AxiConfig there
  //localparam floo_pkg::axi_cfg_t AxiConfig = AxiWide;  // Wide AXI Link
  localparam floo_pkg::axi_cfg_t AxiConfig = AxiNarrow;  // Narrow AXI Link

  /* All Typedef Vars */
  typedef logic [$clog2(RoBChimneyCfg.BRoBSize)-1:0] rob_idx_t;
  typedef logic [1:0] x_bits_t;
  typedef logic [1:0] y_bits_t;
  typedef logic [AxiConfig.DataWidth-1:0] red_data_t;

  // Generate the ID struct???
  `FLOO_TYPEDEF_XY_NODE_ID_T(id_t, x_bits_t, y_bits_t, logic)
  // Generate the header of the filt - inkl. commtype and reduction type
  `FLOO_TYPEDEF_OFF_REDUCTION_HDR_T(hdr_t, id_t, id_t, id_t, floo_pkg::axi_ch_e, rob_idx_t, floo_pkg::collect_comm_e, floo_pkg::reduction_op_e)
  //// Generate the axi from the test config??? (Num Route also defined there!)
  `FLOO_TYPEDEF_AXI_FROM_CFG(axi, AxiConfig)
  //// Generate all required Floo Types which encompass the axi channels with the generated header
  `FLOO_TYPEDEF_AXI_CHAN_ALL(axi, req, rsp, axi_in, AxiConfig, hdr_t)
  //// Link type to connect flooNoC Elements
  `FLOO_TYPEDEF_AXI_LINK_ALL(req, rsp, req, rsp)

  // Generate the address range for each node
  typedef struct packed {
    int unsigned  idx;
    axi_addr_t start_addr;
    axi_addr_t end_addr;
  } node_addr_region_t;

  // Generate the adress scope of each individal master
  // If you change these value then change the value @ the function generateParticpants!
  localparam node_addr_region_t [floo_pkg::NumDirections-1:0] AddrRegions = '{
    '{idx: Eject, start_addr: 32'h00110000, end_addr: 32'h00120000},  // Local Port TODO: Is this correct?
    '{idx: West,  start_addr: 32'h00100000, end_addr: 32'h01010000},   // West
    '{idx: South, start_addr: 32'h00010000, end_addr: 32'h00020000},  // South
    '{idx: East,  start_addr: 32'h00120000, end_addr: 32'h00130000},   // East
    '{idx: North, start_addr: 32'h00210000, end_addr: 32'h00220000}   // North
  };

  // Due to the build up of the testbench we have some invalid path due to the routing.
  localparam int NumberInvalidPath = 13;
  localparam node_addr_region_t [NumberInvalidPath-1:0] InvalidPath = '{
    '{idx: 0, start_addr: 32'h00010000, end_addr: 32'h00120000},      // S-E
    '{idx: 0, start_addr: 32'h00010000, end_addr: 32'h00100000},      // S-W
    '{idx: 0, start_addr: 32'h00210000, end_addr: 32'h00120000},      // N-E
    '{idx: 0, start_addr: 32'h00210000, end_addr: 32'h00100000},      // N-W
    '{idx: 0, start_addr: 32'h00120000, end_addr: 32'h00010000},      // E-S
    '{idx: 0, start_addr: 32'h00100000, end_addr: 32'h00010000},      // W-S
    '{idx: 0, start_addr: 32'h00120000, end_addr: 32'h00210000},      // E-N
    '{idx: 0, start_addr: 32'h00100000, end_addr: 32'h00210000},      // W-N
    '{idx: 0, start_addr: 32'h00110000, end_addr: 32'h00110000},      // E-E
    '{idx: 0, start_addr: 32'h00100000, end_addr: 32'h00100000},      // W-W
    '{idx: 0, start_addr: 32'h00010000, end_addr: 32'h00010000},      // S-S
    '{idx: 0, start_addr: 32'h00120000, end_addr: 32'h00120000},      // E-E
    '{idx: 0, start_addr: 32'h00210000, end_addr: 32'h00210000}       // N-N
  };

  /* Variable declaration */

  // Control Var
  logic clk, rst_n;
  logic [floo_pkg::NumDirections-1:0] end_of_sim;

  // The Chimney works with a combined struct which encompass the data channel / valid / ready
  // However the router accept them independently so we have to separate them.
  floo_req_t [floo_pkg::NumDirections-1:0] chimney_req_out, chimney_req_in;
  floo_rsp_t [floo_pkg::NumDirections-1:0] chimney_rsp_out, chimney_rsp_in;

  // Data-Channel to parse the chimney request
  floo_req_chan_t [floo_pkg::NumDirections-1:0] chimney_req_out_chan, chimney_req_in_chan;
  floo_rsp_chan_t [floo_pkg::NumDirections-1:0] chimney_rsp_out_chan, chimney_rsp_in_chan;

  // Handshake signal to parse the chimney request
  logic [floo_pkg::NumDirections-1:0]      chimney_req_out_valid, chimney_req_out_ready;
  logic [floo_pkg::NumDirections-1:0]      chimney_rsp_out_valid, chimney_rsp_out_ready;
  logic [floo_pkg::NumDirections-1:0]      chimney_req_in_valid, chimney_req_in_ready;
  logic [floo_pkg::NumDirections-1:0]      chimney_rsp_in_valid, chimney_rsp_in_ready;

  // ID for each irection ???
  id_t [floo_pkg::NumDirections-1:0] xy_id;

  // AXI master & slave variable!
  axi_in_req_t [floo_pkg::NumDirections-1:0] node_mst_req;
  axi_in_rsp_t [floo_pkg::NumDirections-1:0] node_mst_resp;

  axi_out_req_t [floo_pkg::NumDirections-1:0] node_slv_req;
  axi_out_rsp_t [floo_pkg::NumDirections-1:0] node_slv_resp;

  // Connect the offload port of the router directly to the test offload
  floo_pkg::reduction_op_e offload_req_op;
  red_data_t offload_req_operand1;
  red_data_t offload_req_operand2;
  logic offload_req_valid;
  logic offload_req_ready;

  red_data_t offload_resp_result;
  logic offload_resp_valid;
  logic offload_resp_ready;

  /* Module Declaration */

  // Mapping between the "chimney" variable and the "router" variable 
  for (genvar i = 0; i < floo_pkg::NumDirections; i++) begin : gen_directions
    assign chimney_req_out_chan[i] = chimney_req_out[i].req;
    assign chimney_rsp_out_chan[i] = chimney_rsp_out[i].rsp;
    assign chimney_req_in[i].req = chimney_req_in_chan[i];
    assign chimney_rsp_in[i].rsp = chimney_rsp_in_chan[i];
    assign chimney_req_out_valid[i] = chimney_req_out[i].valid;
    assign chimney_req_out_ready[i] = chimney_req_out[i].ready;
    assign chimney_rsp_out_valid[i] = chimney_rsp_out[i].valid;
    assign chimney_rsp_out_ready[i] = chimney_rsp_out[i].ready;
    assign chimney_req_in[i].valid = chimney_req_in_valid[i];
    assign chimney_req_in[i].ready = chimney_req_in_ready[i];
    assign chimney_rsp_in[i].valid = chimney_rsp_in_valid[i];
    assign chimney_rsp_in[i].ready = chimney_rsp_in_ready[i];
  end
/*
  // Determint the reduction participant
  function logic[floo_pkg::NumDirections-1:0] determintParticipant (int master, axi_subfield_user_t mask);
    logic[floo_pkg::NumDirections-1:0] participants;
    logic[AxiConfig.AddrWidth-1:0] mask_dont_care_bits;
    logic[AxiConfig.AddrWidth-1:0] masked_mask;

    logic[AxiConfig.AddrWidth-1:0] rule_mask;
    logic[AxiConfig.AddrWidth-1:0] rule_start_addr;
    logic[AxiConfig.AddrWidth-1:0] mst_start_addr;

    // Change here for new address schem
    mask_dont_care_bits = 32'hFFCCFFFF;
    masked_mask = mask_dont_care_bits | mask.mask;

    participants = '0;
    mst_start_addr = AddrRegions[master].start_addr;

    for(int i = 0; i < floo_pkg::NumDirections; i++) begin
      rule_mask       = AddrRegions[i].end_addr - AddrRegions[i].start_addr - 1;
      rule_start_addr = AddrRegions[i].start_addr;

      if(&((~(mst_start_addr ^ rule_start_addr) | (rule_mask | mask.mask)))) begin
        participants = participants | (1 << i);
      end
    end

    return participants;
  endfunction
*/

  // Determint the reduction participant
  function logic[floo_pkg::NumDirections-1:0] determintParticipant (id_t src, id_t mask);
    logic[floo_pkg::NumDirections-1:0] participants;
    id_t id_masked;
    id_t src_masked;

    participants = '0;
    // mask the source
    src_masked = src | mask;

    for(int i = 0; i < floo_pkg::NumDirections; i++) begin
      id_masked = xy_id[i] | mask;
      // Compare both masked result together
      if(src_masked == id_masked) begin
        participants = participants | (1 << i);
      end
    end

    return participants;
  endfunction

  // Generate golden model
  initial begin
    logic[AxiConfig.DataWidth-1:0] 	data_queue [floo_pkg::NumDirections][$];
    $display($time, "Start Golden Model Generation!");
    while(1) begin
      @(posedge clk);

      // When we receive an incomingrequest push the data to an queue
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_req_out[i].valid == 1'b1) && (chimney_req_in[i].ready == 1'b1) && (chimney_req_out[i].req.generic.hdr.axi_ch == AxiW)) begin
          logic[floo_pkg::NumDirections-1:0] participants;
          axi_subfield_user_t abstraction;
          participants = determintParticipant(chimney_req_out[i].req.axi_w.hdr.src_id, chimney_req_out[i].req.axi_w.hdr.mask);

          // Only add Data if we have an reduction
          abstraction = chimney_req_out[i].req.axi_w.payload.user;
          if(($countones(participants) > 0) && (abstraction.coll_operation_type == floo_pkg::OffloadReduction) && (participants[i] == 1'b1)) begin
            data_queue[i].push_front(chimney_req_out[i].req.axi_w.payload.data);
          end
        end
      end

      // Evaluate all incoming request to the chimney
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_req_in[i].valid == 1'b1) && (chimney_req_out[i].ready == 1'b1) && (chimney_req_in[i].req.generic.hdr.axi_ch == AxiW)) begin
          logic[floo_pkg::NumDirections-1:0] participants;
          logic[floo_pkg::NumDirections-1:0] temp_participants;
          axi_subfield_user_t abstraction;
          participants = '0;

          // Determint all input paricipants to fetch from their queues
          participants = determintParticipant(chimney_req_in[i].req.axi_w.hdr.src_id, chimney_req_in[i].req.axi_w.hdr.mask);

          // Only check reduction if we have more than 0 participant!
          abstraction = chimney_req_in[i].req.axi_w.payload.user;
          if(($countones(participants) > 0) && (abstraction.coll_operation_type == floo_pkg::OffloadReduction)) begin
            logic[31:0] result;
            logic[31:0] fetch_data_mask;
            logic[AxiConfig.DataWidth-1:0] fetch_data;
            logic[31:0] received_result;
            result = '0;

            // Fetch data from all involved Queues and calc the addition
            for(int j = 0; j < floo_pkg::NumDirections; j++) begin
              if(participants[j] == 1'b1) begin
                fetch_data = data_queue[j].pop_back();
                fetch_data_mask = fetch_data[31:0];
                result = result + fetch_data_mask;
              end
            end

            // Compare against received result
            received_result = chimney_req_in[i].req.axi_w.payload.data[31:0];
            if(result == received_result) begin
              $display($time, " MONITOR %1d (W)           > Correct Reduction Result: %h For Masterset: %5b", i, received_result, participants);
            end else begin
              $display($time, " MONITOR %1d (W)           > Wrong Reduction Result: %h Golden Model: %h For Masterset: %5b", i, received_result, result, participants);
            end
          end
        end
      end


    end
  end

  // Debug Prints on all IF
  initial begin
    $display($time, " Start IF Monitoring!");
    while(1) begin // run forever
      @(posedge clk);

      // Evaluate all incoming request to the router
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_req_in[i].valid == 1'b1) && (chimney_req_out[i].ready == 1'b1)) begin
          printChimneyRequest(chimney_req_in[i].req, i, "Ch-In ");
        end
      end

        // Evaluate all outgoing request from the router
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_req_out[i].valid == 1'b1) && (chimney_req_in[i].ready == 1'b1)) begin
          printChimneyRequest(chimney_req_out[i].req, i, "Ch-Out");
        end
      end

      // Evaluate all incoming response to the router
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_rsp_in[i].valid == 1'b1) && (chimney_rsp_out[i].ready == 1'b1)) begin
          printChimneyResponse(chimney_rsp_in[i].rsp, i, "Ch-In ");

        end
      end
      
      // Evaluate all outgoing response from the router
      for(int i = 0; i < floo_pkg::NumDirections; i++) begin
        if((chimney_rsp_out[i].valid == 1'b1) && (chimney_rsp_in[i].ready == 1'b1)) begin
          printChimneyResponse(chimney_rsp_out[i].rsp, i, "Ch-Out");
        end  
      end

      // Evaluate the offload port request
      if((offload_req_valid == 1'b1) && (offload_req_ready == 1'b1)) begin
        $display($time, " OFFLOAD   (RQ) > T: %4b OP1: %h OP2: %h", offload_req_op, offload_req_operand1, offload_req_operand2);
      end

      // Evaluate the offload port response
      if((offload_resp_valid == 1'b1) && (offload_resp_ready == 1'b1)) begin
        $display($time, " OFFLOAD   (RS) > Res: %h", offload_resp_result);
      end
    end
  end

  // Function to plot request (No support for AR channel)
  function void printChimneyRequest (floo_req_chan_t req, int i, string s);
    if(req.generic.hdr.axi_ch == AxiAw) begin
      $display($time, " MONITOR %1d (AW) [%s] > M(Floo): %b M(AXI): %b C: %2b T: %4b Id: %4b Addr:%h Part: %5b", i, s, req.axi_aw.hdr.mask, req.axi_aw.payload.user, req.axi_aw.hdr.commtype, req.axi_aw.hdr.reduction_op, req.axi_aw.payload.id, req.axi_aw.payload.addr, determintParticipant(req.axi_aw.hdr.src_id, req.axi_aw.hdr.mask));
    end else if(req.generic.hdr.axi_ch == AxiW) begin
      $display($time, " MONITOR %1d (W)  [%s] > M(Floo): %b M(AXI): %b C: %2b T: %4b Data:%h Last: %1d", i, s, req.axi_w.hdr.mask, req.axi_w.payload.user, req.axi_w.hdr.commtype, req.axi_w.hdr.reduction_op, req.axi_w.payload.data, req.axi_w.payload.last);
    end else if(req.generic.hdr.axi_ch == AxiAr) begin
      $display($time, " MONITOR %1d (AR) [%s] > Dedected!", i, s);
    end
	endfunction

    // Function to plot response
  function void printChimneyResponse (floo_rsp_chan_t rsp, int i, string s);
    if(rsp.generic.hdr.axi_ch == AxiB) begin
      $display($time, " MONITOR %1d (B)  [%s] > M(Floo): %b M(AXI): %b C: %2b T: %4b Id: %4b", i, s, rsp.axi_b.hdr.mask, rsp.axi_b.payload.user, rsp.axi_b.hdr.commtype, rsp.axi_b.hdr.reduction_op, rsp.axi_b.payload.id);
    end else if(rsp.generic.hdr.axi_ch == AxiR) begin
      $display($time, " MONITOR %1d (R)  [%s] > Dedected!", i, s);
    end
	endfunction




  // clock and reset generation
  clk_rst_gen #(
    .ClkPeriod    ( CyclTime ),
    .RstClkCycles ( 5        )
  ) i_clk_gen (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );

  // Request router with reduction enabled
  floo_router #(
    .NumRoutes                      (floo_pkg::NumDirections),
    .NumVirtChannels                (1),
    .InFifoDepth                    (2),
    .OutFifoDepth                   (2),
    .RouteAlgo                      (floo_pkg::XYRouting),
    .id_t                           (id_t),
    .NoLoopback                     (1'b1),
    .XYRouteOpt                     (1'b0),
    .EnMultiCast                    (1'b0),
    .EnReduction                    (1'b0),
    .EnOffloadReduction             (1'b1),
    .flit_t                         (floo_req_generic_flit_t),
    .hdr_t                          (hdr_t),
    .NarrowRspMask                  ('0),
    .WideRspMask                    ('0),
    .RdOperation_t                  (floo_pkg::reduction_op_e),
    .RdData_t                       (red_data_t),
    .RdFifoFallThrough              (1'b1),
    .RdFifoDepth                    (2),
    .RdPipelineDepth                (3),
    .RdControllerComplex            (2),
    .RdPartialBufferSize            (3),
    .RdTagBits                      (4),
    .InversedSrcDst                 (1'b0)
  ) i_dut_req (
    .clk_i                          (clk),
    .rst_ni                         (rst_n),
    .test_enable_i                  (1'b0),
    .xy_id_i                        (xy_id[floo_pkg::Eject]),
    .id_route_map_i                 ('0),
    .valid_i                        (chimney_req_out_valid),
    .ready_o                        (chimney_req_in_ready),
    .data_i                         (chimney_req_out_chan),
    .valid_o                        (chimney_req_in_valid),
    .ready_i                        (chimney_req_out_ready),
    .data_o                         (chimney_req_in_chan),
    .offload_req_op_o               (offload_req_op),
    .offload_req_operand1_o         (offload_req_operand1),
    .offload_req_operand2_o         (offload_req_operand2),
    .offload_req_valid_o            (offload_req_valid),
    .offload_req_ready_i            (offload_req_ready),
    .offload_resp_result_i          (offload_resp_result),
    .offload_resp_valid_i           (offload_resp_valid),
    .offload_resp_ready_o           (offload_resp_ready)
  );
  
  // Request router with reduction enabled
  floo_router #(
    .NumRoutes                      (floo_pkg::NumDirections),
    .NumVirtChannels                (1),
    .InFifoDepth                    (2),
    .OutFifoDepth                   (2),
    .RouteAlgo                      (floo_pkg::XYRouting),
    .id_t                           (id_t),
    .NoLoopback                     (1'b1),
    .XYRouteOpt                     (1'b0),
    .EnMultiCast                    (1'b1),
    .EnReduction                    (1'b0),
    .EnOffloadReduction             (1'b0),
    .flit_t                         (floo_rsp_generic_flit_t),
    .hdr_t                          (hdr_t),
    .NarrowRspMask                  ('0),
    .WideRspMask                    ('0),
    .RdOperation_t                  (floo_pkg::reduction_op_e),
    .RdData_t                       (red_data_t),
    .RdFifoFallThrough              (1'b1),
    .RdFifoDepth                    (2),
    .RdPipelineDepth                (3),
    .RdControllerComplex            (2),
    .RdPartialBufferSize            (3),
    .RdTagBits                      (4),
    .InversedSrcDst                 (1'b0)
  ) i_dut_resp (
    .clk_i                          (clk),
    .rst_ni                         (rst_n),
    .test_enable_i                  (1'b0),
    .xy_id_i                        (xy_id[floo_pkg::Eject]),
    .id_route_map_i                 ('0),
    .valid_i                        (chimney_rsp_out_valid),
    .ready_o                        (chimney_rsp_in_ready),
    .data_i                         (chimney_rsp_out_chan),
    .valid_o                        (chimney_rsp_in_valid),
    .ready_i                        (chimney_rsp_out_ready),
    .data_o                         (chimney_rsp_in_chan),
    .offload_req_op_o               (),
    .offload_req_operand1_o         (),
    .offload_req_operand2_o         (),
    .offload_req_valid_o            (),
    .offload_req_ready_i            ('0),
    .offload_resp_result_i          ('0),
    .offload_resp_valid_i           ('0),
    .offload_resp_ready_o           ()
  );

  // Add the Reduction Offload here
  if(AxiConfig.DataWidth == 64) begin : gen_narrow_reduction
    floo_reduction_wrapper #(
      .RdData_t                       (red_data_t),
      .RdElements                     (1),
      .FPU_ACTIVE                     (1'b0),
      .ALU_ACTIVE                     (1'b1),
      .DEBUG_PRINT_TRACE              (1'b1)
    ) i_wrapper_narrow (
      .clk_i                          (clk),
      .rst_ni                         (rst_n),
      .flush_i                        (1'b0),
      .reduction_req_op1_i            (offload_req_operand1),
      .reduction_req_op2_i            (offload_req_operand2),
      .reduction_req_type_i           (offload_req_op),
      .reduction_req_valid_i          (offload_req_valid),
      .reduction_req_ready_o          (offload_req_ready),
      .reduction_resp_data_o          (offload_resp_result),
      .reduction_resp_valid_o         (offload_resp_valid),
      .reduction_resp_ready_i         (offload_resp_ready)
    );
  end else if(AxiConfig.DataWidth == 512) begin : gen_wide_reduction
    floo_reduction_wrapper #(
      .RdData_t                       (red_data_t),
      .RdElements                     (8),
      .FPU_ACTIVE                     (1'b1),
      .ALU_ACTIVE                     (1'b0),
      .DEBUG_PRINT_TRACE              (1'b1)
    ) i_wrapper_wide (
      .clk_i                          (clk),
      .rst_ni                         (rst_n),
      .flush_i                        (1'b0),
      .reduction_req_op1_i            (offload_req_operand1),
      .reduction_req_op2_i            (offload_req_operand2),
      .reduction_req_type_i           (offload_req_op),
      .reduction_req_valid_i          (offload_req_valid),
      .reduction_req_ready_o          (offload_req_ready),
      .reduction_resp_data_o          (offload_resp_result),
      .reduction_resp_valid_o         (offload_resp_valid),
      .reduction_resp_ready_i         (offload_resp_ready)
    );
  end else begin : gen_unkown_red
    $fatal(1, "Unkown Data Width - Please implement it!");
  end


  // Inst. module which generats the data for the test
  floo_reduction_full_mst #(
    .ApplTime           (ApplTime),
    .TestTime           (TestTime),
    .AxiCfg             (AxiConfig),
    .mst_req_t          (axi_in_req_t),
    .mst_rsp_t          (axi_in_rsp_t),
    .slv_req_t          (axi_out_req_t),
    .slv_rsp_t          (axi_out_rsp_t),
    .rule_t             (node_addr_region_t),
    .AxiMaxBurstLen     (6),
    .NumAddrRegions     (floo_pkg::NumDirections),
    .AddrRegions        (AddrRegions),
    .NumInvalidPath     (NumberInvalidPath),
    .InvalidPath        (InvalidPath),
    .NumReductions      (NumReductions),
    .NumTestPorts       (floo_pkg::NumDirections),
    .NumInfligthElem    (2)
  ) i_gen_reductions (
    .clk_i              (clk),
    .rst_ni             (rst_n),
    .mst_port_req_o     (node_mst_req),
    .mst_port_rsp_i     (node_mst_resp),
    .slv_port_req_i     (node_slv_req),
    .slv_port_rsp_o     (node_slv_resp),
    .end_of_sim_o       (end_of_sim)
  );

  // Iterate over all ports to assign a chimney and the coresspondng AXI logger
  for (genvar i = North; i <= Eject; i++) begin : gen_slaves

    // Assign FlooNoC ID to the router destinations
    if (i == North) begin : gen_north
      assign xy_id[i] = '{x: 2'd1, y: 2'd2, port_id: 1'd0};
    end else if (i == South) begin : gen_south
      assign xy_id[i] = '{x: 2'd1, y: 2'd0, port_id: 1'd0};
    end else if (i == East) begin : gen_east
      assign xy_id[i] = '{x: 2'd2, y: 2'd1, port_id: 1'd0};
    end else if (i == West) begin : gen_west
      assign xy_id[i] = '{x: 2'd0, y: 2'd1, port_id: 1'd0};
    end else if (i == Eject) begin : gen_eject
      assign xy_id[i] = '{x: 2'd1, y: 2'd1, port_id: 1'd0};
    end

    // Generate the Chimneys to connect the router to AXI Testbench
    floo_axi_chimney #(
      .AxiCfg                 (AxiNarrow),
      .ChimneyCfg             (RoBChimneyCfg), // Needs RoB
      .RouteCfg               (floo_test_pkg::RouteCfg),
      .AtopSupport            (floo_test_pkg::AtopSupport),
      .MaxAtomicTxns          (floo_test_pkg::MaxAtomicTxns),
      .axi_in_req_t           (axi_in_req_t),
      .axi_in_rsp_t           (axi_in_rsp_t),
      .axi_out_req_t          (axi_out_req_t),
      .axi_out_rsp_t          (axi_out_rsp_t),
      .rob_idx_t              (rob_idx_t),
      .id_t                   (id_t),
      .hdr_t                  (hdr_t),
      .floo_req_t             (floo_req_t),
      .floo_rsp_t             (floo_rsp_t),
      .user_struct_t          (axi_subfield_user_t),
      .user_mask_t            (axi_subfield_user_t),
      .EnMultiCast            (1'b1),
      .EnCollectiveOperation  (1'b1)
    ) i_floo_axi_chimney (
      .clk_i                  (clk),
      .rst_ni                 (rst_n),
      .sram_cfg_i             ('0),
      .test_enable_i          (1'b0),
      .axi_in_req_i           (node_mst_req[i]),
      .axi_in_rsp_o           (node_mst_resp[i]),
      .axi_out_req_o          (node_slv_req[i]),
      .axi_out_rsp_i          (node_slv_resp[i]),
      .id_i                   (xy_id[i]),
      .route_table_i          ('0),
      .floo_req_o             (chimney_req_out[i]),
      .floo_rsp_o             (chimney_rsp_out[i]),
      .floo_req_i             (chimney_req_in[i]),
      .floo_rsp_i             (chimney_rsp_in[i])
    );

    // Axi Dumper to Monitor the Master Bus
    axi_dumper #(
      .BusName    ($sformatf("AXI Master %0d", i)),
      .LogAR      (1'b1),
      .LogW       (1'b1),
      .LogB       (1'b0),
      .LogR       (1'b0),
      .axi_req_t  (axi_in_req_t),
      .axi_resp_t (axi_in_rsp_t)
    ) i_axi_dumper_mst (
      .clk_i      (clk),
      .rst_ni     (rst_n),
      .axi_req_i  (node_mst_req[i]),
      .axi_resp_i (node_mst_resp[i])
    );

    // Axi Dumper to Monitor the Slave Bus
    axi_dumper #(
      .BusName    ($sformatf("AXI Slave %0d", i)),
      .LogAW      (1'b0),
      .LogAR      (1'b0),
      .LogW       (1'b0),
      .LogB       (1'b1),
      .LogR       (1'b0),
      .axi_req_t  (axi_in_req_t),
      .axi_resp_t (axi_in_rsp_t)
    ) i_axi_dumper_slv (
      .clk_i      (clk),
      .rst_ni     (rst_n),
      .axi_req_i  (node_slv_req[i]),
      .axi_resp_i (node_slv_resp[i])
    );
  end

// End the simultation here if all stop signals are asserted!
initial begin
    wait(&end_of_sim);
    $stop;
end


endmodule
