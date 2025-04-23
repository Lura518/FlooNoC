// Copyright 2022 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51
//
// Tim Fischer <fischeti@iis.ee.ethz.ch>

`include "axi/typedef.svh"
`include "axi/assign.svh"
`include "floo_noc/typedef.svh"

`define TARGET_SIMULATION

module tb_floo_reduction_axi_test;

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

  localparam int unsigned NumReductions = 100;

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
  localparam node_addr_region_t [floo_pkg::NumDirections-1:0] AddrRegions = '{
    '{idx: North, start_addr: 32'h00210000, end_addr: 32'h0021FFFF},  // North
    '{idx: East, start_addr: 32'h00120000, end_addr: 32'h0012FFFF},   // East
    '{idx: South, start_addr: 32'h00010000, end_addr: 32'h0001FFFF},  // South
    '{idx: West, start_addr: 32'h00100000, end_addr: 32'h0010FFFF},   // West
    '{idx: Eject, start_addr: 32'h00000000, end_addr: 32'h00008000}   // Local Port TODO: Is this correct?
  };

  /* Variable declaration */

  // Control Var
  logic clk, rst_n;
  logic [floo_pkg::NumDirections-1:0] end_of_sim;

  // AXI master & slave variable!
  axi_in_req_t [floo_pkg::NumDirections-1:0] node_mst_req;
  axi_in_rsp_t [floo_pkg::NumDirections-1:0] node_mst_resp;
  axi_in_req_t [floo_pkg::NumDirections-1:0] node_mst_req_q;
  axi_in_rsp_t [floo_pkg::NumDirections-1:0] node_mst_resp_q;

  axi_out_req_t [floo_pkg::NumDirections-1:0] node_slv_req;
  axi_out_rsp_t [floo_pkg::NumDirections-1:0] node_slv_resp;
  axi_out_req_t [floo_pkg::NumDirections-1:0] node_slv_req_q;
  axi_out_rsp_t [floo_pkg::NumDirections-1:0] node_slv_resp_q;

  /* Module Declaration */

  for (genvar i = 0; i < 5; i++) begin : gen_cuts
    axi_cut #(
      .Bypass             (1'b0),
      .aw_chan_t          (axi_in_aw_chan_t),
      .w_chan_t           (axi_in_w_chan_t),
      .b_chan_t           (axi_in_b_chan_t),
      .ar_chan_t          (axi_in_ar_chan_t),
      .r_chan_t           (axi_in_r_chan_t),
      .axi_req_t          (axi_in_req_t),
      .axi_resp_t         (axi_in_rsp_t)
    ) cut_master (
      .clk_i              (clk),
      .rst_ni             (rst_n),
      .slv_req_i          (node_mst_req[i]),
      .slv_resp_o         (node_mst_resp[i]),
      .mst_req_o          (node_mst_req_q[i]),
      .mst_resp_i         (node_mst_resp_q[i])
    );

    axi_cut #(
      .Bypass             (1'b0),
      .aw_chan_t          (axi_out_aw_chan_t),
      .w_chan_t           (axi_out_w_chan_t),
      .b_chan_t           (axi_out_b_chan_t),
      .ar_chan_t          (axi_out_ar_chan_t),
      .r_chan_t           (axi_out_r_chan_t),
      .axi_req_t          (axi_out_req_t),
      .axi_resp_t         (axi_out_rsp_t)
    ) cut_slave (
      .clk_i              (clk),
      .rst_ni             (rst_n),
      .slv_req_i          (node_slv_req_q[i]),
      .slv_resp_o         (node_slv_resp_q[i]),
      .mst_req_o          (node_slv_req[i]),
      .mst_resp_i         (node_slv_resp[i])
    );
  end

  // clock and reset generation
  clk_rst_gen #(
    .ClkPeriod    ( CyclTime ),
    .RstClkCycles ( 5        )
  ) i_clk_gen (
    .clk_o  ( clk   ),
    .rst_no ( rst_n )
  );

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

  for (genvar i = 0; i < 5; i++) begin : gen_connections
    assign node_slv_req_q[i] = node_mst_req_q[i];
    assign node_mst_resp_q[i] = node_slv_resp_q[i];

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
      .axi_req_t  (axi_out_req_t),
      .axi_resp_t (axi_out_rsp_t)
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
