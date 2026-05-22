// =============================================================================
// pcie_axi_dma_top.sv
//
// Top-level integration of the PCIe-AXI DMA Streaming Architecture.
//
// Pipeline (left to right):
//
//  gen_*          pcie_*          dma_*          mm2s_*         pre_fifo_*
//   [TB]  -----> [pcie_model] --> [dma_model] --> [mm2s_adap] --> [axis_fifo]
//
//                           post_fifo_*          batch_*
//                               [axis_fifo] --> [batching_unit] --> [stream_sink]
//                                                                        |
//                                                               [latency_monitor]
//
// All AXI-Stream inter-module buses follow the naming convention:
//   <stage>_tvalid / <stage>_tready / <stage>_tdata / <stage>_tkeep /
//   <stage>_tlast  / <stage>_tuser  / <stage>_tid
//
// Parameters override the package defaults to enable sweep experiments:
//   - FIFO_DEPTH          → FIFO depth study
//   - BATCH_COUNT/TIMEOUT → batching study
//   - BATCHING_EN         → 0=baseline, 1=optimized
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module pcie_axi_dma_top #(
  // --- Datapath widths (usually kept at defaults) ---------------------------
  parameter int DATA_WIDTH      = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W      = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W        = pcie_axi_pkg::PKT_ID_W,

  // --- PCIe model -----------------------------------------------------------
  parameter int PCIE_BASE_LAT   = pcie_axi_pkg::PCIE_BASE_LATENCY,
  parameter int PCIE_PER_DW_LAT = pcie_axi_pkg::PCIE_PER_DW_LATENCY,

  // --- DMA model ------------------------------------------------------------
  parameter int DMA_SETUP_CYC   = pcie_axi_pkg::DMA_SETUP_CYCLES,
  parameter int DMA_BPC         = pcie_axi_pkg::DMA_BYTES_PER_CYC,

  // --- FIFO -----------------------------------------------------------------
  parameter int FIFO_DEPTH      = pcie_axi_pkg::FIFO_DEPTH_DEFAULT,

  // --- Batching unit --------------------------------------------------------
  parameter int BATCH_COUNT     = pcie_axi_pkg::BATCH_COUNT_DEFAULT,
  parameter int BATCH_TIMEOUT   = pcie_axi_pkg::BATCH_TIMEOUT_DEFAULT,
  parameter bit BATCHING_EN     = 1'b0   // 0 = baseline, 1 = optimized
) (
  input  logic                   clk,
  input  logic                   rst_n,

  // ---- Input: Packet Generator (testbench) ---------------------------------
  input  logic                   gen_tvalid,
  output logic                   gen_tready,
  input  logic [DATA_WIDTH-1:0]  gen_tdata,
  input  logic [DATA_WIDTH/8-1:0]gen_tkeep,
  input  logic                   gen_tlast,
  input  logic [PKT_SIZE_W-1:0]  gen_tuser,   // packet byte-length
  input  logic [PKT_ID_W-1:0]    gen_tid,

  // ---- Latency monitor injection event (from testbench) -------------------
  input  logic                   pkt_inject,
  input  logic [PKT_ID_W-1:0]    inject_id,
  input  logic [PKT_SIZE_W-1:0]  inject_size,

  // ---- Latency monitor output (to testbench for logging) ------------------
  output logic [31:0]            latency_out,
  output logic                   latency_valid,
  output logic [PKT_ID_W-1:0]    latency_id,

  // ---- Stream sink diagnostics --------------------------------------------
  output logic [31:0]            pkt_count,

  // ---- FIFO status (optional debug) ---------------------------------------
  output logic [$clog2(pcie_axi_pkg::FIFO_DEPTH_DEFAULT):0] fifo_occupancy,
  output logic                   fifo_full,
  output logic                   fifo_empty
);

  // ===========================================================================
  // Local parameters
  // ===========================================================================
  localparam int STRB_W = DATA_WIDTH / 8;

  // ===========================================================================
  // Inter-module AXI-Stream buses
  // ===========================================================================

  // [gen] -> [pcie_model]
  // (gen_* ports are the source — already on the module port list above)

  // [pcie_model] -> [dma_model]
  logic                   pcie_m_tvalid;
  logic                   pcie_m_tready;
  logic [DATA_WIDTH-1:0]  pcie_m_tdata;
  logic [STRB_W-1:0]      pcie_m_tkeep;
  logic                   pcie_m_tlast;
  logic [PKT_SIZE_W-1:0]  pcie_m_tuser;
  logic [PKT_ID_W-1:0]    pcie_m_tid;

  // [dma_model] -> [axi_mm2s_adapter]
  logic                   dma_m_tvalid;
  logic                   dma_m_tready;
  logic [DATA_WIDTH-1:0]  dma_m_tdata;
  logic [STRB_W-1:0]      dma_m_tkeep;
  logic                   dma_m_tlast;
  logic [PKT_SIZE_W-1:0]  dma_m_tuser;
  logic [PKT_ID_W-1:0]    dma_m_tid;

  // [axi_mm2s_adapter] -> [axis_fifo]
  logic                   mm2s_m_tvalid;
  logic                   mm2s_m_tready;
  logic [DATA_WIDTH-1:0]  mm2s_m_tdata;
  logic [STRB_W-1:0]      mm2s_m_tkeep;
  logic                   mm2s_m_tlast;
  logic [PKT_SIZE_W-1:0]  mm2s_m_tuser;
  logic [PKT_ID_W-1:0]    mm2s_m_tid;

  // [axis_fifo] -> [batching_unit]
  logic                   fifo_m_tvalid;
  logic                   fifo_m_tready;
  logic [DATA_WIDTH-1:0]  fifo_m_tdata;
  logic [STRB_W-1:0]      fifo_m_tkeep;
  logic                   fifo_m_tlast;
  logic [PKT_SIZE_W-1:0]  fifo_m_tuser;
  logic [PKT_ID_W-1:0]    fifo_m_tid;

  // [batching_unit] -> [stream_sink]
  logic                   batch_m_tvalid;
  logic                   batch_m_tready;
  logic [DATA_WIDTH-1:0]  batch_m_tdata;
  logic [STRB_W-1:0]      batch_m_tkeep;
  logic                   batch_m_tlast;
  logic [PKT_SIZE_W-1:0]  batch_m_tuser;
  logic [PKT_ID_W-1:0]    batch_m_tid;

  // [stream_sink] -> [latency_monitor]
  logic                   sink_pkt_done;
  logic [PKT_ID_W-1:0]    sink_done_id;

  // ===========================================================================
  // Module instantiations
  // ===========================================================================

  // --- Stage 1: PCIe model --------------------------------------------------
  pcie_model #(
    .DATA_WIDTH   (DATA_WIDTH),
    .PKT_SIZE_W   (PKT_SIZE_W),
    .PKT_ID_W     (PKT_ID_W),
    .BASE_LATENCY (PCIE_BASE_LAT),
    .PER_DW_LATENCY(PCIE_PER_DW_LAT)
  ) u_pcie (
    .clk      (clk),
    .rst_n    (rst_n),
    .s_tvalid (gen_tvalid),
    .s_tready (gen_tready),
    .s_tdata  (gen_tdata),
    .s_tkeep  (gen_tkeep),
    .s_tlast  (gen_tlast),
    .s_tuser  (gen_tuser),
    .s_tid    (gen_tid),
    .m_tvalid (pcie_m_tvalid),
    .m_tready (pcie_m_tready),
    .m_tdata  (pcie_m_tdata),
    .m_tkeep  (pcie_m_tkeep),
    .m_tlast  (pcie_m_tlast),
    .m_tuser  (pcie_m_tuser),
    .m_tid    (pcie_m_tid)
  );

  // --- Stage 2: DMA model ---------------------------------------------------
  dma_model #(
    .DATA_WIDTH   (DATA_WIDTH),
    .PKT_SIZE_W   (PKT_SIZE_W),
    .PKT_ID_W     (PKT_ID_W),
    .SETUP_CYCLES (DMA_SETUP_CYC),
    .BYTES_PER_CYC(DMA_BPC)
  ) u_dma (
    .clk      (clk),
    .rst_n    (rst_n),
    .s_tvalid (pcie_m_tvalid),
    .s_tready (pcie_m_tready),
    .s_tdata  (pcie_m_tdata),
    .s_tkeep  (pcie_m_tkeep),
    .s_tlast  (pcie_m_tlast),
    .s_tuser  (pcie_m_tuser),
    .s_tid    (pcie_m_tid),
    .m_tvalid (dma_m_tvalid),
    .m_tready (dma_m_tready),
    .m_tdata  (dma_m_tdata),
    .m_tkeep  (dma_m_tkeep),
    .m_tlast  (dma_m_tlast),
    .m_tuser  (dma_m_tuser),
    .m_tid    (dma_m_tid)
  );

  // --- Stage 3: AXI-MM to AXI-Stream adapter --------------------------------
  axi_mm2s_adapter #(
    .DATA_WIDTH (DATA_WIDTH),
    .PKT_SIZE_W (PKT_SIZE_W),
    .PKT_ID_W   (PKT_ID_W)
  ) u_mm2s (
    .clk      (clk),
    .rst_n    (rst_n),
    .s_tvalid (dma_m_tvalid),
    .s_tready (dma_m_tready),
    .s_tdata  (dma_m_tdata),
    .s_tkeep  (dma_m_tkeep),
    .s_tlast  (dma_m_tlast),
    .s_tuser  (dma_m_tuser),
    .s_tid    (dma_m_tid),
    .m_tvalid (mm2s_m_tvalid),
    .m_tready (mm2s_m_tready),
    .m_tdata  (mm2s_m_tdata),
    .m_tkeep  (mm2s_m_tkeep),
    .m_tlast  (mm2s_m_tlast),
    .m_tuser  (mm2s_m_tuser),
    .m_tid    (mm2s_m_tid)
  );

  // --- Stage 4: AXI-Stream FIFO ---------------------------------------------
  axis_fifo #(
    .DATA_WIDTH (DATA_WIDTH),
    .PKT_SIZE_W (PKT_SIZE_W),
    .PKT_ID_W   (PKT_ID_W),
    .DEPTH      (FIFO_DEPTH)
  ) u_fifo (
    .clk        (clk),
    .rst_n      (rst_n),
    .s_tvalid   (mm2s_m_tvalid),
    .s_tready   (mm2s_m_tready),
    .s_tdata    (mm2s_m_tdata),
    .s_tkeep    (mm2s_m_tkeep),
    .s_tlast    (mm2s_m_tlast),
    .s_tuser    (mm2s_m_tuser),
    .s_tid      (mm2s_m_tid),
    .m_tvalid   (fifo_m_tvalid),
    .m_tready   (fifo_m_tready),
    .m_tdata    (fifo_m_tdata),
    .m_tkeep    (fifo_m_tkeep),
    .m_tlast    (fifo_m_tlast),
    .m_tuser    (fifo_m_tuser),
    .m_tid      (fifo_m_tid),
    .occupancy  (fifo_occupancy),
    .full       (fifo_full),
    .empty      (fifo_empty)
  );

  // --- Stage 5: Batching unit (bypass when BATCHING_EN=0) ------------------
  batching_unit #(
    .DATA_WIDTH   (DATA_WIDTH),
    .PKT_SIZE_W   (PKT_SIZE_W),
    .PKT_ID_W     (PKT_ID_W),
    .BATCH_COUNT  (BATCH_COUNT),
    .BATCH_TIMEOUT(BATCH_TIMEOUT)
  ) u_batch (
    .clk      (clk),
    .rst_n    (rst_n),
    .enable   (BATCHING_EN),
    .s_tvalid (fifo_m_tvalid),
    .s_tready (fifo_m_tready),
    .s_tdata  (fifo_m_tdata),
    .s_tkeep  (fifo_m_tkeep),
    .s_tlast  (fifo_m_tlast),
    .s_tuser  (fifo_m_tuser),
    .s_tid    (fifo_m_tid),
    .m_tvalid (batch_m_tvalid),
    .m_tready (batch_m_tready),
    .m_tdata  (batch_m_tdata),
    .m_tkeep  (batch_m_tkeep),
    .m_tlast  (batch_m_tlast),
    .m_tuser  (batch_m_tuser),
    .m_tid    (batch_m_tid)
  );

  // --- Stage 6: Stream sink -------------------------------------------------
  stream_sink #(
    .DATA_WIDTH (DATA_WIDTH),
    .PKT_SIZE_W (PKT_SIZE_W),
    .PKT_ID_W   (PKT_ID_W)
  ) u_sink (
    .clk       (clk),
    .rst_n     (rst_n),
    .s_tvalid  (batch_m_tvalid),
    .s_tready  (batch_m_tready),
    .s_tdata   (batch_m_tdata),
    .s_tkeep   (batch_m_tkeep),
    .s_tlast   (batch_m_tlast),
    .s_tuser   (batch_m_tuser),
    .s_tid     (batch_m_tid),
    .pkt_done  (sink_pkt_done),
    .done_id   (sink_done_id),
    .pkt_count (pkt_count)
  );

  // --- Stage 7: Latency monitor ---------------------------------------------
  latency_monitor #(
    .PKT_ID_W    (PKT_ID_W),
    .PKT_SIZE_W  (PKT_SIZE_W),
    .TIMESTAMP_W (32),
    .MAX_PACKETS (pcie_axi_pkg::MAX_PACKETS)
  ) u_latmon (
    .clk          (clk),
    .rst_n        (rst_n),
    .pkt_inject   (pkt_inject),
    .inject_id    (inject_id),
    .inject_size  (inject_size),
    .pkt_done     (sink_pkt_done),
    .done_id      (sink_done_id),
    .latency_out  (latency_out),
    .latency_valid(latency_valid),
    .latency_id   (latency_id)
  );

endmodule : pcie_axi_dma_top
