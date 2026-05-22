// =============================================================================
// axi_mm2s_adapter.sv
//
// AXI Memory-Mapped to AXI-Stream adapter — behavioral, zero-latency model.
//
// WHAT IT MODELS
//   In a real system the DMA engine issues AXI-MM read bursts (AR channel:
//   araddr, arlen, arburst; R channel: rdata, rresp, rlast) to fetch payload
//   bytes from local BRAM or DDR.  Those read-data responses are then packed
//   and serialised onto an AXI4-Stream bus that feeds the FIFO.
//
//   For this behavioural model the DMA has already pre-staged all payload
//   bytes, so the adapter's only real job is to:
//
//     1. Pass data straight through (zero pipeline stages).
//     2. Compute the correct tkeep mask for the FINAL beat of each packet.
//
//   tkeep for non-last beats  = all-ones (every byte lane is valid)
//   tkeep for the last beat   = (1 << remainder) - 1
//     where  remainder = pkt_size % STRB_W
//            if remainder == 0 the last beat is also full → all-ones
//
//   This tkeep correction is important downstream: the FIFO and batching
//   unit use it to compute byte-accurate output counts for the IEEE-paper
//   latency-vs-packet-size graphs.
//
// INTERFACE
//   Combinational pass-through: s_tready = m_tready (no internal state).
//   clk/rst_n are present for interface uniformity; unused by logic.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module axi_mm2s_adapter #(
  parameter int DATA_WIDTH = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W   = pcie_axi_pkg::PKT_ID_W
) (
  input  logic                    clk,     // unused — kept for interface uniformity
  input  logic                    rst_n,   // unused — kept for interface uniformity

  // ---- Slave port (from DMA model) -----------------------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,  // kept in case caller already correct
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,  // packet byte-length (const across beats)
  input  logic [PKT_ID_W-1:0]     s_tid,

  // ---- Master port (to axis_fifo) ------------------------------------------
  output logic                    m_tvalid,
  input  logic                    m_tready,
  output logic [DATA_WIDTH-1:0]   m_tdata,
  output logic [DATA_WIDTH/8-1:0] m_tkeep,
  output logic                    m_tlast,
  output logic [PKT_SIZE_W-1:0]   m_tuser,
  output logic [PKT_ID_W-1:0]     m_tid
);

  localparam int STRB_W = DATA_WIDTH / 8;

  // ---------------------------------------------------------------------------
  // tkeep computation for the last beat
  //
  //   remainder = pkt_size mod STRB_W   (how many bytes land in the last beat)
  //   remainder == 0 means the final beat is fully populated → all-ones tkeep
  //
  // Example with DATA_WIDTH=64 (STRB_W=8):
  //   64 B pkt → 8 full beats, remainder = 0 → tkeep_last = 8'hFF
  //  128 B pkt → 16 full beats, remainder = 0 → tkeep_last = 8'hFF
  //  130 B pkt → 16 full beats + 2 leftover → tkeep_last = 8'h03
  // ---------------------------------------------------------------------------
  logic [STRB_W-1:0]  tkeep_last;
  logic [PKT_SIZE_W-1:0] remainder;

  always_comb begin
    remainder  = s_tuser & PKT_SIZE_W'(STRB_W - 1);   // mod STRB_W (power-of-2)
    tkeep_last = (remainder == '0) ? {STRB_W{1'b1}}
                                   : {STRB_W{1'b0}} | ((STRB_W'(1) << remainder) - STRB_W'(1));
  end

  // ---------------------------------------------------------------------------
  // Pass-through wiring (zero latency, full combinational path)
  // ---------------------------------------------------------------------------
  assign s_tready = m_tready;
  assign m_tvalid = s_tvalid;
  assign m_tdata  = s_tdata;
  assign m_tkeep  = s_tlast ? tkeep_last : {STRB_W{1'b1}};
  assign m_tlast  = s_tlast;
  assign m_tuser  = s_tuser;
  assign m_tid    = s_tid;

  // ---------------------------------------------------------------------------
  // Simulation check: warn if tkeep supplied by upstream differs from computed
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (s_tvalid && s_tready && s_tlast && (s_tkeep != tkeep_last))
      $display("[mm2s] t=%0t  pkt_id=%0d  tkeep corrected: upstream=0x%0h  computed=0x%0h",
               $time, s_tid, s_tkeep, tkeep_last);
  end
  // synthesis translate_on

endmodule : axi_mm2s_adapter
