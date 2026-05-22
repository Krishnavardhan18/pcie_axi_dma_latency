// =============================================================================
// batching_unit.sv
//
// Small-packet batching optimization module.
//
// WHAT IT MODELS
//   One of the dominant sources of PCIe inefficiency for small transfers is
//   per-packet header overhead: a 64 B payload still pays the same TLP
//   header cost as a 4 KB payload.  Batching N small packets into one
//   larger frame amortises that overhead N-fold.
//
// ONLINE PASS-THROUGH DESIGN
//   Beats flow from slave to master with zero buffering — only the tlast
//   signal is modified.  Intermediate tlast pulses (inner-packet boundaries)
//   are suppressed; a single m_tlast is asserted when either:
//     (a) BATCH_COUNT packets have been seen (pkt_cnt reaches BATCH_COUNT-1)
//     (b) The idle-cycle timeout fires and the next tlast arrives (flush)
//   m_tuser carries the total accumulated byte count of the batch.
//   m_tid  carries the ID of the first packet in the batch so that the
//   latency_monitor attributes the entire frame to the first injection.
//
// BYPASS MODE (enable = 0)
//   Zero-latency combinational pass-through: all s_* wired directly to m_*.
//   Used for the baseline (no-batching) scenario so that a single Makefile
//   parameter produces a fair A/B comparison.
//
// TIMEOUT FLUSH
//   When batch_active is set but no beats arrive for BATCH_TIMEOUT consecutive
//   cycles, flush_pending is asserted.  The next accepted tlast then emits
//   m_tlast regardless of how many packets have accumulated.
//
// LIMITATION
//   If the TB sends fewer than BATCH_COUNT packets and then stops forever,
//   the final partial batch will hang (no incoming tlast to trigger the
//   flush emission).  In the thesis scenario, BATCH_COUNT packets are always
//   injected, so this corner case does not arise.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module batching_unit #(
  parameter int DATA_WIDTH    = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W    = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W      = pcie_axi_pkg::PKT_ID_W,
  parameter int BATCH_COUNT   = pcie_axi_pkg::BATCH_COUNT_DEFAULT,
  parameter int BATCH_TIMEOUT = pcie_axi_pkg::BATCH_TIMEOUT_DEFAULT
) (
  input  logic                    clk,
  input  logic                    rst_n,
  input  logic                    enable,    // 1 = batch mode, 0 = bypass

  // ---- Slave port (from axis_fifo) -----------------------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,
  input  logic [PKT_ID_W-1:0]     s_tid,

  // ---- Master port (to stream_sink) ----------------------------------------
  output logic                    m_tvalid,
  input  logic                    m_tready,
  output logic [DATA_WIDTH-1:0]   m_tdata,
  output logic [DATA_WIDTH/8-1:0] m_tkeep,
  output logic                    m_tlast,
  output logic [PKT_SIZE_W-1:0]   m_tuser,  // total batch byte count
  output logic [PKT_ID_W-1:0]     m_tid     // first-packet ID in batch
);

  // ---------------------------------------------------------------------------
  // Internal counter widths
  //   PCNT_W  : holds 0 .. BATCH_COUNT-1 (pkt_cnt)
  //   ICNT_W  : holds 0 .. BATCH_TIMEOUT-1 (idle_cnt)
  //   BSIZE_W : accumulates up to (BATCH_COUNT-1) × max_pkt_bytes
  // ---------------------------------------------------------------------------
  localparam int PCNT_W  = $clog2(BATCH_COUNT  + 1);
  localparam int ICNT_W  = $clog2(BATCH_TIMEOUT + 1);
  localparam int BSIZE_W = PKT_SIZE_W + $clog2(BATCH_COUNT + 1);

  // ---------------------------------------------------------------------------
  // Batch state registers
  // ---------------------------------------------------------------------------
  logic [PCNT_W-1:0]   pkt_cnt;       // completed inner-packet count in batch
  logic [BSIZE_W-1:0]  batch_bytes;   // sum of bytes from completed inner pkts
  logic [PKT_ID_W-1:0] first_id;      // ID of first packet in current batch
  logic                batch_active;  // 1 after first beat of new batch
  logic [ICNT_W-1:0]   idle_cnt;      // consecutive idle cycles while active
  logic                flush_pending; // 1 = timeout fired, flush on next tlast

  // ---------------------------------------------------------------------------
  // Combinational helpers
  // ---------------------------------------------------------------------------

  // A beat is being transferred right now (s_tready == m_tready in both modes)
  wire beat_accepted = s_tvalid && m_tready;

  // This tlast closes the current batch: BATCH_COUNT-th packet, or flush
  wire close_batch = enable && beat_accepted && s_tlast &&
                     ((pkt_cnt == PCNT_W'(BATCH_COUNT - 1)) || flush_pending);

  // ---------------------------------------------------------------------------
  // Batch state machine
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pkt_cnt       <= '0;
      batch_bytes   <= '0;
      first_id      <= '0;
      batch_active  <= 1'b0;
      idle_cnt      <= '0;
      flush_pending <= 1'b0;
    end else begin

      if (close_batch) begin
        // ---- Batch closed: reset all state for the next batch ---------------
        pkt_cnt       <= '0;
        batch_bytes   <= '0;
        first_id      <= '0;
        batch_active  <= 1'b0;
        idle_cnt      <= '0;
        flush_pending <= 1'b0;

      end else if (beat_accepted) begin
        // ---- Beat flowing through -------------------------------------------
        idle_cnt <= '0;          // reset idle timer on any activity

        if (!batch_active) begin
          first_id     <= s_tid; // latch first packet's ID
          batch_active <= 1'b1;
        end

        if (s_tlast) begin
          // Intermediate packet boundary: accumulate; suppress tlast in outputs
          pkt_cnt     <= pkt_cnt     + PCNT_W'(1);
          batch_bytes <= batch_bytes + BSIZE_W'(s_tuser);
        end

      end else if (batch_active) begin
        // ---- Batch in progress, no beats: run idle timeout ------------------
        if (!flush_pending) begin
          if (idle_cnt == ICNT_W'(BATCH_TIMEOUT - 1))
            flush_pending <= 1'b1;
          else
            idle_cnt <= idle_cnt + ICNT_W'(1);
        end
      end

    end
  end

  // ---------------------------------------------------------------------------
  // Output mux
  //
  // enable=0 (bypass): all outputs are direct wires from slave port.
  // enable=1 (batch) : data/valid/ready pass through; tlast, tuser, tid
  //                    are transformed.
  //
  // m_tuser note: computed as batch_bytes (previous pkts) + s_tuser (current
  // pkt).  This is only architecturally meaningful at m_tlast=1 (when
  // stream_sink reads it).  For non-tlast beats the value is ignored.
  // The sum is truncated to PKT_SIZE_W bits; for the thesis batch sizes
  // (max 4 × 64 B = 256 B) this is well within range.
  // ---------------------------------------------------------------------------
  assign s_tready = m_tready;
  assign m_tvalid = s_tvalid;
  assign m_tdata  = s_tdata;
  assign m_tkeep  = s_tkeep;
  assign m_tlast  = enable ? close_batch                                      : s_tlast;
  assign m_tuser  = enable ? PKT_SIZE_W'(batch_bytes + BSIZE_W'(s_tuser))    : s_tuser;
  assign m_tid    = enable ? first_id                                         : s_tid;

  // ---------------------------------------------------------------------------
  // Simulation diagnostics
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  initial begin
    if (enable)
      $display("[batching_unit] Batch mode  BATCH_COUNT=%0d  TIMEOUT=%0d",
               BATCH_COUNT, BATCH_TIMEOUT);
    else
      $display("[batching_unit] Bypass mode (enable=0)");
  end

  always_ff @(posedge clk) begin
    if (close_batch)
      $display("[batmon]  t=%0t  batch CLOSED  pkts=%0d  total_bytes=%0d  first_id=%0d",
               $time,
               pkt_cnt + 32'd1,
               batch_bytes + BSIZE_W'(s_tuser),
               first_id);
    else if (enable && beat_accepted && s_tlast)
      $display("[batmon]  t=%0t  inner tlast suppressed  pkt_cnt=%0d→%0d  bytes_so_far=%0d",
               $time, pkt_cnt, pkt_cnt + 32'd1,
               batch_bytes + BSIZE_W'(s_tuser));
  end

  always_ff @(posedge clk) begin
    if (enable && !flush_pending && batch_active && !beat_accepted) begin
      if (idle_cnt == ICNT_W'(BATCH_TIMEOUT - 1))
        $display("[batmon]  t=%0t  TIMEOUT fired after %0d idle cycles — flush on next tlast",
                 $time, BATCH_TIMEOUT);
    end
  end
  // synthesis translate_on

endmodule : batching_unit
