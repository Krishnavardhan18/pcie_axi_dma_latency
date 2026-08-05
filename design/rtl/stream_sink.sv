// =============================================================================
// stream_sink.sv
//
// AXI-Stream sink — always-ready consumer, drives latency completion signals.
//
// WHAT IT MODELS
//   The endpoint consumer of the DMA stream (e.g., a PCIe device receive
//   buffer, a network MAC, or a host-side DMA write target).  In a real
//   design this destination imposes back-pressure; by default (STALL_EN=0)
//   s_tready is permanently asserted so the sink never stalls the pipeline,
//   isolating latency to the DMA/PCIe path rather than the consumer.
//
//   Optional back-pressure model (STALL_EN=1): the sink accepts STALL_READY
//   cycles out of every STALL_PERIOD cycles. This is the only way axis_fifo
//   can ever actually accumulate occupancy — with an always-ready sink the
//   FIFO drains every cycle it holds data (FWFT), so FIFO_DEPTH can never
//   affect latency. Enable this to make the FIFO depth-sweep experiment
//   (Step 4 axis_fifo) actually exercise fifo_full.
//
// COMPLETION SIGNALLING
//   On every cycle where s_tvalid=1, s_tready=1, and s_tlast=1 (the beat is
//   actually transferred, and it's the last beat of a packet):
//     pkt_done  ← 1-cycle pulse
//     done_id   ← s_tid of the completing packet/batch (first-packet ID in batch mode)
//     done_size ← s_tuser byte count at tlast (= total batch bytes in batch mode)
//     pkt_count ← increments (running total for TB termination check)
//
//   The latency_monitor uses pkt_done + done_id + done_size to compute
//   end-to-end latency and correct efficiency for both single-packet and
//   batched frames.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module stream_sink #(
  parameter int DATA_WIDTH   = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W   = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W     = pcie_axi_pkg::PKT_ID_W,

  // ---- Optional consumer back-pressure model (see header) ------------------
  parameter bit STALL_EN     = pcie_axi_pkg::SINK_STALL_EN_DEFAULT,
  parameter int STALL_PERIOD = pcie_axi_pkg::SINK_STALL_PERIOD_DEFAULT,
  parameter int STALL_READY  = pcie_axi_pkg::SINK_STALL_READY_DEFAULT
) (
  input  logic                    clk,
  input  logic                    rst_n,

  // ---- Slave port (from batching_unit or axis_fifo) ------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,   // permanently 1 unless STALL_EN=1
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,    // pkt_size in bytes
  input  logic [PKT_ID_W-1:0]     s_tid,      // packet tag

  // ---- Completion signals (to latency_monitor) ----------------------------
  output logic                    pkt_done,              // 1-cycle pulse at last beat
  output logic [PKT_ID_W-1:0]     done_id,               // tag of completing packet/batch
  output logic [PKT_SIZE_W-1:0]   done_size,             // actual byte count (batch-aware)

  // ---- Diagnostics ---------------------------------------------------------
  output logic [31:0]             pkt_count   // running count of received pkts
);

  // ---------------------------------------------------------------------------
  // Always-ready by default (STALL_EN=0): sink never back-pressures the
  // pipeline. With STALL_EN=1, accept STALL_READY cycles out of every
  // STALL_PERIOD cycles — models a slow/rate-limited consumer so the
  // upstream FIFO can actually accumulate occupancy.
  // ---------------------------------------------------------------------------
  generate
    if (STALL_EN) begin : g_stall
      localparam int CNT_W = $clog2(STALL_PERIOD);
      logic [CNT_W-1:0] stall_cnt;

      always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
          stall_cnt <= '0;
        else if (stall_cnt == CNT_W'(STALL_PERIOD - 1))
          stall_cnt <= '0;
        else
          stall_cnt <= stall_cnt + CNT_W'(1);
      end

      assign s_tready = (stall_cnt < CNT_W'(STALL_READY));
    end else begin : g_no_stall
      assign s_tready = 1'b1;
    end
  endgenerate

  // ---------------------------------------------------------------------------
  // Completion detection — registered on posedge
  //
  // Registering ensures pkt_done is a clean 1-cycle pulse with no glitches.
  // The latency_monitor (Step 6) captures it with a single always_ff.
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pkt_done  <= 1'b0;
      done_id   <= '0;
      done_size <= '0;
      pkt_count <= 32'd0;
    end else begin
      pkt_done <= 1'b0;    // default: deassert each cycle

      // Proper AXI-S handshake: a beat only transfers when tvalid && tready.
      // With STALL_EN=0, s_tready is permanently 1, so this simplifies back
      // to the original s_tvalid && s_tlast check. With STALL_EN=1, tvalid/
      // tlast can stay asserted for several cycles while waiting for
      // tready — omitting the s_tready qualifier would fire pkt_done once
      // per waiting cycle instead of once per actual transfer.
      if (s_tvalid && s_tready && s_tlast) begin
        pkt_done  <= 1'b1;
        done_id   <= s_tid;
        done_size <= s_tuser;   // captures total batch bytes in batch mode
        pkt_count <= pkt_count + 32'd1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Simulation diagnostics
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (s_tvalid && s_tready && s_tlast)
      $display("[sink]  t=%0t  pkt_id=%0d  size=%0dB  count=%0d",
               $time, s_tid, s_tuser, pkt_count + 1);
  end
  // synthesis translate_on

endmodule : stream_sink
