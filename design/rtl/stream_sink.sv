// =============================================================================
// stream_sink.sv
//
// AXI-Stream sink — always-ready consumer, drives latency completion signals.
//
// WHAT IT MODELS
//   The endpoint consumer of the DMA stream (e.g., a PCIe device receive
//   buffer, a network MAC, or a host-side DMA write target).  In a real
//   design this destination imposes back-pressure; here s_tready is
//   permanently asserted so the sink never stalls the pipeline.
//   This isolates latency to the DMA/PCIe path rather than the consumer.
//
// COMPLETION SIGNALLING
//   On every cycle where s_tvalid=1 and s_tlast=1 (last beat of a packet):
//     pkt_done  ← 1-cycle pulse
//     done_id   ← s_tid of the completing packet
//     pkt_count ← increments (running total for TB termination check)
//
//   The latency_monitor (Step 6) uses pkt_done + done_id to compute
//   end-to-end latency by subtracting the stored injection timestamp.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module stream_sink #(
  parameter int DATA_WIDTH = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W   = pcie_axi_pkg::PKT_ID_W
) (
  input  logic                    clk,
  input  logic                    rst_n,

  // ---- Slave port (from batching_unit or axis_fifo) ------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,   // permanently 1
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,    // pkt_size in bytes
  input  logic [PKT_ID_W-1:0]     s_tid,      // packet tag

  // ---- Completion signals (to latency_monitor) ----------------------------
  output logic                    pkt_done,   // 1-cycle pulse at last beat
  output logic [PKT_ID_W-1:0]     done_id,    // tag of completing packet

  // ---- Diagnostics ---------------------------------------------------------
  output logic [31:0]             pkt_count   // running count of received pkts
);

  // ---------------------------------------------------------------------------
  // Always-ready: sink never back-pressures the pipeline
  // ---------------------------------------------------------------------------
  assign s_tready = 1'b1;

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
      pkt_count <= 32'd0;
    end else begin
      pkt_done <= 1'b0;    // default: deassert each cycle

      // s_tready is permanently 1, so qualification simplifies to s_tvalid & s_tlast
      if (s_tvalid && s_tlast) begin
        pkt_done  <= 1'b1;
        done_id   <= s_tid;
        pkt_count <= pkt_count + 32'd1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Simulation diagnostics
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (s_tvalid && s_tlast)
      $display("[sink]  t=%0t  pkt_id=%0d  size=%0dB  count=%0d",
               $time, s_tid, s_tuser, pkt_count + 1);
  end
  // synthesis translate_on

endmodule : stream_sink
