// =============================================================================
// pcie_axi_pkg.sv
//
// Shared parameters, types, and constants for the PCIe-AXI DMA streaming
// architecture.  Every RTL module and the testbench imports this package so
// that a single change here propagates everywhere.
//
// System Architecture (pipeline order):
//
//   [Packet Generator / TB]
//          |  gen_* (valid/ready/data/size/id/last)
//          v
//   [pcie_model]          -- adds BASE_LATENCY + ceil(size/4)*PER_DW_LAT cycles
//          |  pcie_* (AXI-S style + size/id sidechannel)
//          v
//   [dma_model]           -- adds SETUP_CYCLES descriptor delay + data transfer
//          |  dma_* (AXI-S style + size/id sidechannel)
//          v
//   [axi_mm2s_adapter]    -- re-packetizes MM burst words into AXI-Stream beats
//          |  AXIS (tvalid/tready/tdata/tkeep/tlast/tuser/tid)
//          v
//   [axis_fifo]           -- depth-parameterized store-and-forward buffer
//          |  AXIS
//          v
//   [batching_unit]       -- accumulates N small pkts, emits one large frame
//          |  AXIS                     (bypassed when BATCHING_EN=0)
//          v
//   [stream_sink]         -- absorbs output, drives latency_monitor signals
//          |  pkt_done / done_id
//          v
//   [latency_monitor]     -- records inject_cycle[id] and done_cycle[id],
//                            computes and prints latency per packet
//
// =============================================================================

`ifndef PCIE_AXI_PKG_SV
`define PCIE_AXI_PKG_SV

package pcie_axi_pkg;

  // ---------------------------------------------------------------------------
  // Data path widths
  // ---------------------------------------------------------------------------
  parameter int DATA_WIDTH   = 64;   // bits per AXI-S beat  (8 bytes)
  parameter int STRB_WIDTH   = DATA_WIDTH / 8;
  parameter int PKT_SIZE_W   = 13;   // max packet size field: 0-8191 bytes
  parameter int PKT_ID_W     = 8;    // packet ID / tag width (0-255 unique IDs)
  parameter int ADDR_WIDTH   = 32;   // AXI-MM address width

  // ---------------------------------------------------------------------------
  // PCIe model timing parameters
  // These mimic PCIe Gen3 x1 round-trip at 10 ns/cycle sim clock.
  // BASE_LATENCY  : fixed overhead per TLP (header processing, credit check)
  // PER_DW_LATENCY: additional cycles per 4-byte DWORD in the payload
  // ---------------------------------------------------------------------------
  parameter int PCIE_BASE_LATENCY  = 20;   // cycles
  parameter int PCIE_PER_DW_LATENCY = 1;   // cycles per DW

  // ---------------------------------------------------------------------------
  // DMA model timing parameters
  // SETUP_CYCLES  : descriptor fetch + doorbell overhead (fixed per packet)
  // BYTES_PER_CYC : effective DMA throughput (= DATA_WIDTH/8 at full rate)
  // ---------------------------------------------------------------------------
  parameter int DMA_SETUP_CYCLES  = 8;    // cycles
  parameter int DMA_BYTES_PER_CYC = 8;   // bytes/cycle (matches 64-bit bus)

  // ---------------------------------------------------------------------------
  // FIFO configuration
  // ---------------------------------------------------------------------------
  parameter int FIFO_DEPTH_DEFAULT = 64;  // entries (each = one 64-bit beat)

  // ---------------------------------------------------------------------------
  // Batching unit configuration
  // ---------------------------------------------------------------------------
  parameter int BATCH_COUNT_DEFAULT   = 4;   // packets per batch
  parameter int BATCH_TIMEOUT_DEFAULT = 64;  // flush after N idle cycles

  // ---------------------------------------------------------------------------
  // Stream sink back-pressure model (FIFO depth-sweep experiment)
  // Default is always-ready (SINK_STALL_EN=0), matching the original
  // "isolate DMA/PCIe latency from the consumer" design. Setting
  // SINK_STALL_EN=1 makes the sink accept SINK_STALL_READY cycles out of
  // every SINK_STALL_PERIOD cycles, which is the only way axis_fifo can
  // ever actually fill — needed to make FIFO_DEPTH visible in latency.
  // ---------------------------------------------------------------------------
  parameter bit SINK_STALL_EN_DEFAULT     = 1'b0;
  parameter int SINK_STALL_PERIOD_DEFAULT = 4;
  parameter int SINK_STALL_READY_DEFAULT  = 1;

  // ---------------------------------------------------------------------------
  // Latency monitor
  // ---------------------------------------------------------------------------
  parameter int TIMESTAMP_WIDTH = 32;    // cycle counter width
  parameter int MAX_PACKETS     = 256;   // max tracked in-flight packets

  // ---------------------------------------------------------------------------
  // Packet size constants used by testbench (bytes)
  // ---------------------------------------------------------------------------
  parameter int PKT_64B   =   64;
  parameter int PKT_128B  =  128;
  parameter int PKT_256B  =  256;
  parameter int PKT_512B  =  512;
  parameter int PKT_1KB   = 1024;
  parameter int PKT_4KB   = 4096;

  // ---------------------------------------------------------------------------
  // AXI-Stream sideband carried through the pipeline via tuser / tid
  //
  //   tuser[PKT_SIZE_W-1 : 0]  = packet byte-length (injected at source)
  //   tid  [PKT_ID_W-1   : 0]  = packet tag/ID      (injected at source)
  //
  // tkeep follows standard AXI-S rules (one bit per byte).
  // tlast is asserted on the final beat of each packet.
  // ---------------------------------------------------------------------------

  // Convenience: total sideband width bundled into tuser
  parameter int TUSER_WIDTH = PKT_SIZE_W;

  // ---------------------------------------------------------------------------
  // Derived helper function: compute number of 64-bit beats for a byte count
  // ---------------------------------------------------------------------------
  function automatic int beats_for_bytes(input int bytes);
    return (bytes + (DATA_WIDTH/8 - 1)) / (DATA_WIDTH/8);
  endfunction

endpackage : pcie_axi_pkg

`endif // PCIE_AXI_PKG_SV
