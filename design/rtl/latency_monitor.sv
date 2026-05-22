// =============================================================================
// latency_monitor.sv
//
// Hardware performance counter — measures per-packet end-to-end latency.
//
// ARCHITECTURE
//
//   Free-running cycle counter (TIMESTAMP_W bits, wraps after ~42 s @ 100 MHz)
//
//   Two storage tables indexed by pkt_id (0 .. MAX_PACKETS-1):
//     inject_time[id]   — cycle counter sampled when pkt_inject fires
//     size_store[id]    — packet byte-count sampled at the same injection
//     seen[id]          — 1 after inject event; guards against stale ids
//
//   When pkt_done fires (from stream_sink):
//     latency = cycle_cnt_at_done - inject_time[done_id]
//
// TIMING NOTE
//   pkt_inject and pkt_done are 1-cycle registered pulses.
//   Both cycle_cnt reads inside always_ff see the PRE-NBA (current-cycle) value,
//   so the computed latency = done_posedge_count - inject_posedge_count.
//   This matches the TB's tb_cycle measurements to within ±2 cycles
//   (constant offset from the inject-event setup delay in send_packet).
//
// OUTPUT INTERFACE
//   latency_out / latency_valid / latency_id are asserted for ONE cycle,
//   exactly one clock after pkt_done.  They can be used by:
//     • The testbench (Step 5-6 cross-check)
//     • A hypothetical hardware performance counter register
//
// SIMULATION OUTPUTS  (inside synthesis translate_off)
//   Per-packet:
//     [latmon] pkt_id=N  size=SB  latency=L cyc  eff=E B/cyc
//
//   End-of-simulation summary (final block):
//     [latmon] === LATENCY SUMMARY ===
//     [latmon]  Packets : N
//     [latmon]  Min     : X cyc
//     [latmon]  Max     : Y cyc
//     [latmon]  Avg     : Z.z cyc
//     [latmon]  Throughput: T MB/s  (at 100 MHz sim clock)
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module latency_monitor #(
  parameter int PKT_ID_W    = pcie_axi_pkg::PKT_ID_W,
  parameter int PKT_SIZE_W  = pcie_axi_pkg::PKT_SIZE_W,
  parameter int TIMESTAMP_W = pcie_axi_pkg::TIMESTAMP_WIDTH,
  parameter int MAX_PACKETS = pcie_axi_pkg::MAX_PACKETS
) (
  input  logic                    clk,
  input  logic                    rst_n,

  // ---- Injection event (from testbench, coincides with first gen_tvalid) ---
  input  logic                    pkt_inject,
  input  logic [PKT_ID_W-1:0]     inject_id,
  input  logic [PKT_SIZE_W-1:0]   inject_size,

  // ---- Completion event (from stream_sink, one cycle after tlast accepted) -
  input  logic                    pkt_done,
  input  logic [PKT_ID_W-1:0]     done_id,

  // ---- Registered result output (valid one cycle after pkt_done) -----------
  output logic [TIMESTAMP_W-1:0]  latency_out,
  output logic                    latency_valid,
  output logic [PKT_ID_W-1:0]     latency_id
);

  // ---------------------------------------------------------------------------
  // Free-running cycle counter
  // At posedge N (inside always_ff), cycle_cnt reads as N-1 (pre-NBA value).
  // Both injection and completion timestamps use the same reference so the
  // computed difference correctly equals (done_posedge - inject_posedge).
  // ---------------------------------------------------------------------------
  logic [TIMESTAMP_W-1:0] cycle_cnt;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) cycle_cnt <= '0;
    else        cycle_cnt <= cycle_cnt + TIMESTAMP_W'(1);
  end

  // ---------------------------------------------------------------------------
  // Injection timestamp & size storage (indexed by packet ID)
  // ---------------------------------------------------------------------------
  logic [TIMESTAMP_W-1:0] inject_time [0:MAX_PACKETS-1];
  logic [PKT_SIZE_W-1:0]  size_store  [0:MAX_PACKETS-1];
  logic                   seen        [0:MAX_PACKETS-1];   // valid flag

  integer init_k;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (init_k = 0; init_k < MAX_PACKETS; init_k++) begin
        inject_time[init_k] <= '0;
        size_store [init_k] <= '0;
        seen       [init_k] <= 1'b0;
      end
    end else if (pkt_inject) begin
      inject_time[inject_id] <= cycle_cnt;   // cycle_cnt read = inject posedge index
      size_store [inject_id] <= inject_size;
      seen       [inject_id] <= 1'b1;
    end
  end

  // ---------------------------------------------------------------------------
  // Latency computation + output registers
  //
  // lat_comb: wire driven by current cycle_cnt and inject_time[done_id].
  // Inside the always_ff below, lat_comb reads the PRE-NBA active-region
  // values → cycle_cnt = done_posedge_index, inject_time = stored inject index
  // → lat = done - inject (exact cycle count between the two events).
  // ---------------------------------------------------------------------------
  wire [TIMESTAMP_W-1:0] lat_comb = cycle_cnt - inject_time[done_id];

  // Summary accumulators (simulation only — wrap in translate_off later)
  logic [TIMESTAMP_W-1:0] stat_min;
  logic [TIMESTAMP_W-1:0] stat_max;
  logic [63:0]            stat_sum_bytes;   // total bytes, for throughput
  logic [63:0]            stat_sum_lat;     // sum of latencies (for avg)
  logic [31:0]            stat_count;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      latency_out   <= '0;
      latency_valid <= 1'b0;
      latency_id    <= '0;
      stat_min      <= {TIMESTAMP_W{1'b1}};  // initialise to max value
      stat_max      <= '0;
      stat_sum_bytes<= '0;
      stat_sum_lat  <= '0;
      stat_count    <= '0;
    end else begin
      latency_valid <= 1'b0;    // default: deassert

      if (pkt_done && seen[done_id]) begin
        // Latch result
        latency_out   <= lat_comb;
        latency_valid <= 1'b1;
        latency_id    <= done_id;

        // Update running stats
        if (lat_comb < stat_min)  stat_min <= lat_comb;
        if (lat_comb > stat_max)  stat_max <= lat_comb;
        stat_sum_lat   <= stat_sum_lat   + 64'(lat_comb);
        stat_sum_bytes <= stat_sum_bytes + 64'(size_store[done_id]);
        stat_count     <= stat_count + 32'd1;
      end
    end
  end

  // ---------------------------------------------------------------------------
  // Simulation: per-packet $display and end-of-sim summary
  // ---------------------------------------------------------------------------
  // synthesis translate_off

  always_ff @(posedge clk) begin
    if (pkt_done && seen[done_id]) begin
      $display("[latmon] pkt_id=%0d  size=%4dB  latency=%5d cyc  eff=%5.2f B/cyc",
               done_id,
               size_store[done_id],
               lat_comb,
               real'(size_store[done_id]) / real'(lat_comb));
    end
  end

  // Breakdown estimator: prints theoretical stage contributions alongside
  // measured latency.  Uses package parameters as a reference model.
  // (Measured may differ slightly due to state-transition overhead.)
  always_ff @(posedge clk) begin
    if (pkt_done && seen[done_id]) begin : breakdown
      automatic int sz    = int'(size_store[done_id]);
      automatic int dws   = (sz + 3) / 4;
      automatic int beats = (sz + (pcie_axi_pkg::DATA_WIDTH/8 - 1))
                            / (pcie_axi_pkg::DATA_WIDTH/8);
      automatic int pcie_est = pcie_axi_pkg::PCIE_BASE_LATENCY
                              + dws * pcie_axi_pkg::PCIE_PER_DW_LATENCY;
      automatic int dma_est  = pcie_axi_pkg::DMA_SETUP_CYCLES + beats;
      $display("[latmon]  ↳ est: PCIe=%0d  DMA=%0d  other=%0d  total_measured=%0d",
               pcie_est + beats,     // PCIe delay + pcie recv
               dma_est  + beats,     // DMA delay  + dma recv
               int'(lat_comb) - (pcie_est + beats) - (dma_est + beats),
               int'(lat_comb));
    end
  end

  // End-of-simulation summary (fires after $finish is called)
  final begin : summary
    if (stat_count > 0) begin
      $display("");
      $display("[latmon] ╔══════════════════════════════════════════╗");
      $display("[latmon] ║        LATENCY MONITOR SUMMARY           ║");
      $display("[latmon] ╠══════════════════════════════════════════╣");
      $display("[latmon] ║  Packets measured : %-4d                 ║", stat_count);
      $display("[latmon] ║  Min latency      : %-6d cycles          ║", stat_min);
      $display("[latmon] ║  Max latency      : %-6d cycles          ║", stat_max);
      $display("[latmon] ║  Avg latency      : %-8.1f cycles        ║",
               real'(stat_sum_lat) / real'(stat_count));
      $display("[latmon] ║  Total bytes      : %-8d bytes           ║", stat_sum_bytes);
      $display("[latmon] ║  Avg efficiency   : %-5.2f B/cyc         ║",
               real'(stat_sum_bytes) / real'(stat_sum_lat));
      // 100 MHz → each cycle = 10 ns → 1 cyc/B = 100 MB/s
      $display("[latmon] ║  Eff throughput   : %-6.1f MB/s (100MHz) ║",
               (real'(stat_sum_bytes) / real'(stat_sum_lat)) * 100.0);
      $display("[latmon] ╚══════════════════════════════════════════╝");
      $display("");
    end
  end

  // synthesis translate_on

endmodule : latency_monitor
