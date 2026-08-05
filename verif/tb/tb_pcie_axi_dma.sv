// =============================================================================
// tb_pcie_axi_dma.sv
//
// Top-level testbench for the PCIe-AXI DMA Streaming Architecture.
//
// TEST SCENARIOS (TB_SCENARIO parameter, override via Makefile)
//   0 = Baseline   : no batching, FIFO depth = FIFO_DEPTH
//   1 = Batching   : batching_unit enabled (Step 7), FIFO_DEPTH
//   2 = FIFO sweep : batching off, vary FIFO_DEPTH={16,64,256}
//
// PACKET SIZES TESTED: 64 B, 128 B, 256 B, 512 B, 1 KB, 4 KB
//
// LATENCY MEASUREMENT
//   All latency values in [RESULT] rows come from the hardware latency_monitor
//   (latency_out), not from a software tb_cycle difference.  This eliminates
//   the constant 2-cycle offset that arose from TB-side reference-point
//   misalignment, and ensures [RESULT] and [latmon] display identical numbers.
//
//   Flow:
//     1. pkt_inject pulse → latency_monitor records inject_time
//     2. TB waits for pkt_count to change (stream_sink accepted tlast)
//     3. TB waits one additional posedge for latency_valid to assert
//     4. TB reads latency_out and prints [RESULT]
//
// TIMING CONVENTION
//   All signal drives happen at #1 after posedge (setup margin).
//   gen_tready is sampled at @(posedge clk) BEFORE the #1 delay (pre-NBA
//   active region) — this is the correct AXI-S handshake check point.
//
// OUTPUT FORMAT
//   [RESULT]  PKT_ID | SIZE (B) | LATENCY (cyc) | EFFICIENCY (B/cyc) | CONFIG
//
// =============================================================================

`timescale 1ns/1ps
`include "../../design/rtl/pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module tb_pcie_axi_dma;

  // ---------------------------------------------------------------------------
  // Parameters — override with -generic_top on xelab command line
  // ---------------------------------------------------------------------------
  parameter int  TB_SCENARIO   = 0;
  parameter int  FIFO_DEPTH    = pcie_axi_pkg::FIFO_DEPTH_DEFAULT;
  parameter int  BATCH_COUNT   = pcie_axi_pkg::BATCH_COUNT_DEFAULT;
  parameter int  BATCH_TIMEOUT = pcie_axi_pkg::BATCH_TIMEOUT_DEFAULT;
  parameter bit  BATCHING_EN   = (TB_SCENARIO == 1);

  // Consumer back-pressure model — must be enabled for FIFO_DEPTH to have
  // any effect on latency (an always-ready sink never lets the FIFO fill).
  // Off by default so scenarios 0/1 and the un-stalled scenario 2 sweep are
  // unaffected. Override with SINK_STALL_EN=1 for the real depth-sweep run.
  parameter bit  SINK_STALL_EN     = pcie_axi_pkg::SINK_STALL_EN_DEFAULT;
  parameter int  SINK_STALL_PERIOD = pcie_axi_pkg::SINK_STALL_PERIOD_DEFAULT;
  parameter int  SINK_STALL_READY  = pcie_axi_pkg::SINK_STALL_READY_DEFAULT;

  // ---------------------------------------------------------------------------
  // Clock & reset
  // ---------------------------------------------------------------------------
  localparam real CLK_PERIOD_NS = 10.0;   // 100 MHz

  logic clk   = 1'b0;
  logic rst_n = 1'b0;

  always #(CLK_PERIOD_NS / 2.0) clk = ~clk;

  // ---------------------------------------------------------------------------
  // Local width aliases
  // ---------------------------------------------------------------------------
  localparam int DW   = pcie_axi_pkg::DATA_WIDTH;
  localparam int PSW  = pcie_axi_pkg::PKT_SIZE_W;
  localparam int IDW  = pcie_axi_pkg::PKT_ID_W;
  localparam int STRB = DW / 8;

  // ---------------------------------------------------------------------------
  // DUT port signals
  // ---------------------------------------------------------------------------
  logic              gen_tvalid;
  logic              gen_tready;
  logic [DW-1:0]     gen_tdata;
  logic [STRB-1:0]   gen_tkeep;
  logic              gen_tlast;
  logic [PSW-1:0]    gen_tuser;
  logic [IDW-1:0]    gen_tid;

  logic              pkt_inject;
  logic [IDW-1:0]    inject_id;
  logic [PSW-1:0]    inject_size;

  logic [31:0]       latency_out;
  logic              latency_valid;
  logic [IDW-1:0]    latency_id;

  logic [31:0]       pkt_count;
  logic [$clog2(FIFO_DEPTH):0] fifo_occupancy;   // sized to actual FIFO_DEPTH, not the package default
  logic              fifo_full;
  logic              fifo_empty;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  pcie_axi_dma_top #(
    .DATA_WIDTH      (DW),
    .PKT_SIZE_W      (PSW),
    .PKT_ID_W        (IDW),
    .FIFO_DEPTH      (FIFO_DEPTH),
    .BATCH_COUNT     (BATCH_COUNT),
    .BATCH_TIMEOUT   (BATCH_TIMEOUT),
    .BATCHING_EN     (BATCHING_EN),
    .SINK_STALL_EN     (SINK_STALL_EN),
    .SINK_STALL_PERIOD (SINK_STALL_PERIOD),
    .SINK_STALL_READY  (SINK_STALL_READY)
  ) dut (
    .clk           (clk),
    .rst_n         (rst_n),
    .gen_tvalid    (gen_tvalid),
    .gen_tready    (gen_tready),
    .gen_tdata     (gen_tdata),
    .gen_tkeep     (gen_tkeep),
    .gen_tlast     (gen_tlast),
    .gen_tuser     (gen_tuser),
    .gen_tid       (gen_tid),
    .pkt_inject    (pkt_inject),
    .inject_id     (inject_id),
    .inject_size   (inject_size),
    .latency_out   (latency_out),
    .latency_valid (latency_valid),
    .latency_id    (latency_id),
    .pkt_count     (pkt_count),
    .fifo_occupancy(fifo_occupancy),
    .fifo_full     (fifo_full),
    .fifo_empty    (fifo_empty)
  );

  // ---------------------------------------------------------------------------
  // Cycle counter — used for latency measurement
  // Increments at every posedge; reset clears to 0.
  // ---------------------------------------------------------------------------
  int tb_cycle;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) tb_cycle <= 0;
    else        tb_cycle <= tb_cycle + 1;
  end

  // ---------------------------------------------------------------------------
  // Packet sizes under test (bytes)
  // ---------------------------------------------------------------------------
  int pkt_sizes[6] = '{
    pcie_axi_pkg::PKT_64B,
    pcie_axi_pkg::PKT_128B,
    pcie_axi_pkg::PKT_256B,
    pcie_axi_pkg::PKT_512B,
    pcie_axi_pkg::PKT_1KB,
    pcie_axi_pkg::PKT_4KB
  };

  // ---------------------------------------------------------------------------
  // Config string (printed in each result row)
  // ---------------------------------------------------------------------------
  function automatic string cfg_string();
    string base;
    if (BATCHING_EN)
      base = $sformatf("BATCH=%0d,FIFO=%0d", BATCH_COUNT, FIFO_DEPTH);
    else
      base = $sformatf("BASELINE,FIFO=%0d", FIFO_DEPTH);
    if (SINK_STALL_EN)
      base = $sformatf("%s,STALL=%0d/%0d", base, SINK_STALL_READY, SINK_STALL_PERIOD);
    return base;
  endfunction

  // ---------------------------------------------------------------------------
  // FIFO stress tracking — peak occupancy and whether fifo_full was ever
  // asserted during the window between two reset points (set by the caller
  // right before injecting the packet(s) being measured). Only meaningful
  // when SINK_STALL_EN=1; with an always-ready sink these will always read
  // back near-zero / never-full regardless of FIFO_DEPTH (see stream_sink.sv).
  // ---------------------------------------------------------------------------
  logic [$clog2(FIFO_DEPTH):0] peak_occ;
  logic                        fifo_full_seen;

  always @(posedge clk) begin
    if (fifo_occupancy > peak_occ) peak_occ = fifo_occupancy;
    if (fifo_full)                 fifo_full_seen = 1'b1;
  end

  task automatic reset_occ_tracking();
    peak_occ       = '0;
    fifo_full_seen = 1'b0;
  endtask

  // ===========================================================================
  // Task: send_packet
  //
  // Drives one complete AXI-S packet (all beats) into the DUT.
  //
  // HANDSHAKE TIMING (critical for last beat):
  //   gen_tready is checked at @(posedge clk) in the ACTIVE region, BEFORE
  //   the DUT's NBA assignments fire.  At this moment:
  //     - pcie_model.state is still S_RECV (hasn't transitioned yet)
  //     - gen_tready = 1 for ALL beats including the last one
  //   After the #1 delay, for the last beat gen_tready drops to 0 (pcie now
  //   in S_DELAY).  We deassert gen_tvalid at #1 to prevent a spurious accept
  //   when pcie eventually returns to S_RECV.
  //
  // The pkt_inject pulse fires for exactly ONE clock cycle: the cycle in which
  // the first beat of the packet is presented to the DUT input.
  // ===========================================================================
  task automatic send_packet(
    input int pkt_id,
    input int pkt_size_bytes
  );
    int beats;
    int last_bytes;
    logic [STRB-1:0] keep_last;
    int b;

    beats      = (pkt_size_bytes + STRB - 1) / STRB;
    last_bytes = pkt_size_bytes % STRB;
    keep_last  = (last_bytes == 0) ? {STRB{1'b1}}
                                   : {STRB{1'b0}} | ((STRB'(1) << last_bytes) - STRB'(1));

    // --- Setup pkt_inject one cycle before the loop starts -----------------
    @(posedge clk); #1;
    pkt_inject  = 1'b1;
    inject_id   = IDW'(pkt_id);
    inject_size = PSW'(pkt_size_bytes);

    // --- Drive all beats ----------------------------------------------------
    for (b = 0; b < beats; b++) begin
      gen_tvalid = 1'b1;
      gen_tdata  = {$urandom(), $urandom()};
      gen_tkeep  = (b == beats-1) ? keep_last : {STRB{1'b1}};
      gen_tlast  = (b == beats-1);
      gen_tuser  = PSW'(pkt_size_bytes);
      gen_tid    = IDW'(pkt_id);

      // Wait for DUT to accept this beat.
      // Checked at @(posedge clk) — ACTIVE region, pre-NBA.
      // gen_tready = (pcie_model.state == S_RECV) using OLD state value.
      // This is 1 for ALL beats (including last) since S_RECV hasn't
      // transitioned to S_DELAY in the NBA region yet.
      do begin @(posedge clk); end while (!gen_tready);

      // Now in active region: beat b was just accepted.
      #1;  // wait for NBA to settle

      pkt_inject = 1'b0;   // deassert after first clock (harmless on later beats)

      if (b == beats - 1) begin
        // Last beat accepted; deassert valid before pcie returns to S_RECV
        // (avoids spurious beat when pcie exits S_DELAY → S_RECV)
        gen_tvalid = 1'b0;
        gen_tlast  = 1'b0;
      end
    end
  endtask

  // ===========================================================================
  // Module-level variables for the main test loop
  // ===========================================================================
  int prev_count;
  int timeout_cnt;

  // ===========================================================================
  // Main test sequence
  // ===========================================================================
  initial begin : main_test

    // ---- Initialise all driven signals to safe defaults --------------------
    gen_tvalid  = 1'b0;
    gen_tdata   = '0;
    gen_tkeep   = '0;
    gen_tlast   = 1'b0;
    gen_tuser   = '0;
    gen_tid     = '0;
    pkt_inject  = 1'b0;
    inject_id   = '0;
    inject_size = '0;

    // ---- Reset sequence ----------------------------------------------------
    rst_n = 1'b0;
    repeat (10) @(posedge clk);
    rst_n = 1'b1;
    repeat (5)  @(posedge clk);

    // ---- Print result table header -----------------------------------------
    $display("");
    $display("[RESULT] ================================================================");
    $display("[RESULT]  PCIe-AXI DMA Latency Simulation  |  Config: %s", cfg_string());
    $display("[RESULT] ================================================================");
    $display("[RESULT]  PKT_ID | SIZE (B) | LATENCY (cyc) | EFFICIENCY (B/cyc) | FIFO_PEAK | FULL?");
    $display("[RESULT] ----------------------------------------------------------------");

    // ====================================================================
    // Scenario 0/2: one packet per size, wait for each completion
    // Scenario 1:   BATCH_COUNT × 64B back-to-back, wait for one batch
    // ====================================================================

    if (TB_SCENARIO != 1) begin

      for (int i = 0; i < 6; i++) begin

        prev_count = pkt_count;
        reset_occ_tracking();

        // Drive the packet into the pipeline.
        // pkt_inject fires inside send_packet; latency_monitor latches inject_time.
        send_packet(i, pkt_sizes[i]);

        // ----------------------------------------------------------------
        // Wait for stream_sink to accept the final beat (pkt_count changes).
        // Timeout covers worst-case 4 KB pipeline delay (~3000 cycles).
        // ----------------------------------------------------------------
        timeout_cnt = 0;
        do begin
          @(posedge clk);
          timeout_cnt++;
        end while ((pkt_count == prev_count) && (timeout_cnt < 15000));

        if (pkt_count == prev_count) begin
          $error("[TB] Timeout: pkt_id=%0d  size=%0dB  never completed", i, pkt_sizes[i]);
          $finish;
        end

        // Wait one more posedge: latency_monitor registers latency_out one cycle
        // after pkt_done (which itself is one cycle after stream_sink sees tlast).
        @(posedge clk);

        // Print result row using hardware-measured latency (latency_out).
        // This matches [latmon] exactly — no tb_cycle offset.
        $display("[RESULT]  %6d | %8d | %13d | %18.2f | %9d | %5s | %s",
                 i,
                 pkt_sizes[i],
                 latency_out,
                 real'(pkt_sizes[i]) / real'(latency_out),
                 peak_occ,
                 fifo_full_seen ? "YES" : "no",
                 cfg_string());

        // Brief gap: let pipeline fully return to idle before next packet.
        repeat (8) @(posedge clk);
      end

    end else begin

      // ====================================================================
      // Scenario 1: Batching experiment
      //
      // Send BATCH_COUNT × 64B packets sequentially into the pipeline.
      // The batching_unit accumulates them (inter-packet gaps are below
      // BATCH_TIMEOUT) and emits a single m_tlast for the whole batch.
      // stream_sink increments pkt_count exactly ONCE for the batch.
      //
      // Latency is measured from first inject_cyc to that one pkt_count
      // increment — same methodology as Scenario 0 (constant ±2 cyc offset).
      // ====================================================================
      $display("[RESULT]  Batching %0d × 64B packets into one frame", BATCH_COUNT);
      $display("[RESULT] ----------------------------------------------------------------");
      $display("[RESULT]  %6s | %8s | %13s | %18s | %9s | %5s | %s",
               "BATCH#", "SIZE (B)", "LATENCY (cyc)", "EFFICIENCY (B/cyc)", "FIFO_PEAK", "FULL?", "CONFIG");
      $display("[RESULT] ................................................................");

      prev_count = pkt_count;
      reset_occ_tracking();

      // Inject all BATCH_COUNT × 64B packets.  pkt_inject fires for each
      // individual packet; latency_monitor records inject_time for pkt_id=0
      // (first packet). pcie_model serialises them so inter-packet gaps at
      // the batching_unit are ~43 cycles < BATCH_TIMEOUT.
      for (int i = 0; i < BATCH_COUNT; i++) begin
        send_packet(i, pcie_axi_pkg::PKT_64B);
      end

      // Wait for the single batch-level completion (one pkt_count increment).
      // Allow extra timeout margin: 4 packets × ~300 cyc/pkt + overhead.
      timeout_cnt = 0;
      do begin
        @(posedge clk);
        timeout_cnt++;
      end while ((pkt_count == prev_count) && (timeout_cnt < 30000));

      if (pkt_count == prev_count) begin
        $error("[TB] Timeout: batch of %0d × 64B never completed", BATCH_COUNT);
        $finish;
      end

      // Wait one posedge for latency_monitor to register latency_out.
      @(posedge clk);

      // Batch result row — SIZE is total batch bytes; latency from hardware monitor.
      $display("[RESULT]  %6d | %8d | %13d | %18.2f | %9d | %5s | %s",
               0,
               BATCH_COUNT * pcie_axi_pkg::PKT_64B,
               latency_out,
               real'(BATCH_COUNT * pcie_axi_pkg::PKT_64B) / real'(latency_out),
               peak_occ,
               fifo_full_seen ? "YES" : "no",
               cfg_string());

      repeat (8) @(posedge clk);

    end

    // ---- Summary footer ----------------------------------------------------
    $display("[RESULT] ----------------------------------------------------------------");
    $display("[RESULT]  Total packets received: %0d", pkt_count);
    $display("[RESULT] ================================================================");
    $display("");

    repeat (20) @(posedge clk);
    $finish;
  end

  // ---------------------------------------------------------------------------
  // Global watchdog — prevents infinite simulation on unexpected hang
  // ---------------------------------------------------------------------------
  initial begin : watchdog
    // 4 KB pipeline + some margin: 3000 × 6 + overhead ≈ 30 000 cycles
    // At 10 ns/cycle → 400 µs
    #4_000_000;
    $display("[ERROR] Watchdog fired — simulation hung at %0t ns", $time / 1000);
    $finish;
  end

endmodule : tb_pcie_axi_dma
