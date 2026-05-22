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
// LATENCY MEASUREMENT (Steps 5 & 6)
//   Step 5 (this step): TB measures latency directly using tb_cycle counter:
//     inject_cyc = tb_cycle when gen_tvalid first presented to pcie_model
//     done_cyc   = tb_cycle when pkt_count increments (stream_sink accepted tlast)
//     latency    = done_cyc - inject_cyc
//   Step 6: latency_monitor takes over (same numbers, with per-stage breakdown)
//
// TIMING CONVENTION
//   All signal drives happen at #1 after posedge (setup margin).
//   gen_tready is sampled at @(posedge clk) BEFORE the #1 delay (pre-NBA
//   active region) — this is the correct AXI-S handshake check point.
//   Measured latency has a constant ±2 cycle offset vs true latency;
//   this is negligible for the thesis trend analysis.
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
  logic [$clog2(pcie_axi_pkg::FIFO_DEPTH_DEFAULT):0] fifo_occupancy;
  logic              fifo_full;
  logic              fifo_empty;

  // ---------------------------------------------------------------------------
  // DUT
  // ---------------------------------------------------------------------------
  pcie_axi_dma_top #(
    .DATA_WIDTH    (DW),
    .PKT_SIZE_W    (PSW),
    .PKT_ID_W      (IDW),
    .FIFO_DEPTH    (FIFO_DEPTH),
    .BATCH_COUNT   (BATCH_COUNT),
    .BATCH_TIMEOUT (BATCH_TIMEOUT),
    .BATCHING_EN   (BATCHING_EN)
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
    if (BATCHING_EN)
      return $sformatf("BATCH=%0d,FIFO=%0d", BATCH_COUNT, FIFO_DEPTH);
    else
      return $sformatf("BASELINE,FIFO=%0d", FIFO_DEPTH);
  endfunction

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
  int inject_cyc;
  int done_cyc;
  int measured_lat;
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
    $display("[RESULT]  PKT_ID | SIZE (B) | LATENCY (cyc) | EFFICIENCY (B/cyc)");
    $display("[RESULT] ----------------------------------------------------------------");

    // ====================================================================
    // Scenario 0/2: one packet per size, wait for each completion
    // Scenario 1:   BATCH_COUNT × 64B back-to-back, wait for one batch
    // ====================================================================

    if (TB_SCENARIO != 1) begin

      for (int i = 0; i < 6; i++) begin

        // Record inject cycle BEFORE driving gen_tvalid.
        // One cycle early vs. true first-beat acceptance; error is constant
        // across all packets so trends are accurate.
        inject_cyc = tb_cycle;
        prev_count = pkt_count;

        // Drive the packet into the pipeline
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

        // Record completion cycle (tb_cycle incremented at the posedge where
        // pkt_count changed — exact cycle stream_sink saw tlast)
        done_cyc     = tb_cycle;
        measured_lat = done_cyc - inject_cyc;

        // Print result row
        $display("[RESULT]  %6d | %8d | %13d | %18.2f | %s",
                 i,
                 pkt_sizes[i],
                 measured_lat,
                 real'(pkt_sizes[i]) / real'(measured_lat),
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
      $display("[RESULT]  %6s | %8s | %13s | %18s | %s",
               "BATCH#", "SIZE (B)", "LATENCY (cyc)", "EFFICIENCY (B/cyc)", "CONFIG");
      $display("[RESULT] ................................................................");

      // Capture inject time from the first packet's perspective
      inject_cyc = tb_cycle;
      prev_count = pkt_count;

      // Inject all BATCH_COUNT × 64B packets.  pcie_model serialises them
      // (each packet waits for pcie to cycle back to S_RECV) so the
      // inter-packet gaps at the batching_unit are ~43 cycles < BATCH_TIMEOUT.
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

      done_cyc     = tb_cycle;
      measured_lat = done_cyc - inject_cyc;

      // Batch result row — SIZE is the total batch bytes
      $display("[RESULT]  %6d | %8d | %13d | %18.2f | %s",
               0,
               BATCH_COUNT * pcie_axi_pkg::PKT_64B,
               measured_lat,
               real'(BATCH_COUNT * pcie_axi_pkg::PKT_64B) / real'(measured_lat),
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
