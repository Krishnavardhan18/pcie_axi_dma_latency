// =============================================================================
// axis_fifo.sv
//
// Parameterized AXI-Stream FIFO — synchronous, first-word-fall-through.
//
// WHAT IT MODELS
//   The DMA write buffer that decouples bursty DMA writes from smooth
//   downstream consumption.  In a real Xilinx design this would be a
//   BRAM-backed FIFO generator instance.
//
//   THESIS EXPERIMENT — FIFO depth sweep
//     DEPTH=16  : shallow FIFO → frequent full-stalls → latency spikes
//     DEPTH=64  : balanced default
//     DEPTH=256 : deep FIFO → rarely stalls → lowest latency at cost of area
//
//   Run with:  make sim FIFO_DEPTH=16   (or 64, 256)
//
// IMPLEMENTATION — First-Word Fall-Through (FWFT) circular buffer
//
//   The output (m_tdata etc.) is driven combinatorially from mem[rd_ptr].
//   Downstream can sample on any cycle that m_tvalid & m_tready is true
//   without an extra register stage — this minimises the FIFO's latency
//   contribution to the overall pipeline.
//
//   Write: on (s_tvalid & s_tready)
//          mem[wr_ptr] ← s_*
//          wr_ptr      ← (wr_ptr + 1) mod DEPTH   (natural wrap for 2^N)
//          count       ← count + 1
//
//   Read:  on (m_tvalid & m_tready)
//          rd_ptr ← (rd_ptr + 1) mod DEPTH
//          count  ← count - 1
//
//   Simultaneous R+W: both pointers advance, count unchanged (no stall).
//
// FLAGS
//   full      : count == DEPTH   → s_tready deasserted (back-pressure)
//   empty     : count == 0       → m_tvalid deasserted
//   occupancy : count (0..DEPTH) → exposed for latency monitor correlation
//
// ASSUMPTION
//   DEPTH must be a power of two for natural pointer wrap.
//   The thesis uses DEPTH ∈ {16, 64, 256} — all powers of two.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module axis_fifo #(
  parameter int DATA_WIDTH = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W   = pcie_axi_pkg::PKT_ID_W,
  parameter int DEPTH      = pcie_axi_pkg::FIFO_DEPTH_DEFAULT
) (
  input  logic                     clk,
  input  logic                     rst_n,

  // ---- Slave port (write side) ---------------------------------------------
  input  logic                     s_tvalid,
  output logic                     s_tready,
  input  logic [DATA_WIDTH-1:0]    s_tdata,
  input  logic [DATA_WIDTH/8-1:0]  s_tkeep,
  input  logic                     s_tlast,
  input  logic [PKT_SIZE_W-1:0]    s_tuser,
  input  logic [PKT_ID_W-1:0]      s_tid,

  // ---- Master port (read side) ---------------------------------------------
  output logic                     m_tvalid,
  input  logic                     m_tready,
  output logic [DATA_WIDTH-1:0]    m_tdata,
  output logic [DATA_WIDTH/8-1:0]  m_tkeep,
  output logic                     m_tlast,
  output logic [PKT_SIZE_W-1:0]    m_tuser,
  output logic [PKT_ID_W-1:0]      m_tid,

  // ---- Status --------------------------------------------------------------
  output logic [$clog2(DEPTH):0]   occupancy,  // 0 .. DEPTH (one extra bit)
  output logic                     full,
  output logic                     empty
);

  localparam int STRB_W = DATA_WIDTH / 8;
  localparam int PTR_W  = $clog2(DEPTH);          // pointer bit-width
  localparam int CNT_W  = $clog2(DEPTH) + 1;      // count: 0 .. DEPTH

  // ---------------------------------------------------------------------------
  // Memory arrays (separate arrays per field — Vivado-friendly)
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] mem_data [0:DEPTH-1];
  logic [STRB_W-1:0]     mem_keep [0:DEPTH-1];
  logic                  mem_last [0:DEPTH-1];
  logic [PKT_SIZE_W-1:0] mem_user [0:DEPTH-1];
  logic [PKT_ID_W-1:0]   mem_tid  [0:DEPTH-1];

  // ---------------------------------------------------------------------------
  // Pointers and count
  // ---------------------------------------------------------------------------
  logic [PTR_W-1:0]  wr_ptr;
  logic [PTR_W-1:0]  rd_ptr;
  logic [CNT_W-1:0]  count;   // number of valid entries currently stored

  // ---------------------------------------------------------------------------
  // Handshake enables
  // ---------------------------------------------------------------------------
  logic wr_en, rd_en;
  assign wr_en = s_tvalid & s_tready;   // qualified write
  assign rd_en = m_tvalid & m_tready;   // qualified read

  // ---------------------------------------------------------------------------
  // Flags (combinational)
  // ---------------------------------------------------------------------------
  assign full      = (count == CNT_W'(DEPTH));
  assign empty     = (count == '0);
  assign occupancy = count;

  // ---------------------------------------------------------------------------
  // Sequential: write path, pointer updates, count
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_ptr <= '0;
      rd_ptr <= '0;
      count  <= '0;
    end else begin
      // Write
      if (wr_en) begin
        mem_data[wr_ptr] <= s_tdata;
        mem_keep[wr_ptr] <= s_tkeep;
        mem_last[wr_ptr] <= s_tlast;
        mem_user[wr_ptr] <= s_tuser;
        mem_tid [wr_ptr] <= s_tid;
        wr_ptr           <= wr_ptr + PTR_W'(1);  // wraps at 2^PTR_W = DEPTH
      end

      // Read pointer advance
      if (rd_en) begin
        rd_ptr <= rd_ptr + PTR_W'(1);
      end

      // Occupancy counter — one-hot priority to avoid double-counting
      if      ( wr_en && !rd_en) count <= count + CNT_W'(1);
      else if (!wr_en &&  rd_en) count <= count - CNT_W'(1);
      // simultaneous R+W or neither: count unchanged
    end
  end

  // ---------------------------------------------------------------------------
  // Output assignments (FWFT — combinational read)
  // ---------------------------------------------------------------------------
  assign s_tready = ~full;
  assign m_tvalid = ~empty;
  assign m_tdata  = mem_data[rd_ptr];
  assign m_tkeep  = mem_keep[rd_ptr];
  assign m_tlast  = mem_last[rd_ptr];
  assign m_tuser  = mem_user[rd_ptr];
  assign m_tid    = mem_tid [rd_ptr];

  // ---------------------------------------------------------------------------
  // Simulation assertions
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (s_tvalid && full)
      $warning("[axis_fifo] FIFO full (depth=%0d) — upstream back-pressured at t=%0t",
               DEPTH, $time);

    if (count > CNT_W'(DEPTH))
      $fatal(1, "[axis_fifo] count overflow: count=%0d DEPTH=%0d at t=%0t",
             count, DEPTH, $time);

    // Verify power-of-2 depth at time-0 (parameter sanity)
    if ($time == 0 && (DEPTH & (DEPTH-1)) != 0)
      $warning("[axis_fifo] DEPTH=%0d is not a power-of-2; pointer wrap may be incorrect",
               DEPTH);
  end
  // synthesis translate_on

endmodule : axis_fifo
