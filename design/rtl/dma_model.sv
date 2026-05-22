// =============================================================================
// dma_model.sv
//
// Behavioral model of a DMA engine.
//
// WHAT IT MODELS
//   A real DMA engine adds two kinds of overhead:
//
//   1. DESCRIPTOR SETUP (S_SETUP, fixed per packet)
//      The host writes a descriptor ring entry (src_addr, dst_addr, length,
//      flags) and rings a doorbell register.  The DMA controller must fetch
//      that descriptor over PCIe (another TLP round-trip), parse it, and
//      set up internal address counters before the first byte of data moves.
//      → Modelled as SETUP_CYCLES fixed cycles per packet regardless of size.
//      → This is the KEY thesis insight: small packets are penalised by the
//        same fixed overhead as large ones, making per-byte efficiency poor.
//
//   2. DATA TRANSFER (S_SEND, size-dependent)
//      The DMA engine reads from source memory and writes to a stream FIFO
//      at a rate of BYTES_PER_CYC bytes/cycle.  The transfer time is:
//        ceil(pkt_size / BYTES_PER_CYC) cycles
//      With the default BYTES_PER_CYC = DATA_WIDTH/8 = 8, this equals the
//      number of 64-bit AXI-S beats — no throughput gap, full pipeline rate.
//
//   Total DMA contribution = SETUP_CYCLES + ceil(pkt_size / BYTES_PER_CYC)
//
// STATE MACHINE  (3 states)
//
//   S_RECV  -- accept upstream beats into internal buffer (s_tready = 1)
//              On s_tlast: latch sideband, load setup_cnt → S_SETUP
//
//   S_SETUP -- descriptor-fetch countdown (s_tready = 0)
//              Decrement setup_cnt each cycle; when 0 → S_SEND
//              setup_cnt loaded as SETUP_CYCLES so we spend exactly
//              SETUP_CYCLES + 1 cycles here (including the entry cycle).
//
//   S_SEND  -- drive buffered beats downstream (s_tready = 0)
//              m_tvalid asserted when throttle_cnt == 0 (beat is ready)
//              CYCLES_PER_BEAT = ceil(STRB_W / BYTES_PER_CYC) gives the
//              inter-beat holdoff; for full-rate (default) this is 1.
//              On m_tready + last beat → S_RECV
//
// BANDWIDTH THROTTLE
//   When BYTES_PER_CYC < DATA_WIDTH/8, each beat requires CYCLES_PER_BEAT
//   cycles to produce (models bandwidth-limited DMA reads from slow DRAM).
//   m_tvalid goes low between beats during the holdoff.  At full rate
//   (BYTES_PER_CYC = STRB_W) CYCLES_PER_BEAT = 1 and there is no holdoff.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module dma_model #(
  parameter int DATA_WIDTH    = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W    = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W      = pcie_axi_pkg::PKT_ID_W,
  parameter int SETUP_CYCLES  = pcie_axi_pkg::DMA_SETUP_CYCLES,
  parameter int BYTES_PER_CYC = pcie_axi_pkg::DMA_BYTES_PER_CYC
) (
  input  logic                    clk,
  input  logic                    rst_n,

  // ---- Slave port (from PCIe model) ----------------------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,   // packet byte-length (per-packet const)
  input  logic [PKT_ID_W-1:0]     s_tid,     // packet tag         (per-packet const)

  // ---- Master port (to AXI-MM2S adapter) -----------------------------------
  output logic                    m_tvalid,
  input  logic                    m_tready,
  output logic [DATA_WIDTH-1:0]   m_tdata,
  output logic [DATA_WIDTH/8-1:0] m_tkeep,
  output logic                    m_tlast,
  output logic [PKT_SIZE_W-1:0]   m_tuser,
  output logic [PKT_ID_W-1:0]     m_tid
);

  // ---------------------------------------------------------------------------
  // Local constants
  // ---------------------------------------------------------------------------
  localparam int STRB_W        = DATA_WIDTH / 8;
  localparam int MAX_BEATS     = 4096 / STRB_W;              // 512 for 64-bit bus
  localparam int PTR_W         = $clog2(MAX_BEATS) + 1;
  // Cycles to produce each beat (1 = full bandwidth; >1 = BW throttle)
  localparam int CYCLES_PER_BEAT = (BYTES_PER_CYC >= STRB_W) ? 1
                                 : (STRB_W / BYTES_PER_CYC);
  localparam int THR_W         = $clog2(CYCLES_PER_BEAT + 1) + 1;

  // ---------------------------------------------------------------------------
  // Packet buffer — holds one full packet during setup and playback
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] buf_data [0:MAX_BEATS-1];
  logic [STRB_W-1:0]     buf_keep [0:MAX_BEATS-1];
  logic                  buf_last [0:MAX_BEATS-1];
  logic [PKT_SIZE_W-1:0] buf_user;
  logic [PKT_ID_W-1:0]   buf_tid;

  logic [PTR_W-1:0] wr_ptr;
  logic [PTR_W-1:0] rd_ptr;

  // ---------------------------------------------------------------------------
  // FSM
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    S_RECV  = 2'b00,   // receiving upstream beats
    S_SETUP = 2'b01,   // descriptor-fetch overhead countdown
    S_SEND  = 2'b10    // data transfer to downstream
  } state_e;
  state_e state;

  // ---------------------------------------------------------------------------
  // Counters
  // ---------------------------------------------------------------------------
  logic [31:0]    setup_cnt;     // counts down SETUP_CYCLES in S_SETUP
  logic [THR_W:0] throttle_cnt;  // inter-beat holdoff in S_SEND

  // ---------------------------------------------------------------------------
  // FSM — sequential
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state        <= S_RECV;
      wr_ptr       <= '0;
      rd_ptr       <= '0;
      setup_cnt    <= '0;
      throttle_cnt <= '0;
      buf_user     <= '0;
      buf_tid      <= '0;
    end else begin
      case (state)

        // --------------------------------------------------------------------
        // S_RECV: buffer incoming beats from pcie_model.
        // s_tready is driven combinatorially (= 1 in this state).
        // --------------------------------------------------------------------
        S_RECV: begin
          if (s_tvalid) begin
            buf_data[wr_ptr] <= s_tdata;
            buf_keep[wr_ptr] <= s_tkeep;
            buf_last[wr_ptr] <= s_tlast;
            wr_ptr           <= wr_ptr + PTR_W'(1);

            if (s_tlast) begin
              buf_user     <= s_tuser;
              buf_tid      <= s_tid;
              setup_cnt    <= 32'(SETUP_CYCLES);  // see timing note in header
              wr_ptr       <= '0;
              state        <= S_SETUP;
            end
          end
        end

        // --------------------------------------------------------------------
        // S_SETUP: fixed descriptor-fetch penalty.
        // Counts down from SETUP_CYCLES; transition when it reaches zero.
        // Total cycles in this state = SETUP_CYCLES + 1 (including entry).
        // --------------------------------------------------------------------
        S_SETUP: begin
          if (setup_cnt != 32'd0) begin
            setup_cnt <= setup_cnt - 32'd1;
          end else begin
            rd_ptr       <= '0;
            throttle_cnt <= THR_W'(CYCLES_PER_BEAT - 1);
            state        <= S_SEND;
          end
        end

        // --------------------------------------------------------------------
        // S_SEND: drive buffered beats downstream at BYTES_PER_CYC rate.
        //
        // throttle_cnt > 0 → DMA not ready yet (m_tvalid = 0, holdoff)
        // throttle_cnt = 0 → beat ready; wait for m_tready
        //
        // When m_tready accepted:
        //   - reload throttle for next beat
        //   - advance rd_ptr (or return to S_RECV on last beat)
        // --------------------------------------------------------------------
        S_SEND: begin
          if (throttle_cnt != '0) begin
            // Producing next beat — not ready to present yet
            throttle_cnt <= throttle_cnt - THR_W'(1);
          end else begin
            // Beat is ready on m_* outputs; wait for downstream acceptance
            if (m_tready) begin
              if (buf_last[rd_ptr]) begin
                rd_ptr <= '0;
                state  <= S_RECV;
              end else begin
                rd_ptr       <= rd_ptr + PTR_W'(1);
                throttle_cnt <= THR_W'(CYCLES_PER_BEAT - 1);
              end
            end
          end
        end

        default: state <= S_RECV;
      endcase
    end
  end

  // ---------------------------------------------------------------------------
  // Output assignments (combinational)
  // ---------------------------------------------------------------------------
  // s_tready: only high when we can accept new beats (S_RECV)
  assign s_tready = (state == S_RECV);

  // m_tvalid: asserted in S_SEND only when the current beat is fully produced
  // (throttle_cnt == 0 means the DMA's internal read has completed for this beat)
  assign m_tvalid = (state == S_SEND) && (throttle_cnt == '0);
  assign m_tdata  = m_tvalid ? buf_data[rd_ptr] : '0;
  assign m_tkeep  = m_tvalid ? buf_keep[rd_ptr] : '0;
  assign m_tlast  = m_tvalid & buf_last[rd_ptr];
  assign m_tuser  = buf_user;
  assign m_tid    = buf_tid;

  // ---------------------------------------------------------------------------
  // Simulation helpers
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    // Report when descriptor setup completes (useful for latency breakdown)
    if ((state == S_SETUP) && (setup_cnt == 32'd0))
      $display("[dma_model] t=%0t  pkt_id=%0d  descriptor setup done  (SETUP_CYCLES=%0d)",
               $time, buf_tid, SETUP_CYCLES);

    // Warn on buffer overflow
    if (s_tvalid && s_tready && (wr_ptr == PTR_W'(MAX_BEATS - 1)) && !s_tlast)
      $warning("[dma_model] Buffer almost full (wr_ptr=%0d). Increase MAX_BEATS.", wr_ptr);
  end
  // synthesis translate_on

endmodule : dma_model
