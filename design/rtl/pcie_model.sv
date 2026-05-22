// =============================================================================
// pcie_model.sv
//
// Behavioral model of a PCIe transaction layer.
//
// WHAT IT MODELS
//   Real PCIe has a fixed per-TLP overhead (header framing, credit checks,
//   ACK/NAK round-trip) PLUS a throughput-limited data transfer phase.
//   Both effects are captured with two parameters:
//
//     BASE_LATENCY   -- fixed cycles per packet regardless of size
//                       (models: TLP header assembly + credit-check RTT +
//                        link-level framing; typ. ~100-200 ns on Gen3)
//     PER_DW_LATENCY -- extra cycles added per 4-byte DWORD in the payload
//                       (models link bandwidth: at 8 GT/s Gen3 x1 a DWORD
//                        takes ~0.5 ns, so 1 cycle/DW @ 100 MHz is a slight
//                        over-estimate — tunable for realism)
//
//   Total PCIe delay = BASE_LATENCY + ceil(pkt_size / 4) * PER_DW_LATENCY
//
// STATE MACHINE  (3 states)
//
//   S_RECV  -- accept upstream beats into internal packet buffer
//              s_tready = 1; on s_tlast compute delay and go to S_DELAY
//
//   S_DELAY -- PCIe latency countdown; s_tready = 0 (back-pressure upstream)
//              count delay_cnt down to 0 then go to S_SEND
//
//   S_SEND  -- drive buffered beats out to DMA model via m_* port
//              honour m_tready back-pressure; on m_tlast go back to S_RECV
//
// SIMPLIFICATIONS
//   - One packet in-flight at a time (matches sequential TB injection).
//   - No real credit flow; s_tready is low only during delay+send phases.
//   - Payload data is stored verbatim and replayed; no TLP header insertion.
//
// =============================================================================

`include "pcie_axi_pkg.sv"
import pcie_axi_pkg::*;

module pcie_model #(
  parameter int DATA_WIDTH     = pcie_axi_pkg::DATA_WIDTH,
  parameter int PKT_SIZE_W     = pcie_axi_pkg::PKT_SIZE_W,
  parameter int PKT_ID_W       = pcie_axi_pkg::PKT_ID_W,
  parameter int BASE_LATENCY   = pcie_axi_pkg::PCIE_BASE_LATENCY,
  parameter int PER_DW_LATENCY = pcie_axi_pkg::PCIE_PER_DW_LATENCY
) (
  input  logic                    clk,
  input  logic                    rst_n,

  // ---- Slave port (from Packet Generator) ----------------------------------
  input  logic                    s_tvalid,
  output logic                    s_tready,
  input  logic [DATA_WIDTH-1:0]   s_tdata,
  input  logic [DATA_WIDTH/8-1:0] s_tkeep,
  input  logic                    s_tlast,
  input  logic [PKT_SIZE_W-1:0]   s_tuser,   // packet byte-length (constant across beats)
  input  logic [PKT_ID_W-1:0]     s_tid,     // packet tag         (constant across beats)

  // ---- Master port (to DMA model) ------------------------------------------
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
  localparam int STRB_W    = DATA_WIDTH / 8;
  // Buffer sized for the largest packet in the test suite (4 KB)
  localparam int MAX_BEATS = 4096 / STRB_W;   // = 512 for 64-bit bus
  localparam int PTR_W     = $clog2(MAX_BEATS) + 1;  // one extra bit for safety

  // ---------------------------------------------------------------------------
  // Internal packet buffer — holds one full packet during the delay phase
  // ---------------------------------------------------------------------------
  logic [DATA_WIDTH-1:0] buf_data [0:MAX_BEATS-1];
  logic [STRB_W-1:0]     buf_keep [0:MAX_BEATS-1];
  logic                  buf_last [0:MAX_BEATS-1];
  logic [PKT_SIZE_W-1:0] buf_user;   // latched pkt_size from s_tuser
  logic [PKT_ID_W-1:0]   buf_tid;    // latched pkt_id  from s_tid

  logic [PTR_W-1:0] wr_ptr;   // next write slot (cleared on packet boundary)
  logic [PTR_W-1:0] rd_ptr;   // next read  slot (cleared on packet boundary)

  // ---------------------------------------------------------------------------
  // FSM state
  // ---------------------------------------------------------------------------
  typedef enum logic [1:0] {
    S_RECV  = 2'b00,
    S_DELAY = 2'b01,
    S_SEND  = 2'b10
  } state_e;
  state_e state;

  // ---------------------------------------------------------------------------
  // Delay counter
  // ---------------------------------------------------------------------------
  logic [31:0] delay_cnt;

  // ---------------------------------------------------------------------------
  // Delay computation (done combinatorially at tlast so it is ready on the
  // same edge that triggers the S_DELAY entry)
  // ceil(pkt_size / 4) = (pkt_size + 3) >> 2
  // ---------------------------------------------------------------------------
  logic [31:0] computed_delay;
  always_comb begin
    computed_delay = 32'(BASE_LATENCY) +
                     ((32'(s_tuser) + 32'd3) >> 2) * 32'(PER_DW_LATENCY);
  end

  // ---------------------------------------------------------------------------
  // FSM — sequential logic
  // ---------------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state     <= S_RECV;
      wr_ptr    <= '0;
      rd_ptr    <= '0;
      delay_cnt <= '0;
      buf_user  <= '0;
      buf_tid   <= '0;
    end else begin
      case (state)

        // --------------------------------------------------------------------
        // S_RECV: accept upstream beats one by one into the buffer.
        // s_tready is driven combinatorially (see below).
        // --------------------------------------------------------------------
        S_RECV: begin
          if (s_tvalid) begin
            buf_data[wr_ptr] <= s_tdata;
            buf_keep[wr_ptr] <= s_tkeep;
            buf_last[wr_ptr] <= s_tlast;
            wr_ptr           <= wr_ptr + PTR_W'(1);

            if (s_tlast) begin
              // Latch per-packet sideband (same value on every beat)
              buf_user  <= s_tuser;
              buf_tid   <= s_tid;
              // Load delay counter and transition
              delay_cnt <= computed_delay;
              wr_ptr    <= '0;
              state     <= S_DELAY;
            end
          end
        end

        // --------------------------------------------------------------------
        // S_DELAY: count down; upstream is back-pressured (s_tready = 0).
        // --------------------------------------------------------------------
        S_DELAY: begin
          if (delay_cnt != 32'd0) begin
            delay_cnt <= delay_cnt - 32'd1;
          end else begin
            rd_ptr <= '0;
            state  <= S_SEND;
          end
        end

        // --------------------------------------------------------------------
        // S_SEND: drive buffered beats downstream, honouring m_tready.
        // --------------------------------------------------------------------
        S_SEND: begin
          if (m_tready) begin
            if (buf_last[rd_ptr]) begin
              // Final beat consumed — ready for next packet
              rd_ptr <= '0;
              state  <= S_RECV;
            end else begin
              rd_ptr <= rd_ptr + PTR_W'(1);
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
  // Slave ready: only accept when in S_RECV
  assign s_tready = (state == S_RECV);

  // Master valid/data: driven from buffer during S_SEND
  assign m_tvalid = (state == S_SEND);
  assign m_tdata  = (state == S_SEND) ? buf_data[rd_ptr] : '0;
  assign m_tkeep  = (state == S_SEND) ? buf_keep[rd_ptr] : '0;
  assign m_tlast  = (state == S_SEND) & buf_last[rd_ptr];
  assign m_tuser  = buf_user;   // valid throughout S_SEND (latched at entry)
  assign m_tid    = buf_tid;

  // ---------------------------------------------------------------------------
  // Simulation assertion: warn if buffer overflow (pkt > MAX_BEATS)
  // ---------------------------------------------------------------------------
  // synthesis translate_off
  always_ff @(posedge clk) begin
    if (s_tvalid && s_tready && (wr_ptr == PTR_W'(MAX_BEATS - 1)) && !s_tlast)
      $warning("[pcie_model] Buffer almost full (wr_ptr=%0d). Increase MAX_BEATS.", wr_ptr);
  end
  // synthesis translate_on

endmodule : pcie_model
