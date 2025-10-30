`timescale 1ns/1ps

// ============================================================================
// spi_bit_engine_v2 — SPI bit-level engine with optional auto-reload
// ============================================================================
module spi_bit_engine_v2 #(
           parameter integer MAX_BITS = 1024
       )(
           // Inputs
           input  wire                 clk,
           input  wire                 rst_n,
           input  wire                 start,
           input  wire [15:0]          bit_count,
           input  wire                 cpol,
           input  wire                 cpha,
           input  wire                 present_first_bit,
           input  wire [MAX_BITS-1:0]  tx_bits,
           input  wire                 keep_cs,
           input  wire                 qd_mode_en,
           input  wire                 auto_reload,     // auto-reload mode

           // Outputs
           output reg  [MAX_BITS-1:0]  rx_bits,
           output reg                  sclk,
           output reg                  cs_n,
           output reg                  busy,
           output reg                  done,

           // Inouts
           inout  wire                 dq0,
           inout  wire                 dq1,
           inout  wire                 dq2,
           inout  wire                 dq3
       );
// ============================================================================
// Local Parameters
// ============================================================================
localparam S_IDLE = 2'd0;
localparam S_CS   = 2'd1;
localparam S_RUN  = 2'd2;
localparam S_END  = 2'd3;

// ============================================================================
// Register Declarations
// ============================================================================
reg                  dout;
reg                  dq2_oe_n;            // active-low OE for dq2
reg                  dq3_oe_n;            // active-low OE for dq3
reg                  int_keep_cs;
reg [MAX_BITS-1:0]   sh_rx;
reg [15:0]           idx_tx;
reg [15:0]           shifts_left;
reg [15:0]           samples_left;
reg [16:0]           half_edges_rem;
reg                  running;
reg [15:0]           bit_count_latch;     // latched bit_count for auto-reload
reg                  auto_reload_latch;   // latched auto_reload for auto-reload FSM
reg [1:0]            state;
reg [1:0]            nstate;

// ============================================================================
// Wire Declarations
// ============================================================================
wire                 sg_din;
wire [3:0]           qd_din;
wire                 leading_next;
wire                 trailing_next;
wire                 do_sample_next;
wire                 do_shift_next;

// ============================================================================
// Assignments
// ============================================================================
assign sg_din     = dq1;
assign qd_din[0]  = dq0;
assign qd_din[1]  = dq1;
assign qd_din[2]  = dq2;
assign qd_din[3]  = dq3;

assign dq0        = (qd_mode_en ? 1'bz : dout);
assign dq1        = (qd_mode_en ? 1'bz : 1'bz);
// Original intent preserved; dq2/dq3 are driven high when enabled (not in quad)
assign dq2        = (dq2_oe_n ? 1'bz : 1'b1);
assign dq3        = (dq3_oe_n ? 1'bz : 1'b1);

assign leading_next   = (sclk == cpol);
assign trailing_next  = ~leading_next;
assign do_sample_next = (cpha == 1'b0) ? leading_next  : trailing_next;
assign do_shift_next  = (cpha == 1'b0) ? trailing_next : leading_next;

// ============================================================================
// FSM: State register (posedge clk, async active-low reset)
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state <= S_IDLE;
    end
    else begin
        state <= nstate;
    end
end

// ============================================================================
// FSM: Next-state combinational logic
// ============================================================================
always @* begin
    nstate = state;
    case (state)
        S_IDLE:
            if (start && bit_count != 0)
                nstate = S_CS;
        S_CS  :
            nstate = S_RUN;
        S_RUN :
            if (!running)
                nstate = S_END;
            else
                nstate = S_RUN;
        S_END :
            nstate = S_IDLE;
        default:
            nstate = S_IDLE;
    endcase
end

// ============================================================================
// Core: Control SCLK/CS, shift/sample data, handle auto-reload
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sclk               <= 1'b0;
        cs_n               <= 1'b1;
        dout               <= 1'b0;
        busy               <= 1'b0;
        done               <= 1'b0;
        running            <= 1'b0;
        sh_rx              <= {MAX_BITS{1'b0}};
        rx_bits            <= {MAX_BITS{1'b0}};
        idx_tx             <= 16'd0;
        shifts_left        <= 16'd0;
        samples_left       <= 16'd0;
        half_edges_rem     <= 17'd0;
        int_keep_cs        <= 1'b0;
        bit_count_latch    <= 16'd0;
        auto_reload_latch  <= 1'b0;
        dq2_oe_n           <= 1'b1;
        dq3_oe_n           <= 1'b1;
    end
    else begin
        done      <= 1'b0;
        dq2_oe_n  <= qd_mode_en;
        dq3_oe_n  <= qd_mode_en;

        case (state)
            // ----------------------------------------------------------
            // IDLE: Latch parameters and optionally present first bit
            // ----------------------------------------------------------
            S_IDLE: begin
                busy        <= 1'b0;
                cs_n        <= (int_keep_cs ? cs_n : 1'b1);
                sclk        <= cpol;
                dout        <= 1'b0;
                running     <= 1'b0;

                if (start && bit_count != 0) begin
                    busy            <= 1'b1;
                    sh_rx           <= {MAX_BITS{1'b0}};
                    rx_bits         <= {MAX_BITS{1'b0}};
                    samples_left    <= bit_count;
                    half_edges_rem  <= (qd_mode_en ? {1'b0, bit_count} >> 1
                                        : {1'b0, bit_count} << 1);
                    int_keep_cs     <= keep_cs;

                    // Latch parameters for auto-reload
                    bit_count_latch   <= bit_count;
                    auto_reload_latch <= auto_reload;

                    if (!cpha && present_first_bit) begin
                        dout        <= tx_bits[bit_count-1];
                        shifts_left <= (bit_count > 1) ? (bit_count - 1) : 16'd0;
                        idx_tx      <= (bit_count > 1) ? (bit_count - 2) : 16'd0;
                    end
                    else begin
                        shifts_left <= bit_count;
                        idx_tx      <= bit_count - 1;
                    end
                end
                else begin
                    int_keep_cs <= keep_cs;
                end
            end

            // ----------------------------------------------------------
            // CS: Assert chip-select and mark running
            // ----------------------------------------------------------
            S_CS: begin
                cs_n    <= 1'b0;
                running <= 1'b1;
            end

            // ----------------------------------------------------------
            // RUN: Shift out, sample in, toggle clock, manage auto-reload
            // ----------------------------------------------------------
            S_RUN: begin
                if (running) begin
                    // Shift out next bit(s)
                    if (do_shift_next && (shifts_left != 0)) begin
                        dout        <= tx_bits[idx_tx];
                        idx_tx      <= idx_tx - 16'd1;
                        shifts_left <= shifts_left - 16'd1;
                    end

                    // Sample incoming bit(s)
                    if (do_sample_next && (samples_left != 0)) begin
                        if (qd_mode_en) begin
                            sh_rx        <= {sh_rx[MAX_BITS-5:0], qd_din};
                            samples_left <= samples_left - 16'd4;
                        end
                        else begin
                            sh_rx        <= {sh_rx[MAX_BITS-2:0], sg_din};
                            samples_left <= samples_left - 16'd1;
                        end
                    end

                    // Toggle SCLK and handle auto-reload at end of transfer
                    if (half_edges_rem > 17'd1) begin
                        sclk           <= ~sclk;
                        half_edges_rem <= half_edges_rem - 17'd1;
                    end
                    else begin
                        sclk           <= ~sclk;
                        half_edges_rem <= 17'd0;

                        // Auto-reload: emit done pulse and immediately reload counters
                        if (auto_reload_latch && auto_reload) begin
                            rx_bits        <= sh_rx;
                            sh_rx          <= {MAX_BITS{1'b0}};
                            samples_left   <= bit_count_latch;
                            shifts_left    <= bit_count_latch;
                            idx_tx         <= bit_count_latch - 1;
                            half_edges_rem <= (qd_mode_en ? {1'b0, bit_count_latch} >> 1
                                               : {1'b0, bit_count_latch} << 1);
                            done           <= 1'b1;  // pulse for external capture
                            // running stays asserted
                        end
                        else begin
                            running <= 1'b0;
                        end
                    end
                end
            end

            // ----------------------------------------------------------
            // END: Deassert CS (unless kept), present received bits, pulse done
            // ----------------------------------------------------------
            S_END: begin
                cs_n    <= (int_keep_cs ? cs_n : 1'b1);
                rx_bits <= sh_rx;
                done    <= 1'b1;
            end
        endcase
    end
end


endmodule

