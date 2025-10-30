`timescale 1ns/1ps
module spi_bit_engine_sim #(
           parameter integer MAX_BITS = 1024
       )(
           input  wire                   clk,    // 200MHz -> SCLK 100MHz
           input  wire                   rst_n,
           input  wire                   start,    // 1-cycle pulse (clk domain)
           input  wire [15:0]            bit_count,
           input  wire                   cpol,     // idle level
           input  wire                   cpha,
           input  wire                   present_first_bit, // only meaningful when CPHA=0
           input  wire [MAX_BITS-1:0]    tx_bits,  // MSB-first
           output reg  [MAX_BITS-1:0]    rx_bits,  // N bits at [N-1:0], last at bit0
           input  wire                   din,      // MISO
           output reg                    dout,     // MOSI
           output reg                    sclk,
           output reg                    cs_n,
           output reg                    busy,
           output reg                    done
       );

reg [MAX_BITS-1:0] sh_rx;
reg [15:0]         idx_tx;
reg [15:0]         shifts_left, samples_left;
reg [16:0]         half_edges_rem;   // = 2*bit_count
reg                running;          // "toggle in progress" flag

// Determine the property of the "next edge" (check current sclk before toggling)
wire leading_next  = (sclk == cpol);
wire trailing_next = ~leading_next;
wire do_sample_next = (cpha==1'b0) ? leading_next  : trailing_next;
wire do_shift_next  = (cpha==1'b0) ? trailing_next : leading_next;

// FSM (use running as main control to avoid race conditions when decrementing and changing state in the same cycle)
localparam S_IDLE=2'd0, S_CS=2'd1, S_RUN=2'd2, S_END=2'd3;
reg [1:0] state, nstate;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= S_IDLE;
    else
        state <= nstate;
end

always @* begin
    nstate = state;
    case (state)
        S_IDLE:
            if (start && bit_count!=0)
                nstate = S_CS;
        S_CS  :
            nstate = S_RUN;
        S_RUN :
            if (!running)
                nstate = S_END;   // Finish before wrap-up
            else
                nstate = S_RUN;
        S_END :
            nstate = S_IDLE;
        default:
            nstate = S_IDLE;
    endcase
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // reset
        sclk <= 1'b0;
        cs_n <= 1'b1;
        dout <= 1'b0;
        busy <= 1'b0;
        done <= 1'b0;
        running <= 1'b0;
        sh_rx <= {MAX_BITS{1'b0}};
        rx_bits <= {MAX_BITS{1'b0}};
        idx_tx <= 16'd0;
        shifts_left <= 16'd0;
        samples_left <= 16'd0;
        half_edges_rem <= 17'd0;
    end
    else begin
        done <= 1'b0;

        case (state)
            // -------- IDLE --------
            S_IDLE: begin
                busy  <= 1'b0;
                cs_n  <= 1'b1;
                sclk  <= cpol;       // Only set idle during idle; do not "force back" during operation
                dout  <= 1'b0;
                running <= 1'b0;

                if (start && bit_count!=0) begin
                    busy          <= 1'b1;
                    sh_rx         <= {MAX_BITS{1'b0}};
                    rx_bits       <= {MAX_BITS{1'b0}};
                    samples_left  <= bit_count;
                    half_edges_rem<= {1'b0, bit_count} << 1; // 2*bit_count

                    if (!cpha && present_first_bit) begin
                        // Assert MSB immediately when CS goes low
                        dout        <= tx_bits[bit_count-1];
                        shifts_left <= (bit_count>1) ? (bit_count-1) : 16'd0;
                        idx_tx      <= (bit_count>1) ? (bit_count-2) : 16'd0;
                    end
                    else begin
                        shifts_left <= bit_count;
                        idx_tx      <= bit_count-1;
                    end
                end
            end

            // -------- CS Assert --------
            S_CS: begin
                cs_n   <= 1'b0;
                running<= 1'b1;    // Start toggling from next cycle
            end

            // -------- RUN (on each clk posedge: do actions first, then toggle)--------
            S_RUN: begin
                if (running) begin
                    // 1) Align actions with the "upcoming edge"
                    if (do_shift_next && (shifts_left!=0)) begin
                        dout        <= tx_bits[idx_tx];
                        idx_tx      <= idx_tx - 16'd1;
                        shifts_left <= shifts_left - 16'd1;
                    end
                    if (do_sample_next && (samples_left!=0)) begin
                        sh_rx        <= {sh_rx[MAX_BITS-2:0], din};
                        samples_left <= samples_left - 16'd1;
                    end

                    // 2) Toggle and decrement (ensure 50% duty cycle)
                    if (half_edges_rem > 17'd1) begin
                        sclk <= ~sclk;
                        half_edges_rem <= half_edges_rem - 17'd1;
                    end
                    else begin
                        // half_edges_rem == 1: do the last toggle
                        sclk <= ~sclk;
                        half_edges_rem <= 17'd0;
                        running <= 1'b0;   // Stop on next clk cycle (sclk holds this value for half a cycle)
                    end
                end
            end

            // -------- END (only handshake, do not touch sclk)--------
            S_END: begin
                cs_n   <= 1'b1;
                rx_bits<= sh_rx;
                done   <= 1'b1;
            end

        endcase
    end
end
endmodule
