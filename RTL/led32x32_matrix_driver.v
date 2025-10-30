`timescale 1ns/1ps

// ============================================================================
// led32x32_matrix_driver — Fixed row0 dwell-time and end-row timing
// ============================================================================
module led32x32_matrix_driver #(
           parameter integer REFRESH_DIVIDER = 16
       )(
           // Inputs
           input  wire        clk,
           input  wire        rst_n,
           input  wire        enable,
           input  wire        row_valid,
           input  wire [4:0]  row_idx,
           input  wire [31:0] row_data,

           // Outputs
           output reg  [31:0] led_row,
           output reg  [31:0] led_col,
           output reg         refresh_pulse,
           output reg         frame_done
       );

// ============================================================================
// Frame Buffer
// ============================================================================
reg [31:0] fb [0:31];

// ============================================================================
// FSM
// ============================================================================
localparam [1:0] S_IDLE = 2'd0;
localparam [1:0] S_SCAN = 2'd1;
localparam [1:0] S_HOLD = 2'd2;

reg [1:0]  st_q;
reg [1:0]  st_d;
reg [4:0]  rptr_q;         // current row index being displayed
reg [4:0]  rptr_d;
reg [15:0] div_q;          // refresh divider counter
reg [15:0] div_d;
reg [15:0] hold_q;         // hold counter for the last row
reg [15:0] hold_d;
reg        sync_req_q;     // set when full frame (row 31) has been loaded
reg        sync_req_d;
reg        first_cycle_q;  // marks the very first scan cycle for row 0
reg        first_cycle_d;

integer i;
initial begin
    for (i = 0; i < 32; i = i + 1) begin
        fb[i] = 32'd0;
    end
end

// ============================================================================
// Combinational Defaults and Next-State/Output Logic
// ============================================================================
reg [31:0] led_row_d;
reg [31:0] led_col_d;
reg        refresh_pulse_d;
reg        frame_done_d;

// COMB: Next-state and output logic for scan/hold timing
always @* begin
    // Defaults (hold)
    st_d            = st_q;
    rptr_d          = rptr_q;
    div_d           = div_q;
    hold_d          = hold_q;
    sync_req_d      = sync_req_q;
    first_cycle_d   = first_cycle_q;

    led_row_d       = led_row;
    led_col_d       = led_col;
    refresh_pulse_d = 1'b0;
    frame_done_d    = 1'b0;

    // Trigger sync when last row of a frame has been written into fb
    if (row_valid && (row_idx == 5'd31)) begin
        sync_req_d = 1'b1;
    end

    case (st_q)
        // ========================================================================
        // IDLE
        // ========================================================================
        S_IDLE: begin
            if (enable && sync_req_q) begin
                // Immediately output row0 but do not advance rptr yet
                led_row_d       = 32'h0000_0001;  // bit[0]
                led_col_d       = fb[5'd0];
                refresh_pulse_d = 1'b1;

                rptr_d         = 5'd0;       // keep at row0 for a full period
                div_d          = 16'd0;      // start divider
                first_cycle_d  = 1'b1;       // mark first cycle
                st_d           = S_SCAN;
                sync_req_d     = 1'b0;
            end
            else if (!enable) begin
                led_row_d = 32'd0;
                led_col_d = 32'd0;
            end
        end

        // ========================================================================
        // SCAN
        // ========================================================================
        S_SCAN: begin
            if (!enable) begin
                led_row_d = 32'd0;
                led_col_d = 32'd0;
                st_d      = S_IDLE;
            end
            else begin
                if (div_q == (REFRESH_DIVIDER - 1)) begin
                    // One full refresh period completed
                    div_d           = 16'd0;
                    refresh_pulse_d = 1'b1;

                    if (first_cycle_q) begin
                        // First cycle: row0 has been displayed long enough; switch to row1
                        first_cycle_d = 1'b0;
                        rptr_d        = 5'd1;
                        led_row_d     = 32'h0000_0002;  // bit[1]
                        led_col_d     = fb[5'd1];
                    end
                    else begin
                        // Subsequent cycles
                        if (rptr_q == 5'd31) begin
                            // Last row: go to HOLD to keep full period
                            frame_done_d = 1'b1;
                            st_d         = S_HOLD;
                            hold_d       = REFRESH_DIVIDER - 1;
                        end
                        else begin
                            // Advance to next row
                            rptr_d    = rptr_q + 5'd1;
                            led_row_d = (32'h1 << (rptr_q + 5'd1));
                            led_col_d = fb[rptr_q + 5'd1];
                        end
                    end
                end
                else begin
                    // Continue counting within the refresh period
                    div_d = div_q + 16'd1;
                end
            end
        end

        // ========================================================================
        // HOLD (final row full-period hold before returning to IDLE)
        // ========================================================================
        S_HOLD: begin
            if (!enable) begin
                led_row_d = 32'd0;
                led_col_d = 32'd0;
                st_d      = S_IDLE;
            end
            else begin
                if (hold_q == 16'd0) begin
                    // Hold complete; clear and return to IDLE
                    led_row_d     = 32'd0;
                    led_col_d     = 32'd0;
                    st_d          = S_IDLE;
                    rptr_d        = 5'd0;
                    first_cycle_d = 1'b0;
                end
                else begin
                    hold_d = hold_q - 16'd1;
                end
            end
        end

        default: begin
            st_d = S_IDLE;
        end
    endcase
end

// ============================================================================
// Sequential Logic
// ============================================================================
// SEQ: State/counters, frame buffer write, and registered outputs
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        st_q          <= S_IDLE;
        rptr_q        <= 5'd0;
        div_q         <= 16'd0;
        hold_q        <= 16'd0;
        sync_req_q    <= 1'b0;
        first_cycle_q <= 1'b0;

        led_row       <= 32'd0;
        led_col       <= 32'd0;
        refresh_pulse <= 1'b0;
        frame_done    <= 1'b0;
    end
    else begin
        // Frame buffer update
        if (row_valid) begin
            fb[row_idx] <= row_data;
        end

        // State and counters
        st_q          <= st_d;
        rptr_q        <= rptr_d;
        div_q         <= div_d;
        hold_q        <= hold_d;
        sync_req_q    <= sync_req_d;
        first_cycle_q <= first_cycle_d;

        // Registered outputs
        led_row       <= led_row_d;
        led_col       <= led_col_d;
        refresh_pulse <= refresh_pulse_d;
        frame_done    <= frame_done_d;
    end
end

endmodule
