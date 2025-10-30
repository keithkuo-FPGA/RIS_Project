`timescale 1ns/1ps

// ============================================================================
// led32x32_row_streamer — stream 32 rows from BRAM with 2-cycle read latency
// ============================================================================
module led32x32_row_streamer #(
           parameter integer FRAME_SEL_WIDTH = 3
       )(
           // Inputs
           input  wire                       clk,
           input  wire                       rst_n,
           input  wire                       start,
           input  wire [FRAME_SEL_WIDTH-1:0] frame_sel,
           input  wire [31:0]                bram_rd_data,

           // Outputs
           output reg                        bram_rd_en,
           output reg  [7:0]                 bram_rd_addr,
           output reg                        row_valid,
           output reg  [4:0]                 row_idx,
           output reg  [31:0]                row_data,
           output reg                        busy,
           output reg                        done
       );

// ============================================================================
// Local Parameters
// ============================================================================
localparam [1:0] S_IDLE  = 2'd0;
localparam [1:0] S_ISSUE = 2'd1;  // issue read requests
localparam [1:0] S_DRAIN = 2'd2;  // drain remaining pipeline words

// ============================================================================
// Register Declarations
// ============================================================================
reg [1:0] state;
reg [4:0] issue_row;       // next row index to request (0..31)
reg [5:0] issued_cnt;      // number of requests issued (0..32)
reg [5:0] output_cnt;      // number of rows output (0..32)
reg [4:0] pipe_row [0:1];  // pipeline row tags: [0]=stage1, [1]=stage2
reg       pipe_vld [0:1];

// ============================================================================
// Wire Declarations
// ============================================================================
wire kick;

// ============================================================================
// Assignments
// ============================================================================
assign kick = (start == 1'b1) && (state == S_IDLE);

// ============================================================================
// Initialization: seed pipeline arrays (synthesizable on FPGA targets)
// ============================================================================
integer i;
initial begin
    for (i = 0; i < 2; i = i + 1) begin
        pipe_row[i] = 5'd0;
        pipe_vld[i] = 1'b0;
    end
end

// ROW: Drive BRAM reads, align 2-cycle latency, and emit row stream on clk
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state        <= S_IDLE;
        busy         <= 1'b0;
        done         <= 1'b0;
        bram_rd_en   <= 1'b0;
        bram_rd_addr <= 8'd0;
        issue_row    <= 5'd0;
        issued_cnt   <= 6'd0;
        output_cnt   <= 6'd0;
        row_valid    <= 1'b0;
        row_idx      <= 5'd0;
        row_data     <= 32'd0;

        for (i = 0; i < 2; i = i + 1) begin
            pipe_row[i] <= 5'd0;
            pipe_vld[i] <= 1'b0;
        end
    end
    else begin
        done <= 1'b0;  // one-cycle pulse

        case (state)
            // --------------------------------------------------------------------
            // IDLE: wait for start; immediately issue row0 and seed pipeline
            // --------------------------------------------------------------------
            S_IDLE: begin
                busy       <= 1'b0;
                row_valid  <= 1'b0;
                bram_rd_en <= 1'b0;

                if (kick) begin
                    // Issue row0 request
                    busy         <= 1'b1;
                    state        <= S_ISSUE;
                    bram_rd_en   <= 1'b1;
                    bram_rd_addr <= {frame_sel, 5'd0};
                    issue_row    <= 5'd1;   // next: row1
                    issued_cnt   <= 6'd1;   // issued 1 request (row0)
                    output_cnt   <= 6'd0;   // none output yet

                    // Seed pipeline (stage1 holds row0 tag)
                    pipe_row[0]  <= 5'd0;
                    pipe_vld[0]  <= 1'b1;

                    // Clear stage2
                    pipe_row[1]  <= 5'd0;
                    pipe_vld[1]  <= 1'b0;
                end
            end

            // --------------------------------------------------------------------
            // ISSUE: keep issuing until 32 requests; stream outputs as they return
            // --------------------------------------------------------------------
            S_ISSUE: begin
                // Advance pipeline (2-cycle BRAM latency alignment)
                pipe_row[1] <= pipe_row[0];
                pipe_vld[1] <= pipe_vld[0];

                // Emit valid output aligned to stage2
                if (pipe_vld[1]) begin
                    row_valid  <= 1'b1;
                    row_idx    <= pipe_row[1];
                    row_data   <= bram_rd_data;
                    output_cnt <= output_cnt + 6'd1;
                end
                else begin
                    row_valid <= 1'b0;
                end

                // Keep issuing requests until all 32 are sent
                if (issued_cnt < 6'd32) begin
                    bram_rd_en   <= 1'b1;
                    bram_rd_addr <= {frame_sel, issue_row};
                    pipe_row[0]  <= issue_row;
                    pipe_vld[0]  <= 1'b1;
                    issue_row    <= issue_row + 5'd1;
                    issued_cnt   <= issued_cnt + 6'd1;
                end
                else begin
                    // No more requests; stop feeding stage1 and switch to drain phase
                    bram_rd_en  <= 1'b0;
                    pipe_row[0] <= 5'd0;
                    pipe_vld[0] <= 1'b0;
                    state       <= S_DRAIN;
                end
            end

            // --------------------------------------------------------------------
            // DRAIN: flush final two pipeline stages; finish when 32 outputs seen
            // --------------------------------------------------------------------
            S_DRAIN: begin
                // Push pipeline; stage1 no longer receives new data
                pipe_row[1] <= pipe_row[0];
                pipe_vld[1] <= pipe_vld[0];
                pipe_vld[0] <= 1'b0;

                // Emit remaining outputs from stage2
                if (pipe_vld[1]) begin
                    row_valid  <= 1'b1;
                    row_idx    <= pipe_row[1];
                    row_data   <= bram_rd_data;
                    output_cnt <= output_cnt + 6'd1;
                end
                else begin
                    row_valid <= 1'b0;
                end

                // Completion check
                if (output_cnt >= 6'd32) begin
                    busy  <= 1'b0;
                    done  <= 1'b1;
                    state <= S_IDLE;
                end
            end

            default: begin
                state <= S_IDLE;
            end
        endcase
    end
end

endmodule

