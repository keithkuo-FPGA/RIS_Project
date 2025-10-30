`timescale 1ns/1ps

// ============================================================================
// dp_bram_256x32 — Dual-port BRAM (Port A write, Port B synchronous read)
// ============================================================================
module dp_bram_256x32 (
           // Inputs
           input  wire        clk,
           input  wire        rst_n,
           input  wire        we_a,
           input  wire [7:0]  addr_a,
           input  wire [31:0] wdata_a,
           input  wire        rd_en_b,
           input  wire [7:0]  addr_b,

           // Outputs
           output reg  [31:0] rdata_b
       );

// ============================================================================
// Register Declarations
// ============================================================================
reg [31:0] mem [0:255];  // Frame buffer storage

// ============================================================================
// Integer Declarations
// ============================================================================
integer i;

// ============================================================================
// Initialization (retain original behavior)
// ============================================================================
initial begin
    for (i = 0; i < 256; i = i + 1) begin
        mem[i] = 32'd0;
    end
end

// ============================================================================
// BRAM: Port A write, Port B synchronous read (posedge clk, async active-low rst)
// ============================================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rdata_b <= 32'd0;
    end
    else begin
        // Port A: write
        if (we_a) begin
            mem[addr_a] <= wdata_a;
        end

        // Port B: synchronous read (no extra address register)
        if (rd_en_b) begin
            rdata_b <= mem[addr_b];
        end
    end
end

endmodule
