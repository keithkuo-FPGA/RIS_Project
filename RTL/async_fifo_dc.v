`timescale 1ns/1ps

// ============================================================================
// async_fifo_dc — Wrapper for Lattice SCUBA FIFO (module name: fifo)
// - DW must be 8, DEPTH must be 256 (must match SCUBA configuration)
// - Non-FWFT: Q is enabled by RdEn (internal IP has ORE = RdEn)
// - Reset: async assert / sync release (tie to Reset and RPReset, both active-high)
// ============================================================================
module async_fifo_dc #(
           parameter DW    = 8,    // fixed 8 (must match SCUBA)
           parameter DEPTH = 256   // fixed 256 (must match SCUBA)
       )(
           // Inputs
           input  wire           wr_clk,
           input  wire           wr_rst_n,    // active-low
           input  wire           wr_en,
           input  wire [DW-1:0]  din,
           input  wire           rd_clk,
           input  wire           rd_rst_n,    // active-low
           input  wire           rd_en,

           // Outputs
           output wire [DW-1:0]  dout,
           output wire           rd_empty,
           output wire           wr_full
       );

// ============================================================================
// Internal Signals
// ============================================================================
wire           rst_hi;       // active-high reset to SCUBA
wire [DW-1:0]  q_w;          // SCUBA output (IP already has OUTREG)
wire           empty_w;
wire           full_w;
wire           aempty_w;     // kept for future use
wire           afull_w;      // kept for future use
wire           wr_en_ip;     // gated write enable (protect against full)
wire           rd_en_ip;     // gated read enable (protect against empty)

// ============================================================================
// Assignments
// ============================================================================
// Combine resets: any side asserting reset resets the FIFO
assign rst_hi   = ~(wr_rst_n & rd_rst_n);

// Gate enables: only drive IP when not full/empty
assign wr_en_ip = wr_en & ~full_w;
assign rd_en_ip = rd_en & ~empty_w;

// External connections
assign dout     = q_w;
assign rd_empty = empty_w;
assign wr_full  = full_w;

// ============================================================================
// Module Instantiations
// ============================================================================
// SCUBA-generated FIFO instance (commonly named fifo.v / fifo_netlist.v)
fifo u_fifo (
         .Data        (din),         // [7:0]
         .WrClock     (wr_clk),
         .RdClock     (rd_clk),
         .WrEn        (wr_en_ip),
         .RdEn        (rd_en_ip),
         .Reset       (rst_hi),      // active-high
         .RPReset     (rst_hi),      // active-high
         .Q           (q_w),         // [7:0]
         .Empty       (empty_w),
         .Full        (full_w),
         .AlmostEmpty (aempty_w),
         .AlmostFull  (afull_w)
     );

endmodule

