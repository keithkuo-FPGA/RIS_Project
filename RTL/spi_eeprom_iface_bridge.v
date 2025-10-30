`timescale 1ns/1ps

// ============================================================================
// Module: spi_eeprom_iface_bridge
// Description: Clock Domain Crossing (CDC) bridge for SPI EEPROM interface
//              - 3-stage CDC synchronizers for control signals
//              - Asynchronous FIFOs for TX/RX data paths
//              - Handshake protocol for address/control transfer
//              - Fixed FIFO read timing and RX data alignment
// ============================================================================

module spi_eeprom_iface_bridge (
           // System clock domain (x2osc_clk)
           input  wire        sys_clk,
           input  wire        clk,
           input  wire        rst_n,
           // EEPROM SPI interface
           output wire        sclk,
           output wire        cs_n,
           // Control signals (system clock domain)
           input  wire [23:0] addr,
           input  wire        addr_vld,
           input  wire        wr_rd_sel,
           input  wire        id_mem_sel,
           input  wire [7:0]  erase_ctrl,
           input  wire [9:0]  data_len,
           // TX path (system -> EEPROM)
           input  wire [7:0]  wdata,
           input  wire        wdata_vld,
           output wire        wdata_rdy,
           // RX path (EEPROM -> system)
           output wire [7:0]  rdata,
           output wire        rdata_vld,
           input  wire        rdata_rdy,
           // Status outputs
           output wire [9:0]  data_cntr,
           output wire        busy,
           // EEPROM bidirectional data lines
           inout  wire        d_dq0,
           inout  wire        q_dq1,
           inout  wire        w_n_dq2,
           inout  wire        hold_n_dq3
       );

// ==========================================================================
// Internal Signals - Wire Declarations
// ==========================================================================

// TX FIFO interface
wire        tx_full;
wire        tx_empty;
wire [7:0]  tx_dout;

// RX FIFO interface
wire        rx_full;
wire        rx_empty;
wire [7:0]  rx_dout;

// Internal RX FIFO signals
wire        rx_fifo_rd_en_int;
wire [7:0]  rx_fifo_dout_int;
wire        rx_fifo_empty_int;

// Core interface signals
wire [7:0]  core_rdata;
wire        core_rdata_vld;
wire        core_wdata_rdy;

// CDC helper signals
wire        addr_req_edge_eif;
wire        addr_ack_sync_sys;
wire        can_issue_req;

// ==========================================================================
// Internal Signals - Register Declarations
// ==========================================================================

// Address/control CDC registers (toggle synchronization)
reg         addr_req_tgl_sys;
reg         addr_ack_q1_sys;
reg         addr_ack_q2_sys;
reg         addr_ack_q3_sys;
reg         addr_req_q1_eif;
reg         addr_req_q2_eif;
reg         addr_req_q3_eif;
reg         addr_ack_tgl_eif;

// Address/control storage (system clock domain)
reg [23:0]  addr_sys;
reg         wr_rd_sys;
reg         id_sel_sys;
reg [9:0]   len_sys;

// Address/control storage (EEPROM interface clock domain)
reg [23:0]  addr_eif;
reg         wr_rd_eif;
reg         id_sel_eif;
reg [9:0]   len_eif;
reg         addr_vld_rise;

// TX path control registers
reg         tx_rd_en;
reg         tx_rd_en_q;
reg [7:0]   core_wdata;
reg [7:0]   core_wdata_q;
reg         core_wdata_vld;
reg         core_wdata_vld_q;

// Core ready signal synchronizers (3-stage)
reg         core_wdata_rdy_q1;
reg         core_wdata_rdy_q2;
reg         core_wdata_rdy_q3;

// RX FIFO full synchronizers (3-stage)
reg         rx_full_q1;
reg         rx_full_q2;
reg         rx_full_q3;

// RX path output buffer
reg [7:0]   rdata_buf;
reg         rdata_vld_buf;

// RX state machine
reg [1:0]   rx_state;

// ==========================================================================
// RX State Machine Parameters
// ==========================================================================

localparam RX_IDLE = 2'd0;
localparam RX_READ = 2'd1;
localparam RX_HOLD = 2'd2;

// ==========================================================================
// Assignments
// ==========================================================================

// CDC helper signals
assign addr_ack_sync_sys = addr_ack_q3_sys;
assign can_issue_req = (addr_req_tgl_sys == addr_ack_sync_sys);
assign addr_req_edge_eif = addr_req_q2_eif ^ addr_req_q3_eif;

// TX interface
assign wdata_rdy = ~tx_full;

// RX interface
assign rdata = rdata_buf;
assign rdata_vld = rdata_vld_buf;
assign rx_fifo_rd_en_int = (rx_state == RX_IDLE) && !rx_fifo_empty_int;

// ==========================================================================
// CDC: Address/Control Transfer (System -> EEPROM Interface Clock Domain)
// Toggle-based handshake with 3-stage synchronizers
// ==========================================================================

// System clock domain: Capture address/control and toggle request
always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin
        addr_req_tgl_sys <= 1'b0;
        addr_ack_q1_sys  <= 1'b0;
        addr_ack_q2_sys  <= 1'b0;
        addr_ack_q3_sys  <= 1'b0;
        addr_sys         <= 24'd0;
        wr_rd_sys        <= 1'b0;
        id_sel_sys       <= 1'b0;
        len_sys          <= 10'd0;
    end
    else begin
        // 3-stage synchronizer for acknowledge
        addr_ack_q1_sys <= addr_ack_tgl_eif;
        addr_ack_q2_sys <= addr_ack_q1_sys;
        addr_ack_q3_sys <= addr_ack_q2_sys;

        // Capture new address/control when not busy
        if (addr_vld && can_issue_req) begin
            addr_sys         <= addr;
            wr_rd_sys        <= wr_rd_sel;
            id_sel_sys       <= id_mem_sel;
            len_sys          <= data_len;
            addr_req_tgl_sys <= ~addr_req_tgl_sys;
        end
    end
end

// EEPROM clock domain: Synchronize request and generate acknowledge
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        addr_req_q1_eif  <= 1'b0;
        addr_req_q2_eif  <= 1'b0;
        addr_req_q3_eif  <= 1'b0;
        addr_ack_tgl_eif <= 1'b0;
    end
    else begin
        // 3-stage synchronizer for request
        addr_req_q1_eif <= addr_req_tgl_sys;
        addr_req_q2_eif <= addr_req_q1_eif;
        addr_req_q3_eif <= addr_req_q2_eif;

        // Toggle acknowledge on request edge detection
        if (addr_req_edge_eif) begin
            addr_ack_tgl_eif <= ~addr_ack_tgl_eif;
        end
    end
end

// EEPROM clock domain: Latch address/control on request edge
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        addr_eif      <= 24'd0;
        wr_rd_eif     <= 1'b0;
        id_sel_eif    <= 1'b0;
        len_eif       <= 10'd0;
        addr_vld_rise <= 1'b0;
    end
    else begin
        addr_vld_rise <= 1'b0;
        if (addr_req_edge_eif) begin
            addr_eif      <= addr_sys;
            wr_rd_eif     <= wr_rd_sys;
            id_sel_eif    <= id_sel_sys;
            len_eif       <= len_sys;
            addr_vld_rise <= 1'b1;
        end
    end
end

// ==========================================================================
// TX Path: System Clock Domain -> EEPROM Interface
// 3-stage pipeline to handle FIFO read latency
// ==========================================================================

// EEPROM clock domain: Synchronize core ready signal (3-stage)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        core_wdata_rdy_q1 <= 1'b0;
        core_wdata_rdy_q2 <= 1'b0;
        core_wdata_rdy_q3 <= 1'b0;
    end
    else begin
        core_wdata_rdy_q1 <= core_wdata_rdy;
        core_wdata_rdy_q2 <= core_wdata_rdy_q1;
        core_wdata_rdy_q3 <= core_wdata_rdy_q2;
    end
end

// EEPROM clock domain: TX FIFO read control with proper timing
// Stage 1: Issue read request
// Stage 2: Capture FIFO output (1 cycle latency)
// Stage 3: Present stable data to core
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        tx_rd_en         <= 1'b0;
        tx_rd_en_q       <= 1'b0;
        core_wdata       <= 8'd0;
        core_wdata_q     <= 8'd0;
        core_wdata_vld   <= 1'b0;
        core_wdata_vld_q <= 1'b0;
    end
    else begin
        // Stage 1: Issue read when FIFO has data and core is ready
        tx_rd_en <= 1'b0;
        if (!tx_empty && core_wdata_rdy_q3 && !tx_rd_en_q && !core_wdata_vld_q) begin
            tx_rd_en <= 1'b1;
        end

        // Stage 2: Capture FIFO output after 1 cycle delay
        tx_rd_en_q <= tx_rd_en;
        if (tx_rd_en_q) begin
            core_wdata_q     <= tx_dout;
            core_wdata_vld_q <= 1'b1;
        end
        else if (core_wdata_vld) begin
            core_wdata_vld_q <= 1'b0;
        end

        // Stage 3: Present stable data to core
        core_wdata     <= core_wdata_q;
        core_wdata_vld <= core_wdata_vld_q;
    end
end

// ==========================================================================
// RX Path: EEPROM Interface -> System Clock Domain
// State machine to handle FIFO read timing and output buffering
// ==========================================================================

// EEPROM clock domain: Synchronize RX FIFO full signal (3-stage)
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_full_q1 <= 1'b0;
        rx_full_q2 <= 1'b0;
        rx_full_q3 <= 1'b0;
    end
    else begin
        rx_full_q1 <= rx_full;
        rx_full_q2 <= rx_full_q1;
        rx_full_q3 <= rx_full_q2;
    end
end

// System clock domain: RX output state machine
// States: IDLE -> READ (wait for FIFO data) -> HOLD (wait for rdata_rdy)
always @(posedge sys_clk or negedge rst_n) begin
    if (!rst_n) begin
        rx_state      <= RX_IDLE;
        rdata_buf     <= 8'd0;
        rdata_vld_buf <= 1'b0;
    end
    else begin
        case (rx_state)
            // IDLE: Monitor FIFO for new data
            RX_IDLE: begin
                rdata_vld_buf <= 1'b0;
                if (!rx_fifo_empty_int) begin
                    rx_state <= RX_READ;
                end
            end

            // READ: Wait for FIFO data to be ready (1 cycle after rd_en)
            RX_READ: begin
                rdata_buf     <= rx_fifo_dout_int;
                rdata_vld_buf <= 1'b1;
                rx_state      <= RX_HOLD;
            end

            // HOLD: Wait for downstream to accept data
            RX_HOLD: begin
                if (rdata_rdy) begin
                    rdata_vld_buf <= 1'b0;
                    rx_state      <= RX_IDLE;
                end
            end

            default: begin
                rx_state <= RX_IDLE;
            end
        endcase
    end
end

// ==========================================================================
// Module Instantiations
// ==========================================================================

// TX Path: Async FIFO (System clk -> EEPROM interface clk)
async_fifo_dc #(
                  .DW    (8),
                  .DEPTH (512)
              ) u_tx_fifo (
                  .wr_clk   (sys_clk),
                  .wr_rst_n (rst_n),
                  .wr_en    (wdata_vld),
                  .din      (wdata),
                  .wr_full  (tx_full),
                  .rd_clk   (clk),
                  .rd_rst_n (rst_n),
                  .rd_en    (tx_rd_en),
                  .dout     (tx_dout),
                  .rd_empty (tx_empty)
              );

// RX Path: Async FIFO (EEPROM interface clk -> System clk)
async_fifo_dc #(
                  .DW    (8),
                  .DEPTH (512)
              ) u_rx_fifo (
                  .wr_clk   (clk),
                  .wr_rst_n (rst_n),
                  .wr_en    (core_rdata_vld),
                  .din      (core_rdata),
                  .wr_full  (rx_full),
                  .rd_clk   (sys_clk),
                  .rd_rst_n (rst_n),
                  .rd_en    (rx_fifo_rd_en_int),
                  .dout     (rx_fifo_dout_int),
                  .rd_empty (rx_fifo_empty_int)
              );

// SPI EEPROM Interface Core
spi_eeprom_iface u_core (
                     .clk        (clk),
                     .rst_n      (rst_n),
                     .sclk       (sclk),
                     .cs_n       (cs_n),
                     .d_dq0      (d_dq0),
                     .q_dq1      (q_dq1),
                     .w_n_dq2    (w_n_dq2),
                     .hold_n_dq3 (hold_n_dq3),
                     .addr       (addr_eif),
                     .addr_vld   (addr_vld_rise),
                     .wr_rd_sel  (wr_rd_eif),
                     .id_mem_sel (id_sel_eif),
                     .erase_ctrl (erase_ctrl),
                     .data_len   (len_eif),
                     .wdata      (core_wdata),
                     .wdata_rdy  (core_wdata_rdy),
                     .wdata_vld  (core_wdata_vld),
                     .rdata      (core_rdata),
                     .rdata_rdy  (~rx_full_q3),
                     .rdata_vld  (core_rdata_vld),
                     .data_cntr  (data_cntr),
                     .busy       (busy)
                 );

endmodule

