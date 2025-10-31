`timescale 1ns/1ps

// ============================================================================
// spi_slave_engine — SPI slave with EEPROM and 32x32 control
// ============================================================================
module spi_slave_engine #(
           parameter integer ADDR_BITS = 8,
           parameter integer DATA_BITS = 8,
           parameter [7:0]   ACK_BYTE  = 8'hA5,
           parameter integer CPOL      = 0,
           parameter integer CPHA      = 0
       )(
           // Inputs
           input  wire        rst_n,
           input  wire        sclk_i,
           input  wire        cs_n_i,
           input  wire        mosi_i,
           input  wire        pat_rx_done_clr,
           input  wire        eeprom_erase_clr,
           input  wire        cfg_clk,
           input  wire        cfg_tx_we,
           input  wire [1:0]  cfg_tx_bank,
           input  wire [7:0]  cfg_tx_addr,
           input  wire [7:0]  cfg_tx_wdata,
           input  wire        cfg_rx_re,
           input  wire [1:0]  cfg_rx_bank,
           input  wire [7:0]  cfg_rx_addr,

           // Outputs
           output wire        miso_o,
           output reg  [7:0]  cmd,
           output reg  [7:0]  addr,
           output reg  [7:0]  data,
           output reg         got_packet,
           output reg         led0_rx_done,
           output reg         led1_rx_done,
           output reg         pat_rx_128_done,
           output reg  [7:0]  eeprom_erase_ctrl,
           output reg         eeprom_erase_done,
           output reg  [7:0]  cfg_rx_rdata
       );

// ============================================================================
// Local Parameters
// ============================================================================
localparam integer MEM_SIZE        = 256;
localparam integer HEAD_SIZE       = 2;
localparam integer RD_DUMMY_BYTES  = 1;

// ============================================================================
// Register Declarations
// ============================================================================
// TX/RX memories (TX memories are readable by SPI; RX memories are written by SPI)
reg [7:0] mem_tx_sys [0:MEM_SIZE-1];
reg [7:0] mem_tx_pat [0:MEM_SIZE-1];
reg [7:0] mem_tx_eep [0:MEM_SIZE-1];
reg [7:0] mem_rx_sys [0:MEM_SIZE-1];
reg [7:0] mem_rx_pat [0:MEM_SIZE-1];
reg [7:0] mem_rx_eep [0:MEM_SIZE-1];

// RX/TX state
reg [7:0] rx_shift;
reg [7:0] rx_bit_cnt;
reg [7:0] rx_byte_cnt;
reg       rx_cmd_addr_done;
reg       rw_is_write;
reg [3:0] mode_low4;
reg [1:0] sel_bank;

reg [7:0] tx_shift;
reg [2:0] tx_bit_cnt;
reg [7:0] tx_byte_cnt;
reg       miso_reg;
reg [7:0] rd_index;  // reserved for future use
reg [7:0] tx_next;

// Pattern helper
reg       pat_arm;
reg [7:0] pat_cnt;

// EEPROM erase tracking (kept for completeness; not modified here)
reg [7:0] eep_erase_ctrl_q;
reg [7:0] eep_erase_ctrl_d;
reg       eep_erase_start_q;
reg       eep_erase_start_d;

// ============================================================================
// Integer Declarations
// ============================================================================
integer i;

// ============================================================================
// Assignments
// ============================================================================
assign miso_o = miso_reg;

// ============================================================================
// Initialization (kept as original behavior)
// ============================================================================
initial begin
    for (i = 0; i < MEM_SIZE; i = i + 1) begin
        mem_tx_sys[i] = 8'h00;
        mem_tx_pat[i] = 8'h00;
        mem_tx_eep[i] = 8'h00;
        mem_rx_sys[i] = 8'h00;
        mem_rx_pat[i] = 8'h00;
        mem_rx_eep[i] = 8'h00;
    end
    mem_tx_sys[8'h00] = 8'h94;  // for 2000-01-01
    mem_tx_sys[8'h01] = 8'h25;  //

    // EEPROM bank default values
    mem_tx_eep[8'h02] = 8'h00;  // Operation: 0 = none
    mem_tx_eep[8'h03] = 8'h00;  // Erase Ctrl: 0 = idle
    mem_tx_eep[8'h04] = 8'h0F;  // Erase Status: ready
end

// ============================================================================
// Bank mapping helper (kept as original behavior)
// ============================================================================
function [1:0] mode2bank;
    input [3:0] m;
    begin
        case (m)
            4'h8:
                mode2bank = 2'd0;  // SYS
            4'h4:
                mode2bank = 2'd1;  // PAT
            4'h2:
                mode2bank = 2'd2;  // EEP
            default:
                mode2bank = 2'd0;
        endcase
    end
endfunction

// ============================================================================
// Writable register check (extended to include EEP:0x03)
// ============================================================================
function is_writable_rx;
    input [1:0] b;
    input [7:0] a;
    begin
        case (b)
            2'd0: begin  // SYS
                case (a)
                    8'h04, 8'h05, 8'hF0, 8'hF1:
                        is_writable_rx = 1'b1;
                    default:
                        is_writable_rx = 1'b0;
                endcase
            end
            2'd1:
                is_writable_rx = 1'b1;  // PAT: all writable
            2'd2: begin                   // EEP
                case (a)
                    8'h00, 8'h01, 8'h02, 8'h03:
                        is_writable_rx = 1'b1;
                    default:
                        is_writable_rx = 1'b0;
                endcase
            end
            default:
                is_writable_rx = 1'b0;
        endcase
    end
endfunction

function use_rx_for_read;
    input [1:0] b;
    input [7:0] a;
    begin
        use_rx_for_read = is_writable_rx(b, a);
    end
endfunction

// ============================================================================
// CFG interface: write TX memories, read RX memories
// ============================================================================
always @(posedge cfg_clk or negedge rst_n) begin
    if (!rst_n) begin
        cfg_rx_rdata <= 8'h00;
    end
    else begin
        if (cfg_tx_we) begin
            case (cfg_tx_bank)
                2'd0:
                    mem_tx_sys[cfg_tx_addr] <= cfg_tx_wdata;
                2'd1:
                    mem_tx_pat[cfg_tx_addr] <= cfg_tx_wdata;
                default:
                    mem_tx_eep[cfg_tx_addr] <= cfg_tx_wdata;
            endcase
        end
        if (cfg_rx_re) begin
            case (cfg_rx_bank)
                2'd0:
                    cfg_rx_rdata <= mem_rx_sys[cfg_rx_addr];
                2'd1:
                    cfg_rx_rdata <= mem_rx_pat[cfg_rx_addr];
                default:
                    cfg_rx_rdata <= mem_rx_eep[cfg_rx_addr];
            endcase
        end
    end
end

// ============================================================================
// SPI modes: Mode0/Mode3 use posedge sample (RX), negedge shift (TX)
// ============================================================================
generate
    if ((CPOL == 0 && CPHA == 0) || (CPOL == 1 && CPHA == 1)) begin : POSRX_NEGTX
        // RX: Sample MOSI on posedge sclk_i, reset on cs_n_i/rst_n; build CMD/ADDR/DATA
        always @(posedge sclk_i or posedge cs_n_i or negedge rst_n) begin
            if (!rst_n) begin
                rx_shift         <= 8'd0;
                rx_bit_cnt       <= 8'd0;
                rx_byte_cnt      <= 8'd0;
                rx_cmd_addr_done <= 1'b0;
                cmd              <= 8'd0;
                addr             <= 8'd0;
                data             <= 8'd0;
                got_packet       <= 1'b0;
                rw_is_write      <= 1'b0;
                mode_low4        <= 4'h0;
                sel_bank         <= 2'd0;
                pat_arm          <= 1'b0;
                pat_cnt          <= 8'd0;
            end
            else if (cs_n_i) begin
                rx_shift         <= 8'd0;
                rx_bit_cnt       <= 8'd0;
                rx_byte_cnt      <= 8'd0;
                rx_cmd_addr_done <= 1'b0;
                cmd              <= 8'd0;
                addr             <= 8'd0;
                data             <= 8'd0;
                got_packet       <= 1'b0;
                rw_is_write      <= 1'b0;
                mode_low4        <= 4'h0;
                sel_bank         <= 2'd0;
                pat_arm          <= 1'b0;
                pat_cnt          <= 8'd0;
            end
            else begin
                rx_shift   <= {rx_shift[6:0], mosi_i};
                rx_bit_cnt <= rx_bit_cnt + 8'd1;

                if (rx_bit_cnt == 8'd7) begin
                    rx_bit_cnt  <= 8'd0;
                    rx_byte_cnt <= rx_byte_cnt + 8'd1;

                    // Byte 0: CMD
                    if (rx_byte_cnt == 8'd0) begin
                        cmd         <= {rx_shift[6:0], mosi_i};
                        rw_is_write <= ~rx_shift[3];
                        mode_low4   <= {rx_shift[2:0], mosi_i};
                        sel_bank    <= mode2bank({rx_shift[2:0], mosi_i});
                        pat_arm     <= 1'b0;
                        pat_cnt     <= 8'd0;
                    end
                    // Byte 1: ADDR
                    else if (rx_byte_cnt == 8'd1) begin
                        addr[7:0] <= {rx_shift[6:0], mosi_i};
                        if (rw_is_write && (sel_bank == 2'd1) && ({rx_shift[6:0], mosi_i} == 8'h00)) begin
                            pat_arm <= 1'b1;
                            pat_cnt <= 8'd0;
                        end
                        else begin
                            pat_arm <= 1'b0;
                            pat_cnt <= 8'd0;
                        end
                        if (!rw_is_write) begin
                            rx_cmd_addr_done <= 1'b1;
                        end
                    end
                    // Byte 2+: DATA
                    else begin
                        data       <= {rx_shift[6:0], mosi_i};
                        got_packet <= 1'b1;

                        if (rw_is_write) begin
                            if (is_writable_rx(sel_bank, addr[7:0] + (rx_byte_cnt - HEAD_SIZE))) begin
                                case (sel_bank)
                                    2'd0:
                                        mem_rx_sys[addr[7:0] + (rx_byte_cnt - HEAD_SIZE)] <= {rx_shift[6:0], mosi_i};
                                    2'd1:
                                        mem_rx_pat[addr[7:0] + (rx_byte_cnt - HEAD_SIZE)] <= {rx_shift[6:0], mosi_i};
                                    default:
                                        mem_rx_eep[addr[7:0] + (rx_byte_cnt - HEAD_SIZE)] <= {rx_shift[6:0], mosi_i};
                                endcase
                            end
                        end

                        if (pat_arm && (sel_bank == 2'd1) && (rx_byte_cnt >= HEAD_SIZE)) begin
                            pat_cnt <= addr[7:0] + (rx_byte_cnt - HEAD_SIZE) + 8'd1;
                        end
                    end
                end
            end
        end

        // TX: Shift MISO on negedge sclk_i, send dummy then data; reset on cs_n_i/rst_n
        always @(negedge sclk_i or posedge cs_n_i or negedge rst_n) begin
            if (!rst_n) begin
                tx_shift    <= 8'd0;
                tx_bit_cnt  <= 3'd0;
                tx_byte_cnt <= 8'd0;
                miso_reg    <= 1'b0;
                rd_index    <= 8'd0;
                tx_next     <= 8'd0;
            end
            else if (cs_n_i) begin
                tx_shift    <= 8'd0;
                tx_bit_cnt  <= 3'd0;
                tx_byte_cnt <= 8'd0;
                miso_reg    <= 1'b0;
                rd_index    <= 8'd0;
                tx_next     <= 8'd0;
            end
            else begin
                if (rx_cmd_addr_done) begin
                    if (tx_bit_cnt == 3'd7) begin
                        if (!rw_is_write) begin
                            if (tx_byte_cnt + 1 < RD_DUMMY_BYTES[7:0]) begin
                                tx_next <= 8'h00;
                            end
                            else begin
                                case (sel_bank)
                                    2'd0: begin
                                        if (use_rx_for_read(2'd0, addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0]))) begin
                                            tx_next <= mem_rx_sys[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                        else begin
                                            tx_next <= mem_tx_sys[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                    end
                                    2'd1: begin
                                        if (mem_rx_eep[2] == 8'h02 || mem_rx_sys[5] == 8'h02) begin
                                            tx_next <= mem_tx_pat[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                        else begin
                                            tx_next <= mem_rx_pat[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                    end
                                    default: begin
                                        if (use_rx_for_read(2'd2, addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0]))) begin
                                            tx_next <= mem_rx_eep[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                        else begin
                                            tx_next <= mem_tx_eep[addr[7:0] + (tx_byte_cnt + 8'd1 - RD_DUMMY_BYTES[7:0])];
                                        end
                                    end
                                endcase
                            end
                        end
                        else begin
                            tx_next <= 8'h00;
                        end
                    end

                    if (tx_bit_cnt == 3'd0) begin
                        tx_shift   <= tx_next;
                        miso_reg   <= tx_next[7];
                        tx_bit_cnt <= 3'd1;
                    end
                    else if (tx_bit_cnt < 3'd7) begin
                        tx_shift   <= {tx_shift[6:0], 1'b0};
                        miso_reg   <= tx_shift[6];
                        tx_bit_cnt <= tx_bit_cnt + 3'd1;
                    end
                    else begin
                        miso_reg    <= tx_shift[6];
                        tx_bit_cnt  <= 3'd0;
                        tx_byte_cnt <= tx_byte_cnt + 8'd1;
                    end
                end
                else begin
                    miso_reg <= 1'b0;
                end
            end
        end
    end
    else begin : NEG_RX_POS_TX
        // RX (other SPI modes): placeholder to preserve reset behavior
        always @(posedge sclk_i or negedge rst_n) begin
            if (!rst_n) begin
                cmd        <= 8'd0;
                addr       <= 8'd0;
                data       <= 8'd0;
                got_packet <= 1'b0;
            end
        end

        // TX (other SPI modes): drive 0 on MISO
        always @(negedge sclk_i or negedge rst_n) begin
            if (!rst_n) begin
                miso_reg <= 1'b0;
            end
            else begin
                miso_reg <= 1'b0;
            end
        end
    end
endgenerate

// ============================================================================
// Pattern and erase flags: detect specific write completions on sclk_i domain
// ============================================================================
always @(posedge sclk_i or posedge pat_rx_done_clr or posedge eeprom_erase_clr or negedge rst_n) begin
    if (!rst_n) begin
        pat_rx_128_done  <= 1'b0;
        eeprom_erase_done <= 1'b0;
        eeprom_erase_ctrl <= 8'd0;
    end
    else if (pat_rx_done_clr) begin
        pat_rx_128_done <= 1'b0;
    end
    else if (eeprom_erase_clr) begin
        eeprom_erase_done <= 1'b0;
    end
    else begin
        if (pat_arm && (sel_bank == 2'd1) && (rx_byte_cnt >= HEAD_SIZE) &&
                (rx_bit_cnt == 8'd7) && (pat_cnt == 8'd127)) begin
            pat_rx_128_done <= 1'b1;
        end
        else if (rw_is_write && (sel_bank == 2'd2) && (addr == 8'h02) &&
                 (rx_byte_cnt >= HEAD_SIZE) && (rx_bit_cnt == 8'd7) &&
                 ({rx_shift[6:0], mosi_i} == 8'h02)) begin
            pat_rx_128_done <= 1'b1;
        end
        else if (rw_is_write && (sel_bank == 2'd0) && (addr == 8'hF0 || addr == 8'hF1) &&
                 (rx_byte_cnt >= HEAD_SIZE) && (rx_bit_cnt == 8'd7)) begin
            pat_rx_128_done <= 1'b1;
        end
        else if (rw_is_write && (sel_bank == 2'd2) && (addr == 8'h03) &&
                 (rx_byte_cnt >= HEAD_SIZE) && (rx_bit_cnt == 8'd7)) begin
            eeprom_erase_done <= 1'b1;
            eeprom_erase_ctrl <= {rx_shift[6:0], mosi_i};
        end
    end
end

// ============================================================================
// LED flags: sample specific RX_SYS bits into LED indicators on cfg_clk domain
// ============================================================================
always @(posedge cfg_clk or negedge rst_n) begin
    if (!rst_n) begin
        led0_rx_done <= 1'b0;
        led1_rx_done <= 1'b0;
    end
    else begin
        led0_rx_done <= mem_rx_sys[4][0];
        led1_rx_done <= mem_rx_sys[4][1];
    end
end

endmodule

