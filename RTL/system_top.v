//----------------------------------------------------------------------------------------------------------------------
// Module Name: system_top.v
// Design Name:
// Description: Processor
//
// Company: TMYTEK
// Engineer: Keith - keith_kuo@tmytek.com
// Created Date: 07/10/2025
// Revision: 0.0.001 (look release_notes)
//
// Standard Language: Verilog
// Target Devices:
// Tool Versions:
//
// COPYRIGHT 2025 BY TMYTEK
// THE INFORMATION CONTAINED IN THIS DOCUMENT IS SUBJECT TO LICENSING RESTRICTIONS
// FROM TMYTEK CORP. IF YOU ARE NOT IN POSSESSION OF WRITTEN AUTHORIZATION FROM
// TMYTEK FOR USE OF THIS FILE, THEN THE FILE SHOULD BE IMMEDIATELY DESTROYED AND
// NO BACK-UP OF THE FILE SHOULD BE MADE.
//----------------------------------------------------------------------------------------------------------------------
`timescale 1 ps / 1 ps

// `define SIM

module system_top #
       (
           parameter integer         SPI_DEBUG_EN               = 0
       )
       (
           // System Clock 100MHz
           // input   wire        	  sysclk_clk_n,	            //	System Clock
           // input   wire        	  sysclk_clk_p,		        //	System Clock

           // Reset
           input    wire                    sys_rst_n,               // B3 , CREST, RST_FPGA_B
           // UART

           // SPI Slave
           input    wire                    sclk,
           input    wire                    cs_n,
           input    wire                    mosi,
           output   wire                    miso,


           // SPI Master EEPROM
           output   wire                    eep_sclk,
           output   wire                    eep_cs_n,
           inout    wire                    eep_dq0,
           inout    wire                    eep_dq1,
           inout    wire                    eep_dq2,
           inout    wire                    eep_dq3,

           // Clock
`ifdef SIM
           // debug
           output   wire   [1:0]            debug_o,
`endif
           // GPIO
           input    wire   [3:0]            HW_L1_i,
           input    wire   [2:0]            HW_L2_i,
           input    wire   [3:0]            HW_Frequency_i,
           output   wire   [31:0]           cloumn_o,
           output   wire   [31:0]           row_o,
           output   wire   [1:0]            gpio_LED


       );
// ---------------- Local params ----------------
localparam FRAME_SEL_WIDTH = 1;

`ifdef SIM
localparam [24:0] HALF_SEC_MAX = 25'd2000; //for 66.5MHz
`else
localparam [24:0] HALF_SEC_MAX = 25'd33249999; //for 66.5MHz
`endif

//-------------------------------------------------------------
wire                        Osc_Clk, Osc_Clk_EEP;
wire                        SEDSTDBY;
wire                        x2osc_clk;       // for cfg/BRAM/LED
wire                        eif_clk_x2;      // for EEPROM iface 99.75Mz
wire                        debug_clk;

//----------------- SYS HW VER --------------------------------
reg  [2:0]                  hw_l2_meta, hw_l2_sync;
reg  [3:0]                  hw_l1_meta, hw_l1_sync;
reg  [7:0]                  hw_ver_shadow;
wire [7:0]                  hw_ver_byte;

reg  [3:0]                  hw_frequency_meta, hw_frequency_sync;
reg  [7:0]                  hw_freq_shadow;
wire [7:0]                  hw_freq_byte;

//----------------- SYS LED Ctrl -------------------------------
reg  [24:0]                 tick_cnt;               // 共用 0.5s 計數器
reg                         halfsec_tick;           // 共用 tick（1 個 clk 週期寬）

wire                        led0_rx_done_s;         // from slave (sclk_i domain)
wire                        led1_rx_done_s;         // from slave (sclk_i domain)
reg                         led0_q;                 // LED0 輸出狀態
reg                         led1_q;                 // LED1 輸出狀態
wire [1:0]                  led_mode;

// ---------------- SPI slave interface -----------------------
wire [7:0]                  spi_cmd;
wire [7:0]                  spi_addr;
wire [7:0]                  spi_data;
wire                        spi_got;

reg                         cfg_tx_we;
reg  [1:0]                  cfg_tx_bank;
reg  [7:0]                  cfg_tx_addr;
reg  [7:0]                  cfg_tx_wdata;
reg                         cfg_rx_re;
reg  [1:0]                  cfg_rx_bank;
reg  [7:0]                  cfg_rx_addr;
wire [7:0]                  cfg_rx_rdata;

// ---------------- EEPROM interface --------------------------
wire                        eif_busy;
reg                         eif_addr_vld;
reg                         eif_wr_rd_n;
reg                         eif_mem_sel;
reg [23:0]                  eif_addr;
reg [9:0]                   eif_len;
reg [7:0]                   eif_wdata;
wire                        eif_wready;
reg                         eif_wvalid;
wire [7:0]                  eif_rdata;
reg                         eif_rready;
wire                        eif_rvalid;
wire [9:0]                  eif_cnt;

// wire                        eep_ctrl_change_s;
// reg                         eep_ctrl_d1, eep_ctrl_d2;
// wire                        eep_ctrl_rise;

reg  [7:0]                 sys_pat_mode; // SYS:0x05
reg  [7:0]                 eep_op_now;   // EEP:0x02
reg  [7:0]                 eep_addr_h;   // EEP:0x00
reg  [7:0]                 eep_addr_l;   // EEP:0x01
reg                        load_from_eep;// 0:從 PAT(mem_rx_pat)；1:從 EEPROM 讀入

reg                        eif_wready_q1, eif_wready_q2;
reg                        eif_rvalid_q1, eif_rvalid_q2;
reg                        eif_busy_q1,   eif_busy_q2;

wire                       eif_wready_sync = eif_wready_q2;
wire                       eif_rvalid_sync = eif_rvalid_q2;
wire                       eif_busy_sync   = eif_busy_q2;
reg  [9:0]                 wr_cnt;

reg                        eep_disp_ready;

wire                       eeprom_erase_done_s;
wire [7:0]                 eeprom_erase_ctrl_s;
reg  [7:0]                 erase_ctrl_q1, erase_ctrl_q2;
wire [7:0]                 erase_ctrl_sync;
reg  [7:0]                 eif_erase_ctrl;

reg                        erase_done_d1, erase_done_d2;
wire                       erase_done_rise;
reg                        erase_auto_pend;
reg                        erase_auto_pend_ack;
reg                        slave_erase_clr;      // 回寫到 slave 的清旗標
reg  [3:0]                 slave_erase_clr_cnt;  // 伸展清旗標
reg                        auto_erase_kick_in_progress;
reg                        auto_erase_kick_set_pulse;

reg  [1:0]                 eep_byte_cnt;   // 0..3：每 4 個 byte 組一列
reg                        eep_rd_need_display;  // 1=需要顯示(Fast Switch), 0=僅供MCU(R/W mode)

reg  [3:0]                 eep_status_reg;       // Status 寄存器本體
reg                        eep_write_set;        // Write
reg                        eep_read_set;         // Read
reg                        eep_erase_set;        // Erase

// ---------------- LED(32x32) interface -----------------------------
reg                        start_stream;
reg  [FRAME_SEL_WIDTH-1:0] frame_sel_disp;
wire                       row_valid;
wire [4:0]                 row_idx;
wire [31:0]                row_data;
wire                       busy_stream;
wire                       done_stream;
wire                       refresh_pulse, frame_done;
reg                        stream_start_armed;

reg                        restart_req;          // FSM -> pulser
reg                        start_ack;            // pulser -> FSM
reg                        led_enable;           // 在載入期間關閉，PDONE 開啟
reg                        disp_gate;
wire                       drv_enable_gate;
reg  [3:0]                 enable_delay_cnt;

// ---------------- BRAM 256x32 ----------------
reg                        bram_we_a;
reg  [7:0]                 bram_addr_a;
reg  [31:0]                bram_wdata_a;

wire                       bram_rd_en_b;
wire [7:0]                 bram_rd_addr_b;
wire [31:0]                bram_rd_data_b;

// ---------------- SYS 協定暫存 ----------------
reg  [7:0]                 sys_frame_sel;
reg  [7:0]                 sys_pat_seq,  pat_seq_seen;
reg  [7:0]                 sys_eep_seq,  eep_seq_seen;
reg  [23:0]                sys_eep_addr;
reg  [7:0]                 sys_eep_len;

// ---------------- 主控 FSM ----------------
// 主狀態
localparam ST_IDLE          = 5'd0;   // 閒置等待

// Pattern Control Mode 判斷流程
localparam ST_READ_MODE     = 5'd1;   // 讀取 SYS:0x05 (Pattern Control Mode)

// Fast Pattern Switch 流程
localparam ST_FAST_ID_L     = 5'd2;   // 讀取 SYS:0xF0 (Fast Switch ID Low)
localparam ST_FAST_ID_H     = 5'd3;   // 讀取 SYS:0xF1 (Fast Switch ID High)

// EEPROM R/W 流程
localparam ST_EEP_OP        = 5'd4;   // 讀取 EEP:0x02 (Operation Mode)
localparam ST_EEP_ID_L      = 5'd5;   // 讀取 EEP:0x00 (Pattern ID Low)
localparam ST_EEP_ID_H      = 5'd6;   // 讀取 EEP:0x01 (Pattern ID High)

// Erase 流程
localparam ST_ERASE_CTRL    = 5'd7;   // 讀取 EEP:0x03 (Erase Control)
localparam ST_ERASE_ID_L    = 5'd8;   // 讀取 Erase Pattern ID Low
localparam ST_ERASE_ID_H    = 5'd9;   // 讀取 Erase Pattern ID High

// Pattern 載入流程（PAT 或 EEPROM → BRAM）
localparam ST_PAT_BYTE0     = 5'd10;  // 載入 Pattern Byte 0
localparam ST_PAT_BYTE1     = 5'd11;  // 載入 Pattern Byte 1
localparam ST_PAT_BYTE2     = 5'd12;  // 載入 Pattern Byte 2
localparam ST_PAT_BYTE3     = 5'd13;  // 載入 Pattern Byte 3 並寫入 BRAM
localparam ST_PAT_DONE      = 5'd14;  // Pattern 載入完成，啟動顯示

// EEPROM 寫入流程（PAT → EEPROM）
localparam ST_EEP_WR_START  = 4'd15;  // EEPROM 寫入啟動

// EEPROM Read
localparam ST_EEP_RD_DATA   = 5'd16;  //
// ============================================================================
// Pattern Control 相關定義
// ============================================================================
localparam MODE_DIRECT      = 8'h00;  // 直接從 PAT buffer 顯示
localparam MODE_EEPROM_RW   = 8'h01;  // EEPROM 讀寫模式
localparam MODE_FAST_SWITCH = 8'h02;  // Fast Pattern Switch 模式

localparam EEP_OP_NONE      = 8'h00;
localparam EEP_OP_WRITE     = 8'h01;  // PAT → EEPROM
localparam EEP_OP_READ      = 8'h02;  // EEPROM → BRAM
localparam EEP_OP_ID_WR     = 8'h03;
localparam EEP_OP_ID_RD     = 8'h04;


reg  [4:0]                 state;
reg                        cfg_pipe_valid;
reg  [FRAME_SEL_WIDTH-1:0] next_frame_sel;
reg  [4:0]                 pat_row;
reg  [15:0]                pat_src_index;
reg  [31:0]                pat_assemble;

// ---------------- Slave PAT Flag and clear ----------------
wire                       pat_rx_128_done_s;   // from slave (sclk_i domain)
reg                        pat_done_d1;         // sync 1
reg                        pat_done_d2;         // sync 2
wire                       pat_done_rise;       // 上升沿（同步到 x2osc_clk）
reg                        pat_auto_pend;
reg                        pat_auto_pend_ack;
reg                        pat_auto_consume_pulse;

reg                        slave_pat_clr;      // 回寫到 slave 的清旗標
reg  [3:0]                 slave_pat_clr_cnt;  // 伸展清旗標
reg                        auto_kick_in_progress;
reg                        auto_kick_set_pulse;
// wire                       auto_kick_set;      // 在 ST_DECIDE 看到上升沿時觸發
reg                        scan_kick_pulse;
reg                        scan_pending;
reg                        force_pat_clr_pulse;

reg                        fsm_busy;
reg                        pend_overflow;
reg  [1:0]                 pend_count;
//-------------------------------------------------------------

//-------------------------------------------------------------
//=============================================================
// Ver
assign hw_ver_byte = {1'b0, hw_l2_sync, hw_l1_sync};
// frequency
assign hw_freq_byte = {4'd0, hw_frequency_sync};

// SPI

// GPIO
//

// HW Reset
// assign  bd_reset = CPU_RESET;

// LED Output
assign gpio_LED[0] = led0_q;
assign gpio_LED[1] = led1_q;

assign led_mode = {led1_rx_done_s, led0_rx_done_s};
assign drv_enable_gate = disp_gate & led_enable & (pat_rx_128_done_s | eep_disp_ready) & (enable_delay_cnt == 4'd6);

//
assign pat_done_rise = (~pat_done_d2) & pat_done_d1;
assign erase_done_rise = (~erase_done_d2) & erase_done_d1;
assign erase_ctrl_sync = erase_ctrl_q2;

// EEPROM
// assign eep_ctrl_rise = (eep_ctrl_d1 & ~eep_ctrl_d2);

// debug
assign debug_o = frame_done;

// ---------------- HW VER -------------------------------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        hw_l2_meta <= 3'd0;
        hw_l2_sync <= 3'd0;
        hw_l1_meta <= 4'd0;
        hw_l1_sync <= 4'd0;
        hw_frequency_meta <= 4'd0;
        hw_frequency_sync <= 4'd0;
    end
    else begin
        hw_l2_meta <= HW_L2_i;
        hw_l2_sync <= hw_l2_meta;
        hw_l1_meta <= HW_L1_i;
        hw_l1_sync <= hw_l1_meta;
        hw_frequency_meta <= HW_Frequency_i;
        hw_frequency_sync <= hw_frequency_meta;
    end
end

// ---------------- LED Auto-Kick ------------------------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        tick_cnt            <= 25'd0;
        halfsec_tick        <= 1'b0;
        enable_delay_cnt    <= 4'd0;
    end
    else begin
        if (tick_cnt == HALF_SEC_MAX) begin
            tick_cnt     <= 25'd0;
            halfsec_tick <= 1'b1;   // 僅此拍為 1
        end
        else begin
            tick_cnt     <= tick_cnt + 25'd1;
            halfsec_tick <= 1'b0;
        end


        if (led_enable && enable_delay_cnt < 4'd6)
            enable_delay_cnt <= enable_delay_cnt + 1;
        else if (!led_enable)
            enable_delay_cnt <= 0;
    end
end


always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        led0_q    <= 1'b0;
        led1_q    <= 1'b0;
    end
    else begin
        case (led_mode)
            2'b00: begin
                // 關
                led0_q <= 1'b1;
                led1_q <= 1'b0;
            end

            2'b01: begin
                // LED0 閃爍（0.5s），啟動當拍定相為 1
                led1_q <= 1'b0;
                if (halfsec_tick) begin
                    led0_q <= ~led0_q;      // 之後每 0.5s 翻轉
                end
            end

            2'b10: begin
                // LED1 閃爍（0.5s），啟動當拍定相為 1
                led0_q <= 1'b0;
                if (halfsec_tick) begin
                    led1_q <= ~led1_q;
                end
            end

            default: begin
                // 2'b11：LED1 恆亮（Error），LED0 關
                led0_q <= 1'b0;
                led1_q <= 1'b1;
            end
        endcase
    end
end

// ---------------- - ----------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        eif_wready_q1 <= 1'b0;
        eif_wready_q2 <= 1'b0;
        eif_rvalid_q1 <= 1'b0;
        eif_rvalid_q2 <= 1'b0;
        eif_busy_q1   <= 1'b0;
        eif_busy_q2   <= 1'b0;
    end
    else begin
        eif_wready_q1 <= eif_wready;  // 跨域輸入
        eif_wready_q2 <= eif_wready_q1;

        eif_rvalid_q1 <= eif_rvalid;  // 跨域輸入
        eif_rvalid_q2 <= eif_rvalid_q1;

        eif_busy_q1   <= eif_busy;    // 若 ST_EK 用到了 busy，一併同步
        eif_busy_q2   <= eif_busy_q1;
    end
end


// ---------------- Synchronization & Auto-Kick ----------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pat_done_d1           <= 1'b0;
        pat_done_d2           <= 1'b0;
        erase_done_d1         <= 1'b0;
        erase_done_d2         <= 1'b0;
        pat_auto_pend         <= 1'b0;
        auto_kick_in_progress <= 1'b0;
        slave_pat_clr_cnt     <= 4'd0;
        slave_pat_clr         <= 1'b0;
    end
    else begin
        pat_done_d1 <= pat_rx_128_done_s;
        pat_done_d2 <= pat_done_d1;

        erase_done_d1 <= eeprom_erase_done_s;
        erase_done_d2 <= erase_done_d1;

        erase_ctrl_q1  <= eeprom_erase_ctrl_s;
        erase_ctrl_q2  <= erase_ctrl_q1;

        // Any rising edge → set pending flag (decoupled from FSM state)
        if (pat_done_rise)
            pat_auto_pend <= 1'b1;
        else if (pat_auto_pend_ack)
            pat_auto_pend <= 1'b0;

        if (erase_done_rise)
            erase_auto_pend <= 1'b1;
        else if (erase_auto_pend_ack)
            erase_auto_pend <= 1'b0;

        // After auto-kick starts, wait for first frame_done to clear slave flag
        if (pat_done_rise)
            auto_kick_in_progress <= 1'b1;
        else if (auto_kick_in_progress && frame_done)
            auto_kick_in_progress <= 1'b0;

        if (auto_kick_in_progress && frame_done)
            slave_pat_clr_cnt <= 4'd8;
        else if (force_pat_clr_pulse)
            slave_pat_clr_cnt <= 4'd8;
        else if (slave_pat_clr_cnt != 4'd0)
            slave_pat_clr_cnt <= slave_pat_clr_cnt - 4'd1;

        slave_pat_clr <= (slave_pat_clr_cnt != 4'd0) ? 1'b1 : 1'b0;

        // ERASE
        if (erase_done_rise)
            auto_erase_kick_in_progress <= 1'b1;
        else if (auto_erase_kick_in_progress)
            auto_erase_kick_in_progress <= 1'b0;

        if (auto_erase_kick_in_progress)
            slave_erase_clr_cnt <= 4'd8;
        else if (force_pat_clr_pulse)
            slave_erase_clr_cnt <= 4'd8;
        else if (slave_erase_clr_cnt != 4'd0)
            slave_erase_clr_cnt <= slave_erase_clr_cnt - 4'd1;

        slave_erase_clr <= (slave_erase_clr_cnt != 4'd0) ? 1'b1 : 1'b0;
    end
end

// ---------------- Streamer Start Control ----------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        start_stream <= 1'b0;
        start_ack    <= 1'b0;
    end
    else begin
        start_stream <= 1'b0;   // 預設 0，條件達成送 1 拍
        start_ack    <= 1'b0;
        if (!busy_stream && restart_req) begin
            start_stream <= 1'b1;
            start_ack    <= 1'b1;
        end
    end
end

// ---------------- Slave EEP Flag ----------------
// always @(posedge x2osc_clk or negedge sys_rst_n) begin
//     if (!sys_rst_n) begin
//         eep_ctrl_d1 <= 1'b0;
//         eep_ctrl_d2 <= 1'b0;
//     end
//     else begin
//         eep_ctrl_d1 <= eep_ctrl_change_s;
//         eep_ctrl_d2 <= eep_ctrl_d1;
//     end
// end

// ---------------- Main FSM (x2osc_clk domain) ----------------
always @(posedge x2osc_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        state                <= ST_IDLE;
        cfg_rx_re            <= 1'b0;
        cfg_rx_bank          <= 2'd0;
        cfg_rx_addr          <= 8'd0;
        cfg_tx_we            <= 1'b0;
        cfg_tx_bank          <= 2'd0;
        cfg_tx_addr          <= 8'd0;
        cfg_tx_wdata         <= 8'd0;

        eif_addr_vld         <= 1'b0;
        eif_wr_rd_n          <= 1'b0;
        eif_mem_sel          <= 1'b0;
        eif_addr             <= 24'd0;
        eif_len              <= 10'd0;
        eif_wdata            <= 8'd0;
        eif_wvalid           <= 1'b0;
        eif_rready           <= 1'b0;
        eif_erase_ctrl       <= 8'd0;

        next_frame_sel       <= {FRAME_SEL_WIDTH{1'b0}};
        frame_sel_disp       <= {FRAME_SEL_WIDTH{1'b0}};
        pat_row              <= 5'd0;
        pat_src_index        <= 16'd0;
        pat_assemble         <= 32'd0;

        bram_we_a            <= 1'b0;
        bram_addr_a          <= 8'd0;
        bram_wdata_a         <= 32'd0;

        led_enable           <= 1'b0;
        disp_gate            <= 1'b0;
        force_pat_clr_pulse  <= 1'b0;
        restart_req          <= 1'b1;

        pat_auto_pend_ack    <= 1'b0;
        erase_auto_pend_ack  <= 1'b0;
        cfg_pipe_valid       <= 1'b0;

        hw_ver_shadow        <= 8'hFF;
        hw_freq_shadow       <= 8'hFF;
        wr_cnt               <= 10'd0;
        eep_disp_ready       <= 1'b0;

        sys_pat_mode         <= 8'd0;
        eep_op_now           <= 8'd0;
        eep_addr_h           <= 8'd0;
        eep_addr_l           <= 8'd0;
        load_from_eep        <= 1'b0;
        eep_byte_cnt         <= 2'd0;
        eep_rd_need_display  <= 1'b0;

        eep_status_reg       <= 4'hF;
        eep_write_set        <= 1'b0;
        eep_read_set         <= 1'b0;
        eep_erase_set        <= 1'b0;
    end
    else begin
        cfg_rx_re            <= 1'b0;
        bram_we_a            <= 1'b0;
        pat_auto_pend_ack    <= 1'b0;
        erase_auto_pend_ack  <= 1'b0;
        force_pat_clr_pulse  <= 1'b0;
        eif_addr_vld         <= 1'b0;
        cfg_tx_we            <= 1'b0;

        if (frame_done) begin
            eep_disp_ready   <= 1'b0;
            led_enable       <= 1'b0;
            disp_gate        <= 1'b0;
        end

        if (cfg_pipe_valid)
            cfg_pipe_valid   <= 1'b0;

        if (start_ack)
            restart_req      <= 1'b0;

        // HW Version 更新
        if (hw_ver_byte != hw_ver_shadow && state == ST_IDLE) begin
            cfg_tx_bank   <= 2'd0;
            cfg_tx_addr   <= 8'h02;
            cfg_tx_wdata  <= hw_ver_byte;
            cfg_tx_we     <= 1'b1;
            hw_ver_shadow <= hw_ver_byte;
        end
        else if (hw_freq_byte != hw_freq_shadow && state == ST_IDLE) begin
            cfg_tx_bank    <= 2'd0;
            cfg_tx_addr    <= 8'h03;
            cfg_tx_wdata   <= hw_freq_byte;
            cfg_tx_we      <= 1'b1;
            hw_freq_shadow <= hw_freq_byte;
        end

        case (state)
            ST_IDLE: begin
                if ((eep_write_set | eep_read_set | eep_erase_set) && !eif_busy_sync) begin
                    cfg_tx_bank  <= 2'd2;  // EEP bank
                    cfg_tx_addr  <= 8'h04;
                    if (eep_erase_set) begin
                        eep_erase_set <= 1'b0;
                        cfg_tx_wdata <= {4'b0000, (eep_status_reg | 4'b0011)};
                        eep_status_reg <= (eep_status_reg | 4'b0011);
                    end
                    else if (eep_write_set) begin
                        eep_write_set <= 1'b0;
                        cfg_tx_wdata <= {4'b0000, (eep_status_reg | 4'b1010)};
                        eep_status_reg <= (eep_status_reg | 4'b1010);
                    end
                    else if (eep_read_set) begin
                        eep_read_set <= 1'b0;
                        cfg_tx_wdata <= {4'b0000, (eep_status_reg | 4'b0110)};
                        eep_status_reg <= (eep_status_reg | 4'b0110);
                    end
                    cfg_tx_we    <= 1'b1;
                end
                else if (erase_auto_pend) begin
                    cfg_rx_bank    <= 2'd2;
                    cfg_rx_addr    <= 8'h03;
                    cfg_rx_re      <= 1'b1;
                    cfg_pipe_valid <= 1'b1;
                    state          <= ST_ERASE_CTRL;
                end
                else if (pat_auto_pend) begin
                    eep_disp_ready <= 1'b0;
                    disp_gate      <= 1'b0;

                    cfg_rx_bank    <= 2'd0;
                    cfg_rx_addr    <= 8'h05;
                    cfg_rx_re      <= 1'b1;
                    cfg_pipe_valid <= 1'b1;
                    state          <= ST_READ_MODE;
                end
            end

            ST_READ_MODE: begin
                if (!cfg_pipe_valid) begin
                    sys_pat_mode <= cfg_rx_rdata;

                    if (cfg_rx_rdata == MODE_FAST_SWITCH) begin
                        // Fast Switch: EEPROM → BRAM
                        led_enable          <= 1'b0;
                        force_pat_clr_pulse <= 1'b1;

                        cfg_rx_bank    <= 2'd0;
                        cfg_rx_addr    <= 8'hF0;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_FAST_ID_L;

                        if(!eep_read_set) begin
                            eep_read_set  <= 1'b1;
                            cfg_tx_bank  <= 2'd2;  // EEP bank
                            cfg_tx_addr  <= 8'h04;
                            cfg_tx_wdata <= {4'b0000, eep_status_reg & 4'b1011};
                            cfg_tx_we    <= 1'b1;
                            eep_status_reg  <= eep_status_reg & 4'b1011;
                        end
                    end
                    else if (cfg_rx_rdata == MODE_EEPROM_RW) begin
                        // EEPROM R/W: 需要進一步判斷 Operation
                        led_enable          <= 1'b0;
                        force_pat_clr_pulse <= 1'b1;

                        cfg_rx_bank    <= 2'd2;
                        cfg_rx_addr    <= 8'h02;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_EEP_OP;
                    end
                    else begin
                        // Direct Mode: PAT → BRAM → 顯示
                        load_from_eep  <= 1'b0;
                        next_frame_sel <= frame_sel_disp;
                        pat_row        <= 5'd0;
                        pat_src_index  <= 16'd0;
                        led_enable     <= 1'b0;

                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= 8'd0;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_PAT_BYTE0;
                    end
                end
            end

            // ====== Fast Switch 流程 ======
            ST_FAST_ID_L: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_l     <= cfg_rx_rdata;

                    cfg_rx_bank    <= 2'd0;
                    cfg_rx_addr    <= 8'hF1;
                    cfg_rx_re      <= 1'b1;
                    cfg_pipe_valid <= 1'b1;
                    state          <= ST_FAST_ID_H;
                end
            end

            ST_FAST_ID_H: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_h <= cfg_rx_rdata;

                    // Fast Switch: 啟動 EEPROM 讀取並載入 BRAM
                    eif_addr       <= {8'h00, cfg_rx_rdata, eep_addr_l, 7'd0};
                    eif_len        <= 10'd128;
                    eif_wr_rd_n    <= 1'b0;  // Read
                    eif_mem_sel    <= 1'b0;
                    eif_addr_vld   <= 1'b1;

                    load_from_eep  <= 1'b1;  // 標記從 EEPROM 載入
                    next_frame_sel <= frame_sel_disp;
                    pat_row        <= 5'd0;
                    pat_assemble   <= 32'd0;
                    eif_rready     <= 1'b1;

                    eep_rd_need_display <= 1'b1;  // Fast Switch 一定要顯示

                    state             <= ST_PAT_BYTE0;  // 進入 BRAM 載入流程
                    pat_auto_pend_ack <= 1'b1;
                end
            end

            // ====== EEPROM R/W 流程 ======
            ST_EEP_OP: begin
                if (!cfg_pipe_valid) begin
                    eep_op_now <= cfg_rx_rdata;

                    if (cfg_rx_rdata == EEP_OP_WRITE || cfg_rx_rdata == EEP_OP_READ || cfg_rx_rdata == EEP_OP_ID_WR || cfg_rx_rdata == EEP_OP_ID_RD) begin
                        cfg_rx_bank    <= 2'd2;
                        cfg_rx_addr    <= 8'h00;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_EEP_ID_L;
                    end
                    else begin
                        pat_auto_pend_ack <= 1'b1;
                        state             <= ST_IDLE;
                    end
                end
            end

            ST_EEP_ID_L: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_l     <= cfg_rx_rdata;

                    cfg_rx_bank    <= 2'd2;
                    cfg_rx_addr    <= 8'h01;
                    cfg_rx_re      <= 1'b1;
                    cfg_pipe_valid <= 1'b1;
                    state          <= ST_EEP_ID_H;
                end
            end

            ST_EEP_ID_H: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_h <= cfg_rx_rdata;
                    eif_addr   <= {8'h00, cfg_rx_rdata, eep_addr_l, 7'd0};
                    eif_len    <= 10'd128;

                    if (eep_op_now == EEP_OP_WRITE || eep_op_now == EEP_OP_ID_WR) begin
                        // Write: PAT → EEPROM
                        eif_wr_rd_n    <= 1'b1;  // Write
                        eif_addr_vld   <= 1'b1;
                        eif_mem_sel    <= (eep_op_now == EEP_OP_ID_WR) ? 1'b1 : 1'b0;

                        wr_cnt         <= 10'd0;
                        pat_src_index  <= 16'd0;
                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= 8'd0;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;

                        pat_auto_pend_ack <= 1'b1;
                        state             <= ST_EEP_WR_START;
                    end
                    else begin
                        // Read: EEPROM → mem_rx_eep（不顯示！）
                        eif_wr_rd_n    <= 1'b0;  // Read
                        eif_addr_vld   <= 1'b1;
                        eif_mem_sel    <= (eep_op_now == EEP_OP_ID_RD) ? 1'b1 : 1'b0;

                        pat_src_index  <= 16'd0;  // 用於計數
                        eif_rready     <= 1'b1;

                        // 判斷是否需要顯示
                        if (sys_pat_mode == MODE_EEPROM_RW) begin
                            eep_rd_need_display <= 1'b0;  // 不顯示，僅寫 mem_tx_pat
                        end
                        else begin
                            eep_rd_need_display <= 1'b1;  // 需要顯示（預設安全值）
                        end

                        pat_auto_pend_ack <= 1'b1;  // 立即清除 pending
                        state             <= ST_EEP_RD_DATA;  // 進入專用狀態

                        if(!eep_read_set) begin
                            eep_read_set  <= 1'b1;
                            cfg_tx_bank  <= 2'd2;  // EEP bank
                            cfg_tx_addr  <= 8'h04;
                            cfg_tx_wdata <= {4'b0000, eep_status_reg & 4'b1011};
                            cfg_tx_we    <= 1'b1;
                            eep_status_reg  <= eep_status_reg & 4'b1011;
                        end
                    end
                end
            end

            // EEPROM Read → mem_tx_pat（供 MCU 讀取）
            ST_EEP_RD_DATA: begin
                // 建議在該 always 頂端或這裡先清預設值（避免連續寫）
                bram_we_a  <= 1'b0;
                // eif_rready <= 1'b0;

                if (eif_rvalid_sync) begin
                    eif_rready <= 1'b1;

                    // 原有：鏡寫到 SPI Slave 的 mem_tx_pat[]（給 MCU，保留）
                    cfg_tx_bank  <= 2'd1;                   // EEP bank
                    cfg_tx_addr  <= pat_src_index[7:0];     // 0..127
                    cfg_tx_wdata <= eif_rdata;
                    cfg_tx_we    <= 1'b1;
                    pat_src_index <= pat_src_index + 16'd1;

                    // 新增：4 個 byte 組成一個 32-bit，寫 BRAM_A 的一列
                    if (eep_rd_need_display) begin
                        case (eep_byte_cnt)
                            2'd0:
                                pat_assemble[7:0]   <= eif_rdata;
                            2'd1:
                                pat_assemble[15:8]  <= eif_rdata;
                            2'd2:
                                pat_assemble[23:16] <= eif_rdata;
                            2'd3: begin
                                pat_assemble[31:24] <= eif_rdata;

                                // 寫入 BRAM
                                bram_addr_a  <= {next_frame_sel, pat_row};
                                bram_wdata_a <= {eif_rdata, pat_assemble[23:0]};
                                bram_we_a    <= 1'b1;
                                pat_row      <= pat_row + 5'd1;
                            end
                        endcase

                        if (eep_byte_cnt == 2'd3)
                            eep_byte_cnt <= 2'd0;
                        else
                            eep_byte_cnt <= eep_byte_cnt + 2'd1;

                        // 128 Bytes 完成後進入顯示流程
                        if ((eep_byte_cnt == 2'd3) && (pat_row == 5'd31)) begin
                            eep_disp_ready <= 1'b1;
                            load_from_eep  <= 1'b0;
                            state          <= ST_PAT_DONE;
                        end
                    end
                    else begin
                        // 不需要顯示的情況：僅計數，不寫 BRAM
                        if (pat_src_index == 16'd128) begin
                            // 128 Bytes 讀取完成，直接回 IDLE（不觸發顯示）
                            state <= ST_IDLE;
                        end
                    end
                end
            end


            // ====== Erase 流程 ======
            ST_ERASE_CTRL: begin
                if (!cfg_pipe_valid) begin
                    eif_erase_ctrl      <= erase_ctrl_sync;

                    cfg_rx_bank         <= 2'd2;
                    cfg_rx_addr         <= 8'h00;
                    cfg_rx_re           <= 1'b1;
                    cfg_pipe_valid      <= 1'b1;
                    force_pat_clr_pulse <= 1'b1;

                    state               <= ST_ERASE_ID_L;
                end
            end

            ST_ERASE_ID_L: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_l     <= cfg_rx_rdata;

                    cfg_rx_bank    <= 2'd2;
                    cfg_rx_addr    <= 8'h01;
                    cfg_rx_re      <= 1'b1;
                    cfg_pipe_valid <= 1'b1;
                    state          <= ST_ERASE_ID_H;
                end
            end

            ST_ERASE_ID_H: begin
                if (!cfg_pipe_valid) begin
                    eep_addr_h <= cfg_rx_rdata;

                    if ((eif_erase_ctrl[2:1]) == 2'b00) begin
                        eif_addr <= 24'd0;
                    end
                    else begin
                        eif_addr <= {8'h00, cfg_rx_rdata, eep_addr_l, 7'd0};
                    end

                    eif_wr_rd_n         <= 1'b1;
                    eif_addr_vld        <= 1'b1;

                    state               <= ST_IDLE;
                    erase_auto_pend_ack <= 1'b1;
                end
            end

            // ====== Pattern 載入流程（PAT 或 EEPROM → BRAM）======
            ST_PAT_BYTE0: begin
                if (!load_from_eep) begin
                    if (!cfg_pipe_valid) begin
                        pat_assemble[7:0] <= cfg_rx_rdata;

                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= pat_src_index + 16'd1;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_PAT_BYTE1;
                    end
                end
                else begin
                    if (eif_rvalid_sync) begin
                        pat_assemble[7:0] <= eif_rdata;
                        eif_rready        <= 1'b1;
                        state             <= ST_PAT_BYTE1;
                    end
                end
            end

            ST_PAT_BYTE1: begin
                if (!load_from_eep) begin
                    if (!cfg_pipe_valid) begin
                        pat_assemble[15:8] <= cfg_rx_rdata;

                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= pat_src_index + 16'd2;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_PAT_BYTE2;
                    end
                end
                else begin
                    if (eif_rvalid_sync) begin
                        pat_assemble[15:8] <= eif_rdata;
                        eif_rready         <= 1'b1;
                        state              <= ST_PAT_BYTE2;
                    end
                end
            end

            ST_PAT_BYTE2: begin
                if (!load_from_eep) begin
                    if (!cfg_pipe_valid) begin
                        pat_assemble[23:16] <= cfg_rx_rdata;

                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= pat_src_index + 16'd3;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        state          <= ST_PAT_BYTE3;
                    end
                end
                else begin
                    if (eif_rvalid_sync) begin
                        pat_assemble[23:16] <= eif_rdata;
                        eif_rready          <= 1'b1;
                        state               <= ST_PAT_BYTE3;
                    end
                end
            end

            ST_PAT_BYTE3: begin
                if (!load_from_eep) begin
                    if (!cfg_pipe_valid) begin
                        pat_assemble[31:24] <= cfg_rx_rdata;

                        bram_addr_a  <= {next_frame_sel, pat_row};
                        bram_wdata_a <= {cfg_rx_rdata, pat_assemble[23:0]};
                        bram_we_a    <= 1'b1;

                        pat_row       <= pat_row + 5'd1;
                        pat_src_index <= pat_src_index + 16'd4;

                        if (pat_row == 5'd31) begin
                            state <= ST_PAT_DONE;
                        end
                        else begin
                            cfg_rx_bank    <= 2'd1;
                            cfg_rx_addr    <= pat_src_index + 16'd4;
                            cfg_rx_re      <= 1'b1;
                            cfg_pipe_valid <= 1'b1;
                            state          <= ST_PAT_BYTE0;
                        end
                    end
                end
                else begin
                    if (eif_rvalid_sync) begin
                        pat_assemble[31:24] <= eif_rdata;
                        eif_rready          <= 1'b1;

                        bram_addr_a  <= {next_frame_sel, pat_row};
                        bram_wdata_a <= {eif_rdata, pat_assemble[23:0]};
                        bram_we_a    <= 1'b1;

                        pat_row <= pat_row + 5'd1;

                        if (pat_row == 5'd31) begin
                            eep_disp_ready <= 1'b1;
                            load_from_eep  <= 1'b0;
                            state          <= ST_PAT_DONE;
                        end
                        else begin
                            state <= ST_PAT_BYTE0;
                        end
                    end
                end
            end

            ST_PAT_DONE: begin
                // ✅ 載入完成，啟動顯示
                frame_sel_disp      <= next_frame_sel;
                led_enable          <= 1'b1;
                disp_gate           <= 1'b1;
                restart_req         <= 1'b1;

                pat_auto_pend_ack   <= 1'b1;
                erase_auto_pend_ack <= 1'b1;

                state <= ST_IDLE;
            end

            // ====== EEPROM Write 流程 ======
            ST_EEP_WR_START: begin
                if (!cfg_pipe_valid && !eif_wvalid && (eif_busy_sync || eif_wready_sync)) begin
                    eif_wdata  <= cfg_rx_rdata;
                    eif_wvalid <= 1'b1;
                end

                if (eif_wvalid && eif_wready_sync) begin
                    eif_wvalid <= 1'b0;
                    wr_cnt     <= wr_cnt + 10'd1;

                    if (wr_cnt >= (eif_len - 10'd1)) begin
                        state <= ST_IDLE;
                    end
                    else begin
                        cfg_rx_bank    <= 2'd1;
                        cfg_rx_addr    <= pat_src_index + 16'd1;
                        cfg_rx_re      <= 1'b1;
                        cfg_pipe_valid <= 1'b1;
                        pat_src_index  <= pat_src_index + 16'd1;
                    end
                end

                if(!eep_write_set) begin
                    eep_write_set  <= 1'b1;
                    cfg_tx_bank  <= 2'd2;  // EEP bank
                    cfg_tx_addr  <= 8'h04;
                    cfg_tx_wdata <= {4'b0000, eep_status_reg & 4'b0111};
                    cfg_tx_we    <= 1'b1;
                    eep_status_reg  <= eep_status_reg & 4'b0111;
                end
            end

            default: begin
                state <= ST_IDLE;
            end
        endcase
    end
end



//=============================================================
pll pll_u0 (
        .CLKI       (Osc_Clk  ),
        .CLKOP      (x2osc_clk)
        // .CLKOS      (eif_clk_x2)
        // .CLKOS2     (debug_clk)
    );

pll2 pll_u1 (
         .CLKI       (Osc_Clk),
         .CLKOP      (eif_clk_x2)
     );

OSCH #(.NOM_FREQ("133"))
     OSCH_u0 (
         .STDBY     (1'b0    ), // 0=Enabled, 1=Disabled
         .OSC       (Osc_Clk ),
         .SEDSTDBY  (SEDSTDBY)
     );

dp_bram_256x32 u_bram (
                   .clk             (x2osc_clk),
                   .rst_n           (sys_rst_n),
                   .we_a            (bram_we_a),
                   .addr_a          (bram_addr_a),
                   .wdata_a         (bram_wdata_a),
                   .rd_en_b         (bram_rd_en_b),
                   .addr_b          (bram_rd_addr_b),
                   .rdata_b         (bram_rd_data_b)
               );

spi_slave_engine #(.CPOL(0), .CPHA(0)) u_spi (
                     //  .clk                   (x2osc_clk),
                     .rst_n                 (sys_rst_n),
                     .sclk_i                (sclk),
                     .cs_n_i                (cs_n),
                     .mosi_i                (mosi),
                     .miso_o                (miso),
                     .cmd                   (spi_cmd),
                     .addr                  (spi_addr),
                     .data                  (spi_data),
                     .got_packet            (spi_got),
                     .led0_rx_done          (led0_rx_done_s),
                     .led1_rx_done          (led1_rx_done_s),
                     .pat_rx_128_done       (pat_rx_128_done_s),
                     .pat_rx_done_clr       (slave_pat_clr),
                     .eeprom_erase_ctrl     (eeprom_erase_ctrl_s),
                     .eeprom_erase_done     (eeprom_erase_done_s),
                     .eeprom_erase_clr      (slave_erase_clr),
                     .cfg_clk               (x2osc_clk),
                     .cfg_tx_we             (cfg_tx_we),
                     .cfg_tx_bank           (cfg_tx_bank),
                     .cfg_tx_addr           (cfg_tx_addr),
                     .cfg_tx_wdata          (cfg_tx_wdata),
                     .cfg_rx_re             (cfg_rx_re),
                     .cfg_rx_bank           (cfg_rx_bank),
                     .cfg_rx_addr           (cfg_rx_addr),
                     .cfg_rx_rdata          (cfg_rx_rdata)
                 );

led32x32_row_streamer #(.FRAME_SEL_WIDTH(FRAME_SEL_WIDTH)) u_stream (
                          .clk              (x2osc_clk),
                          .rst_n            (sys_rst_n),
                          .start            (start_stream),
                          .frame_sel        (frame_sel_disp),
                          .bram_rd_en       (bram_rd_en_b),
                          .bram_rd_addr     (bram_rd_addr_b),
                          .bram_rd_data     (bram_rd_data_b),
                          .row_valid        (row_valid),
                          .row_idx          (row_idx),
                          .row_data         (row_data),
                          .busy             (busy_stream),
                          .done             (done_stream)
                      );


led32x32_matrix_driver #(.REFRESH_DIVIDER(2)) u_drv (
                           .clk             (x2osc_clk),
                           .rst_n           (sys_rst_n),
                           .enable          (drv_enable_gate),
                           .row_valid       (row_valid),
                           .row_idx         (row_idx),
                           .row_data        (row_data),
                           .led_row         (row_o),
                           .led_col         (cloumn_o),
                           .refresh_pulse   (refresh_pulse),
                           .frame_done      (frame_done)
                       );


spi_eeprom_iface_bridge u_eep (
                            .sys_clk        (x2osc_clk),
                            .clk            (eif_clk_x2),
                            .rst_n          (sys_rst_n),
                            .sclk           (eep_sclk),
                            .cs_n           (eep_cs_n),
                            .d_dq0          (eep_dq0),
                            .q_dq1          (eep_dq1),
                            .w_n_dq2        (eep_dq2),
                            .hold_n_dq3     (eep_dq3),
                            .addr           (eif_addr),
                            .addr_vld       (eif_addr_vld),
                            .wr_rd_sel      (eif_wr_rd_n),
                            .id_mem_sel     (eif_mem_sel),
                            .erase_ctrl     (eif_erase_ctrl),
                            .data_len       (eif_len),
                            .wdata          (eif_wdata),
                            .wdata_rdy      (eif_wready),
                            .wdata_vld      (eif_wvalid),
                            .rdata          (eif_rdata),
                            .rdata_rdy      (eif_rready),
                            .rdata_vld      (eif_rvalid),
                            .data_cntr      (),
                            .busy           (eif_busy)
                        );




generate
    if(SPI_DEBUG_EN==1) begin: SPI_ILA_DEBUG








































    end
endgenerate

endmodule
