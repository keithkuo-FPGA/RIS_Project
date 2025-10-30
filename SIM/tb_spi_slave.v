`timescale 1ns/1ps
`include "../RTL/define/protocol_defs.vh"

module tb_system_top_v2;

// ============================================================================
// DUT I/O Signals
// ============================================================================
reg         sys_rst_n;
wire        spi_sclk, spi_cs_n, spi_mosi, spi_miso;
wire        eep_sclk, eep_cs_n, eep_dq0, eep_dq1, eep_dq2, eep_dq3;
wire [31:0] cloumn_o, row_o;
wire [1:0]  gpio_LED;
wire [1:0]  debug_o;
reg  [2:0]  hw_l2;
reg  [3:0]  hw_l1;
reg  [3:0]  hw_freq;

// ============================================================================
// SPI Master Interface
// ============================================================================
localparam integer TB_MAX_BITS = 4096;
localparam PRESENT_FIRST_BIT = 1'b1;

reg                   spi_start;
reg [15:0]            spi_bit_count;
reg [TB_MAX_BITS-1:0] spi_tx_bits;
wire [TB_MAX_BITS-1:0] spi_rx_bits;
wire                  spi_busy, spi_done;
reg                   spi_cpol, spi_cpha;

// ============================================================================
// Test Variables
// ============================================================================
integer err_count;
integer I, J, K, B, N_MOSI, TEST_ALL_CNT;
integer REQ_BYTES, TOTAL_BITS;
integer timeout_cnt;

// Buffers
reg [7:0] MOSI_BUF [0:255];
reg [7:0] RD_BUF [0:255];
reg [7:0] RX_BYTES [0:255];
reg [7:0] expected_pattern [0:127];

reg [7:0] test_pattern [0:127];
reg [7:0] test_pattern2 [0:127];
reg [7:0] test_pattern3 [0:127];
reg [31:0] expected_bram [0:31];

// Task temporary variables
reg [TB_MAX_BITS-1:0] build_bits_tmp;
reg [15:0]            build_nbits_tmp;
reg [7:0]             t_cmd_task;
reg [7:0]             rd_dummy_b, rd_data_b;

localparam integer RD_DUMMY_BYTES = 1;

// Task internal variables
integer   j;
reg       frame_ok;
reg       poll_ok;
integer   mismatch;
reg [7:0] erase_ctrl_val;
reg [7:0] read_val;
reg       read_ok;
integer   row_idx_task;
integer   mismatch_task;
reg [31:0] captured_data_task;
integer poll_cnt;


// ============================================================================
// Lattice Global Reset (Simulation Only)
// ============================================================================
`ifdef SIM
reg glbl_gsr, glbl_pur;
initial begin
    glbl_gsr = 1'b1;
    glbl_pur = 1'b1;
    #100;
    glbl_gsr = 1'b0;
    glbl_pur = 1'b0;
end

GSR GSR_INST (.GSR(glbl_gsr));
PUR PUR_INST (.PUR(glbl_pur));
`endif

// ============================================================================
// DUT Clock
// ============================================================================
wire dut_clk = u_dut.x2osc_clk;

// ============================================================================
// DUT Instantiation
// ============================================================================
system_top u_dut (
               .sys_rst_n      (sys_rst_n),
               .sclk           (spi_sclk),
               .cs_n           (spi_cs_n),
               .mosi           (spi_mosi),
               .miso           (spi_miso),
               .eep_sclk       (eep_sclk),
               .eep_cs_n       (eep_cs_n),
               .eep_dq0        (eep_dq0),
               .eep_dq1        (eep_dq1),
               .eep_dq2        (eep_dq2),
               .eep_dq3        (eep_dq3),
               .HW_L1_i        (hw_l1),
               .HW_L2_i        (hw_l2),
               .HW_Frequency_i (hw_freq),
               .debug_o        (debug_o),
               .cloumn_o       (cloumn_o),
               .row_o          (row_o),
               .gpio_LED       (gpio_LED)
           );

// ============================================================================
// SPI Master Instantiation
// ============================================================================
spi_master_top #(
                   .MAX_BITS(TB_MAX_BITS),
                   .CLK_DIV(1)
               ) u_spi_master (
                   .clk                (dut_clk),
                   .rst_n              (sys_rst_n),
                   .start              (spi_start),
                   .bit_count          (spi_bit_count),
                   .tx_bits            (spi_tx_bits),
                   .rx_bits            (spi_rx_bits),
                   .busy               (spi_busy),
                   .done               (spi_done),
                   .mode_awmf          (1'b0),
                   .use_pdi            (1'b0),
                   .cpol               (spi_cpol),
                   .cpha               (spi_cpha),
                   .present_first_bit  (PRESENT_FIRST_BIT),
                   .sclk               (spi_sclk),
                   .cs_n               (spi_cs_n),
                   .mosi               (spi_mosi),
                   .miso               (spi_miso),
                   .sdi                (),
                   .pdi                (),
                   .sdo                (1'b0)
               );

// ============================================================================
// EEPROM Model Instantiation
// ============================================================================
eeprom_m95p32_emu #(
                      .MEM_SIZE(1024)
                  ) u_eep_model (
                      .c          (eep_sclk),
                      .s_n        (eep_cs_n),
                      .d_dq0      (eep_dq0),
                      .q_dq1      (eep_dq1),
                      .w_n_dq2    (eep_dq2),
                      .hold_n_dq3 (eep_dq3)
                  );

// ============================================================================
// Pattern Initialization
// ============================================================================
initial begin
    for (I=0; I<128; I=I+1)
        test_pattern[I] = I[7:0];

    for (I=0; I<128; I=I+1)
        test_pattern2[I] = 8'hA5;

    for (I=0; I<128; I=I+1)
        test_pattern3[I] = $urandom() & 8'hFF;

    for (I=0; I<32; I=I+1)
        expected_bram[I] = {test_pattern[I*4+3], test_pattern[I*4+2],
                            test_pattern[I*4+1], test_pattern[I*4+0]};
end

// ============================================================================
// Helper Functions
// ============================================================================
function [7:0] make_cmd;
    input [1:0] bank;
    input       is_read;
    reg [3:0] mode_low;
    begin
        case (bank)
            2'd0:
                mode_low = 4'h8;
            2'd1:
                mode_low = 4'h4;
            2'd2:
                mode_low = 4'h2;
            default:
                mode_low = 4'h0;
        endcase
        make_cmd = {3'b000, is_read, mode_low};
    end
endfunction

function is_legal_status;
    input [7:0] status;
    begin
        case (status)
            8'h0F, 8'h0E, 8'h0B, 8'h07:
                is_legal_status = 1'b1;
            default:
                is_legal_status = 1'b0;
        endcase
    end
endfunction

// ============================================================================
// Frame Builder Tasks
// ============================================================================
task build_frame_write;
    input  [7:0]             t_cmd;
    input  [7:0]             t_regaddr;
    input  integer           t_n_mosi;
    output [TB_MAX_BITS-1:0] t_bits_out;
    output [15:0]            t_bitlen_out;
    begin
        REQ_BYTES    = 2 + t_n_mosi;
        TOTAL_BITS   = REQ_BYTES * 8;
        t_bitlen_out = TOTAL_BITS;
        t_bits_out   = {TB_MAX_BITS{1'b0}};
        t_bits_out[t_bitlen_out-1 - 8*0 -: 8] = t_cmd;
        t_bits_out[t_bitlen_out-1 - 8*1 -: 8] = t_regaddr;
        for (I = 0; I < t_n_mosi; I = I + 1)
            t_bits_out[t_bitlen_out-1 - 8*(2+I) -: 8] = MOSI_BUF[I];
    end
endtask

task build_frame_read1;
    input  [7:0]             t_cmd;
    input  [7:0]             t_regaddr;
    output [TB_MAX_BITS-1:0] t_bits_out;
    output [15:0]            t_bitlen_out;
    begin
        REQ_BYTES    = 4;
        TOTAL_BITS   = REQ_BYTES * 8;
        t_bitlen_out = TOTAL_BITS;
        t_bits_out   = {TB_MAX_BITS{1'b0}};
        t_bits_out[t_bitlen_out-1 - 8*0 -: 8] = t_cmd;
        t_bits_out[t_bitlen_out-1 - 8*1 -: 8] = t_regaddr;
        t_bits_out[t_bitlen_out-1 - 8*2 -: 8] = 8'h00;
        t_bits_out[t_bitlen_out-1 - 8*3 -: 8] = 8'h00;
    end
endtask

task build_frame_readN;
    input  [7:0]             t_cmd;
    input  [7:0]             t_regaddr;
    input  integer           n_read;
    input  integer           n_dummy;
    output [TB_MAX_BITS-1:0] t_bits_out;
    output [15:0]            t_bitlen_out;
    begin
        REQ_BYTES    = 2 + n_dummy + n_read;
        TOTAL_BITS   = REQ_BYTES * 8;
        t_bitlen_out = TOTAL_BITS;
        t_bits_out   = {TB_MAX_BITS{1'b0}};
        t_bits_out[t_bitlen_out-1 - 8*0 -: 8] = t_cmd;
        t_bits_out[t_bitlen_out-1 - 8*1 -: 8] = t_regaddr;
        for (K = 0; K < (n_dummy + n_read); K = K + 1)
            t_bits_out[t_bitlen_out-1 - 8*(2+K) -: 8] = 8'h00;
    end
endtask

// ============================================================================
// Basic SPI Operations
// ============================================================================
task do_spi_write;
    input [1:0]   bank;
    input [7:0]   regaddr;
    input integer n_bytes;
    input [127:0] tag;
    begin
        t_cmd_task = make_cmd(bank, 1'b0);
        build_frame_write(t_cmd_task, regaddr, n_bytes, build_bits_tmp, build_nbits_tmp);

        $display("[%0t] %s: SPI WRITE bank=%0d addr=0x%02h len=%0d",
                 $time, tag, bank, regaddr, n_bytes);

        @(posedge dut_clk);
        spi_tx_bits   <= build_bits_tmp;
        spi_bit_count <= build_nbits_tmp;
        spi_start     <= 1'b1;
        @(posedge dut_clk);
        spi_start     <= 1'b0;

        wait(spi_done);
        @(posedge dut_clk);
        $display("[%0t] %s: SPI WRITE Done", $time, tag);
    end
endtask

task do_spi_read1_raw;
    input  [1:0]   bank;
    input  [7:0]   regaddr;
    output [7:0]   read_value;
    output         read_success;
    input  [127:0] tag;

    integer timeout;
    begin
        t_cmd_task = make_cmd(bank, 1'b1);
        build_frame_read1(t_cmd_task, regaddr, build_bits_tmp, build_nbits_tmp);

        @(posedge dut_clk);
        spi_tx_bits   <= build_bits_tmp;
        spi_bit_count <= build_nbits_tmp;
        spi_start     <= 1'b1;
        @(posedge dut_clk);
        spi_start     <= 1'b0;

        timeout = 0;
        while (!spi_done && timeout < 5000) begin
            @(posedge dut_clk);
            timeout = timeout + 1;
        end

        if (timeout >= 5000) begin
            read_success = 1'b0;
            read_value   = 8'hFF;
        end
        else begin
            rd_dummy_b   = spi_rx_bits[15:8];
            rd_data_b    = spi_rx_bits[7:0];
            read_success = (rd_dummy_b == 8'h00);
            read_value   = rd_data_b;
        end

        @(posedge dut_clk);
    end
endtask

task do_spi_read1_and_check;
    input [1:0]   bank;
    input [7:0]   regaddr;
    input [7:0]   expect;
    input [127:0] tag;
    begin
        do_spi_read1_raw(bank, regaddr, read_val, read_ok, tag);

        if (!read_ok) begin
            $display("[%0t] %s: READ FAILED", $time, tag);
            err_count = err_count + 1;
        end
        else if (read_val !== expect) begin
            $display("[%0t] %s: MISMATCH - Got 0x%02h, Expected 0x%02h",
                     $time, tag, read_val, expect);
            err_count = err_count + 1;
        end
        else begin
            $display("[%0t] %s: PASS (0x%02h)", $time, tag, read_val);
        end
    end
endtask

task do_spi_readN;
    input [1:0]   bank;
    input [7:0]   regaddr;
    input integer n_read;
    input [127:0] tag;
    begin
        t_cmd_task = make_cmd(bank, 1'b1);
        build_frame_readN(t_cmd_task, regaddr, n_read, RD_DUMMY_BYTES,
                          build_bits_tmp, build_nbits_tmp);

        @(posedge dut_clk);
        spi_tx_bits   <= build_bits_tmp;
        spi_bit_count <= build_nbits_tmp;
        spi_start     <= 1'b1;
        @(posedge dut_clk);
        spi_start     <= 1'b0;

        wait(spi_done);
        @(posedge dut_clk);

        for (B = 0; B < (RD_DUMMY_BYTES + n_read); B = B + 1)
            RX_BYTES[B] = spi_rx_bits[(build_nbits_tmp-1 - 8*2) - 8*B -: 8];

        for (B = 0; B < n_read; B = B + 1)
            RD_BUF[B] = RX_BYTES[RD_DUMMY_BYTES + B];

        $display("[%0t] %s: READ %0d bytes from 0x%02h",
                 $time, tag, n_read, regaddr);
    end
endtask

// ============================================================================
// Frame Done Waiting Task
// ============================================================================
task wait_frame_done;
    input integer timeout_cycles;
    output        success;

    integer wait_cnt;
    reg     prev_frame;
    begin
        success    = 1'b0;
        wait_cnt   = 0;
        prev_frame = debug_o[0];

        while (wait_cnt < timeout_cycles) begin
            @(posedge dut_clk);
            if (debug_o[0] && !prev_frame) begin
                success  = 1'b1;
                wait_cnt = timeout_cycles;
            end
            prev_frame = debug_o[0];
            wait_cnt   = wait_cnt + 1;
        end
    end
endtask

// ============================================================================
// BRAM Verification Task
// ============================================================================
task verify_bram_content;
    input [127:0] tag;
    begin
        $display("[%0t] %s: Verifying BRAM Content...", $time, tag);
        mismatch_task = 0;
        repeat(10) @(posedge dut_clk);
        for (row_idx_task=0; row_idx_task<8; row_idx_task=row_idx_task+1) begin
            if (captured_data_task !== expected_bram[row_idx_task]) begin
                $display("  [ERROR] Row[%0d]: Got 0x%08h, Expected 0x%08h",
                         row_idx_task, captured_data_task, expected_bram[row_idx_task]);
                mismatch_task = mismatch_task + 1;
            end
            else begin
                $display("  [OK] Row[%0d]: 0x%08h", row_idx_task, captured_data_task);
            end
        end
        if (mismatch_task==0)
            $display("[%0t] %s: BRAM Verification PASSED ✓", $time, tag);
        else begin
            $display("[%0t] %s: BRAM Verification FAILED ✗ (%0d errors)",
                     $time, tag, mismatch_task);
            err_count = err_count + mismatch_task;
        end
    end
endtask

// ============================================================================
// Main Test Sequence
// ============================================================================
initial begin
    // Initialize
    sys_rst_n     = 1'b0;
    spi_start     = 1'b0;
    spi_bit_count = 16'd0;
    spi_tx_bits   = {TB_MAX_BITS{1'b0}};
    spi_cpol      = 1'b0;
    spi_cpha      = 1'b0;
    err_count     = 0;
    hw_l2         = 3'h0;
    hw_l1         = 4'h0;
    hw_freq       = 4'h2;
    poll_cnt = 0;
    poll_ok = 1'b0;

    // $dumpfile("tb_system_top_v2.vcd");
    // $dumpvars(0, tb_system_top_v2);

    #100;
    sys_rst_n = 1'b1;

    #200000;  // 等待 200us，確保 PLL 穩定
    wait(dut_clk);
    repeat(5000) @(posedge dut_clk);  // 再等 5000 cycles

    $display("[%0t] System Ready - Starting Tests", $time);

    $display("\n========================================");
    $display("  System Top Complete Test (V2)");
    $display("========================================\n");

    for (TEST_ALL_CNT = 0; TEST_ALL_CNT < 10; TEST_ALL_CNT = TEST_ALL_CNT + 1) begin
        // ========================================
        // Test 1: Basic Sanity Check
        // ========================================
        $display("\n--- Test 1: Basic Sanity Check ---");
        do_spi_read1_and_check(2'd0, 8'h00, `FPGA_VerH, "READ FPGA_VER_H");
        do_spi_read1_and_check(2'd0, 8'h01, `FPGA_VerL, "READ FPGA_VER_L");

        // ========================================
        // Test 2: 第一次 Pattern Write (等待 Frame Done)
        // ========================================
        $display("\n--- Test 2: First Pattern Write ---");

        MOSI_BUF[0] = 8'h0;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=0x00");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h00, 1, "SET ADDR_L=0x00");
        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h01, 1, "SET ADDR_H=0x00");
        MOSI_BUF[0] = 8'h01;
        do_spi_write(2'd2, 8'h02, 1, "SET EEP_OP=0x01");

        for (I = 0; I < 128; I = I + 1)
            MOSI_BUF[I] = test_pattern2[I];

        N_MOSI = 128;
        do_spi_write(2'd1, 8'h00, N_MOSI, "WRITE_PATTERN");
        repeat(50) @(posedge dut_clk);

        wait(debug_o[0]);
        $display("[%0t] LED Frame Done detected", $time);
        repeat(64) @(posedge dut_clk);

        // ========================================
        // Test 3: 第二次 Pattern Write (MODE_EEPROM_RW)
        // ========================================
        $display("\n--- Test 3: Second Pattern Write (EEPROM_RW) ---");

        MOSI_BUF[0] = `MODE_EEPROM_RW;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=EEPROM_RW");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h00, 1, "SET ADDR_L=0x00");
        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h01, 1, "SET ADDR_H=0x00");

        for (I = 0; I < 128; I = I + 1)
            MOSI_BUF[I] = test_pattern[I];

        N_MOSI = 128;
        do_spi_write(2'd1, 8'h00, N_MOSI, "WRITE_PATTERN");
        repeat(50) @(posedge dut_clk);

        // ⚠️ 這裡原始 TB 沒有觸發 EEP_OP，直接等待
        repeat(3064) @(posedge dut_clk);

        // ========================================
        // Test 4: EEPROM Read Trigger
        // ========================================
        $display("\n--- Test 4: EEPROM Read Trigger ---");

        MOSI_BUF[0] = `MODE_EEPROM_RW;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=EEPROM_RW");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h00, 1, "SET ADDR_L=0x00");
        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h01, 1, "SET ADDR_H=0x00");
        MOSI_BUF[0] = 8'h02;
        do_spi_write(2'd2, 8'h02, 1, "SET EEP_OP=READ");

        repeat(1000) @(posedge dut_clk);

        // Test 4: MCU Read Back from EEPROM via PAT Bank
        $display("\n--- Test 4: MCU ReadBack EEPROM Data ---");

        // Step 1: 觸發 EEPROM Read（已在 Test 4 完成）
        MOSI_BUF[0] = `MODE_EEPROM_RW;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=EEPROM_RW");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h00, 1, "SET ADDR_L");
        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h01, 1, "SET ADDR_H");
        MOSI_BUF[0] = 8'h02;  // Read operation
        do_spi_write(2'd2, 8'h02, 1, "Trigger EEPROM Read");

        // Step 2: 等待完成（輪詢 status）
        for (poll_cnt=0; poll_cnt<10 && !poll_ok; poll_cnt=poll_cnt+1) begin
            do_spi_read1_raw(2'd2, 8'h04, read_val, read_ok, "Poll Status");
            if (read_ok && (read_val == 8'h0F)) begin
                $display("  ✅ Read Complete!");
                poll_ok = 1'b1;
            end
            else begin
                repeat(50) @(posedge dut_clk);
            end
        end

        // Step 3: 從 PAT Bank 讀取 128 bytes
        do_spi_readN(2'd1, 8'h00, 128, "ReadBack from PAT");

        // Step 4: 比對數據
        mismatch = 0;
        for (I=0; I<128; I=I+1) begin
            if (RD_BUF[I] !== test_pattern[I]) begin
                $display("  [ERROR] Byte[%0d]: Got 0x%02h, Expect 0x%02h",
                         I, RD_BUF[I], test_pattern[I]);
                mismatch = mismatch + 1;
            end
        end

        if (!poll_ok) begin
            $display("  ⚠️ WARNING: Status polling timeout");
        end

        if (mismatch == 0)
            $display("✅ Test 4 PASSED - All 128 bytes match!");
        else begin
            $display("❌ Test 4 FAILED - %0d mismatches", mismatch);
            err_count = err_count + mismatch;
        end

        // ========================================
        // Test 5: MODE_FAST_SWITCH
        // ========================================
        $display("\n--- Test 5: MODE_FAST_SWITCH ---");

        MOSI_BUF[0] = `MODE_FAST_SWITCH;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=FAST_SWITCH");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd0, 8'hF0, 1, "SET FastSwitch ID=0x00");

        repeat(1500) @(posedge dut_clk);

        // Test 4: MCU Read Back from EEPROM via PAT Bank
        $display("\n--- Test 4: MCU ReadBack EEPROM Data ---");

        // Step 1: 觸發 EEPROM Read（已在 Test 4 完成）
        MOSI_BUF[0] = `MODE_EEPROM_RW;
        do_spi_write(2'd0, 8'h05, 1, "SET Mode=EEPROM_RW");

        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h00, 1, "SET ADDR_L");
        MOSI_BUF[0] = 8'h00;
        do_spi_write(2'd2, 8'h01, 1, "SET ADDR_H");
        MOSI_BUF[0] = 8'h02;  // Read operation
        do_spi_write(2'd2, 8'h02, 1, "Trigger EEPROM Read");

        // Step 2: 等待完成（輪詢 status）
        for (poll_cnt=0; poll_cnt<10 && !poll_ok; poll_cnt=poll_cnt+1) begin
            do_spi_read1_raw(2'd2, 8'h04, read_val, read_ok, "Poll Status");
            if (read_ok && (read_val == 8'h0F)) begin
                $display("  ✅ Read Complete!");
                poll_ok = 1'b1;
            end
            else begin
                repeat(50) @(posedge dut_clk);
            end
        end

        // Step 3: 從 PAT Bank 讀取 128 bytes
        do_spi_readN(2'd1, 8'h00, 128, "ReadBack from PAT");

        // Step 4: 比對數據
        mismatch = 0;
        for (I=0; I<128; I=I+1) begin
            if (RD_BUF[I] !== test_pattern[I]) begin
                $display("  [ERROR] Byte[%0d]: Got 0x%02h, Expect 0x%02h",
                         I, RD_BUF[I], test_pattern[I]);
                mismatch = mismatch + 1;
            end
        end

        if (!poll_ok) begin
            $display("  ⚠️ WARNING: Status polling timeout");
        end

        if (mismatch == 0)
            $display("✅ Test 4 PASSED - All 128 bytes match!");
        else begin
            $display("❌ Test 4 FAILED - %0d mismatches", mismatch);
            err_count = err_count + mismatch;
        end

        // ========================================
        // Final Report
        // ========================================
        repeat(200) @(posedge dut_clk);
    end
    $display("\n========================================");
    $display("  TEST SUMMARY (V2 Migration)");
    $display("========================================");
    $display("Total Errors: %0d", err_count);
    $display("========================================");

    if (err_count == 0) begin
        $display("🎉 ALL TESTS PASSED ✓");
        $display("========================================\n");
    end
    else begin
        $display("❌ TESTS FAILED - %0d error(s)", err_count);
        $display("========================================\n");
    end

    #1000 $finish;
end

// ============================================================================
// Monitors
// ============================================================================
initial begin
    #500000000;
    $display("\n[ERROR] Simulation Timeout!");
    $display("Last known state:");
    $display("  err_count = %0d", err_count);
    $finish;
end

always @(posedge gpio_LED[0])
    $display("[%0t] 🚨 GPIO_LED[0] toggled (Frame Done)", $time);

always @(posedge gpio_LED[1])
    $display("[%0t] 🚨 GPIO_LED[1] toggled (EEPROM Busy)", $time);

endmodule
