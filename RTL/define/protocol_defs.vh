`define SIM
// ============================================================================
// protocol_defs.vh - FPGA 版本
// ============================================================================
`define FPGA_VerH          8'h94
`define FPGA_VerL          8'h22

// ============================================================================
// protocol_defs.vh - 系統協議定義
// ============================================================================

// 命令類型定義
`define CMD_IDLE              8'h00
`define CMD_TRIGGER_DISPLAY   8'h01  // 觸發顯示（Direct Mode）
`define CMD_TRIGGER_EEPROM    8'h02  // 觸發 EEPROM 操作
`define CMD_FAST_SWITCH       8'h03  // Fast Switch
`define CMD_EEPROM_ERASE      8'h04  // Erase

// Pattern Control Mode
`define MODE_DIRECT        8'h00
`define MODE_EEPROM_RW     8'h01
`define MODE_FAST_SWITCH   8'h02

// EEPROM Operation
`define EEP_OP_NONE        8'h00
`define EEP_OP_WRITE       8'h01
`define EEP_OP_READ        8'h02
`define EEP_OP_ID_WR       8'h03
`define EEP_OP_ID_RD       8'h04
// Erase Type
`define ERASE_TYPE_PAGE    2'b00
`define ERASE_TYPE_SECTOR  2'b01
`define ERASE_TYPE_BLOCK   2'b10
`define ERASE_TYPE_CHIP    2'b11

// 狀態定義
`define TASK_STATUS_IDLE   4'h0
`define TASK_STATUS_BUSY   4'h1
`define TASK_STATUS_DONE   4'h2
`define TASK_STATUS_ERROR  4'hF

// EEPROM Status Bit Position
`define STATUS_BIT_PROG    0  // Program Done
`define STATUS_BIT_ERASE   1  // Erase Done
`define STATUS_BIT_READ    2  // Read Done
`define STATUS_BIT_WRITE   3  // Write Done

// EEPROM slave 定義
`define PAT_BUFFER_SIZE    8'd128   // 單個 Pattern 大小
`define PAT_MAX_COUNT      2        // 最多存儲 2 個 Pattern
`define PAT_ADDR_MASK      8'h01    // 地址遮罩 (0x00~0x01)
`define EEP_MAX_PATTERNS   8        // EEPROM 最多 8 個 Pattern

`define EEP_TOTAL_SIZE     10'd1024 // EEPROM 總大小
`define EEP_ADDR_MASK      10'h3FF  // EEPROM 地址遮罩
