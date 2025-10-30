# LED Matrix Control System (32x32)

## Description
This is a complete FPGA-based 32x32 LED matrix control system with integrated SPI slave interface and external EEPROM support. 
The design supports three operating modes: Direct Pattern Control, EEPROM Read/Write Mode, and Fast Pattern Switch Mode, 
enabling flexible pattern management and display capabilities for LED matrix applications.

### Key Features
- **32x32 LED Matrix Driver** with hardware refresh control
- **Dual-port BRAM** (256x32) for frame buffering
- **SPI Slave Interface** with 3-bank register architecture
- **EEPROM Bridge** with quad-SPI support (up to 99.75 MHz)
- **Three Operating Modes** for pattern management
- **Hardware Version Detection** via GPIO pins
- **Dual LED Status Indicators** with configurable modes

### Target Device
- Lattice LCMXO3LF-6900C
- System Clock: 66.5 MHz (x2osc_clk)
- EEPROM Interface Clock: 99.75 MHz (eif_clk_x2)

---

## Block Design

```
┌────────────────────────────────────────────────────────────────────┐
│                           MCU/Host System                          │
│                      (SPI Master Controller)                       │
└────────────────┬───────────────────────────────────────────────────┘
                 │ SPI (SCLK, CS_N, MOSI, MISO)
                 │
┌────────────────▼───────────────────────────────────────────────────┐
│                         system_top.v                               │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │         SPI Slave Engine (spi_slave_top.v)                   │  │
│  │  ┌──────────────┬──────────────┬──────────────┐              │  │
│  │  │   Bank 0:    │   Bank 1:    │   Bank 2:    │              │  │
│  │  │   System     │   Pattern    │   EEPROM     │              │  │
│  │  │   (mem_rx/tx)│   (mem_rx/tx)│   (mem_rx/tx)│              │  │
│  │  └──────────────┴──────────────┴──────────────┘              │  │
│  └────────────────────┬─────────────────────────────────────────┘  │
│                       │ cfg_clk domain (x2osc_clk)                 │
│  ┌────────────────────▼─────────────────────────────────────────┐  │
│  │              Main Control FSM                                │  │
│  │  - Mode Selection (Direct/EEPROM_RW/Fast_Switch)             │  │
│  │  - Pattern Loading Control                                   │  │
│  │  - EEPROM Operation Management                               │  │
│  └────────────────────┬─────────────────────────────────────────┘  │
│                       │                                            │
│  ┌────────────────────▼─────────────────────────────────────────┐  │
│  │         Dual-Port BRAM (256x32)                              │  │
│  │  Port A: Write (Pattern Loading)                             │  │
│  │  Port B: Read (LED Display)                                  │  │
│  └────────────────────┬─────────────────────────────────────────┘  │
│                       │                                            │
│  ┌────────────────────▼─────────────────────────────────────────┐  │
│  │    LED Row Streamer (led32x32_row_streamer.v)                │  │
│  │  - Fetches 32 rows sequentially from BRAM                    │  │
│  │  - 2-cycle pipeline for timing alignment                     │  │
│  └────────────────────┬─────────────────────────────────────────┘  │
│                       │                                            │
│  ┌────────────────────▼─────────────────────────────────────────┐  │
│  │   LED Matrix Driver (led32x32_matrix_driver.v)               │  │
│  │  - Row scanning with configurable refresh rate               │  │
│  │  - 32-bit column data output                                 │  │
│  └────────────────────┬─────────────────────────────────────────┘  │
│                       │                                            │
└───────────────────────┼────────────────────────────────────────────┘
                        │
                        ▼
              ┌───────────────────┐
              │  32x32 LED        │
              │  Matrix Panel     │
              │  (row_o[31:0])    │
              │  (cloumn_o[31:0]) │
              └───────────────────┘
-------------------------------------------------------------------------------
-------------------------------------------------------------------------------
┌────────────────────────────────────────────────────────────────────┐
│              EEPROM Interface (Parallel Path)                      │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │   spi_eeprom_iface_bridge.v (CDC Bridge)                     │  │
│  │  ┌──────────────────┐          ┌──────────────────┐          │  │
│  │  │  sys_clk Domain  │ ◄──CDC──►│  eif_clk Domain  │          │  │
│  │  │  (66.5 MHz)      │          │  (99.75 MHz)     │          │  │
│  │  │  - TX FIFO (512) │          │                  │          │  │
│  │  │  - RX FIFO (512) │          │                  │          │  │
│  │  └──────────────────┘          └──────────────────┘          │  │
│  │                                          │                   │  │
│  │  ┌───────────────────────────────────────▼─────────────────┐ │  │
│  │  │   spi_eeprom_iface.v (Core Controller)                  │ │  │
│  │  │  - FQREAD: Fast Quad Read (6Bh)                         │ │  │
│  │  │  - PGWR: Page Write (02h)                               │ │  │
│  │  │  - Auto-reload continuous read mode                     │ │  │
│  │  └───────────────────────────────────────┬─────────────────┘ │  │
│  │                                          │                   │  │
│  │  ┌───────────────────────────────────────▼─────────────────┐ │  │
│  │  │   spi_bit_engine_v2.v (SPI Master)                      │ │  │
│  │  │  - Quad-SPI support (DQ0-DQ3)                           │ │  │
│  │  │  - Configurable CPOL/CPHA                               │ │  │
│  │  │  - Auto-reload for continuous transfer                  │ │  │
│  │  └─────────────────────────────────────────────────────────┘ │  │
│  └──────────────────────────────────────────────────────────────┘  │
└───────────────────────┬────────────────────────────────────────────┘
                        │ QSPI (SCLK, CS_N, DQ0-DQ3)
                        ▼
              ┌──────────────────┐
              │  External EEPROM │
              │  (M95P32 or      │
              │   compatible)    │
              └──────────────────┘
```

---

## Table of Contents

* [Hierarchy](#hierarchy)
* [Register Architecture](#register-architecture)
  * [Bank 0: System Register Map](#bank-0-system-register-map)
  * [Bank 1: Pattern Buffer Map](#bank-1-pattern-buffer-map)
  * [Bank 2: EEPROM Control Map](#bank-2-eeprom-control-map)
* [Operating Modes](#operating-modes)
* [State Machine Flow](#state-machine-flow)
* [Timing Specifications](#timing-specifications)
* [Known Issues](#known-issues)
* [References](#references)

---

## Hierarchy

```txt
+ system_top.v
  ├── pll (Lattice PLL)
  │   ├── CLKOP: x2osc_clk (66.5 MHz)
  │   └── CLKOS: eif_clk_x2 (99.75 MHz)
  │
  ├── OSCH (Internal Oscillator: 133 MHz)
  │
  ├── spi_slave_engine (SPI Slave Interface)
  │   ├── Only Mode 0 support (CPOL=0, CPHA=0)
  │   ├── 3-bank memory architecture
  │   │   ├── mem_tx_sys[0:255] (Bank 0: System TX)
  │   │   ├── mem_rx_sys[0:255] (Bank 0: System RX)
  │   │   ├── mem_tx_pat[0:255] (Bank 1: Pattern TX)
  │   │   ├── mem_rx_pat[0:255] (Bank 1: Pattern RX)
  │   │   ├── mem_tx_eep[0:255] (Bank 2: EEPROM TX)
  │   │   └── mem_rx_eep[0:255] (Bank 2: EEPROM RX)
  │   └── Auto-detection of 128-byte pattern write
  │
  ├── dp_bram_256x32 (Dual-Port Block RAM)
  │   ├── Port A: Write port (Pattern loading)
  │   ├── Port B: Read port (Display streaming)
  │   └── Depth: 256 words x 32 bits
  │
  ├── led32x32_row_streamer (Row Data Streamer)
  │   ├── State machine: IDLE → ISSUE → DRAIN
  │   ├── 2-stage pipeline for BRAM read latency
  │   └── Outputs: row_valid, row_idx[4:0], row_data[31:0]
  │
  ├── led32x32_matrix_driver (LED Matrix Driver)
  │   ├── Refresh divider: configurable (default=2)
  │   ├── Row scanning: sequential 0→31
  │   ├── Frame buffer: 32 words x 32 bits
  │   └── Outputs: led_row[31:0], led_col[31:0]
  │
  └── spi_eeprom_iface_bridge (EEPROM Bridge with CDC)
      ├── Clock Domain Crossing
      │   ├── sys_clk (66.5 MHz) ← → eif_clk (99.75 MHz)
      │   ├── addr_vld toggle-based handshake
      │   └── 3-stage synchronizers for all control signals
      │
      ├── TX Path: sys_clk → async_fifo_dc (512x8) → eif_clk
      ├── RX Path: eif_clk → async_fifo_dc (512x8) → sys_clk
      │
      └── spi_eeprom_iface (EEPROM Core Controller)
          ├── State machine: IDLE → WRCTL → INST → TRANS → END
          ├── Supported commands:
          │   ├── 0x06: WREN (Write Enable)
          │   ├── 0x05: RDSR (Read Status Register)
          │   ├── 0x6B: FQREAD (Fast Quad Read)
          │   ├── 0x02: PGWR (Page Write)
          │   ├── 0xDB: PGER (Page Erase)
          │   ├── 0x20: SCER (Sector Erase 4KB)
          │   ├── 0xD8: BKER (Block Erase 64KB)
          │   ├── 0xC7: CHER (Chip Erase)
          │   ├── 0x8B: FRDID (Fast Read ID)
          │   └── 0x82: WRID (Write ID)
          │
          └── spi_bit_engine_v2 (SPI Master Bit Engine)
              ├── Quad-SPI mode support
              ├── Auto-reload for continuous read
              └── Maximum throughput: 50 Mbps (@ 99.75 MHz)
```

---

## Register Architecture

The system uses a **3-bank SPI register architecture** for flexible control and data access:

| Bank | Name | Address Range | Description |
|------|------|---------------|-------------|
| 0 | System | 0x00 - 0xFF | System configuration and status |
| 1 | Pattern | 0x00 - 0x7F | Pattern data buffer (128 bytes) |
| 2 | EEPROM | 0x00 - 0xFF | EEPROM control and status |

**SPI Command Format:**
```
┌────────┬─────────┬──────────────┐
│  CMD   │  ADDR   │  DATA[0..N]  │
│ (8bit) │ (8bit)  │  (N bytes)   │
└────────┴─────────┴──────────────┘

CMD[7:4] = Reserved (0000)
CMD[3]   = R/W (0=Write, 1=Read)
CMD[2:0] = Bank Select
           000 = Invalid
           001 = Bank 2 (EEPROM)
           010 = Bank 1 (Pattern)
           100 = Bank 0 (System)
```

---

### Bank 0: System Register Map

**Base Command: 0x08 (Read) / 0x00 (Write)**

| Address | Register Name | Size | Default | Access | Description |
|---------|---------------|------|---------|--------|-------------|
| **0x00** | **FPGA_VER_H** | 8 bits | 0x16 | R | **FPGA Version High Byte**<br>• [7:4] Version Major (1)<br>• [3:0] Version Minor (6)<br>Example: 0x16 = Version 1.6 |
| **0x01** | **FPGA_VER_L** | 8 bits | 0x8D | R | **FPGA Version Low Byte**<br>• [7:4] Patch Version (8)<br>• [3:0] Build Number (D)<br>Example: 0x8D = Patch 8, Build 13 |
| **0x02** | **HW_VERSION** | 8 bits | Dynamic | R | **Hardware Version (Board Detection)**<br>• [7] Reserved (0)<br>• [6:4] HW_L2[2:0] (Antenna Version)<br>• [3:0] HW_L1[3:0] (FPGA Board Version)<br>*Read from GPIO pins HW_L2_i and HW_L1_i* |
| **0x03** | **RIS_FREQUENCY_INFO** | 8 bits | Dynamic | R | **Module Frequency Information**<br>• [7:4] Reserved (0)<br>• [3:0] Frequency Code:<br>&nbsp;&nbsp;- 0x0 = 28G<br>&nbsp;&nbsp;- 0x1 = 4G7<br>&nbsp;&nbsp;- 0x2 = 3G5<br>&nbsp;&nbsp;- 0x3 = Special<br>*Read from GPIO pins HW_Frequency_i[3:0]* |
| **0x04** | **LED_CONTROL** | 8 bits | 0x00 | R/W | **LED Status Control**<br>• [7:2] Reserved<br>• [1] LED1 Control (gpio_LED[1])<br>• [0] LED0 Control (gpio_LED[0])<br>**Mode Effects:**<br>• 0x00: Idle (LED0=ON, LED1=OFF)<br>• 0x01: Find me (Both blink @ 0.5s)<br>• 0x02: Update in progress (LED0 blinks)<br>• 0x03: System error (LED1=ON, LED0=OFF) |
| **0x05** | **PATTERN_CONTROL_MODE** | 8 bits | 0x00 | R/W | **Pattern Control Mode Selection**<br>• [7:2] Reserved<br>• [1:0] Operating Mode:<br>&nbsp;&nbsp;**0x00:** Direct control mode<br>&nbsp;&nbsp;&nbsp;&nbsp;(pattern buffer → BRAM → display)<br>&nbsp;&nbsp;**0x01:** EEPROM R/W mode<br>&nbsp;&nbsp;&nbsp;&nbsp;(controlled by Bank 2 registers)<br>&nbsp;&nbsp;**0x02:** Fast pattern switch mode<br>&nbsp;&nbsp;&nbsp;&nbsp;(EEPROM → BRAM → display) |
| **0x06** | **STATUS** | 8 bits | 0x00 | R | **System and Pattern Status**<br>• [7:2] Reserved<br>• [1] System Status<br>&nbsp;&nbsp;- 0 = Normal<br>&nbsp;&nbsp;- 1 = Error (show via LED)<br>• [0] Pattern Status<br>&nbsp;&nbsp;- 0 = Normal (update done)<br>&nbsp;&nbsp;- 1 = Busy (update in progress) |
| **0x07 - 0xEF** | **RESERVED** | - | 0x00 | - | Reserved for future use |
| **0xF0** | **FAST_SWITCH_ID_L** | 8 bits | 0x00 | R/W | **Fast Pattern Switch ID Low Byte [7:0]**<br>Used in MODE_FAST_SWITCH (0x02)<br>Combined with 0xF1 to form 16-bit pattern ID |
| **0xF1** | **FAST_SWITCH_ID_H** | 8 bits | 0x00 | R/W | **Fast Pattern Switch ID High Byte [15:8]**<br>Pattern EEPROM Address = {ID_H, ID_L, 7'b0}<br>Max patterns: 65536 (128-byte each) |


---

### Bank 1: Pattern Buffer Map

**Base Command: 0x04 (Read) / 0x00 (Write)**

| Address | Register Name | Size | Default | Access | Description |
|---------|---------------|------|---------|--------|-------------|
| **0x00** | **PATTERN_BYTE_0** | 8 bits | 0x00 | R/W | **Pattern Data Byte 0**<br>Row 0, Column bits [7:0] |
| **0x01** | **PATTERN_BYTE_1** | 8 bits | 0x00 | R/W | **Pattern Data Byte 1**<br>Row 0, Column bits [15:8] |
| **0x02** | **PATTERN_BYTE_2** | 8 bits | 0x00 | R/W | **Pattern Data Byte 2**<br>Row 0, Column bits [23:16] |
| **0x03** | **PATTERN_BYTE_3** | 8 bits | 0x00 | R/W | **Pattern Data Byte 3**<br>Row 0, Column bits [31:24] |
| **0x04 - 0x7C** | **PATTERN_BYTE_4 - 124** | 8 bits | 0x00 | R/W | **Pattern continues...**<br>128 bytes total (32 rows × 4 bytes) |
| **0x7D** | **PATTERN_BYTE_125** | 8 bits | 0x00 | R/W | **Pattern Data Byte 125**<br>Row 31, Column bits [7:0] |
| **0x7E** | **PATTERN_BYTE_126** | 8 bits | 0x00 | R/W | **Pattern Data Byte 126**<br>Row 31, Column bits [15:8] |
| **0x7F** | **PATTERN_BYTE_127** | 8 bits | 0x00 | R/W | **Pattern Data Byte 127**<br>Row 31, Column bits [31:24] |


**Pattern Memory Layout:**
```
BRAM Address (8-bit):
┌───────────┬──────────────┐
│ [7:5]     │ [4:0]        │
│ Frame Sel │ Row Index    │
└───────────┴──────────────┘

BRAM Data (32-bit):
┌───────────┬──────────┬─────────┬─────────┐
│ Byte 3    │ Byte 2   │ Byte 1  │ Byte 0  │
│ Col[31:24] Col[23:16] Col[15:8] Col[7:0] │
└───────────┴──────────┴─────────┴─────────┘
```

---

### Bank 2: EEPROM Control Map

**Base Command: 0x02 (Read) / 0x00 (Write)**

| Address | Register Name | Size | Default | Access | Description |
|---------|---------------|------|---------|--------|-------------|
| **0x00** | **EEPROM_PATTERN_ID_L** | 8 bits | 0x00 | R/W | **EEPROM Pattern ID Low Byte [7:0]**<br>Used in EEPROM_RW mode (0x01)<br>Pattern address pointer in EEPROM |
| **0x01** | **EEPROM_PATTERN_ID_H** | 8 bits | 0x00 | R/W | **EEPROM Pattern ID High Byte [15:8]**<br>EEPROM Address = {0x00, ID_H, ID_L, 7'b0}<br>Supports up to 65536 patterns (16-bit address) |
| **0x02** | **EEPROM_OPERATION_MODE** | 8 bits | 0x00 | W | **EEPROM Operation Control**<br>• [7:2] Reserved<br>• [1:0] Operation Mode:<br>&nbsp;&nbsp;**0x00:** No operation<br>&nbsp;&nbsp;**0x01:** Write 128 bytes (Pattern Buffer → EEPROM)<br>&nbsp;&nbsp;**0x02:** Read 128 bytes (EEPROM → Pattern Buffer)<br>&nbsp;&nbsp;**0x03:** Write 128 bytes to ID page<br>&nbsp;&nbsp;**0x04:** Read 128 bytes from ID page<br>*Self-clearing after operation starts* |
| **0x03** | **EEPROM_ERASE_CONTROL** | 8 bits | 0x00 | W | **EEPROM Erase Control**<br>• [7:3] Reserved<br>• [2:1] Erase Type:<br>&nbsp;&nbsp;**00:** Chip Erase (entire EEPROM)<br>&nbsp;&nbsp;**01:** Page Erase (256 bytes at ID address)<br>&nbsp;&nbsp;**10:** Sector Erase (4KB at ID address)<br>&nbsp;&nbsp;**11:** Block Erase (64KB at ID address)<br>• [0] Erase Start (pulse trigger)<br>&nbsp;&nbsp;0→1 transition starts erase operation |
| **0x04** | **EEPROM_STATUS** | 8 bits | 0x0F | R | **EEPROM Operation Status**<br>• [7:4] Reserved<br>• [3] **Write Done**<br>&nbsp;&nbsp;0 = Write in progress<br>&nbsp;&nbsp;1 = Write completed<br>• [2] **Read Done**<br>&nbsp;&nbsp;0 = Read in progress<br>&nbsp;&nbsp;1 = Read completed<br>• [1] **Program Done**<br>&nbsp;&nbsp;0 = Programming in progress<br>&nbsp;&nbsp;1 = Programming completed<br>• [0] **Erase Done**<br>&nbsp;&nbsp;0 = Erase in progress<br>&nbsp;&nbsp;1 = Erase completed<br>*Default 0x0F = All idle/completed* |


---

## Operating Modes

### Mode 0: Direct Control Mode (0x00)
**Description:** Pattern data is directly written to Bank 1, then automatically loaded to BRAM and displayed on LED matrix.

**Sequence:**
```
1. MCU writes to Bank 0, Addr 0x05 = 0x00 (Set Direct Mode)
2. MCU writes 128 bytes to Bank 1, Addr 0x00-0x7F
   → Writing to 0x00 triggers pattern reception mode
   → After 128 bytes → pat_rx_128_done pulse
3. FPGA automatically:
   - Loads pattern from mem_rx_pat to BRAM (4 bytes at a time)
   - Starts LED display when loading completes
   - Sets led_enable = 1, disp_gate = 1
```

**Timing:**
- Pattern loading: ~128 clock cycles (@ 66.5 MHz ≈ 1.9 µs)
- Display refresh: Configurable (default: 16 clocks per row)

---

### Mode 1: EEPROM Read/Write Mode (0x01)
**Description:** Bi-directional data transfer between EEPROM and pattern buffer.

**Write Sequence (Pattern → EEPROM):**
```
1. MCU writes to Bank 0, Addr 0x05 = 0x01 (Set EEPROM_RW Mode)
2. MCU writes 128 bytes to Bank 1, Addr 0x00-0x7F
3. MCU sets Bank 2, Addr 0x00-0x01 = Pattern ID (e.g., 0x0000)
4. MCU writes to Bank 2, Addr 0x02 = 0x01 (Trigger Write)
5. FPGA:
   - Reads pattern from mem_rx_pat
   - Writes to EEPROM at address {0x00, ID_H, ID_L, 7'b0}
   - Sets Status[3] = 0 during write, = 1 when done
6. MCU polls Bank 2, Addr 0x04 until Status[3] = 1
```

**Read Sequence (EEPROM → Pattern):**
```
1. MCU writes to Bank 0, Addr 0x05 = 0x01
2. MCU sets Bank 2, Addr 0x00-0x01 = Pattern ID
3. MCU writes to Bank 2, Addr 0x02 = 0x02 (Trigger Read)
4. FPGA:
   - Reads 128 bytes from EEPROM using FQREAD (0x6B)
   - Writes to mem_tx_pat (for MCU readback)
   - Does NOT load to BRAM (display unchanged)
   - Sets Status[2] = 0 during read, = 1 when done
5. MCU polls Bank 2, Addr 0x04 until Status[2] = 1
6. MCU reads back pattern from Bank 1, Addr 0x00-0x7F
```

---

### Mode 2: Fast Pattern Switch Mode (0x02)
**Description:** Rapidly switch displayed patterns from EEPROM without MCU intervention.

**Sequence:**
```
1. MCU writes to Bank 0, Addr 0x05 = 0x02 (Set Fast Switch Mode)
2. MCU writes to Bank 0, Addr 0xF0-0xF1 = Pattern ID
   → Immediately triggers EEPROM read operation
3. FPGA automatically:
   - Reads 128 bytes from EEPROM at {0x00, ID_H, ID_L, 7'b0}
   - Loads directly to BRAM (4 bytes at a time)
   - Updates LED display when loading completes
   - Clears pat_rx_128_done flag
4. Pattern switch complete
```

**Performance:**
- EEPROM read: ~256 µs (@ 99.75 MHz QSPI, 50 Mbps effective)
- BRAM loading: ~1.9 µs (@ 66.5 MHz)
- Total switch time: <300 µs

**Use Case:** Real-time pattern animation, user interface updates

---

## State Machine Flow

### Main Control FSM (system_top.v)

```
                                    ┌──────────┐
                                    │  IDLE    │◄───────────────┐
                                    └────┬─────┘                │
                                         │ pat_auto_pend        │
                                         │                      │
                                    ┌────▼────────┐             │
                                    │ READ_MODE   │             │
                                    │ (0x05 reg)  │             │
                                    └────┬────────┘             │
                                         │                      │
                    ┌────────────────────┼──────────────────┐   │
                    │                    │                  │   │
         Mode=0x00  │         Mode=0x01  │       Mode=0x02  │   │
                    │                    │                  │   │
        ┌───────────▼─┐     ┌────────────▼──┐   ┌───────────▼─┐ │
        │ PAT_BYTE0-3 │     │  EEP_OP       │   │ FAST_ID_L/H │ │
        │ (mem_rx_pat)│     │  (Read 0x02)  │   │ (0xF0/0xF1) │ │
        └───────┬─────┘     └───────┬───────┘   └──────┬──────┘ │
                │                   │                  │        │
                │         ┌─────────┴─────────┐        │        │
                │         │                   │        │        │
                │    Op=0x01            Op=0x02        │        │
                │    WRITE              READ           │        │
                │         │                   │        │        │
                │    ┌────▼────┐         ┌────▼────┐   │        │
                │    │EEP_WR   │         │EEP_RD   │   │        │
                │    │_START   │         │_DATA    │   │        │
                │    └────┬────┘         └────┬────┘   │        │
                │         │                   │        │        │
                │         └─────────┬─────────┘        │        │
                │                   │                  │        │
                └───────────────────┴──────────────────┘        │
                                    │                           │
                            ┌───────▼────────┐                  │
                            │  PAT_DONE      │                  │
                            │ - Load to BRAM │──────────────────┘
                            │ - Start display│
                            └────────────────┘
```

**State Descriptions:**

| State | Function | Next State Trigger |
|-------|----------|-------------------|
| ST_IDLE | Wait for pattern trigger | pat_auto_pend=1 or erase_auto_pend=1 |
| ST_READ_MODE | Read mode register (0x05) | cfg_pipe_valid=0 |
| ST_FAST_ID_L/H | Read fast switch ID | cfg_pipe_valid=0 |
| ST_EEP_OP | Read EEPROM operation (0x02) | cfg_pipe_valid=0 |
| ST_EEP_ID_L/H | Read EEPROM pattern ID | cfg_pipe_valid=0 |
| ST_PAT_BYTE0-3 | Load 4 bytes from PAT/EEPROM | eif_rvalid or cfg_pipe_valid |
| ST_PAT_DONE | Start LED display | 1 cycle → ST_IDLE |
| ST_EEP_WR_START | Write to EEPROM | wr_cnt = data_len → ST_IDLE |
| ST_EEP_RD_DATA | Read from EEPROM | pat_src_index = 128 → ST_IDLE |
| ST_ERASE_CTRL/ID | Erase operation | cfg_pipe_valid=0 |

---

### EEPROM Interface FSM (spi_eeprom_iface.v)

```
    ┌─────────────┐
    │   ST_IDLE   │◄──────────────────┐
    └──────┬──────┘                   │
           │ addr_vld=1               │
           ├─────────┬────────────┐   │
           │         │            │   │
       WR=1│     RD=1│     ERASE=1│   │
           │         │            │   │
    ┌──────▼──┐ ┌────▼────┐  ┌────▼────┐
    │ WRCTL   │ │ QDDMY   │  │ WRCTL   │
    │ (WREN)  │ │ (Dummy) │  │ (WREN)  │
    └────┬────┘ └────┬────┘  └────┬────┘
         │           │            │
    ┌────▼─────┐┌───▼──────┐┌─────▼────┐
    │  INST    ││  INST    ││  INST    │
    │ (PGWR +  ││ (FQREAD) ││ (ER_CMD) │
    │  ADDR)   ││          ││          │
    └────┬─────┘└────┬─────┘└────┬─────┘
         │           │           │
    ┌────▼─────┐┌───▼──────┐     │
    │  TRANS   ││  TRANS   │     │
    │ (Write   ││ (Read    │     │
    │  data)   ││  data)   │     │
    └────┬─────┘└────┬─────┘     │
         │           │           │
    ┌────▼─────┐┌───▼──────┐┌────▼─────┐
    │ WRPOLL   ││   END    ││  WRPOLL  │
    │ (Wait WIP)│          ││ (Wait WIP)│
    └────┬─────┘└────┬─────┘└────┬─────┘
         │           │           │
    ┌────▼───────────▼───────────▼─────┐
    │              ST_END              │
    └──────────────────┬───────────────┘
                       │ done=1
                       │
                   ┌───▼────┐
                   │ IDLE   │
                   └────────┘
```

**Auto-Reload Feature:**
- In continuous read mode (data_len > 1)
- After each byte transfer, automatically reloads:
  - samples_left = bit_count_latch
  - half_edges_rem = 2 × bit_count_latch
- Enables streaming read without CS_N toggling
- Maximum throughput: 50 Mbps effective

---

## Timing Specifications

### Clock Domains

| Clock | Frequency | Source | Purpose |
|-------|-----------|--------|---------|
| Osc_Clk | 133 MHz | Internal Oscillator | PLL input |
| x2osc_clk | 66.5 MHz | PLL CLKOP | Main system clock |
| eif_clk_x2 | 99.75 MHz | PLL CLKOS | EEPROM interface clock |
| sclk_i | 50 MHz | External MCU | SPI slave clock |

### Timing Constraints

**SPI Slave Interface:**
- Setup time: tSU = 5 ns (MOSI before SCLK rising edge)
- Hold time: tH = 5 ns (MOSI after SCLK rising edge)
- Clock-to-output: tCO = 10 ns (SCLK falling to MISO valid)
- CS_N setup: tCSS = 10 ns (CS_N falling before first SCLK)
- CS_N hold: tCSH = 10 ns (Last SCLK to CS_N rising)

**EEPROM Interface (QSPI):**
- SCLK frequency: Up to 49.875 MHz (99.75 MHz / 2)
- Setup time: tSU = 5 ns (DQ before SCLK edge)
- Hold time: tH = 5 ns (DQ after SCLK edge)
- CS_N pulse width: tCSH = 50 ns minimum

**LED Matrix Output:**
- Row switching time: 16 × (1/66.5 MHz) = 240 ns
- Full frame time: 32 × 240 ns = 7.68 µs
- Refresh rate: 1 / 7.68 µs ≈ 130 kHz

### Critical Paths

**Identified critical paths (>15 ns @ 66.5 MHz):**
1. Main FSM state decoder → BRAM write enable
   - Estimated: 18 ns
   - Suggestion: Insert pipeline register

2. SPI slave cmd/addr decode → Memory bank select
   - Estimated: 12 ns
   - Status: Within timing

3. EEPROM FIFO read → SPI engine TX path
   - Estimated: 20 ns
   - Mitigation: Already pipelined with tx_rd_en_q

---

## Known Issues

---

## Testbench Coverage

### Test Scenarios (tb_spi_slave.v)

| Test Case | Description | Status |
|-----------|-------------|--------|
| Test 1 | Basic sanity check (version read) | ✅ PASS |
| Test 2 | Direct mode pattern write + display | ✅ PASS |
| Test 3 | EEPROM write operation (Pattern → EEPROM) | ✅ PASS |
| Test 4 | EEPROM read operation (EEPROM → Pattern buffer) | ✅ PASS |
| Test 5 | Fast switch mode (EEPROM → BRAM direct) | ✅ PASS |
| Test 6 | MCU readback from pattern buffer | ✅ PASS |

**Coverage Summary:**
- Register access: 90%
- Operating modes: 100% (all 3 modes)

---

## References

---

## Contact & Support

**Design Engineer:** Keith Kuo (keith_kuo@tmytek.com)  
**Company:** TMYTEK  
**Last Updated:** October 30, 2025  

---
