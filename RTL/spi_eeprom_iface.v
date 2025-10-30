`timescale 1ns/1ps

module spi_eeprom_iface (
           input   wire    clk,
           input   wire    rst_n,
           output  wire    sclk,
           output  wire    cs_n,
           inout   wire    d_dq0,
           inout   wire    q_dq1,
           inout   wire    w_n_dq2,
           inout   wire    hold_n_dq3,
           input   wire    [23:0]  addr,
           input   wire    addr_vld,
           input   wire    wr_rd_sel,
           input   wire    id_mem_sel,
           input   wire    [7:0]   erase_ctrl,
           input   wire    [9:0]   data_len,
           input   wire    [7:0]   wdata,
           output  reg     wdata_rdy,
           input   wire    wdata_vld,
           output  reg     [7:0]   rdata,
           input   wire    rdata_rdy,
           output  reg     rdata_vld,
           output  wire    [9:0]   data_cntr,
           output  reg     busy
       );

localparam ST_IDLE      = 3'd0;
localparam ST_WRCTL     = 3'd1;
localparam ST_QDDMY     = 3'd2;
localparam ST_INST      = 3'd3;
localparam ST_TRANS     = 3'd4;
localparam ST_WRPOLL    = 3'd5;
localparam ST_END       = 3'd6;

// EEPROM instructions
localparam  EEP_WREN  = 8'h06;
localparam  EEP_RDSR  = 8'h05;
localparam  EEP_FQREAD = 8'h6B;
localparam  EEP_PGWR  = 8'h02;
localparam  EEP_PGER  = 8'hDB;
localparam  EEP_SCER  = 8'h20;
localparam  EEP_BKER  = 8'hD8;
localparam  EEP_CHER  = 8'hC7;
localparam  EEP_FRDID = 8'h8B;
localparam  EEP_WRID  = 8'h82;

reg [2:0]   state;
reg [1:0]   current_op_mode;
reg erase_trig;
reg [7:0]   int_erase_ctrl;

localparam SPI_CPOL = 1'b0;
localparam SPI_CPHA = 1'b0;
localparam OP_READ  = 2'd0;
localparam OP_WRITE = 2'd1;
localparam OP_ERASE = 2'd2;

reg  spi_start;
reg  [15:0] spi_bit_cnt;
reg  [39:0] spi_tx_bits;
wire [39:0] spi_rx_bits;
wire spi_busy;
wire spi_done;
(* syn_maxfan = 4 *)reg  spi_keep_cs; //(* syn_maxfan = 4 *)
reg  spi_quad_en;
reg  spi_auto_reload;
reg  [3:0] dummy_cnt;
reg [9:0] data_cntr_int;
// wire [39:0] spi_tx_bits_comb;
// wire [15:0] spi_bit_cnt_comb;
reg data_cntr_eq_len;
reg data_cntr_lt_len_m1;

assign data_cntr = data_cntr_int;

// always @(posedge clk or negedge rst_n) begin
//     if (!rst_n) begin
//         spi_tx_bits_next <= 40'd0;
//         spi_bit_cnt_next <= 16'd0;
//     end
//     else begin
//         spi_tx_bits_next <= spi_tx_bits; // ← 改為組合邏輯輸出
//         spi_bit_cnt_next <= spi_bit_cnt;
//     end
// end

spi_bit_engine_v2 #(
                      .MAX_BITS(40)
                  ) spi_master_i (
                      .clk                (clk),
                      .rst_n              (rst_n),
                      .start              (spi_start),
                      .bit_count          (spi_bit_cnt),
                      .present_first_bit  (1'b1),
                      .tx_bits            (spi_tx_bits),
                      .rx_bits            (spi_rx_bits),
                      .busy               (spi_busy),
                      .done               (spi_done),
                      .keep_cs            (spi_keep_cs),
                      .qd_mode_en         (spi_quad_en),
                      .cpol               (SPI_CPOL),
                      .cpha               (SPI_CPHA),
                      .auto_reload        (spi_auto_reload),
                      .dq0                (d_dq0),
                      .dq1                (q_dq1),
                      .dq2                (w_n_dq2),
                      .dq3                (hold_n_dq3),
                      .sclk               (sclk),
                      .cs_n               (cs_n)
                  );

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= ST_IDLE;
        rdata           <= 8'd0;
        spi_start       <= 1'b0;
        spi_tx_bits     <= 40'd0;
        spi_keep_cs     <= 1'b0;
        spi_quad_en     <= 1'b0;
        spi_auto_reload <= 1'b0;
        data_cntr_int   <= 9'd0;
        rdata_vld       <= 1'b0;
        wdata_rdy       <= 1'b0;
        busy            <= 1'b0;
        int_erase_ctrl  <= 8'd0;
        erase_trig      <= 1'b0;
        dummy_cnt       <= 4'd0;
        current_op_mode <= OP_READ;
        data_cntr_eq_len    <= 1'b0;
        data_cntr_lt_len_m1 <= 1'b0;

    end
    else begin
        spi_start <= 1'b0;
        busy <= (state != ST_IDLE);

        data_cntr_eq_len    <= (data_cntr_int == data_len);
        data_cntr_lt_len_m1 <= (data_cntr_int < (data_len - 1));

        case (state)
            ST_IDLE: begin
                rdata       <= 8'd0;
                spi_tx_bits <= 40'd0;
                spi_keep_cs <= 1'b0;
                spi_quad_en <= 1'b0;
                spi_auto_reload <= 1'b0;  // ✅ 清除
                rdata_vld   <= 1'b0;
                data_cntr_int   <= 9'd0;
                wdata_rdy   <= 1'b0;
                dummy_cnt   <= 4'd0;

                if (int_erase_ctrl[0] != erase_ctrl[0]) begin
                    erase_trig      <= 1'b1;
                    int_erase_ctrl  <= erase_ctrl;
                    current_op_mode <= OP_ERASE;
                end
                else
                    erase_trig      <= 1'b0;

                if (erase_trig) begin
                    current_op_mode <= OP_ERASE;
                    spi_bit_cnt <= 16'd8;
                    spi_tx_bits <= EEP_WREN;
                    spi_keep_cs <= 1'b0;
                    spi_quad_en <= 1'b0;
                    spi_start   <= 1'b1;
                    state       <= ST_WRCTL;
                end
                else if (addr_vld) begin
                    if (wr_rd_sel) begin
                        spi_bit_cnt <= 16'd8;
                        spi_tx_bits <= EEP_WREN;
                        spi_keep_cs <= 1'b0;
                        spi_quad_en <= 1'b0;
                        spi_start   <= 1'b1;
                        state       <= ST_WRCTL;
                        current_op_mode <= OP_WRITE;
                    end
                    else begin
                        spi_bit_cnt <= 16'd32;
                        if (id_mem_sel)
                            spi_tx_bits <= (EEP_FRDID << 24) | addr;
                        else
                            spi_tx_bits <= (EEP_FQREAD << 24) | addr;
                        spi_keep_cs <= 1'b1;
                        spi_quad_en <= 1'b0;
                        spi_start <= 1'b1;
                        state <= ST_QDDMY;
                        current_op_mode <= OP_READ;
                    end
                end
            end

            ST_WRCTL: begin
                if (spi_done) begin
                    spi_bit_cnt <= 16'd32;
                    spi_keep_cs <= 1'b1;
                    if (current_op_mode == OP_ERASE) begin
                        spi_keep_cs <= 1'b0;
                        case (int_erase_ctrl[2:1])
                            2'b00: begin
                                spi_tx_bits <= EEP_CHER;
                                spi_bit_cnt <= 16'd8;
                            end
                            2'b01:
                                spi_tx_bits <= (EEP_PGER << 24) | addr;
                            2'b10:
                                spi_tx_bits <= (EEP_SCER << 24) | addr;
                            2'b11:
                                spi_tx_bits <= (EEP_BKER << 24) | addr;
                        endcase
                    end
                    else if (id_mem_sel)
                        spi_tx_bits <= (EEP_WRID << 24) | addr;
                    else
                        spi_tx_bits <= (EEP_PGWR << 24) | addr;
                    spi_quad_en <= 1'b0;
                    spi_start <= 1'b1;
                    state <= ST_INST;
                end
            end

            ST_QDDMY: begin
                if (spi_done) begin
                    spi_tx_bits     <= 40'd0;
                    spi_keep_cs     <= 1'b1;
                    spi_bit_cnt     <= 16'd8;
                    spi_quad_en     <= (id_mem_sel ? 1'b0 : 1'b1);
                    spi_auto_reload <= 1'b0;
                    spi_start       <= 1'b0;
                    state           <= ST_INST;
                end
            end

            ST_INST: begin
                if (current_op_mode == OP_ERASE) begin
                    if (spi_done) begin
                        spi_bit_cnt <= 16'd16;
                        spi_tx_bits <= (EEP_RDSR << 8);
                        spi_keep_cs <= 1'b1;
                        spi_quad_en <= 1'b0;
                        spi_start   <= 1'b1;
                        wdata_rdy   <= 1'b0;
                        state       <= ST_WRPOLL;
                    end
                end
                else if (wr_rd_sel) begin
                    wdata_rdy <= spi_done;
                    if (wdata_vld && !(spi_busy || spi_start)) begin
                        spi_bit_cnt <= 16'd8;
                        spi_tx_bits <= wdata;
                        spi_keep_cs <= data_cntr_lt_len_m1;
                        spi_quad_en <= 1'b0;
                        spi_start   <= 1'b1;
                        data_cntr_int   <= data_cntr_int + 9'd1;
                        wdata_rdy   <= 1'b0;
                        state       <= ST_TRANS;
                    end
                end
                else begin
                    if (!spi_busy && !spi_start) begin
                        spi_bit_cnt     <= 16'd8;
                        spi_tx_bits     <= 40'd0;
                        spi_quad_en     <= (id_mem_sel ? 1'b0 : 1'b1);
                        spi_keep_cs     <= 1'b1;
                        spi_start       <= 1'b1;

                        dummy_cnt       <= dummy_cnt + 4'd1;

                        if (data_len > 1 && dummy_cnt > 3) begin
                            spi_auto_reload <= 1'b1;
                            state <= ST_TRANS;
                        end

                        // state <= ST_TRANS;
                    end
                end
            end

            ST_TRANS: begin
                rdata_vld <= 1'b0;

                if (wr_rd_sel) begin
                    wdata_rdy <= spi_done;
                    if (data_cntr_eq_len) begin
                        if (spi_done) begin
                            spi_bit_cnt <= 16'd16;
                            spi_tx_bits <= (EEP_RDSR << 8);
                            spi_keep_cs <= 1'b1;
                            spi_quad_en <= 1'b0;
                            spi_start <= 1'b1;
                            wdata_rdy <= 1'b0;
                            state <= ST_WRPOLL;
                        end
                    end
                    else if (wdata_vld && !(spi_busy || spi_start)) begin
                        spi_bit_cnt <= 16'd8;
                        spi_tx_bits <= wdata;
                        spi_quad_en <= 1'b0;
                        spi_keep_cs <= data_cntr_lt_len_m1;
                        spi_start <= 1'b1;
                        data_cntr_int <= data_cntr_int + 9'd1;
                        wdata_rdy <= 1'b0;
                    end
                end
                else begin
                    if (spi_done) begin
                        data_cntr_int <= data_cntr_int + 9'd1;
                        rdata     <= spi_rx_bits[7:0];
                        rdata_vld <= 1'b1;


                        if (data_cntr_int >= data_len) begin
                            spi_auto_reload <= 1'b0;
                            state <= ST_END;
                        end
                    end
                end
            end

            ST_WRPOLL: begin
                rdata_vld <= 1'b0;
                if (spi_done) begin
                    spi_bit_cnt <= 16'd8;
                    spi_tx_bits <= 40'd0;
                    spi_quad_en <= 1'b0;
                    wdata_rdy <= 1'b0;
                    spi_keep_cs <= (spi_rx_bits & 8'h1);
                    spi_start <= 1'b1;
                    state <= (!(spi_rx_bits & 8'h1) ? ST_END : ST_WRPOLL);
                end
            end

            ST_END: begin
                if (wr_rd_sel || erase_trig) begin
                    if (spi_done) begin
                        dummy_cnt <= 4'd0;
                        state <= ST_IDLE;
                    end
                end
                else begin
                    rdata_vld <= 1'b0;
                    state <= ST_IDLE;
                end
            end
        endcase
    end
end

endmodule
