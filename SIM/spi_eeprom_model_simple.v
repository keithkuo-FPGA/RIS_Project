`timescale 1ns/1ps

module eeprom_m95p32_emu #(
           parameter MEM_SIZE = 1024
       )(
           input   wire    s_n,
           input   wire    c,
           inout   wire    d_dq0,
           output  wire    q_dq1,
           inout   wire    w_n_dq2,
           inout   wire    hold_n_dq3
       );

// EEPROM instructions macro
localparam  EEP_WREN  = 8'h06;
localparam  EEP_WRDI  = 8'h04;
localparam  EEP_RDSR  = 8'h05;
localparam  EEP_WRSR  = 8'h01;
localparam  EEP_READ  = 8'h03;
localparam  EEP_FREAD = 8'h0B;
localparam  EEP_FDREAD = 8'h3B;
localparam  EEP_FQREAD = 8'h6B;
localparam  EEP_PGWR  = 8'h02;
localparam  EEP_PGPR  = 8'h0A;
localparam  EEP_PGER  = 8'hDB;
localparam  EEP_SCER  = 8'h20;
localparam  EEP_BKER  = 8'hD8;
localparam  EEP_CHER  = 8'hC7;
localparam  EEP_RDID  = 8'h83;
localparam  EEP_FRDID = 8'h8B;
localparam  EEP_WRID  = 8'h82;
localparam  EEP_DPD   = 8'hB9;
localparam  EEP_RDPD  = 8'hAB;
localparam  EEP_JEDID = 8'h9F;
localparam  EEP_RDCR  = 8'h15;
localparam  EEP_RDVR  = 8'h85;
localparam  EEP_WRVR  = 8'h81;
localparam  EEP_CLRSF = 8'h50;
localparam  EEP_RDSFD = 8'h5A;
localparam  EEP_RSTEN = 8'h66;
localparam  EEP_RESET = 8'h99;

localparam  ST_IDLE     = 8'd0;
localparam  ST_RCINST   = 8'd1;
localparam  ST_RCADDR   = 8'd2;
localparam  ST_TRANS    = 8'd3;

reg [7:0] mem_array [0:MEM_SIZE-1];
reg [7:0] id_array  [0:511];
reg [9:0] mem_idxr;
reg [23:0] mem_addr;
reg [3:0] state;

reg [11:0] clk_cntr;
reg [9:0] byte_cntr;
reg byte_cplt;
reg [7:0] rx_data;
reg [7:0] rx_reg;
reg [7:0] tx_data;
reg [7:0] inst;
reg [3:0] dout_quad;
reg dout_sel;
reg quad_mode;
reg dout;
wire din;

assign d_dq0        = (s_n ? 1'bz : (dout_sel ? dout_quad[0] : 1'bz));
assign q_dq1        = (s_n ? 1'bz : (dout_sel ? dout_quad[1] : dout));
assign w_n_dq2      = (s_n ? 1'bz : (dout_sel ? dout_quad[2] : 1'bz));
assign hold_n_dq3   = (s_n ? 1'bz : (dout_sel ? dout_quad[3] : 1'bz));

assign din = d_dq0;

integer i;

always @(c or posedge s_n) begin
    byte_cplt <= 1'b0;
    if (s_n) begin
        dout_sel <= 0;
        byte_cntr <= 10'd0;
        clk_cntr <= 12'd0;
        dout <= 1'b0;
        dout_quad <= 4'd0;
        rx_data <= 8'd0;
        rx_reg <= 8'd0;
    end
    else if (c) begin
        rx_reg <= {rx_reg[6:0], din};
        if (quad_mode) begin
            if ((clk_cntr & 24'b1) == 3'b1) begin
                rx_data <= 8'd0;
                byte_cplt <= 1'b1;
                byte_cntr <= byte_cntr + 1'b1;
            end
        end
        else begin
            if ((clk_cntr & 24'b111) == 3'b111) begin
                rx_data <= {rx_reg[6:0], din};
                byte_cplt <= 1'b1;
                byte_cntr <= byte_cntr + 1'b1;
            end
        end

        clk_cntr <= clk_cntr + 1'b1;
    end
    else if (!c) begin
        dout_sel <= quad_mode;
        if (quad_mode) begin
            dout_quad <= tx_data[(1-(clk_cntr & 12'b1)) * 4 +: 4];
        end
        else begin
            dout <= tx_data[7-(clk_cntr & 12'b111)];
        end
    end
end

always @(*) begin
    case (state)
        ST_IDLE: begin
            tx_data <= 8'd0;
            quad_mode = 1'b0;
            mem_idxr = 9'd0;
            mem_addr = 24'd0;
            inst = 8'd0;
            if (!s_n) begin
                state <= ST_RCINST;
            end
        end
        ST_RCINST: begin
            if (byte_cntr >= 1) begin
                inst = rx_data;
                case (inst)
                    EEP_RDSR:
                        state <= ST_TRANS;
                    // EEP_WREN:   state <= ST_IDLE;
                    default:
                        state <= ST_RCADDR;
                endcase
            end
        end
        ST_RCADDR: begin
            if (byte_cntr >= 2) begin
                mem_addr[(4-byte_cntr)*8 +: 8] = rx_data;
                if (byte_cntr == 4) begin
                    state <= ST_TRANS;
                end
            end
        end
        ST_TRANS: begin
            mem_idxr = (byte_cntr-5) + mem_addr;
            case (inst)
                EEP_RDSR: begin
                    tx_data = 8'h00;
                end
                EEP_FQREAD: begin
                    if (byte_cntr > 4) begin
                        quad_mode = 1'b1;
                        tx_data = mem_array[mem_idxr];
                    end
                end
                EEP_PGWR: begin
                    if (byte_cntr > 4) begin
                        tx_data = 8'h00;
                        mem_array[mem_idxr] <= rx_data;
                    end
                end
                EEP_FRDID: begin
                    if (byte_cntr > 4) begin
                        tx_data = id_array[mem_idxr];
                    end
                end
                EEP_WRID: begin
                    if (byte_cntr > 4) begin
                        tx_data = 8'h00;
                        id_array[mem_idxr] <= rx_data;
                    end
                end
            endcase
        end
        default:
            state <= ST_IDLE;
    endcase
    if (s_n)
        state <= ST_IDLE;
end

endmodule
