
`timescale 1ns/1ps
module spi_master_top #(
           parameter integer MAX_BITS = 100,
           parameter integer CLK_DIV  = 1
       )(
           input  wire                   clk,
           input  wire                   rst_n,
           input  wire                   start,
           input  wire [15:0]            bit_count,
           input  wire [MAX_BITS-1:0]    tx_bits,
           output wire [MAX_BITS-1:0]    rx_bits,
           output wire                   busy,
           output wire                   done,
           input  wire                   mode_awmf,          // 0=Standard, 1=AWMF
           input  wire                   use_pdi,            // AWMF: 1=PDI, 0=SDI
           input  wire                   cpol,
           input  wire                   cpha,
           input  wire                   present_first_bit,  // Recommended=1 for AWMF
           output wire                   sclk,
           output wire                   cs_n,
           output wire                   mosi,
           input  wire                   miso,
           output wire                   sdi,
           output wire                   pdi,
           input  wire                   sdo
       );
wire eng_dout;
wire eng_din = (mode_awmf) ? sdo : miso;

assign mosi = (!mode_awmf)              ? eng_dout : 1'b0;
assign sdi  = ( mode_awmf && !use_pdi ) ? eng_dout : 1'b0; // Serial
assign pdi  = ( mode_awmf &&  use_pdi ) ? eng_dout : 1'b0; // Broadcast；SDI is naturally 0

spi_bit_engine_sim #(
                       .MAX_BITS           (MAX_BITS)
                       // .CLK_DIV            (CLK_DIV)
                   ) u_eng (
                       .clk                (clk),
                       .rst_n              (rst_n),
                       .start              (start),
                       .bit_count          (bit_count),
                       .cpol               (cpol),
                       .cpha               (cpha),
                       .present_first_bit  (present_first_bit),
                       .tx_bits            (tx_bits),
                       .rx_bits            (rx_bits),
                       .din                (eng_din),
                       .dout               (eng_dout),
                       .sclk               (sclk),
                       .cs_n               (cs_n),
                       .busy               (busy),
                       .done               (done)
                   );
endmodule
