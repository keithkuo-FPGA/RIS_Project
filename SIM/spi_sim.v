`timescale 1ns/1ps

module spi_sim ();

wire sim_csn;
wire sim_sck;
wire sim_d_dq0;
wire sim_q_dq1;
wire sim_w_n_dq2;
wire sim_hold_n_dq3;

reg sim_clk     = 1'b0;
reg sim_rstn    = 1'b0;

reg  [23:0] eep_addr;
reg  eep_wr_rd_n;
reg  [7:0] eep_wdata;
wire [7:0] eep_rdata;
wire [9:0] eep_data_cntr;
reg  [9:0] eep_data_len;
reg eep_addr_vld;
reg eep_rdata_rdy;
reg eep_wdata_vld;
wire eep_busy;
wire eep_rdata_vld;
wire eep_wdata_rdy;

always #100 sim_clk = ~sim_clk;

spi_eeprom_iface UUT (
    .clk        (sim_clk),
    .rst_n      (sim_rstn),
    .sclk       (sim_sck),
    .cs_n       (sim_csn),
    .d_dq0      (sim_d_dq0),
    .q_dq1      (sim_q_dq1),
    .w_n_dq2    (sim_w_n_dq2),
    .hold_n_dq3 (sim_hold_n_dq3),
    .addr       (eep_addr),
    .addr_vld   (eep_addr_vld),
    .wr_rd_n    (eep_wr_rd_n),
    .data_len   (eep_data_len),
    .data_cntr  (eep_data_cntr),
    .wdata      (eep_wdata),
    .rdata      (eep_rdata),
    .rdata_rdy  (eep_rdata_rdy),
    .rdata_vld  (eep_rdata_vld),
    .busy       (eep_busy),
    .wdata_rdy  (eep_wdata_rdy),
    .wdata_vld  (eep_wdata_vld)
);

integer i;

initial begin
    sim_rstn = 1'b0;
    eep_addr_vld = 1'b0;
    eep_wdata_vld = 1'b0;
    eep_rdata_rdy = 1'b0;
    #200
    sim_rstn = 1'b1;
    #200

    // Write
    eep_addr = 24'h123456;
    eep_data_len = 9'd2;
    eep_wr_rd_n = 1'b1;

    eep_addr_vld = 1'b1;
    #200
    eep_addr_vld = 1'b0;
    #200

    @(posedge eep_wdata_rdy);
    #200

    eep_wdata = 8'h55;
    #200

    eep_wdata_vld = 1'b1;
    #200
    eep_wdata_vld = 1'b0;
    #200

    @(posedge eep_wdata_rdy);
    #200

    eep_wdata = 8'h88;
    #200

    eep_wdata_vld = 1'b1;
    #200
    eep_wdata_vld = 1'b0;
    #200

    @(negedge eep_busy);
    #200

    // Read
    eep_addr = 24'h123456;
    eep_data_len = 9'd4;
    eep_wr_rd_n = 1'b0;

    eep_rdata_rdy = 1'b1;
    eep_addr_vld = 1'b1;
    #200
    eep_addr_vld = 1'b0;
    #200

    @(posedge eep_rdata_vld);
    #200

    eep_rdata_rdy = 1'b1;
    #200

    @(posedge eep_rdata_vld);
    #200

    eep_rdata_rdy = 1'b1;
    #200
    
    @(posedge eep_rdata_vld);
    #200

    eep_rdata_rdy = 1'b1;
    #200

    @(posedge eep_rdata_vld);
    #200

    @(negedge eep_busy);
    #200

    $finish;
end

reg [7:0] eesim_data [0:5];
reg [7:0] eesim_sck_cntr = 0;
reg [7:0] eesim_data_idxr = 0;
reg [3:0] eesim_dq = 4'd0;
reg eesim_quad_mode = 1'b0;

assign sim_d_dq0        = (sim_csn ? 1'bz : (eesim_quad_mode ? eesim_dq[0] : 1'bz));
assign sim_q_dq1        = (sim_csn ? 1'bz : (eesim_quad_mode ? eesim_dq[1] : 1'b1));
assign sim_w_n_dq2      = (sim_csn ? 1'bz : (eesim_quad_mode ? eesim_dq[2] : 1'bz));
assign sim_hold_n_dq3   = (sim_csn ? 1'bz : (eesim_quad_mode ? eesim_dq[3] : 1'bz));

initial begin 
    eesim_data[0] = 8'h12;
    eesim_data[1] = 8'h34;
    eesim_data[2] = 8'h56;
    eesim_data[3] = 8'h78;
    eesim_data[4] = 8'h9A;
end

always @(negedge sim_sck or sim_csn) begin
    if (sim_csn) begin
        eesim_sck_cntr <= 8'd0;
        eesim_quad_mode <= 1'b0;
    end
    else if (!sim_sck) begin
        eesim_sck_cntr <= eesim_sck_cntr + 1'b1;
        if (eesim_sck_cntr >= 40) begin
            eesim_quad_mode <= (eep_wr_rd_n ? 1'b0 : 1'b1);
            eesim_data_idxr = (eesim_sck_cntr - 8'd40) >> 1;
            eesim_dq <= eesim_data[eesim_data_idxr][(eesim_sck_cntr % 2 ? 0 : 4)+:4];
        end
    end
end

endmodule