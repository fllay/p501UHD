`timescale 1ns/1ps
module tb;
  reg aclk = 0, aresetn = 0, clk = 0, resetn = 0;
  always #5 aclk = ~aclk;
  always #16.25 clk = ~clk;
  reg [7:0] awaddr = 0, araddr = 0; reg awvalid = 0, wvalid = 0, arvalid = 0, bready = 1, rready = 1; reg [31:0] wdata = 0;
  wire awready, wready, bvalid, arvalid_o, arready, rvalid; wire [31:0] rdata; wire [1:0] bresp, rresp;
  reg [15:0] rxd0 = 0, rxd1 = 0, rxd2 = 0, rxd3 = 0; reg rxv = 0;
  wire [15:0] rxo0, rxo1, rxo2, rxo3; wire rxov;
  reg tx_req = 0; wire tx_pull; reg [15:0] up0, up1, up2, up3; reg upv = 0;
  wire [15:0] txo0, txo1, txo2, txo3; wire txov, txunf;
  p501_time dut (.s_axi_aclk(aclk), .s_axi_aresetn(aresetn), .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
    .s_axi_wdata(wdata), .s_axi_wstrb(4'hf), .s_axi_wvalid(wvalid), .s_axi_wready(wready), .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
    .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready), .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
    .fs_clk(clk), .fs_resetn(resetn),
    .rx_in_data_0(rxd0), .rx_in_data_1(rxd1), .rx_in_data_2(rxd2), .rx_in_data_3(rxd3), .rx_in_valid(rxv),
    .rx_out_data_0(rxo0), .rx_out_data_1(rxo1), .rx_out_data_2(rxo2), .rx_out_data_3(rxo3), .rx_out_valid(rxov),
    .tx_req(tx_req), .tx_pull(tx_pull), .tx_up_data_0(up0), .tx_up_data_1(up1), .tx_up_data_2(up2), .tx_up_data_3(up3), .tx_up_valid(upv), .tx_up_unf(1'b0),
    .tx_out_data_0(txo0), .tx_out_data_1(txo1), .tx_out_data_2(txo2), .tx_out_data_3(txo3), .tx_out_valid(txov), .tx_out_unf(txunf));

  task axi_w(input [7:0] a, input [31:0] d); begin
    @(posedge aclk); awaddr <= a; wdata <= d; awvalid <= 1; wvalid <= 1;
    @(posedge aclk); while (!(awready && wready)) @(posedge aclk);
    awvalid <= 0; wvalid <= 0; @(posedge aclk); end endtask
  task axi_r(input [7:0] a, output [31:0] d); begin
    @(posedge aclk); araddr <= a; arvalid <= 1; @(posedge aclk); while (!arready) @(posedge aclk);
    arvalid <= 0; while (!rvalid) @(posedge aclk); d = rdata; @(posedge aclk); end endtask
  task cmd(input [3:0] c, input [63:0] a, input [31:0] b); reg [31:0] bs; begin
    axi_w(8'h08, a[31:0]); axi_w(8'h0c, a[63:32]); axi_w(8'h10, b); axi_w(8'h14, {28'd0, c});
    bs = 1; while (bs[0]) axi_r(8'h14, bs); end endtask

  // upack model: data counter advances per pull, valid one cycle later (as util_upack2)
  reg [15:0] cnt = 1;
  always @(posedge clk) begin
    upv <= tx_pull; if (tx_pull) begin up0 <= cnt; up1 <= cnt; up2 <= cnt; up3 <= cnt; cnt <= cnt + 1; end
  end
  // request pattern: 8 on / 0 off continuous (test 1), later bursty
  integer mode = 0; integer ph = 0;
  always @(posedge clk) begin ph <= ph + 1; tx_req <= (mode == 0) ? 1'b1 : ((ph % 12) < 8); end

  // monitor
  integer nout = 0; reg [63:0] tsnow; integer errors = 0;
  always @(posedge clk) if (resetn && txov) begin nout <= nout + 1; end
  integer f; initial f = $fopen("tx_out.txt", "w");
  always @(posedge clk) if (resetn && txov) $fwrite(f, "%0d %0d %0d\n", dut.ts, txo0, txunf);

  integer i; reg [31:0] v, v2;
  initial begin
    #100 aresetn = 1; resetn = 1;
    // ---- identity
    axi_r(8'h00, v); if (v !== 32'h50353031) begin errors = errors + 1; $display("ID bad %h", v); end
    // ---- TX timed bursts
    cmd(4'd5, 0, 1);                 // timed TX
    cmd(4'd7, 0, 0);
    cmd(4'd6, 64'd400, 40);          // t=400, 40 frames
    cmd(4'd6, 64'd440, 30);          // contiguous
    cmd(4'd6, 64'd600, 20);          // gap
    #50000;
    axi_r(8'h2c, v); $display("tx_done=%0d (expect 3)", v); if (v != 3) errors = errors + 1;
    axi_r(8'h30, v); $display("tx_late=%0d (expect 0)", v); if (v != 0) errors = errors + 1;
    // late burst
    axi_r(8'h1c, v); axi_r(8'h20, v2); $display("ts now = %0d", {v2, v});
    cmd(4'd6, 64'd10, 15);           // far in the past -> late, discarded
    #20000;
    axi_r(8'h30, v); $display("tx_late=%0d (expect 1)", v); if (v != 1) errors = errors + 1;
    // ---- bursty req pattern, push relative to ts
    mode = 1;
    axi_r(8'h1c, v); axi_r(8'h20, v2);
    cmd(4'd6, {v2, v} + 64'd2000, 50); cmd(4'd6, {v2, v} + 64'd2050, 50);
    #200000;
    axi_r(8'h2c, v); $display("tx_done=%0d (expect 5)", v); if (v != 5) errors = errors + 1;
    // ---- RX gate
    cmd(4'd4, 0, 3);                 // timed + tag
    axi_r(8'h1c, v); axi_r(8'h20, v2);
    cmd(4'd2, {v2, v} + 64'd300, 0);
    rxv = 1; rxd0 = 16'h0123; rxd1 = 16'h0456; rxd2 = 16'h0789; rxd3 = 16'h0abc;
    #30000;
    axi_r(8'h24, v); axi_r(8'h28, v2); $display("rx first ts = %0d  (armed at ~ ts+300)", {v2, v});
    axi_r(8'h18, v); $display("status = %h", v);
    cmd(4'd3, 0, 0); #1000; axi_r(8'h18, v); $display("status after stop = %h (rx_armed bit1 must be 0)", v);
    if (v[1]) errors = errors + 1;
    $display("DONE errors=%0d", errors);
    $finish;
  end
  // RX tag check
  reg seen = 0;
  always @(posedge clk) if (rxov && !seen && dut.rx_tag) begin
    seen <= 1; $display("first rx frame: tag nibbles %h %h %h %h  ts-low16 %h  data %h", rxo0[15:12], rxo1[15:12], rxo2[15:12], rxo3[15:12], dut.ts[15:0], rxo0[11:0]);
  end
endmodule
