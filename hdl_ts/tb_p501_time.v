`timescale 1ns/1ps
module tb;
  integer errors = 0;
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
  reg all_avail = 1; integer avail = 0; reg feed1 = 0;
  wire ok = all_avail | (avail > 0);
  always @(posedge clk) begin
    upv <= tx_pull & ok; if (tx_pull & ok) begin up0 <= cnt; up1 <= cnt; up2 <= cnt; up3 <= cnt; cnt <= cnt + 1; end
    if (!all_avail) avail <= avail - ((tx_pull & ok) ? 1 : 0) + (feed1 ? 1 : 0);
  end
  task feed(input integer n, input integer gap); integer k; begin
    for (k = 0; k < n; k = k + 1) begin @(posedge clk); feed1 <= 1; @(posedge clk); feed1 <= 0; repeat (gap) @(posedge clk); end end endtask
  // starvation check: emitted data must be a gapless counter
  reg check_on = 0; reg [15:0] expect_v = 0; integer nchk = 0;
  always @(posedge clk) if (check_on && txov && txo0 != 0) begin
    if (txo0 !== expect_v) begin errors = errors + 1; $display("SLIP: got %0d expect %0d", txo0, expect_v); expect_v <= txo0 + 1; end
    else expect_v <= expect_v + 1;
    nchk <= nchk + 1;
  end
  // request pattern: 8 on / 0 off continuous (test 1), later bursty
  integer mode = 0; integer ph = 0;
  always @(posedge clk) begin ph <= ph + 1; tx_req <= (mode == 0) ? 1'b1 : ((ph % 12) < 8); end

  always @(posedge clk) if (dut.start) $display("START ts=%0d head_ts=%0d len=%0d late=%0d pend_eff=%0d disc=%0d", dut.ts, dut.head_ts, dut.head_len, dut.is_late, dut.pend_eff, dut.discard);
  // monitor
  integer nout = 0; reg [63:0] tsnow;
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
    // ---- starvation: the data arrives late and in dribbles; frames must never slip, late burst data must be consumed
    all_avail = 0; avail = 0; mode = 0;
    begin : starve
      reg [15:0] base; reg [63:0] t0;
      base = cnt; expect_v = base + 20; 
      axi_r(8'h1c, v); axi_r(8'h20, v2); t0 = {v2, v};
      cmd(4'd6, 64'd5, 20);              // late: 20 frames to be consumed and dropped
      cmd(4'd6, t0 + 64'd1500, 40);
      cmd(4'd6, t0 + 64'd4000, 40);
      cmd(4'd6, t0 + 64'd7000, 25);
      check_on = 1;
      repeat (300) @(posedge clk); feed(20, 1);
      repeat (1500) @(posedge clk); feed(25, 3);      // window A is already open and starved
      repeat (400) @(posedge clk); feed(35, 2);
      repeat (2500) @(posedge clk); feed(20, 3);      // window B open and starved
      repeat (300) @(posedge clk); feed(25, 1);
      repeat (6000) @(posedge clk);
      check_on = 0;
      $display("starve: avail=%0d pend=%0d consumed=%0d (expect 125) emitted=%0d (expect 105) unf=%0d", avail, dut.pend, cnt - base, nchk, dut.tx_unf);
      if (avail != 0 || dut.pend != 0 || (cnt - base) != 125 || nchk != 105) errors = errors + 1;
    end
    all_avail = 1;
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
