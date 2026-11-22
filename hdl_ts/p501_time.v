// p501_time -- hardware timebase for the PZSDR P501 AD9361 data path.
//
//  * 64-bit tick counter `ts`, one tick per sample-clock cycle (divclk = fs = 30.72 MHz in 2R2T)
//  * RX: optional timed start (gate opens when ts >= start) and an optional in-band tag: the low 16 bits of ts are
//        written into the (unused) top nibble of each of the 4 words of every frame, so lost samples are exact
//  * TX: optional timed bursts.  Descriptors {start tick, frames} are queued over AXI-lite; the burst data come from the
//        DAC DMA in order.  Between bursts zeros are written to the DAC rate-FIFO.  A burst starts when ts >= start
//        (late by more than LATE_TOL ticks: frames are discarded and counted).  rfifo request phase gives a jitter of
//        <= 8 ticks; contiguous bursts are gap free.
//  * mode registers default to pass-through, so the old untimed streaming keeps working unchanged.
//
// AXI-lite map (32 bit, byte addresses)
//   00 ID 'P501'  04 VERSION  08 A_LO  0C A_HI  10 B  14 CMD(w)/BUSY(r)  18 STATUS  1C TS_LO(r, latches TS_HI)  20 TS_HI
//   24 RXFIRST_LO(r, latches)  28 RXFIRST_HI  2C TX_DONE  30 TX_LATE  34 TX_UNF  38 LATE_TOL(rw, ticks)
// Commands (write the code to CMD after A/B are set, wait BUSY=0 before changing A/B/CMD again)
//   1 SET_TS(A)  2 RX_ARM(A=start)  3 RX_STOP  4 RX_MODE(B: bit0 timed, bit1 tag)  5 TX_MODE(B bit0 timed; flushes)
//   6 TX_PUSH(A=start, B=frames)  7 CLEAR (sticky flags + counters)  8 TX_FLUSH
// STATUS: 0 rx_open 1 rx_armed 2 rx_timed 3 rx_tag 4 tx_timed 5 tx_inwin 6 late 7 unf 8 descovf 15:9 -  23:16 desc_count
`timescale 1ns/1ps
module p501_time (
  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axi, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 100000000" *)
  input  s_axi_aclk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  s_axi_aresetn,
  input  [7:0]  s_axi_awaddr,
  input         s_axi_awvalid,
  output        s_axi_awready,
  input  [31:0] s_axi_wdata,
  input  [3:0]  s_axi_wstrb,
  input         s_axi_wvalid,
  output        s_axi_wready,
  output [1:0]  s_axi_bresp,
  output reg    s_axi_bvalid,
  input         s_axi_bready,
  input  [7:0]  s_axi_araddr,
  input         s_axi_arvalid,
  output        s_axi_arready,
  output reg [31:0] s_axi_rdata,
  output [1:0]  s_axi_rresp,
  output reg    s_axi_rvalid,
  input         s_axi_rready,

  (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 fs_clk CLK" *)
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_RESET fs_resetn" *)
  input  fs_clk,
  (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 fs_resetn RST" *)
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  fs_resetn,

  input  [15:0] rx_in_data_0, input [15:0] rx_in_data_1, input [15:0] rx_in_data_2, input [15:0] rx_in_data_3,
  input         rx_in_valid,
  output reg [15:0] rx_out_data_0, output reg [15:0] rx_out_data_1, output reg [15:0] rx_out_data_2, output reg [15:0] rx_out_data_3,
  output reg    rx_out_valid,

  input         tx_req,
  output        tx_pull,
  input  [15:0] tx_up_data_0, input [15:0] tx_up_data_1, input [15:0] tx_up_data_2, input [15:0] tx_up_data_3,
  input         tx_up_valid,
  input         tx_up_unf,
  output [15:0] tx_out_data_0, output [15:0] tx_out_data_1, output [15:0] tx_out_data_2, output [15:0] tx_out_data_3,
  output        tx_out_valid,
  output        tx_out_unf
);

  // =====================================================================
  // AXI clock domain
  // =====================================================================
  reg [31:0] a_lo, a_hi, b_reg, late_tol_r;
  reg [3:0]  cmd_code;
  reg        cmd_req_t = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg cmd_ack_s1, cmd_ack_s2;
  wire       cmd_ack_t;                       // from fs_clk domain
  wire       cmd_busy = cmd_req_t ^ cmd_ack_s2;

  // status snapshot from the fs_clk domain
  wire [255:0] st_data;
  wire         st_req_t;
  reg          st_ack_t = 1'b0;
  (* ASYNC_REG = "TRUE" *) reg st_req_s1, st_req_s2;
  reg [255:0]  st_sh;
  reg [31:0]   ts_hi_lat, rxf_hi_lat;

  wire [63:0] sh_ts    = st_sh[63:0];
  wire [63:0] sh_rxf   = st_sh[127:64];
  wire [15:0] sh_flags = st_sh[143:128];
  wire [7:0]  sh_cnt   = st_sh[151:144];
  wire [31:0] sh_done  = st_sh[183:152];
  wire [31:0] sh_late  = st_sh[215:184];
  wire [31:0] sh_unf   = st_sh[247:216];

  always @(posedge s_axi_aclk) begin
    cmd_ack_s1 <= cmd_ack_t;  cmd_ack_s2 <= cmd_ack_s1;
    st_req_s1  <= st_req_t;   st_req_s2  <= st_req_s1;
    if (!s_axi_aresetn) st_ack_t <= 1'b0;
    else if (st_req_s2 != st_ack_t) begin
      st_sh   <= st_data;
      st_ack_t <= st_req_s2;
    end
  end

  // write channel
  wire wr_go = s_axi_awvalid & s_axi_wvalid & ~s_axi_bvalid;
  assign s_axi_awready = wr_go;
  assign s_axi_wready  = wr_go;
  assign s_axi_bresp   = 2'b00;
  assign s_axi_rresp   = 2'b00;
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      s_axi_bvalid <= 1'b0; cmd_code <= 4'd0;
      a_lo <= 0; a_hi <= 0; b_reg <= 0; late_tol_r <= 32'd64;
    end else begin
      if (s_axi_bvalid & s_axi_bready) s_axi_bvalid <= 1'b0;
      if (wr_go) begin
        s_axi_bvalid <= 1'b1;
        case (s_axi_awaddr[7:2])
          6'h02: a_lo <= s_axi_wdata;
          6'h03: a_hi <= s_axi_wdata;
          6'h04: b_reg <= s_axi_wdata;
          6'h05: if (!cmd_busy) begin cmd_code <= s_axi_wdata[3:0]; cmd_req_t <= ~cmd_req_t; end
          6'h0e: late_tol_r <= s_axi_wdata;
          default: ;
        endcase
      end
    end
  end

  // read channel
  assign s_axi_arready = s_axi_arvalid & ~s_axi_rvalid;
  always @(posedge s_axi_aclk) begin
    if (!s_axi_aresetn) begin
      s_axi_rvalid <= 1'b0; s_axi_rdata <= 32'd0;
    end else begin
      if (s_axi_rvalid & s_axi_rready) s_axi_rvalid <= 1'b0;
      if (s_axi_arvalid & ~s_axi_rvalid) begin
        s_axi_rvalid <= 1'b1;
        case (s_axi_araddr[7:2])
          6'h00: s_axi_rdata <= 32'h50353031;
          6'h01: s_axi_rdata <= 32'd1;
          6'h02: s_axi_rdata <= a_lo;
          6'h03: s_axi_rdata <= a_hi;
          6'h04: s_axi_rdata <= b_reg;
          6'h05: s_axi_rdata <= {31'd0, cmd_busy};
          6'h06: s_axi_rdata <= {8'd0, sh_cnt, sh_flags};
          6'h07: begin s_axi_rdata <= sh_ts[31:0];  ts_hi_lat  <= sh_ts[63:32];  end
          6'h08: s_axi_rdata <= ts_hi_lat;
          6'h09: begin s_axi_rdata <= sh_rxf[31:0]; rxf_hi_lat <= sh_rxf[63:32]; end
          6'h0a: s_axi_rdata <= rxf_hi_lat;
          6'h0b: s_axi_rdata <= sh_done;
          6'h0c: s_axi_rdata <= sh_late;
          6'h0d: s_axi_rdata <= sh_unf;
          6'h0e: s_axi_rdata <= late_tol_r;
          default: s_axi_rdata <= 32'd0;
        endcase
      end
    end
  end

  // =====================================================================
  // sample clock domain
  // =====================================================================
  (* ASYNC_REG = "TRUE" *) reg [2:0] req_s;
  reg  cmd_ack_tr = 1'b0;
  assign cmd_ack_t = cmd_ack_tr;
  wire cmd_pulse = req_s[2] ^ req_s[1];

  reg [63:0] ts;
  reg [63:0] rx_start, rx_first;
  reg        rx_armed, rx_timed, rx_tag, rx_first_seen;
  reg        tx_timed;
  reg        in_win, discard;
  reg [31:0] win_left;
  reg        req_d, pull_d, disc_d;
  reg        late_s, unf_s, ovf_s;
  reg [31:0] tx_done, tx_late, tx_unf;

  // descriptor FIFO (32 entries)
  reg [95:0] dmem [0:31];
  reg [5:0]  wr_ptr, rd_ptr;
  wire [5:0] dcount = wr_ptr - rd_ptr;
  wire head_valid = (wr_ptr != rd_ptr);
  wire [95:0] head = dmem[rd_ptr[4:0]];
  wire [63:0] head_ts  = head[95:32];
  wire [31:0] head_len = head[31:0];

  // command decode (a/b/code are stable while busy)
  wire [63:0] a_arg = {a_hi, a_lo};
  wire c_setts  = cmd_pulse & (cmd_code == 4'd1);
  wire c_rxarm  = cmd_pulse & (cmd_code == 4'd2);
  wire c_rxstop = cmd_pulse & (cmd_code == 4'd3);
  wire c_rxmode = cmd_pulse & (cmd_code == 4'd4);
  wire c_txmode = cmd_pulse & (cmd_code == 4'd5);
  wire c_push   = cmd_pulse & (cmd_code == 4'd6);
  wire c_clear  = cmd_pulse & (cmd_code == 4'd7);
  wire c_flush  = cmd_pulse & (cmd_code == 4'd8);

  // ---- TX window state machine
  wire last_pull  = in_win & tx_req & (win_left == 32'd1);
  wire can_start  = ~in_win | last_pull;
  wire due        = head_valid & (ts >= head_ts);
  wire start      = tx_timed & due & can_start & ~c_flush;
  wire [63:0] late_by = ts - head_ts;
  wire is_late    = (late_by > {32'd0, late_tol_r});

  assign tx_pull = tx_timed ? (tx_req & in_win) : tx_req;

  wire pass_data  = tx_timed ? (pull_d & ~disc_d) : 1'b1;
  assign tx_out_data_0 = pass_data ? tx_up_data_0 : 16'd0;
  assign tx_out_data_1 = pass_data ? tx_up_data_1 : 16'd0;
  assign tx_out_data_2 = pass_data ? tx_up_data_2 : 16'd0;
  assign tx_out_data_3 = pass_data ? tx_up_data_3 : 16'd0;
  assign tx_out_valid  = tx_timed ? req_d : tx_up_valid;
  wire   unf_now       = tx_timed & pull_d & ~disc_d & ~tx_up_valid;
  assign tx_out_unf    = tx_timed ? unf_now : tx_up_unf;

  // ---- RX gate
  wire rx_gate = rx_timed ? (rx_armed & (ts >= rx_start)) : 1'b1;

  always @(posedge fs_clk) begin
    req_s <= {req_s[1:0], cmd_req_t};
    if (!fs_resetn) begin
      ts <= 64'd0; rx_armed <= 1'b0; rx_timed <= 1'b0; rx_tag <= 1'b0; rx_first_seen <= 1'b0; rx_first <= 64'd0; rx_start <= 64'd0;
      tx_timed <= 1'b0; in_win <= 1'b0; discard <= 1'b0; win_left <= 32'd0; wr_ptr <= 6'd0; rd_ptr <= 6'd0;
      req_d <= 1'b0; pull_d <= 1'b0; disc_d <= 1'b0; late_s <= 1'b0; unf_s <= 1'b0; ovf_s <= 1'b0;
      tx_done <= 32'd0; tx_late <= 32'd0; tx_unf <= 32'd0;
      rx_out_valid <= 1'b0;
    end else begin
      ts <= c_setts ? a_arg : ts + 64'd1;
      if (cmd_pulse) cmd_ack_tr <= ~cmd_ack_tr;

      // RX
      if (c_rxarm)  begin rx_armed <= 1'b1; rx_start <= a_arg; rx_first_seen <= 1'b0; end
      if (c_rxstop) begin rx_armed <= 1'b0; end
      if (c_rxmode) begin rx_timed <= b_reg[0]; rx_tag <= b_reg[1]; end
      rx_out_valid <= rx_in_valid & rx_gate;
      rx_out_data_0 <= rx_tag ? {ts[3:0],   rx_in_data_0[11:0]} : rx_in_data_0;
      rx_out_data_1 <= rx_tag ? {ts[7:4],   rx_in_data_1[11:0]} : rx_in_data_1;
      rx_out_data_2 <= rx_tag ? {ts[11:8],  rx_in_data_2[11:0]} : rx_in_data_2;
      rx_out_data_3 <= rx_tag ? {ts[15:12], rx_in_data_3[11:0]} : rx_in_data_3;
      if (rx_in_valid & rx_gate & ~rx_first_seen & rx_armed) begin rx_first <= ts; rx_first_seen <= 1'b1; end

      // TX
      req_d  <= tx_req;
      pull_d <= tx_pull & tx_timed;
      disc_d <= discard;
      if (c_txmode) begin tx_timed <= b_reg[0]; in_win <= 1'b0; wr_ptr <= 6'd0; rd_ptr <= 6'd0; end
      if (c_flush)  begin in_win <= 1'b0; wr_ptr <= 6'd0; rd_ptr <= 6'd0; end
      if (c_push) begin
        if (dcount >= 6'd32) ovf_s <= 1'b1;
        else begin dmem[wr_ptr[4:0]] <= {a_arg, b_reg}; wr_ptr <= wr_ptr + 6'd1; end
      end
      if (start) begin
        rd_ptr  <= rd_ptr + 6'd1;
        discard <= is_late;
        if (is_late) begin late_s <= 1'b1; tx_late <= tx_late + 32'd1; end
        in_win  <= (head_len != 32'd0);
        win_left <= head_len;
        if (last_pull & ~discard) tx_done <= tx_done + 32'd1;
      end else if (in_win & tx_req) begin
        win_left <= win_left - 32'd1;
        if (win_left == 32'd1) begin in_win <= 1'b0; if (~discard) tx_done <= tx_done + 32'd1; end
      end
      if (unf_now) begin unf_s <= 1'b1; tx_unf <= tx_unf + 32'd1; end
      if (c_clear) begin late_s <= 1'b0; unf_s <= 1'b0; ovf_s <= 1'b0; tx_done <= 32'd0; tx_late <= 32'd0; tx_unf <= 32'd0; end
    end
  end

  // ---- status snapshot -> AXI domain
  reg [255:0] st_data_r;
  reg         st_req_tr;
  (* ASYNC_REG = "TRUE" *) reg st_ack_s1, st_ack_s2;
  assign st_data  = st_data_r;
  assign st_req_t = st_req_tr;
  wire [15:0] flags = {7'd0, ovf_s, unf_s, late_s, in_win, tx_timed, rx_tag, rx_timed, rx_armed, rx_gate};
  always @(posedge fs_clk) begin
    st_ack_s1 <= st_ack_t; st_ack_s2 <= st_ack_s1;
    if (!fs_resetn) begin st_req_tr <= 1'b0; st_data_r <= 256'd0; end
    else if (st_req_tr == st_ack_s2) begin
      st_data_r <= {8'd0, tx_unf, tx_late, tx_done, 2'd0, dcount[5:0], flags, rx_first, ts};
      st_req_tr <= ~st_req_tr;
    end
  end

endmodule
