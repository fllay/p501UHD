# Adds p501_time to the P501 block design and wires it into the ADC and DAC paths.
# vivado -mode batch -source build_ts_bd.tcl -tclargs <hdl_ts dir> [impl]
set dir [lindex $argv 0]
set do_impl [expr {[llength $argv] > 1 && [lindex $argv 1] eq "impl"}]
open_project P501_IIO.xpr
add_files -norecurse $dir/p501_time.v
add_files -fileset constrs_1 -norecurse $dir/p501_time.xdc
update_compile_order -fileset sources_1
open_bd_design [get_files system.bd]
if {[llength [get_bd_cells p501_time_0]] == 0} { create_bd_cell -type module -reference p501_time p501_time_0 }
set t p501_time_0
connect_bd_net -net [get_bd_nets sys_ps8_pl_clk0] [get_bd_pins $t/s_axi_aclk]
connect_bd_net -net [get_bd_nets sys_cpu_resetn] [get_bd_pins $t/s_axi_aresetn]
connect_bd_net -net [get_bd_nets util_ad9361_divclk_clk_out] [get_bd_pins $t/fs_clk]
connect_bd_net -net [get_bd_nets util_ad9361_divclk_reset_peripheral_aresetn] [get_bd_pins $t/fs_resetn]
# ---- RX: wfifo -> p501_time -> cpack
for {set i 0} {$i < 4} {incr i} {
  disconnect_bd_net /util_ad9361_adc_fifo_dout_data_$i [get_bd_pins util_ad9361_adc_pack/fifo_wr_data_$i]
  connect_bd_net -net [get_bd_nets util_ad9361_adc_fifo_dout_data_$i] [get_bd_pins $t/rx_in_data_$i]
  connect_bd_net [get_bd_pins $t/rx_out_data_$i] [get_bd_pins util_ad9361_adc_pack/fifo_wr_data_$i]
}
disconnect_bd_net /util_ad9361_adc_fifo_dout_valid_0 [get_bd_pins util_ad9361_adc_pack/fifo_wr_en]
connect_bd_net -net [get_bd_nets util_ad9361_adc_fifo_dout_valid_0] [get_bd_pins $t/rx_in_valid]
connect_bd_net [get_bd_pins $t/rx_out_valid] [get_bd_pins util_ad9361_adc_pack/fifo_wr_en]
# ---- TX: upack <-> p501_time <-> rfifo
disconnect_bd_net /axi_ad9361_dac_fifo_din_valid_0 [get_bd_pins util_ad9361_dac_upack/fifo_rd_en]
connect_bd_net -net [get_bd_nets axi_ad9361_dac_fifo_din_valid_0] [get_bd_pins $t/tx_req]
connect_bd_net [get_bd_pins $t/tx_pull] [get_bd_pins util_ad9361_dac_upack/fifo_rd_en]
for {set i 0} {$i < 4} {incr i} {
  disconnect_bd_net /util_ad9361_dac_upack_fifo_rd_data_$i [get_bd_pins axi_ad9361_dac_fifo/din_data_$i]
  connect_bd_net -net [get_bd_nets util_ad9361_dac_upack_fifo_rd_data_$i] [get_bd_pins $t/tx_up_data_$i]
  connect_bd_net [get_bd_pins $t/tx_out_data_$i] [get_bd_pins axi_ad9361_dac_fifo/din_data_$i]
}
for {set i 0} {$i < 4} {incr i} { disconnect_bd_net /util_ad9361_dac_upack_fifo_rd_valid [get_bd_pins axi_ad9361_dac_fifo/din_valid_in_$i] }
connect_bd_net -net [get_bd_nets util_ad9361_dac_upack_fifo_rd_valid] [get_bd_pins $t/tx_up_valid]
for {set i 0} {$i < 4} {incr i} { connect_bd_net [get_bd_pins $t/tx_out_valid] [get_bd_pins axi_ad9361_dac_fifo/din_valid_in_$i] }
disconnect_bd_net /util_ad9361_dac_upack_fifo_rd_underflow [get_bd_pins axi_ad9361_dac_fifo/din_unf]
connect_bd_net -net [get_bd_nets util_ad9361_dac_upack_fifo_rd_underflow] [get_bd_pins $t/tx_up_unf]
connect_bd_net [get_bd_pins $t/tx_out_unf] [get_bd_pins axi_ad9361_dac_fifo/din_unf]
# ---- AXI-lite
set_property CONFIG.NUM_MI 5 [get_bd_cells axi_cpu_interconnect]
connect_bd_intf_net [get_bd_intf_pins axi_cpu_interconnect/M04_AXI] [get_bd_intf_pins $t/s_axi]
set seg [get_bd_addr_segs -of_objects [get_bd_cells $t]]
puts "SEGS: $seg"
assign_bd_address -offset 0x99030000 -range 64K $seg
foreach s [get_bd_addr_segs] { if {[string match *p501_time* $s]} { puts "ADDR $s off=[get_property OFFSET $s] range=[get_property RANGE $s]" } }
validate_bd_design
save_bd_design
foreach p [get_bd_pins $t/*] { puts "WIRE [get_property NAME $p] net=[get_bd_nets -of_objects $p]" }
if {$do_impl} {
  generate_target all [get_files system.bd]
  reset_run synth_1
  launch_runs impl_1 -to_step write_bitstream -jobs 14
  wait_on_run impl_1
  puts "STATUS: [get_property STATUS [get_runs impl_1]]"
  open_run impl_1
  report_timing_summary -file /tmp/ts_timing.rpt
  report_utilization -file /tmp/ts_util.rpt
}
exit
