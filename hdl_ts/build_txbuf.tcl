# Deepen the DAC (TX) DMA burst memory so a ~0.5 ms DDR/AXI stall or late descriptor submit
# does not starve the DAC (one starvation delays the TX stream permanently in p501_time).
# Run: vivado -mode batch -source build_txbuf.tcl
open_project /home/pi/projects/p501UHD_hdl/hdl/P501_IIO.xpr
open_bd_design [get_files system.bd]
set c [get_bd_cells axi_ad9361_dac_dma]
puts "BEFORE: FIFO_SIZE=[get_property CONFIG.FIFO_SIZE $c] MAX_BYTES_PER_BURST=[get_property CONFIG.MAX_BYTES_PER_BURST $c]"
set mb 4096
while {[catch {set_property -dict [list CONFIG.FIFO_SIZE {32} CONFIG.MAX_BYTES_PER_BURST $mb] $c} err]} {
  puts "TRY $mb failed: $err"
  set mb [expr {$mb / 2}]
  if {$mb < 512} { error "no valid burst size" }
}
puts "AFTER: FIFO_SIZE=[get_property CONFIG.FIFO_SIZE $c] MAX_BYTES_PER_BURST=[get_property CONFIG.MAX_BYTES_PER_BURST $c]"
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 14
wait_on_run impl_1
puts "IMPL_STATUS: [get_property STATUS [get_runs impl_1]]"
open_run impl_1
report_timing_summary -file /tmp/txbuf_timing.rpt
report_utilization -file /tmp/txbuf_util.rpt
close_project
exit
