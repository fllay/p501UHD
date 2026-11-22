open_project P501_IIO.xpr
open_bd_design [get_files system.bd]
catch {update_module_reference [get_bd_cells p501_time_0]}
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
reset_run system_p501_time_0_0_synth_1
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 14
wait_on_run impl_1
puts "STATUS: [get_property STATUS [get_runs impl_1]]"
open_run impl_1
report_timing_summary -file /tmp/ts_timing.rpt
report_utilization -file /tmp/ts_util.rpt
exit
