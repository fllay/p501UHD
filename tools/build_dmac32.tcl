# Rebuild of the vendor P501_IIO project (pzsdr-p501/hdl) with a larger RX DMA buffer.
# Run from a COPY of pzsdr-p501/hdl:  vivado -mode batch -source build_dmac32.tcl
# The ADC DMA (axi_ad9361_adc_dma) store-and-forward buffer is FIFO_SIZE x MAX_BYTES_PER_BURST
# (default 8 x 128 B = 1 KB ~ 4 us at 246 MB/s). When an AXI write stall fills it, a whole 128 B
# burst (16 samples of 2R2T) is dropped. 32 x 512 B = 16 KB (~66 us) cut drops from one cluster
# every ~10-20 s to about one per 100 s. util_ad9361_adc_fifo DIN_ADDRESS_WIDTH 4->12 had no effect.
open_project P501_IIO.xpr
open_bd_design [get_files system.bd]
set_property -dict [list CONFIG.FIFO_SIZE {32} CONFIG.MAX_BYTES_PER_BURST {512}] [get_bd_cells axi_ad9361_adc_dma]
validate_bd_design
save_bd_design
generate_target all [get_files system.bd]
reset_run synth_1
launch_runs impl_1 -to_step write_bitstream -jobs 14
wait_on_run impl_1
exit
