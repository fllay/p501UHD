// AD9361 register-level driver for the P501: a C port of notebooks/ad9361_driver.py (itself a subset of ADI's no-OS
// ad9361.c): BBPLL, sample-rate clock chain, RX/TX RF PLLs, the vendor analog bring-up with its calibrations, manual
// gain, TX attenuation and FDD. Same register sequence as the Python driver (checked with P501_SPI_TRACE).
#pragma once
#include <stdint.h>

typedef struct {
    int spi_fd;
    uint64_t ref_clk_hz, synth_ref_hz;
    int synth_ref_scaler;           // RF PLL ref scaler field: 0 = x1, 1 = /2, 2 = /4, 3 = x2
    int64_t lo_rx, lo_tx;           // last programmed LOs (-1 = none)
    int rx2tx2;
    int rxbbf_div;
    int tx_quad_lpf_tia_match;
    int last_tx_quad_cal_phase;     // 0x7FFFFFFF = none yet
} ad9361_t;

typedef struct {
    int adc_div, r2_div, r1_div, clkrf_div, rx_fir_div, dac_div, t2_div, t1_div, clktf_div, tx_fir_div;
} ad9361_chain_t;

typedef struct { int rx_bb_filter, tx_bb_filter, bb_dc, rf_dc, tx_quad; } ad9361_init_res_t;

int  ad9361_open(ad9361_t *ad, const char *spidev, uint64_t ref_clk_hz);   // releases RESETB, opens SPI
void ad9361_close(ad9361_t *ad);
int  ad9361_reg_read(ad9361_t *ad, int addr);
void ad9361_reg_write(ad9361_t *ad, int addr, int val);
void ad9361_reg_writef(ad9361_t *ad, int addr, int mask, int val);
int  ad9361_reg_readf(ad9361_t *ad, int addr, int mask);
int  ad9361_product_id(ad9361_t *ad);
void ad9361_power_up_clocks(ad9361_t *ad);
int  ad9361_set_bbpll_rate(ad9361_t *ad, uint64_t rate_hz);
void ad9361_set_clock_chain(ad9361_t *ad, const ad9361_chain_t *c);
int  ad9361_synth_cp_calib(ad9361_t *ad, int tx);
int  ad9361_set_lo_freq(ad9361_t *ad, int tx, uint64_t freq_hz);
void ad9361_enable_rx2tx2(ad9361_t *ad);
ad9361_init_res_t ad9361_analog_init(ad9361_t *ad, uint64_t rf_rx_bw_hz, uint64_t rf_tx_bw_hz, uint64_t bbpll_hz,
                                     uint64_t adc_clk_hz, uint64_t rx_freq_hz, int rx_input, uint64_t clkrf_hz,
                                     uint64_t clktf_hz, int tracking, int rx2tx2);
void ad9361_enable_fdd(ad9361_t *ad);
void ad9361_set_rx_manual_gain(ad9361_t *ad, int gain_index, int ch_mask);   // ch_mask bit0 = RX1, bit1 = RX2
void ad9361_set_tx_atten_db(ad9361_t *ad, double atten_db, int ch_mask);
