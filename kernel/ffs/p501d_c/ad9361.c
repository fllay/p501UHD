// C port of notebooks/ad9361_driver.py (see ad9361.h). Function order and register sequence follow the Python file;
// comments there explain the hardware reasons for each step.
#include "ad9361.h"
#include "ad9361_tables.h"
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define REG_TX_ENABLE_FILTER_CTRL 0x002
#define REG_RX_ENABLE_FILTER_CTRL 0x003
#define REG_INPUT_SELECT 0x004
#define REG_RFPLL_DIVIDERS 0x005
#define REG_CLOCK_ENABLE 0x009
#define REG_BBPLL 0x00A
#define REG_ENSM_MODE 0x013
#define REG_ENSM_CONFIG_1 0x014
#define REG_ENSM_CONFIG_2 0x015
#define REG_CALIBRATION_CTRL 0x016
#define REG_REFERENCE_CLOCK_CYCLES 0x03A
#define REG_SDM_CTRL_1 0x03F
#define REG_FRACT_BB_FREQ_WORD_1 0x041
#define REG_FRACT_BB_FREQ_WORD_2 0x042
#define REG_FRACT_BB_FREQ_WORD_3 0x043
#define REG_INTEGER_BB_FREQ_WORD 0x044
#define REG_CLOCK_CTRL 0x045
#define REG_CP_CURRENT 0x046
#define REG_LOOP_FILTER_1 0x048
#define REG_LOOP_FILTER_2 0x049
#define REG_LOOP_FILTER_3 0x04A
#define REG_VCO_CTRL 0x04B
#define REG_VCO_PROGRAM_1 0x04C
#define REG_VCO_PROGRAM_2 0x04D
#define REG_SDM_CTRL 0x04E
#define REG_CH_1_OVERFLOW 0x05E
#define REG_TX1_ATTEN_0 0x073
#define REG_TX1_ATTEN_1 0x074
#define REG_TX2_ATTEN_0 0x075
#define REG_TX2_ATTEN_1 0x076
#define REG_TX_ATTEN_OFFSET 0x077
#define REG_TX2_DIG_ATTEN 0x07C
#define REG_TX_SYMBOL_ATTEN_CONFIG 0x081
#define REG_QUAD_CAL_NCO_FREQ_PHASE_OFFSET 0x0A0
#define REG_QUAD_CAL_CTRL 0x0A1
#define REG_KEXP_1 0x0A2
#define REG_KEXP_2 0x0A3
#define REG_QUAD_SETTLE_COUNT 0x0A4
#define REG_MAG_FTEST_THRESH 0x0A5
#define REG_MAG_FTEST_THRESH_2 0x0A6
#define REG_QUAD_CAL_STATUS_TX1 0x0A7
#define REG_QUAD_CAL_STATUS_TX2 0x0A8
#define REG_QUAD_CAL_COUNT 0x0A9
#define REG_TX_QUAD_FULL_LMT_GAIN 0x0AA
#define REG_TX_QUAD_LPF_GAIN 0x0AE
#define REG_TX_TUNE_CTRL 0x0CA
#define REG_CONFIG0 0x0D0
#define REG_RESISTOR 0x0D1
#define REG_CAPACITOR 0x0D2
#define REG_TX_BBF_TUNE_DIVIDER 0x0D6
#define REG_TX_BBF_TUNE_MODE 0x0D7
#define REG_AGC_CONFIG_1 0x0FA
#define REG_AGC_CONFIG_2 0x0FB
#define REG_MAX_LMT_FULL_GAIN 0x0FD
#define REG_RX1_MANUAL_LMT_FULL_GAIN 0x109
#define REG_RX1_MANUAL_DIGITALFORCED_GAIN 0x10B
#define REG_RX2_MANUAL_LMT_FULL_GAIN 0x10C
#define REG_RX2_MANUAL_DIGITALFORCED_GAIN 0x10E
#define REG_GAIN_TABLE_ADDRESS 0x130
#define REG_GAIN_TABLE_WRITE_DATA1 0x131
#define REG_GAIN_TABLE_WRITE_DATA2 0x132
#define REG_GAIN_TABLE_WRITE_DATA3 0x133
#define REG_GAIN_TABLE_READ_DATA1 0x134
#define REG_GAIN_TABLE_CONFIG 0x137
#define REG_GM_SUB_TABLE_ADDRESS 0x138
#define REG_GM_SUB_TABLE_GAIN_WRITE 0x139
#define REG_GM_SUB_TABLE_BIAS_WRITE 0x13A
#define REG_GM_SUB_TABLE_CTRL_WRITE 0x13B
#define REG_GM_SUB_TABLE_GAIN_READ 0x13C
#define REG_GM_SUB_TABLE_CONFIG 0x13F
#define REG_CALIBRATION_CONFIG_1 0x169
#define REG_CALIBRATION_CONFIG_2 0x16A
#define REG_CALIBRATION_CONFIG_3 0x16B
#define REG_RX_QUAD_GAIN2 0x16F
#define REG_WAIT_COUNT 0x185
#define REG_RF_DC_OFFSET_COUNT 0x186
#define REG_RF_DC_OFFSET_CONFIG_1 0x187
#define REG_RF_DC_OFFSET_ATTEN 0x188
#define REG_INVERT_BITS 0x189
#define REG_DC_OFFSET_CONFIG2 0x18B
#define REG_BB_DC_OFFSET_SHIFT 0x190
#define REG_BB_DC_OFFSET_COUNT 0x193
#define REG_BB_DC_OFFSET_ATTEN 0x194
#define REG_RX_MIX_GM_CONFIG 0x1C0
#define REG_RX_MIX_LO_CM 0x1D5
#define REG_RX_TIA_CONFIG 0x1DB
#define REG_TIA1_C_LSB 0x1DC
#define REG_TIA1_C_MSB 0x1DD
#define REG_TIA2_C_LSB 0x1DE
#define REG_TIA2_C_MSB 0x1DF
#define REG_RX1_TUNE_CTRL 0x1E2
#define REG_RX2_TUNE_CTRL 0x1E3
#define REG_RX_BBF_R2346 0x1E6
#define REG_RX_BBF_C3_MSB 0x1EB
#define REG_RX_BBF_C3_LSB 0x1EC
#define REG_RX_BBF_TUNE_DIVIDE 0x1F8
#define REG_RX_BBF_TUNE_CONFIG 0x1F9
#define REG_RX_BBBW_MHZ 0x1FB
#define REG_RX_BBBW_KHZ 0x1FC
#define REG_RX_PFD_CONFIG 0x230
#define REG_RX_FRACT_BYTE_2 0x235
#define REG_RX_FORCE_VCO_TUNE_1 0x238
#define REG_RX_ALC_VARACTOR 0x239
#define REG_RX_VCO_OUTPUT 0x23A
#define REG_RX_CP_CURRENT 0x23B
#define REG_RX_CP_CONFIG 0x23D
#define REG_RX_LOOP_FILTER_1 0x23E
#define REG_RX_LOOP_FILTER_2 0x23F
#define REG_RX_LOOP_FILTER_3 0x240
#define REG_RX_VCO_BIAS_1 0x242
#define REG_RX_CAL_STATUS 0x244
#define REG_RX_VCO_CAL_REF 0x245
#define REG_RX_VCO_PD_OVERRIDES 0x246
#define REG_RX_CP_OVERRANGE_VCO_LOCK 0x247
#define REG_RX_VCO_LDO 0x248
#define REG_RX_VCO_CAL 0x249
#define REG_RX_CP_LEVEL_DETECT 0x24B
#define REG_RX_DSM_SETUP_1 0x24D
#define REG_RX_VCO_VARACTOR_CTRL_0 0x250
#define REG_RX_VCO_VARACTOR_CTRL_1 0x251
#define REG_RX_LO_GEN_POWER_MODE 0x261
#define REG_TX_PFD_CONFIG 0x270
#define REG_TX_FRACT_BYTE_2 0x275
#define REG_TX_VCO_OUTPUT 0x27A
#define REG_TX_CP_OVERRANGE_VCO_LOCK 0x287
#define REG_REF_DIVIDE_CONFIG_1 0x2AB
#define REG_REF_DIVIDE_CONFIG_2 0x2AC
#define TX_RX_VCO_OFFSET (REG_TX_VCO_OUTPUT - REG_RX_VCO_OUTPUT)

#define FDD_MODE (1 << 0)
#define FORCE_RX_ON (1 << 6)
#define FORCE_TX_ON (1 << 5)
#define BBPLL_LOCK (1 << 7)
#define VCO_LOCK (1 << 1)
#define INIT_BB_FO_CAL (1 << 2)
#define BBPLL_RESET_BAR (1 << 0)
#define FREQ_CAL_ENABLE (1 << 7)
#define DIGITAL_POWER_UP (1 << 2)
#define BBPLL_ENABLE (1 << 0)
#define XO_BYPASS (1 << 4)
#define RX_REF_RESET_BAR (1 << 1)
#define TX_REF_RESET_BAR (1 << 4)
#define DUAL_SYNTH_MODE (1 << 2)
#define BYPASS_LD_SYNTH (1 << 0)
#define TO_ALERT (1 << 0)
#define FORCE_ALERT_STATE (1 << 2)
#define TX_OUTPUT (1 << 6)
#define DEC_PWR_FOR_LOW_PWR (1 << 7)
#define DEC_PWR_FOR_LOCK_LEVEL (1 << 6)
#define DEC_PWR_FOR_GAIN_LOCK_EXIT (1 << 5)
#define RX_BB_TUNE_CAL (1 << 7)
#define RX_TUNE_RESAMPLE (1 << 1)
#define RX_PD_TUNE (1 << 0)
#define AGC_USE_FULL_GAIN_TABLE (1 << 3)
#define WRITE_GAIN_TABLE (1 << 2)
#define START_GAIN_TABLE_CLOCK (1 << 1)
#define TX_BB_TUNE_CAL (1 << 6)
#define TUNER_RESAMPLE (1 << 1)
#define TX_TUNE_CTRL_1 (1 << 5)
#define TX_PD_TUNE (1 << 2)
#define BBDC_CAL (1 << 0)
#define RFDC_CAL (1 << 1)
#define TX_QUAD_CAL (1 << 4)
#define USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL (1 << 7)
#define INVERT_RX1_RF_DC_CGOUT_WORD (1 << 4)
#define INVERT_RX2_RF_DC_CGOUT_WORD (1 << 5)
#define TX1_LO_CONV (1 << 1)
#define TX1_SSB_CONV (1 << 0)
#define CP_OFFSET_OFF (1 << 4)
#define CP_CAL_ENABLE (1 << 2)
#define CP_CAL_VALID (1 << 7)
#define VCO_CAL_EN (1 << 7)
#define WRITE_GM_SUB_TABLE (1 << 2)
#define START_GM_SUB_TABLE_CLOCK (1 << 1)
#define ENABLE_PHASE_CORR (1 << 7)
#define ENABLE_GAIN_CORR (1 << 6)
#define FREE_RUN_MODE (1 << 3)
#define ENABLE_CORR_WORD_DECIMATION (1 << 2)
#define ENABLE_TRACKING_MODE_CH1 (1 << 0)
#define CALIBRATION_CONFIG2_DFLT (0x3 << 5)
#define PREVENT_POS_LOOP_GAIN (1 << 7)
#define ENABLE_BB_DC_OFFSET_TRACKING (1 << 5)
#define ENABLE_RF_OFFSET_TRACKING (1 << 3)

#define BBPLL_MODULUS 2088960ULL
#define RFPLL_MODULUS 8388593ULL
#define MIN_VCO_FREQ_HZ 6000000000ULL
#define SYNTH_MAX_FREF_HZ 80008000ULL
#define MIN_SYNTH_FREF_HZ 9999000ULL

static const int CONTROL_GPIOS[3] = {640, 641, 642};   // EMIO RESETB / ENABLE / TXNRX

static uint64_t drc(uint64_t a, uint64_t b) { return (a + b / 2) / b; }         // div_round_closest
static uint64_t imin(uint64_t a, uint64_t b) { return a < b ? a : b; }
static uint64_t imax(uint64_t a, uint64_t b) { return a > b ? a : b; }
static uint64_t isqrt64(uint64_t n)                                            // floor(sqrt(n)), exact (math.isqrt)
{
    uint64_t x = 0, b = 1ULL << 62;
    while (b > n) b >>= 2;
    while (b) { if (n >= x + b) { n -= x + b; x = (x >> 1) + b; } else x >>= 1; b >>= 2; }
    return x;
}
static int shift_of(int mask) { int s = 0; while (!(mask & 1)) { mask >>= 1; s++; } return s; }

// ---- SPI trace (P501_SPI_TRACE=<file>): every transfer as hex bytes, to compare with the Python driver ----
static FILE *trace_f(void)
{
    static FILE *f; static int init;
    if (!init) { init = 1; const char *p = getenv("P501_SPI_TRACE"); if (p && *p) f = fopen(p, "w"); }
    return f;
}

static void xfer(ad9361_t *ad, uint8_t *buf, int n)
{
    struct spi_ioc_transfer t; memset(&t, 0, sizeof t);
    t.tx_buf = (uintptr_t)buf; t.rx_buf = (uintptr_t)buf; t.len = n; t.speed_hz = 1000000; t.bits_per_word = 8;
    FILE *f = trace_f();
    if (f) { fprintf(f, "T"); for (int i = 0; i < n; i++) fprintf(f, " %02x", buf[i]); }
    if (ioctl(ad->spi_fd, SPI_IOC_MESSAGE(1), &t) < 0) perror("spi");
    if (f) { fprintf(f, " | %02x\n", buf[n - 1]); fflush(f); }
}

int ad9361_reg_read(ad9361_t *ad, int addr)
{
    int cmd = (0 << 15) | (addr & 0x3FF);
    uint8_t b[3] = {(uint8_t)(cmd >> 8), (uint8_t)cmd, 0};
    xfer(ad, b, 3);
    return b[2];
}

void ad9361_reg_write(ad9361_t *ad, int addr, int val)
{
    int cmd = (1 << 15) | (addr & 0x3FF);
    uint8_t b[3] = {(uint8_t)(cmd >> 8), (uint8_t)cmd, (uint8_t)val};
    xfer(ad, b, 3);
}

static void reg_write_multi(ad9361_t *ad, int start, const uint8_t *v, int n)
{
    int cmd = (1 << 15) | (((n - 1) & 0x7) << 12) | (start & 0x3FF);
    uint8_t b[10] = {(uint8_t)(cmd >> 8), (uint8_t)cmd};
    memcpy(b + 2, v, n);
    xfer(ad, b, n + 2);
}

void ad9361_reg_writef(ad9361_t *ad, int addr, int mask, int value)
{
    int cur = ad9361_reg_read(ad, addr), s = shift_of(mask);
    ad9361_reg_write(ad, addr, (cur & ~mask) | ((value << s) & mask));
}

int ad9361_reg_readf(ad9361_t *ad, int addr, int mask) { return (ad9361_reg_read(ad, addr) & mask) >> shift_of(mask); }

static int check_cal_done(ad9361_t *ad, int addr, int mask, int done_state)
{
    for (int i = 0; i < 20000; i++) {
        if (ad9361_reg_readf(ad, addr, mask) == done_state)
            return 1;
        usleep(120);
    }
    return 0;
}

int ad9361_product_id(ad9361_t *ad) { return ad9361_reg_read(ad, 0x037); }

// ---- open / reset ----
static int wr_file(const char *p, const char *v)
{
    FILE *f = fopen(p, "w"); if (!f) return -1;
    int r = fputs(v, f) < 0 ? -1 : 0; if (fclose(f)) r = -1; return r;
}

static void release_control_gpios(void)
{
    char p[96], v[16];
    for (int i = 0; i < 3; i++) {
        snprintf(p, sizeof p, "/sys/class/gpio/gpio%d", CONTROL_GPIOS[i]);
        if (access(p, F_OK)) { snprintf(v, sizeof v, "%d", CONTROL_GPIOS[i]); wr_file("/sys/class/gpio/export", v); }
    }
    usleep(200000);
    for (int i = 0; i < 3; i++) { snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/direction", CONTROL_GPIOS[i]); wr_file(p, "out"); }
    snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/value", CONTROL_GPIOS[0]); wr_file(p, "0");   // RESETB pulse
    usleep(10000);
    for (int i = 0; i < 3; i++) { snprintf(p, sizeof p, "/sys/class/gpio/gpio%d/value", CONTROL_GPIOS[i]); wr_file(p, "1"); }
    usleep(200000);
}

static void synth_ref_div_sel(uint64_t refin, uint64_t max_hz, uint64_t *ref, int *scaler)
{
    max_hz = imin(imax(max_hz, MIN_SYNTH_FREF_HZ), SYNTH_MAX_FREF_HZ);
    if (refin <= max_hz / 2) { *ref = 2 * refin; *scaler = 3; }
    else if (refin <= max_hz) { *ref = refin; *scaler = 0; }
    else if (refin <= 2 * max_hz) { *ref = refin / 2; *scaler = 1; }
    else { *ref = refin / 4; *scaler = 2; }
}

int ad9361_open(ad9361_t *ad, const char *spidev, uint64_t ref_clk_hz)
{
    memset(ad, 0, sizeof *ad);
    ad->ref_clk_hz = ref_clk_hz;
    // RF PLL reference. Default x1 (40 MHz, the 40 MHz VCO table): what the Python daemon has actually run with (it
    // imports the older jupyter_notebooks/ad9361_driver.py). P501_SYNTH_MAX_FREF=80008000 gives the vendor default
    // (reference doubled to 80 MHz, 80 MHz table), as the newer notebooks/ad9361_driver.py does.
    const char *e = getenv("P501_SYNTH_MAX_FREF");
    synth_ref_div_sel(ref_clk_hz, e ? strtoull(e, 0, 0) : 40008000ULL, &ad->synth_ref_hz, &ad->synth_ref_scaler);
    ad->lo_rx = ad->lo_tx = -1;
    ad->tx_quad_lpf_tia_match = -1;
    ad->last_tx_quad_cal_phase = 0x7FFFFFFF;
    release_control_gpios();
    ad->spi_fd = open(spidev, O_RDWR);
    if (ad->spi_fd < 0) { perror(spidev); return -1; }
    uint8_t mode = SPI_MODE_1, bits = 8; uint32_t speed = 1000000;    // CPOL=0, CPHA=1
    ioctl(ad->spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(ad->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(ad->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
    return 0;
}

void ad9361_close(ad9361_t *ad) { if (ad->spi_fd > 0) close(ad->spi_fd); ad->spi_fd = -1; }

// ---- clocks ----
void ad9361_power_up_clocks(ad9361_t *ad)
{
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_1, RX_REF_RESET_BAR, 1);
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_2, TX_REF_RESET_BAR, 1);
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_2, 0x3 << 0, 3);
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_1, 0x3 << 5, 3);
    ad9361_reg_write(ad, REG_CLOCK_ENABLE, DIGITAL_POWER_UP | (1 << 1) | BBPLL_ENABLE | XO_BYPASS);
    ad9361_reg_write(ad, REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE);
    ad9361_reg_write(ad, REG_ENSM_CONFIG_1, TO_ALERT | FORCE_ALERT_STATE);
    ad9361_reg_writef(ad, REG_RX_PFD_CONFIG, BYPASS_LD_SYNTH, 0);
    ad9361_reg_writef(ad, REG_TX_PFD_CONFIG, BYPASS_LD_SYNTH, 0);
    ad9361_reg_write(ad, REG_REFERENCE_CLOCK_CYCLES, (int)((ad->ref_clk_hz / 1000000) - 1) & 0x7F);
}

int ad9361_set_bbpll_rate(ad9361_t *ad, uint64_t rate_hz)
{
    uint64_t parent = ad->ref_clk_hz;
    uint64_t tmp = (rate_hz >> 7) * 150;
    tmp /= (parent >> 7) * 32;
    int64_t icp = (int64_t)drc(tmp, 25) - 1;
    if (icp < 1) icp = 1; if (icp > 63) icp = 63;
    ad9361_reg_write(ad, REG_CP_CURRENT, (int)icp);
    ad9361_reg_write(ad, REG_LOOP_FILTER_1, 0x35);
    ad9361_reg_write(ad, REG_LOOP_FILTER_2, 0x5B);
    ad9361_reg_write(ad, REG_LOOP_FILTER_3, 0xE8);
    ad9361_reg_write(ad, REG_VCO_CTRL, FREQ_CAL_ENABLE | (3 << 5));
    ad9361_reg_write(ad, REG_SDM_CTRL, 0x10);
    uint64_t integer = rate_hz / parent, rem = rate_hz % parent, fract = drc(rem * BBPLL_MODULUS, parent);
    ad9361_reg_write(ad, REG_INTEGER_BB_FREQ_WORD, (int)integer);
    ad9361_reg_write(ad, REG_FRACT_BB_FREQ_WORD_3, (int)(fract & 0xFF));
    ad9361_reg_write(ad, REG_FRACT_BB_FREQ_WORD_2, (int)((fract >> 8) & 0xFF));
    ad9361_reg_write(ad, REG_FRACT_BB_FREQ_WORD_1, (int)((fract >> 16) & 0xFF));
    ad9361_reg_write(ad, REG_SDM_CTRL_1, INIT_BB_FO_CAL | BBPLL_RESET_BAR);
    ad9361_reg_write(ad, REG_SDM_CTRL_1, BBPLL_RESET_BAR);
    ad9361_reg_write(ad, REG_VCO_PROGRAM_1, 0x86);
    ad9361_reg_write(ad, REG_VCO_PROGRAM_2, 0x01);
    ad9361_reg_write(ad, REG_VCO_PROGRAM_2, 0x05);
    return check_cal_done(ad, REG_CH_1_OVERFLOW, BBPLL_LOCK, 1);
}

static int ilog2i(int v) { int r = 0; while (v > 1) { v >>= 1; r++; } return r; }

void ad9361_set_clock_chain(ad9361_t *ad, const ad9361_chain_t *c)
{
    ad9361_reg_write(ad, REG_CLOCK_CTRL, 0);
    int s = ad->synth_ref_scaler;                                  // _write_synth_ref_scaler
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_1, 1 << 0, s >> 1);
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_2, 1 << 7, s & 1);
    ad9361_reg_writef(ad, REG_REF_DIVIDE_CONFIG_2, 0x3 << 2, s);
    ad9361_reg_writef(ad, REG_BBPLL, 0x7, ilog2i(c->adc_div));
    ad9361_reg_writef(ad, REG_BBPLL, 1 << 3, c->dac_div - 1);
    int rxf = c->rx_fir_div == 1 ? 0 : ilog2i(c->rx_fir_div) + 1, txf = c->tx_fir_div == 1 ? 0 : ilog2i(c->tx_fir_div) + 1;
    ad9361_reg_writef(ad, REG_RX_ENABLE_FILTER_CTRL, 0x3 << 4, c->r2_div - 1);
    ad9361_reg_writef(ad, REG_RX_ENABLE_FILTER_CTRL, 1 << 3, c->r1_div - 1);
    ad9361_reg_writef(ad, REG_RX_ENABLE_FILTER_CTRL, 1 << 2, c->clkrf_div - 1);
    ad9361_reg_writef(ad, REG_RX_ENABLE_FILTER_CTRL, 0x3 << 0, rxf);
    ad9361_reg_writef(ad, REG_TX_ENABLE_FILTER_CTRL, 0x3 << 4, c->t2_div - 1);
    ad9361_reg_writef(ad, REG_TX_ENABLE_FILTER_CTRL, 1 << 3, c->t1_div - 1);
    ad9361_reg_writef(ad, REG_TX_ENABLE_FILTER_CTRL, 1 << 2, c->clktf_div - 1);
    ad9361_reg_writef(ad, REG_TX_ENABLE_FILTER_CTRL, 0x3 << 0, txf);
}

// ---- RF PLLs ----
static void rfpll_vco_init(ad9361_t *ad, int tx, uint64_t vco_freq_hz, int use_fdd)
{
    int n; const synth_row_t *t = synth_lut(ad->synth_ref_hz, use_fdd, &n);
    uint64_t vco_mhz = vco_freq_hz / 1000000;
    const synth_row_t *row = &t[n - 1];
    for (int i = 0; i < n; i++) { row = &t[i]; if (t[i].vco_mhz <= vco_mhz) break; }
    const uint8_t *v = row->v;   // out_level varactor bias_ref bias_tcf cal_offset varactor_ref cp_current lf_c2 lf_c1 lf_r1 lf_c3 lf_r3
    int off = tx ? TX_RX_VCO_OFFSET : 0;
    ad9361_reg_write(ad, REG_RX_VCO_OUTPUT + off, (v[0] & 0xF) | (1 << 6));
    ad9361_reg_writef(ad, REG_RX_ALC_VARACTOR + off, 0xF, v[1]);
    ad9361_reg_write(ad, REG_RX_VCO_BIAS_1 + off, (v[2] & 0x7) | ((v[3] & 0x3) << 3));
    ad9361_reg_write(ad, REG_RX_FORCE_VCO_TUNE_1 + off, (v[4] & 0xF) << 3);
    ad9361_reg_write(ad, REG_RX_VCO_VARACTOR_CTRL_1 + off, v[5] & 0xF);
    ad9361_reg_write(ad, REG_RX_VCO_CAL_REF + off, 0);
    ad9361_reg_write(ad, REG_RX_VCO_VARACTOR_CTRL_0 + off, 7 << 4);
    ad9361_reg_writef(ad, REG_RX_CP_CURRENT + off, 0x3F, v[6]);
    ad9361_reg_write(ad, REG_RX_LOOP_FILTER_1 + off, ((v[7] & 0xF) << 4) | (v[8] & 0xF));
    ad9361_reg_write(ad, REG_RX_LOOP_FILTER_2 + off, ((v[9] & 0xF) << 4) | (v[10] & 0xF));
    ad9361_reg_write(ad, REG_RX_LOOP_FILTER_3 + off, v[11] & 0xF);
}

int ad9361_set_lo_freq(ad9361_t *ad, int tx, uint64_t freq_hz)
{
    uint64_t ref = ad->synth_ref_hz, freq = freq_hz;
    int vco_div = -1;
    while (freq <= MIN_VCO_FREQ_HZ) { freq <<= 1; vco_div++; }
    int64_t other = tx ? ad->lo_rx : ad->lo_tx;
    if (tx) ad->lo_tx = (int64_t)freq_hz; else ad->lo_rx = (int64_t)freq_hz;
    rfpll_vco_init(ad, tx, freq, other >= 0 && (uint64_t)other != freq_hz);
    uint64_t integer = freq / ref, rem = freq % ref, fract = drc(rem * RFPLL_MODULUS, ref);
    int reg = tx ? REG_TX_FRACT_BYTE_2 : REG_RX_FRACT_BYTE_2, lock = tx ? REG_TX_CP_OVERRANGE_VCO_LOCK : REG_RX_CP_OVERRANGE_VCO_LOCK;
    int div_mask = tx ? 0xF << 4 : 0xF << 0;
    uint8_t b[5];
    b[0] = (fract >> 16) & 0x7F; b[1] = (fract >> 8) & 0xFF; b[2] = fract & 0xFF;
    int top = ad9361_reg_read(ad, reg - 3) & 0xF8;
    b[3] = ((integer >> 8) & 0x7) | top; b[4] = integer & 0xFF;
    reg_write_multi(ad, reg, b, 5);
    ad9361_reg_writef(ad, REG_RFPLL_DIVIDERS, div_mask, vco_div);
    return check_cal_done(ad, lock, VCO_LOCK, 1);
}

int ad9361_synth_cp_calib(ad9361_t *ad, int tx)
{
    int o = tx ? 0x40 : 0;
    ad9361_reg_write(ad, REG_RX_CP_LEVEL_DETECT + o, 0x17);
    ad9361_reg_write(ad, REG_RX_DSM_SETUP_1 + o, 0x0);
    ad9361_reg_write(ad, REG_RX_LO_GEN_POWER_MODE + o, 0x00);
    ad9361_reg_write(ad, REG_RX_VCO_LDO + o, 0x0B);
    ad9361_reg_write(ad, REG_RX_VCO_PD_OVERRIDES + o, 0x02);
    ad9361_reg_write(ad, REG_RX_CP_CURRENT + o, 0x80);
    ad9361_reg_write(ad, REG_RX_CP_CONFIG + o, CP_OFFSET_OFF);
    ad9361_reg_write(ad, REG_RX_VCO_CAL + o, VCO_CAL_EN | (3 << 2) | 2);
    ad9361_reg_write(ad, REG_ENSM_CONFIG_2, DUAL_SYNTH_MODE);
    ad9361_reg_write(ad, REG_ENSM_CONFIG_1, FORCE_ALERT_STATE | TO_ALERT);
    ad9361_reg_write(ad, REG_ENSM_MODE, FDD_MODE);
    ad9361_reg_write(ad, REG_RX_CP_CONFIG + o, CP_OFFSET_OFF | CP_CAL_ENABLE);
    return check_cal_done(ad, REG_RX_CAL_STATUS + o, CP_CAL_VALID, 1);
}

// ---- analog bring-up ----
static void rf_port_setup(ad9361_t *ad, int rx_input, int tx_b)
{
    int val = rx_input < 3 ? (3 << (rx_input * 2)) : (1 << (rx_input - 3));
    if (tx_b) val |= TX_OUTPUT;
    ad9361_reg_write(ad, REG_INPUT_SELECT, val);
}

static void load_mixer_gm_subtable(ad9361_t *ad)
{
    const int n = (int)sizeof GM_ST_CTRL;
    ad9361_reg_write(ad, REG_GM_SUB_TABLE_CONFIG, START_GM_SUB_TABLE_CLOCK);
    int addr = n;
    for (int i = 0; i < n; i++) {
        addr--;
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_ADDRESS, addr);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_BIAS_WRITE, 0);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_GAIN_WRITE, GM_ST_GAIN[i]);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_CTRL_WRITE, GM_ST_CTRL[i]);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_CONFIG, WRITE_GM_SUB_TABLE | START_GM_SUB_TABLE_CLOCK);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_GAIN_READ, 0);
        ad9361_reg_write(ad, REG_GM_SUB_TABLE_GAIN_READ, 0);
    }
    ad9361_reg_write(ad, REG_GM_SUB_TABLE_CONFIG, START_GM_SUB_TABLE_CLOCK);
    ad9361_reg_write(ad, REG_GM_SUB_TABLE_GAIN_READ, 0);
    ad9361_reg_write(ad, REG_GM_SUB_TABLE_GAIN_READ, 0);
    ad9361_reg_write(ad, REG_GM_SUB_TABLE_CONFIG, 0);
}

static void gain_control_manual(ad9361_t *ad)
{
    ad9361_reg_write(ad, REG_AGC_CONFIG_1, DEC_PWR_FOR_GAIN_LOCK_EXIT | DEC_PWR_FOR_LOCK_LEVEL | DEC_PWR_FOR_LOW_PWR);
    ad9361_reg_write(ad, REG_RX1_MANUAL_DIGITALFORCED_GAIN, 0);
    ad9361_reg_write(ad, REG_RX2_MANUAL_DIGITALFORCED_GAIN, 0);
    ad9361_reg_write(ad, REG_TX_SYMBOL_ATTEN_CONFIG, 0x00);
}

static void load_rx_gain_table(ad9361_t *ad, int dest, uint64_t freq_hz)
{
    const uint8_t (*tab)[3] = RX_GAIN_TABLES[freq_hz <= 1300000000ULL ? 0 : (freq_hz <= 4000000000ULL ? 1 : 2)];
    const int n = 77;
    ad9361_reg_writef(ad, REG_AGC_CONFIG_2, AGC_USE_FULL_GAIN_TABLE, 1);
    ad9361_reg_write(ad, REG_MAX_LMT_FULL_GAIN, n - 1);
    int sel = (dest & 0x3) << 3;
    ad9361_reg_write(ad, REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK | sel);
    ad->tx_quad_lpf_tia_match = -1;
    for (int i = 0; i < n; i++) {
        if ((tab[i][1] & 0x3F) == 0x20) ad->tx_quad_lpf_tia_match = i;
        ad9361_reg_write(ad, REG_GAIN_TABLE_ADDRESS, i);
        ad9361_reg_write(ad, REG_GAIN_TABLE_WRITE_DATA1, tab[i][0]);
        ad9361_reg_write(ad, REG_GAIN_TABLE_WRITE_DATA2, tab[i][1]);
        ad9361_reg_write(ad, REG_GAIN_TABLE_WRITE_DATA3, tab[i][2]);
        ad9361_reg_write(ad, REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK | WRITE_GAIN_TABLE | sel);
        ad9361_reg_write(ad, REG_GAIN_TABLE_READ_DATA1, 0);
        ad9361_reg_write(ad, REG_GAIN_TABLE_READ_DATA1, 0);
    }
    ad9361_reg_write(ad, REG_GAIN_TABLE_CONFIG, START_GAIN_TABLE_CLOCK | sel);
    ad9361_reg_write(ad, REG_GAIN_TABLE_READ_DATA1, 0);
    ad9361_reg_write(ad, REG_GAIN_TABLE_READ_DATA1, 0);
    ad9361_reg_write(ad, REG_GAIN_TABLE_CONFIG, 0);
}

static int rx_bb_analog_filter_calib(ad9361_t *ad, uint64_t bw, uint64_t bbpll)
{
    bw = imax(200000, imin(28000000, bw));
    uint64_t target = 126906 * (bw / 10000);
    ad->rxbbf_div = (int)imin(511, (bbpll + target - 1) / target);
    ad9361_reg_write(ad, REG_RX_BBF_TUNE_DIVIDE, ad->rxbbf_div & 0xFF);
    ad9361_reg_writef(ad, REG_RX_BBF_TUNE_CONFIG, 1 << 0, ad->rxbbf_div >> 8);
    ad9361_reg_write(ad, REG_RX_BBBW_MHZ, (int)(bw / 1000000));
    ad9361_reg_write(ad, REG_RX_BBBW_KHZ, (int)imin(127, drc((bw % 1000000) * 128, 1000000)));
    ad9361_reg_write(ad, REG_RX_MIX_LO_CM, 0x3F);
    ad9361_reg_write(ad, REG_RX_MIX_GM_CONFIG, 3);
    ad9361_reg_write(ad, REG_RX1_TUNE_CTRL, RX_TUNE_RESAMPLE);
    ad9361_reg_write(ad, REG_RX2_TUNE_CTRL, RX_TUNE_RESAMPLE);
    ad9361_reg_write(ad, REG_CALIBRATION_CTRL, RX_BB_TUNE_CAL);
    int ok = check_cal_done(ad, REG_CALIBRATION_CTRL, RX_BB_TUNE_CAL, 0);
    ad9361_reg_write(ad, REG_RX1_TUNE_CTRL, RX_TUNE_RESAMPLE | RX_PD_TUNE);
    ad9361_reg_write(ad, REG_RX2_TUNE_CTRL, RX_TUNE_RESAMPLE | RX_PD_TUNE);
    return ok;
}

static int run_calibration(ad9361_t *ad, int mask)
{
    ad9361_reg_write(ad, REG_CALIBRATION_CTRL, mask);
    return check_cal_done(ad, REG_CALIBRATION_CTRL, mask, 0);
}

static int tx_bb_analog_filter_calib(ad9361_t *ad, uint64_t bw, uint64_t bbpll)
{
    bw = imax(625000, imin(20000000, bw));
    uint64_t target = 145036 * (bw / 10000);
    int div = (int)imin(511, (bbpll + target - 1) / target);
    ad9361_reg_write(ad, REG_TX_BBF_TUNE_DIVIDER, div & 0xFF);
    ad9361_reg_writef(ad, REG_TX_BBF_TUNE_MODE, 1 << 0, div >> 8);
    ad9361_reg_write(ad, REG_TX_TUNE_CTRL, TUNER_RESAMPLE | TX_TUNE_CTRL_1);
    int ok = run_calibration(ad, TX_BB_TUNE_CAL);
    ad9361_reg_write(ad, REG_TX_TUNE_CTRL, TUNER_RESAMPLE | TX_TUNE_CTRL_1 | TX_PD_TUNE);
    return ok;
}

static void rx_tia_calib(ad9361_t *ad, uint64_t bw)
{
    int reg1eb = ad9361_reg_read(ad, REG_RX_BBF_C3_MSB), reg1ec = ad9361_reg_read(ad, REG_RX_BBF_C3_LSB);
    int reg1e6 = ad9361_reg_read(ad, REG_RX_BBF_R2346);
    bw = imax(200000, imin(20000000, bw));
    int64_t cbbf = reg1eb * 160 + reg1ec * 10 + 140, r2346 = 18300 * (reg1e6 & 0x7);
    int64_t ctia_ff = (cbbf * r2346 * 560) / 3500000;   // Python floor division; all terms >= 0
    int reg1db = bw <= 3000000 ? 0xE0 : (bw <= 10000000 ? 0x60 : 0x20), reg1dc, reg1dd, reg1de, reg1df;
    if (ctia_ff > 2920) {
        reg1dc = reg1de = 0x40;
        int64_t t = (ctia_ff - 400 + 160) / 320; if (t > 127) t = 127;
        reg1dd = reg1df = (int)t;
    } else {
        int64_t q = ctia_ff - 400 + 20, t = q >= 0 ? q / 40 : -((-q + 39) / 40);    // Python (x + 20) // 40 (floor)
        reg1dc = reg1de = (int)((t + 0x40) & 0xFF);
        reg1dd = reg1df = 0;
    }
    ad9361_reg_write(ad, REG_RX_TIA_CONFIG, reg1db);
    ad9361_reg_write(ad, REG_TIA1_C_LSB, reg1dc);
    ad9361_reg_write(ad, REG_TIA1_C_MSB, reg1dd);
    ad9361_reg_write(ad, REG_TIA2_C_LSB, reg1de);
    ad9361_reg_write(ad, REG_TIA2_C_MSB, reg1df);
}

static void tx_bb_second_filter_calib(ad9361_t *ad, uint64_t bw)
{
    bw = imax(530000, imin(20000000, bw));
    uint64_t corner = 15708 * (bw / 10000), res = 1, cap = 0;
    for (int i = 0; i < 4; i++) {
        uint64_t div = corner * res;
        cap = (500000000ULL + (div >> 1)) / div - 12;                 // wraps like the Python & M64
        if (cap < 64) break;
        res <<= 1;
    }
    if (cap > 63) cap = 63;
    int conf = bw <= 4500000 ? 0x59 : (bw <= 12000000 ? 0x56 : 0x57);
    int rr = res == 1 ? 0x0C : res == 2 ? 0x04 : res == 4 ? 0x03 : 0x01;
    ad9361_reg_write(ad, REG_CONFIG0, conf);
    ad9361_reg_write(ad, REG_RESISTOR, rr);
    ad9361_reg_write(ad, REG_CAPACITOR, (int)(cap & 0xFF));
}

static void rx_adc_setup(ad9361_t *ad, uint64_t bbpll, uint64_t adc_clk)
{
    typedef unsigned __int128 u128;
    int c3_msb = ad9361_reg_read(ad, REG_RX_BBF_C3_MSB), c3_lsb = ad9361_reg_read(ad, REG_RX_BBF_C3_LSB);
    int r2346 = ad9361_reg_read(ad, REG_RX_BBF_R2346);
    uint64_t bb = (bbpll * 10000) / (126906ULL * (uint64_t)ad->rxbbf_div);
    bb = imax(200000, imin(28000000, bb));
    uint64_t scale_snr = adc_clk < 80000000 ? 1000 : 1585;
    u128 inv = (u128)160975 * (u128)r2346 * (u128)(160 * c3_msb + 10 * c3_lsb + 140) * (u128)bb;
    if (bb >= 18000000) { inv = inv * (u128)(1000 + (10 * (bb - 18000000) / 1000000)); inv /= 1000; }
    inv /= 1000000000;
    uint64_t invrc = (uint64_t)inv;
    if (!invrc) { fprintf(stderr, "ad9361: invrc == 0 (BB filter calibration failed?)\n"); return; }
    uint64_t s = isqrt64(invrc), maxsnr = 640 / 160;
    uint64_t sadc = drc(adc_clk, 640), inv_sadc = drc(640000000ULL, drc(adc_clk, 1000));
    uint64_t tmp = drc(980000 + 20 * imax(1000, drc(inv_sadc, maxsnr)), 1000);
    uint64_t sqt = isqrt64(sadc), msq = imin(1000, isqrt64(maxsnr * sadc));
    uint64_t d[40] = {0}, t;
    d[3] = 0x24; d[4] = 0x24;
    d[7] = imin(124, ((uint64_t)(-50000000LL + (int64_t)(8 * scale_snr * s * msq))) / 100000000);
    t = (invrc >> 1) + (20 * inv_sadc * d[7] / 80) * 1000;
    d[8] = imin(255, t / invrc);
    d[10] = imin(127, ((uint64_t)(-500000LL + (int64_t)(77 * s * msq))) / 1000000);
    d[9] = imin(127, (800 * d[10]) / 1000);
    t = (invrc >> 1) + 20 * inv_sadc * d[10] * 1000;
    d[11] = imin(255, t / (invrc * 77));
    d[12] = imin(127, ((uint64_t)(-500000LL + (int64_t)(80 * s * msq))) / 1000000);
    t = (uint64_t)(-3 * (int64_t)(invrc >> 1) + (int64_t)(inv_sadc * d[12] * 250));
    d[13] = imin(255, t / invrc);
    d[14] = (21 * (inv_sadc / 10000)) & 0xFF;
    d[15] = imin(127, (500 + 1025 * d[7]) / 1000);
    d[16] = imin(127, (d[15] * tmp) / 1000);
    d[17] = d[15];
    d[18] = imin(127, (500 + 975 * d[10]) / 1000);
    d[19] = imin(127, (d[18] * tmp) / 1000);
    d[20] = d[18];
    d[21] = imin(127, (500 + 975 * d[12]) / 1000);
    d[22] = imin(127, (d[21] * tmp) / 1000);
    d[23] = d[21];
    d[24] = 0x2E;
    d[25] = 128 + imin(63000, drc(63 * sadc, 1000)) / 1000;
    d[26] = imin(63, (63 * sadc / 1000000) * (920 + 80 * inv_sadc / 1000) / 1000);
    d[27] = imin(63, (32 * sqt) / 1000);
    d[28] = d[25]; d[29] = d[26]; d[30] = d[27]; d[31] = d[25]; d[32] = d[26];
    d[33] = imin(63, 63 * sqt / 1000);
    d[34] = imin(127, 64 * sqt / 1000);
    d[35] = 0x40; d[36] = 0x40; d[37] = 0x2C;
    for (int i = 0; i < 40; i++) ad9361_reg_write(ad, 0x200 + i, (int)(d[i] & 0xFF));
}

static int bb_dc_offset_calib(ad9361_t *ad)
{
    ad9361_reg_write(ad, REG_BB_DC_OFFSET_COUNT, 0x3F);
    ad9361_reg_write(ad, REG_BB_DC_OFFSET_SHIFT, 0xF);
    ad9361_reg_write(ad, REG_BB_DC_OFFSET_ATTEN, 1);
    return run_calibration(ad, BBDC_CAL);
}

static int rf_dc_offset_calib(ad9361_t *ad, uint64_t rx_freq, int rx2_inv)
{
    ad9361_reg_write(ad, REG_WAIT_COUNT, 0x20);
    if (rx_freq <= 4000000000ULL) {
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_COUNT, 0x32);
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_CONFIG_1, 4 | (2 << 4));
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_ATTEN, 5);
    } else {
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_COUNT, 0x28);
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_CONFIG_1, 4 | (3 << 4));
        ad9361_reg_write(ad, REG_RF_DC_OFFSET_ATTEN, 6);
    }
    ad9361_reg_write(ad, REG_DC_OFFSET_CONFIG2, USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL | 3);
    int inv = INVERT_RX1_RF_DC_CGOUT_WORD; if (!rx2_inv) inv |= INVERT_RX2_RF_DC_CGOUT_WORD;
    ad9361_reg_write(ad, REG_INVERT_BITS, inv);
    return run_calibration(ad, RFDC_CAL);
}

static int tx_quad_once(ad9361_t *ad, int phase, int rxnco, int decim)   // -1 = calibration timed out (Python None)
{
    ad9361_reg_write(ad, REG_QUAD_CAL_NCO_FREQ_PHASE_OFFSET, ((rxnco & 0x3) << 5) | (phase & 0x1F));
    int base = (1 << 6) | (1 << 5) | (1 << 4) | (1 << 3) | (decim & 0x3);
    ad9361_reg_write(ad, REG_QUAD_CAL_CTRL, base | (1 << 2));
    ad9361_reg_write(ad, REG_QUAD_CAL_CTRL, base);
    if (!run_calibration(ad, TX_QUAD_CAL)) return -1;
    int r = ad9361_reg_read(ad, REG_QUAD_CAL_STATUS_TX1) & (TX1_LO_CONV | TX1_SSB_CONV);
    if (ad->rx2tx2) r &= ad9361_reg_read(ad, REG_QUAD_CAL_STATUS_TX2) & (TX1_LO_CONV | TX1_SSB_CONV);
    return r;
}

static int tx_quad_calib(ad9361_t *ad, uint64_t bw_rx, uint64_t bw_tx, int rx_phase, uint64_t clkrf, uint64_t clktf)
{
    if (ad->tx_quad_lpf_tia_match < 0) { fprintf(stderr, "ad9361: load the gain table first\n"); return -1; }
    int64_t txnco = (int64_t)drc(bw_tx * 8, clktf) - 1;
    if (txnco < 0) txnco = 0; if (txnco > 3) txnco = 3;
    int64_t rxnco = txnco;
    int decim = clktf <= 4000000 ? 2 : 3, ph = 0;
    if (clkrf / 2 == clktf) {
        ph = 0x0E;
        if (txnco == 0) txnco++;
        else if (txnco == 1) rxnco--;
        else if (txnco == 2) { rxnco -= 2; txnco--; }
        else if (txnco == 3) { rxnco -= 2; ph = 0x08; }
    } else if (clkrf == clktf) {
        if (txnco == 0 || txnco == 3) ph = 0x15;
        else if (txnco == 2) ph = 0x1F;
        else ph = ad9361_reg_readf(ad, REG_TX_ENABLE_FILTER_CTRL, 0x3F) == 0x22 ? 0x15 : 0x1A;
    } else if (clktf / 2 == clkrf) {
        ph = (-2) & 0xFF;
        if (txnco == 0) rxnco = 1; else { txnco = 1; rxnco = 3; }
    } else if (clktf / 4 == clkrf) { ph = (-2) & 0xFF; txnco = 0; rxnco = 3; }
    else if (clkrf / 4 == clktf) { ph = (-2) & 0xFF; txnco = 3; rxnco = 0; }
    else { fprintf(stderr, "ad9361: unhandled clkrf/clktf\n"); return -1; }
    if (rx_phase >= 0) ph = rx_phase;
    uint64_t txnco_freq = clktf * (uint64_t)(txnco + 1) / 32;
    if (txnco_freq > bw_rx / 4 || txnco_freq > bw_tx / 4) fprintf(stderr, "ad9361: bandwidth too narrow for TX quad cal\n");
    ad9361_reg_writef(ad, REG_KEXP_2, 0x3 << 6, (int)txnco);
    ad9361_reg_write(ad, REG_QUAD_CAL_COUNT, 0xFF);
    ad9361_reg_write(ad, REG_KEXP_1, (1 << 6) | (3 << 4) | (3 << 2) | 3);
    ad9361_reg_write(ad, REG_MAG_FTEST_THRESH, 0x03);
    ad9361_reg_write(ad, REG_MAG_FTEST_THRESH_2, 0x03);
    ad9361_reg_write(ad, REG_TX_QUAD_FULL_LMT_GAIN, ad->tx_quad_lpf_tia_match);
    ad9361_reg_write(ad, REG_QUAD_SETTLE_COUNT, 0xF0);
    ad9361_reg_write(ad, REG_TX_QUAD_LPF_GAIN, 0x00);
    const int want = TX1_LO_CONV | TX1_SSB_CONV;
    int val = 0;
    if (rx_phase != -2) {
        val = tx_quad_once(ad, ph, (int)rxnco, decim);
        if (val != want && ad->last_tx_quad_cal_phase < 31) val = tx_quad_once(ad, ad->last_tx_quad_cal_phase, (int)rxnco, decim);
        else if (val == want) ad->last_tx_quad_cal_phase = ph;
    }
    if (val != want) {
        int field[64];
        for (int i = 0; i < 32; i++) {
            int v = tx_quad_once(ad, i, (int)rxnco, decim); if (v < 0) v = 0;
            field[i] = field[i + 32] = !((v & TX1_LO_CONV) && (v & TX1_SSB_CONV));
        }
        int best_len = 0, best_start = 0, cnt = 0, start = -1;
        for (int i = 0; i < 64; i++) {
            if (field[i] == 0) { if (start == -1) start = i; cnt++; }
            else { if (cnt > best_len) { best_len = cnt; best_start = start; } start = -1; cnt = 0; }
        }
        if (cnt > best_len) { best_len = cnt; best_start = start; }
        ad->last_tx_quad_cal_phase = (best_start + best_len / 2) & 0x1F;
        val = tx_quad_once(ad, ad->last_tx_quad_cal_phase, (int)rxnco, decim);
    }
    return val;
}

static void tracking_control(ad9361_t *ad, int bbdc, int rfdc, int quad, int events)
{
    ad9361_reg_write(ad, REG_CALIBRATION_CONFIG_2, CALIBRATION_CONFIG2_DFLT | 0x15);
    ad9361_reg_write(ad, REG_CALIBRATION_CONFIG_3, PREVENT_POS_LOOP_GAIN | 0x15);
    ad9361_reg_write(ad, REG_DC_OFFSET_CONFIG2, USE_WAIT_COUNTER_FOR_RF_DC_INIT_CAL | (events & 0x7) |
                                                    (bbdc ? ENABLE_BB_DC_OFFSET_TRACKING : 0) | (rfdc ? ENABLE_RF_OFFSET_TRACKING : 0));
    ad9361_reg_writef(ad, REG_RX_QUAD_GAIN2, 0x7 << 5, 0);
    ad9361_reg_write(ad, REG_CALIBRATION_CONFIG_1, ENABLE_PHASE_CORR | ENABLE_GAIN_CORR | FREE_RUN_MODE |
                                                       ENABLE_CORR_WORD_DECIMATION | (quad ? ENABLE_TRACKING_MODE_CH1 : 0));
}

ad9361_init_res_t ad9361_analog_init(ad9361_t *ad, uint64_t rf_rx_bw, uint64_t rf_tx_bw, uint64_t bbpll, uint64_t adc_clk,
                                     uint64_t rx_freq, int rx_input, uint64_t clkrf, uint64_t clktf, int tracking, int rx2tx2)
{
    ad9361_init_res_t r;
    uint64_t rx_bb = rf_rx_bw / 2, tx_bb = rf_tx_bw / 2;
    ad->rx2tx2 = rx2tx2 ? 1 : 0;
    rf_port_setup(ad, rx_input, 0);
    load_mixer_gm_subtable(ad);
    gain_control_manual(ad);
    load_rx_gain_table(ad, 1 | 2, rx_freq);
    r.rx_bb_filter = rx_bb_analog_filter_calib(ad, rx_bb, bbpll);
    r.tx_bb_filter = tx_bb_analog_filter_calib(ad, tx_bb, bbpll);
    rx_tia_calib(ad, rx_bb);
    tx_bb_second_filter_calib(ad, tx_bb);
    rx_adc_setup(ad, bbpll, adc_clk);
    r.bb_dc = bb_dc_offset_calib(ad);
    r.rf_dc = rf_dc_offset_calib(ad, rx_freq, 0);
    r.tx_quad = tx_quad_calib(ad, rx_bb, tx_bb, -1, clkrf, clktf);
    const char *e = getenv("P501_TRACKING");
    if (tracking || (e && !strcmp(e, "1"))) tracking_control(ad, 1, 1, 1, 5);
    return r;
}

void ad9361_enable_fdd(ad9361_t *ad)
{
    ad9361_reg_write(ad, REG_ENSM_MODE, FDD_MODE);
    ad9361_reg_write(ad, REG_ENSM_CONFIG_1, FORCE_RX_ON | FORCE_TX_ON);
    usleep(10000);
}

void ad9361_enable_rx2tx2(ad9361_t *ad)
{
    ad9361_reg_writef(ad, REG_RX_ENABLE_FILTER_CTRL, 0x3 << 6, 1 | 2);
    ad9361_reg_writef(ad, REG_TX_ENABLE_FILTER_CTRL, 0x3 << 6, 1 | 2);
    ad->rx2tx2 = 1;
}

void ad9361_set_rx_manual_gain(ad9361_t *ad, int g, int ch)
{
    if (ch & 1) ad9361_reg_write(ad, REG_RX1_MANUAL_LMT_FULL_GAIN, g);
    if (ch & 2) ad9361_reg_write(ad, REG_RX2_MANUAL_LMT_FULL_GAIN, g);
}

void ad9361_set_tx_atten_db(ad9361_t *ad, double atten_db, int ch)
{
    int code = (int)(atten_db * 4) & 0x3FF;
    ad9361_reg_writef(ad, REG_TX_ATTEN_OFFSET, 1 << 6, 0);
    ad9361_reg_writef(ad, REG_TX2_DIG_ATTEN, 1 << 6, 0);
    if (ch & 1) { ad9361_reg_write(ad, REG_TX1_ATTEN_0, code & 0xFF); ad9361_reg_writef(ad, REG_TX1_ATTEN_1, 0x3, (code >> 8) & 0x3); }
    if (ch & 2) { ad9361_reg_write(ad, REG_TX2_ATTEN_0, code & 0xFF); ad9361_reg_writef(ad, REG_TX2_ATTEN_1, 0x3, (code >> 8) & 0x3); }
    ad9361_reg_writef(ad, REG_TX2_DIG_ATTEN, 1 << 6, 1);
}
