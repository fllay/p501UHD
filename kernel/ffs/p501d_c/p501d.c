#define _GNU_SOURCE
// p501d: P501 board daemon in C (replaces tools/p501d.py; same TCP protocol, same streamer, no Python/PYNQ).
// Loads the bitstream through the kernel FPGA manager, allocates the RX/TX DMA rings from the CMA heap, brings up the
// AD9361 in 2R2T at the selected rate, starts p501_stream and serves the line-based control protocol on :5555:
//   info | get_rx_freq|get_tx_freq c | set_rx_freq|set_tx_freq c Hz | get/set_rx_gain, get/set_tx_gain c dB |
//   get_rx_gain_range|get_tx_gain_range c | lo_locked | loopback 0|1 | agc [0|1] | get_rate | set_rate Hz | quit
// Environment: P501_BIT (bitstream; its .hwh next to it gives the IP addresses), P501_PORT, P501_RATE, P501_TIME_PHYS,
// P501_AGC/_TARGET/_MIN/_MAX/_UL_SLOTS, P501_TRACKING, P501_SPI_TRACE, P501_STREAMER (path of p501_stream).
// Usage: p501d [--bringup-only LO_Hz]   (--bringup-only: load, bring up once, print the result and exit; for tests)
#include "ad9361.h"
#include <arpa/inet.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <math.h>
#include <netinet/in.h>
#include <pthread.h>
#include <signal.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define VERSION "3c"
#define SLOT_BYTES (4u * 1024 * 1024)
#define NSLOTS 16
#define TX_NSLOTS 8
#define RX_GAIN_MAX 76.0
#define TX_GAIN_MAX 89.75
#define LO_MIN 70e6
#define LO_MAX 6000e6
#define RX_GAIN_INDEX 40      // bring-up defaults (ad9361_mimo_test), apply() sets the real values afterwards
#define TX_ATTEN_DB 30
#define DATARATE 3            // DAC DATARATE = l_clk / fs - 1 (2R2T: l_clk = 4 x fs)

static void logf_(const char *fmt, ...)
{
    va_list ap; va_start(ap, fmt); vprintf(fmt, ap); va_end(ap); fflush(stdout);
}
static double now_s(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
static const char *env(const char *k, const char *d) { const char *v = getenv(k); return v && *v ? v : d; }

// ---------------------------------------------------------------- rates
static const struct { int fs, bw; } RATES[] = {
    {7680000, 5000000}, {11520000, 7500000}, {15360000, 10000000}, {23040000, 18000000}, {30720000, 18000000}};
#define NRATES (int)(sizeof RATES / sizeof RATES[0])

static int rate_bw(int fs) { for (int i = 0; i < NRATES; i++) if (RATES[i].fs == fs) return RATES[i].bw; return 0; }
static int parse_rate(const char *s)                          // Hz or MHz; 0 if unsupported
{
    double v = atof(s); int fs = (int)llround(v < 1000 ? v * 1e6 : v);
    return rate_bw(fs) ? fs : 0;
}

// ---------------------------------------------------------------- bitstream + hwh
static int load_bitstream(const char *bit)
{
    FILE *f = fopen(bit, "rb"); if (!f) { perror(bit); return -1; }
    fseek(f, 0, SEEK_END); long n = ftell(f); fseek(f, 0, SEEK_SET);
    uint8_t *b = malloc(n); if (fread(b, 1, n, f) != (size_t)n) { fclose(f); free(b); return -1; } fclose(f);
    // .bit header: <len16><data> <len16=1>, then fields 'a'..'d' (key, len16, data) and 'e' (key, len32, bitstream)
    long o = 0; int l = (b[o] << 8) | b[o + 1]; o += 2 + l; o += 2;
    uint8_t *data = NULL; uint32_t dlen = 0;
    while (o < n) {
        uint8_t key = b[o++];
        if (key == 'e') { dlen = ((uint32_t)b[o] << 24) | (b[o + 1] << 16) | (b[o + 2] << 8) | b[o + 3]; o += 4; data = b + o; break; }
        l = (b[o] << 8) | b[o + 1]; o += 2 + l;
    }
    if (!data || o + dlen > (uint32_t)n) { fprintf(stderr, "%s: not a .bit file\n", bit); free(b); return -1; }
    for (uint32_t i = 0; i + 3 < dlen; i += 4) { uint8_t t0 = data[i], t1 = data[i + 1]; data[i] = data[i + 3]; data[i + 1] = data[i + 2]; data[i + 2] = t1; data[i + 3] = t0; }
    FILE *w = fopen("/lib/firmware/p501d.bin", "wb");
    if (!w || fwrite(data, 1, dlen, w) != dlen) { perror("/lib/firmware/p501d.bin"); if (w) fclose(w); free(b); return -1; }
    fclose(w); free(b);
    FILE *fl = fopen("/sys/class/fpga_manager/fpga0/flags", "w"); if (fl) { fputs("0", fl); fclose(fl); }
    FILE *fw = fopen("/sys/class/fpga_manager/fpga0/firmware", "w");
    if (!fw) { perror("fpga_manager"); return -1; }
    int bad = fputs("p501d.bin", fw) < 0; bad |= fclose(fw) != 0;
    char st[64] = ""; FILE *s = fopen("/sys/class/fpga_manager/fpga0/state", "r"); if (s) { if (!fgets(st, sizeof st, s)) st[0] = 0; fclose(s); }
    if (bad || strncmp(st, "operating", 9)) { fprintf(stderr, "bitstream load failed (state %s)\n", st); return -1; }
    return 0;
}

static uint64_t hwh_base(const char *hwh_text, const char *inst)   // BASEVALUE of the MEMRANGE of an instance, 0 = none
{
    char want[128]; snprintf(want, sizeof want, "INSTANCE=\"%s\"", inst);
    for (const char *p = hwh_text; (p = strstr(p, "<MEMRANGE ")); p++) {
        const char *e = strchr(p, '>'); if (!e) break;
        const char *i = strstr(p, want), *bv = strstr(p, "BASEVALUE=\"");
        if (i && i < e && bv && bv < e) return strtoull(bv + 11, 0, 0);
    }
    return 0;
}

// ---------------------------------------------------------------- DMA rings (CMA heap) + MMIO
typedef struct { int fd; void *va; size_t len; uint64_t phys; } dmabuf_t;

static int dmabuf_alloc(dmabuf_t *d, size_t len)
{
    int h = open("/dev/dma_heap/linux,cma", O_RDWR | O_CLOEXEC); if (h < 0) { perror("dma_heap"); return -1; }
    struct dma_heap_allocation_data a = {.len = len, .fd_flags = O_RDWR | O_CLOEXEC};
    if (ioctl(h, DMA_HEAP_IOCTL_ALLOC, &a) < 0) { perror("DMA_HEAP_IOCTL_ALLOC"); close(h); return -1; }
    close(h);
    d->fd = a.fd; d->len = len;
    d->va = mmap(0, len, PROT_READ | PROT_WRITE, MAP_SHARED, d->fd, 0);
    if (d->va == MAP_FAILED) { perror("mmap dmabuf"); return -1; }
    struct dma_buf_sync sy = {.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_WRITE}; ioctl(d->fd, DMA_BUF_IOCTL_SYNC, &sy);
    memset(d->va, 0, len);                                    // faults the pages in (needed for pagemap) and clears TX
    sy.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_WRITE; ioctl(d->fd, DMA_BUF_IOCTL_SYNC, &sy);
    // physical address from /proc/self/pagemap (root); the CMA heap gives contiguous pages, checked here
    int pm = open("/proc/self/pagemap", O_RDONLY); if (pm < 0) { perror("pagemap"); return -1; }
    const long pg = sysconf(_SC_PAGESIZE); const size_t np = len / pg; uint64_t first = 0;
    for (size_t i = 0; i < np; i++) {
        uint64_t e; off_t off = (off_t)(((uintptr_t)d->va / pg + i) * 8);
        if (pread(pm, &e, 8, off) != 8 || !(e >> 63)) { fprintf(stderr, "pagemap: page %zu not present\n", i); close(pm); return -1; }
        uint64_t pfn = e & ((1ULL << 55) - 1);
        if (!pfn) { fprintf(stderr, "pagemap: no PFN (not root?)\n"); close(pm); return -1; }
        if (i == 0) first = pfn; else if (pfn != first + i) { fprintf(stderr, "dmabuf not contiguous at page %zu\n", i); close(pm); return -1; }
    }
    close(pm);
    d->phys = first * (uint64_t)pg;
    return 0;
}

static volatile uint32_t *map_mmio(uint64_t phys, size_t len)
{
    int fd = open("/dev/mem", O_RDWR | O_SYNC); if (fd < 0) { perror("/dev/mem"); return NULL; }
    void *p = mmap(0, len, PROT_READ | PROT_WRITE, MAP_SHARED, fd, phys); close(fd);
    return p == MAP_FAILED ? NULL : (volatile uint32_t *)p;
}

// axi_ad9361 core: reset, 2R2T (R1_MODE off) on ADC and DAC, 4 ADC channels with sign extension, DAC from DMA
static void axi_ad9361_core_enable(volatile uint32_t *ip, int nch, int datarate, int r1)
{
    ip[0x040 / 4] = 0; ip[0x040 / 4] = 3;
    ip[0x4040 / 4] = 0; ip[0x4040 / 4] = 3;
    usleep(100000);
    ip[0x044 / 4] = r1 ? (1 << 2) : 0;
    ip[0x4048 / 4] = r1 ? (1 << 5) : 0;
    for (int c = 0; c < nch; c++) ip[(0x400 + c * 0x40) / 4] = (1 << 6) | (1 << 4) | 1;
    for (int c = 0; c < nch; c++) ip[(0x4418 + c * 0x40) / 4] = 2;
    ip[0x404C / 4] = datarate;
}

// ---------------------------------------------------------------- board state
static struct {
    pthread_mutex_t lock;
    ad9361_t ad; int have_ad;
    volatile uint32_t *axi;
    double rx_gain[2], tx_gain[2];
    int64_t lo; int locked, loop, fs;
    uint64_t time_phys;
    // AGC
    int agc_on; double agc_target, agc_min, agc_max, agc_level, agc_floor, agc_floor_rel; int agc_have_level, agc_have_floor;
    int agc_ul[16], agc_nul;
} B = {.lock = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP};

static int bring_up(int64_t lo, int fs, int *rx_ok, int *tx_ok)
{
    const uint64_t adc_clk = 4ULL * fs; int div = 0;
    for (int d = 2; d <= 128; d <<= 1) if (adc_clk * d >= 715000000ULL && adc_clk * d <= 1430000000ULL) { div = d; break; }
    const uint64_t bbpll = adc_clk * div; const int bw = rate_bw(fs);
    ad9361_chain_t ch = {.adc_div = div, .r2_div = 2, .r1_div = 2, .clkrf_div = 1, .rx_fir_div = 1,
                         .dac_div = 2, .t2_div = 2, .t1_div = 1, .clktf_div = 1, .tx_fir_div = 1};
    ad9361_t *ad = &B.ad;
    if (ad9361_open(ad, "/dev/spidev1.0", 40000000ULL)) return -1;
    B.have_ad = 1;
    ad9361_power_up_clocks(ad);
    if (!ad9361_set_bbpll_rate(ad, bbpll)) logf_("BBPLL did not lock\n");
    ad9361_set_clock_chain(ad, &ch);
    ad9361_synth_cp_calib(ad, 0);
    ad9361_synth_cp_calib(ad, 1);
    *rx_ok = ad9361_set_lo_freq(ad, 0, (uint64_t)lo);
    *tx_ok = ad9361_set_lo_freq(ad, 1, (uint64_t)lo);
    ad9361_enable_rx2tx2(ad);
    axi_ad9361_core_enable(B.axi, 4, DATARATE, 0);
    ad9361_reg_write(ad, 0x010, 0x80 | 0x40 | 0x08 | (1 << 2));    // PARALLEL_PORT_CONF_1: swap IQ, frame pulse, R2T2 timing
    ad9361_reg_write(ad, 0x011, 0x00);
    ad9361_reg_write(ad, 0x012, 0x10);                             // LVDS
    ad9361_reg_write(ad, 0x006, 4);                                // RX clock/data delay
    ad9361_reg_write(ad, 0x007, 7 << 4);                           // TX clock/data delay
    ad9361_reg_write(ad, 0x03C, ((150 - 75) / 75) | (1 << 5));    // LVDS bias
    ad9361_init_res_t r = ad9361_analog_init(ad, bw, bw, bbpll, adc_clk, (uint64_t)lo, 0, fs, fs, 0, 1);
    ad9361_set_rx_manual_gain(ad, RX_GAIN_INDEX, 3);
    ad9361_set_tx_atten_db(ad, TX_ATTEN_DB, 3);
    ad9361_enable_fdd(ad);
    axi_ad9361_core_enable(B.axi, 4, DATARATE, 0);
    FILE *f = fopen("/tmp/p501_fs", "w"); if (f) { fprintf(f, "%d\n", fs); fclose(f); }   // the streamer scales its limits by this
    logf_("bring-up %.3f MHz %.2f MS/s: rx_bb %d tx_bb %d bb_dc %d rf_dc %d tx_quad %d\n", lo / 1e6, fs / 1e6,
          r.rx_bb_filter, r.tx_bb_filter, r.bb_dc, r.rf_dc, r.tx_quad);
    return 0;
}

static void apply(void)
{
    for (int i = 0; i < 2; i++) {
        ad9361_set_rx_manual_gain(&B.ad, (int)llround(B.rx_gain[i]), 1 << i);
        ad9361_set_tx_atten_db(&B.ad, TX_GAIN_MAX - B.tx_gain[i], 1 << i);
    }
    ad9361_reg_write(&B.ad, 0x3F4, 0);                             // no BIST tone
    int r = ad9361_reg_read(&B.ad, 0x3F5);
    ad9361_reg_write(&B.ad, 0x3F5, B.loop ? (r | 1) : (r & ~1));   // digital loopback
}

static void retune(int64_t f)
{
    pthread_mutex_lock(&B.lock);
    if (f != B.lo) {
        if (B.have_ad) ad9361_close(&B.ad);
        int rx_ok = 0, tx_ok = 0;
        if (bring_up(f, B.fs, &rx_ok, &tx_ok) == 0) {
            B.lo = f; B.locked = rx_ok && tx_ok;
            apply();
            logf_("retuned to %.3f MHz at %.2f MS/s locked=%s\n", f / 1e6, B.fs / 1e6, B.locked ? "True" : "False");
        }
    }
    pthread_mutex_unlock(&B.lock);
}

static void set_gain_all(double g)
{
    pthread_mutex_lock(&B.lock);
    g = fmin(RX_GAIN_MAX, fmax(0.0, round(g)));
    B.rx_gain[0] = B.rx_gain[1] = g;
    ad9361_set_rx_manual_gain(&B.ad, (int)g, 3);
    pthread_mutex_unlock(&B.lock);
}

// ---------------------------------------------------------------- AGC (see tools/p501d.py class Agc)
typedef struct { unsigned long seq; double lvl; long clip; } lvl_t;

static int read_lvl(lvl_t *o)
{
    char buf[1024]; FILE *f = fopen("/tmp/p501_rxlvl", "r"); if (!f) return 0;
    size_t n = fread(buf, 1, sizeof buf - 1, f); fclose(f); buf[n] = 0;
    // "<seq> <n> <dBFS>... | dc <dBFS> | <clips>... | <samples>"
    char *p = buf, *end; unsigned long seq = strtoul(p, &end, 10); if (end == p) return 0; p = end;
    int ns = (int)strtol(p, &end, 10); if (end == p || ns < 1 || ns > 40) return 0; p = end;
    double lv[40]; for (int k = 0; k < ns; k++) { lv[k] = strtod(p, &end); if (end == p) return 0; p = end; }
    char *b1 = strchr(p, '|'); if (!b1) return 0; char *b2 = strchr(b1 + 1, '|'); if (!b2) return 0; p = b2 + 1;
    long cl[40]; for (int k = 0; k < ns; k++) { cl[k] = strtol(p, &end, 10); if (end == p) return 0; p = end; }
    double mx = -1e9; long cs = 0;
    for (int i = 0; i < B.agc_nul; i++) { int k = B.agc_ul[i]; if (k < ns) { if (lv[k] > mx) mx = lv[k]; cs += cl[k]; } }
    o->seq = seq; o->lvl = mx; o->clip = cs; return 1;
}

static void *agc_thread(void *a)
{
    (void)a;
    lvl_t hist[50]; int nh = 0; unsigned long seq = (unsigned long)-1; double t_dec = 0, last_up = 0;
    for (;;) {
        usleep(200000);
        lvl_t r; if (!read_lvl(&r) || r.seq == seq) continue;
        seq = r.seq;
        if (nh == 50) { memmove(hist, hist + 1, 49 * sizeof *hist); nh = 49; }
        hist[nh++] = r;
        double rel = r.lvl - B.rx_gain[0];
        B.agc_floor_rel = B.agc_have_floor ? fmin(rel, B.agc_floor_rel + 0.02) : rel; B.agc_have_floor = 1;
        B.agc_floor = B.agc_floor_rel + B.rx_gain[0];
        double t = now_s();
        if (t - t_dec < 1.0 || nh < 5) continue;
        t_dec = t;
        double lvl = -1e9; long clips = 0;
        for (int i = nh - 5; i < nh; i++) { if (hist[i].lvl > lvl) lvl = hist[i].lvl; clips += hist[i].clip; }
        B.agc_level = lvl; B.agc_have_level = 1;
        if (!B.agc_on) continue;
        double g = B.rx_gain[0], ng;
        if (clips > 50 || lvl > B.agc_target + 4) ng = fmax(B.agc_min, g - 3);
        else if (lvl < B.agc_target - 5 && lvl > B.agc_floor + 8 && t - last_up > 2.0) { ng = fmin(B.agc_max, g + (lvl < B.agc_target - 10 ? 2 : 1)); last_up = t; }
        else continue;
        if (ng != g) {
            set_gain_all(ng); nh = 0;
            logf_("AGC rx_gain %.0f -> %.0f (UL %.1f dBFS, clipped %ld, floor %.1f)\n", g, ng, lvl, clips, B.agc_floor);
        }
    }
    return NULL;
}

// ---------------------------------------------------------------- control protocol
static void fmt_d(char *o, size_t n, double v) { if (v == floor(v)) snprintf(o, n, "%.1f", v); else snprintf(o, n, "%g", v); }  // Python float str

static void handle(char *line, char *out, size_t on)
{
    char *tok[8]; int nt = 0;
    for (char *s = strtok(line, " \t\r\n"); s && nt < 8; s = strtok(NULL, " \t\r\n")) tok[nt++] = s;
    if (!nt) { snprintf(out, on, "ERR empty"); return; }
    const char *c = tok[0];
    int ch = 0; if (nt > 1) { char *e; long v = strtol(tok[1], &e, 10); if (*e == 0) ch = (int)v; }
    if (ch < 0 || ch > 1) ch = 0;
    char v[32];
    if (!strcmp(c, "info")) {
        char host[64] = "p501"; gethostname(host, sizeof host);
        char rates[128] = ""; for (int i = 0; i < NRATES; i++) snprintf(rates + strlen(rates), sizeof rates - strlen(rates), "%s%d", i ? "," : "", RATES[i].fs);
        snprintf(out, on, "OK p501 %s fs=%d rx=2 tx=2 time=%d serial=P501-%s rates=%s", VERSION, B.fs, B.time_phys ? 1 : 0, host, rates);
    } else if (!strcmp(c, "get_rx_freq") || !strcmp(c, "get_tx_freq")) snprintf(out, on, "OK %lld", (long long)B.lo);
    else if (!strcmp(c, "set_rx_freq") || !strcmp(c, "set_tx_freq")) {
        if (nt < 3) { snprintf(out, on, "ERR usage"); return; }
        double f = atof(tok[2]);
        if (f < LO_MIN || f > LO_MAX) { snprintf(out, on, "ERR frequency %g outside %g..%g", f, LO_MIN, LO_MAX); return; }
        retune(llround(f)); snprintf(out, on, "OK %lld", (long long)B.lo);
    } else if (!strcmp(c, "get_rx_gain")) { fmt_d(v, sizeof v, B.rx_gain[ch]); snprintf(out, on, "OK %s", v); }
    else if (!strcmp(c, "get_tx_gain")) { fmt_d(v, sizeof v, B.tx_gain[ch]); snprintf(out, on, "OK %s", v); }
    else if (!strcmp(c, "set_rx_gain")) {
        if (nt < 3) { snprintf(out, on, "ERR usage"); return; }
        pthread_mutex_lock(&B.lock);
        B.rx_gain[ch] = fmin(RX_GAIN_MAX, fmax(0.0, round(atof(tok[2]))));
        ad9361_set_rx_manual_gain(&B.ad, (int)B.rx_gain[ch], 1 << ch);
        pthread_mutex_unlock(&B.lock);
        fmt_d(v, sizeof v, B.rx_gain[ch]); snprintf(out, on, "OK %s", v);
    } else if (!strcmp(c, "set_tx_gain")) {
        if (nt < 3) { snprintf(out, on, "ERR usage"); return; }
        pthread_mutex_lock(&B.lock);
        double g = fmin(TX_GAIN_MAX, fmax(0.0, atof(tok[2])));
        B.tx_gain[ch] = round(g * 4) / 4;
        ad9361_set_tx_atten_db(&B.ad, TX_GAIN_MAX - B.tx_gain[ch], 1 << ch);
        pthread_mutex_unlock(&B.lock);
        fmt_d(v, sizeof v, B.tx_gain[ch]); snprintf(out, on, "OK %s", v);
    } else if (!strcmp(c, "get_rx_gain_range")) snprintf(out, on, "OK 0 %.1f 1", RX_GAIN_MAX);
    else if (!strcmp(c, "get_tx_gain_range")) snprintf(out, on, "OK 0 %g 0.25", TX_GAIN_MAX);
    else if (!strcmp(c, "lo_locked")) snprintf(out, on, "OK %d", B.locked);
    else if (!strcmp(c, "loopback")) {
        pthread_mutex_lock(&B.lock); B.loop = nt > 1 && !strcmp(tok[1], "1"); apply(); pthread_mutex_unlock(&B.lock);
        snprintf(out, on, "OK %d", B.loop);
    } else if (!strcmp(c, "agc")) {
        if (nt > 1) B.agc_on = !strcmp(tok[1], "1");
        fmt_d(v, sizeof v, B.rx_gain[0]);
        if (B.agc_have_level) snprintf(out, on, "OK %d %s %.1f", B.agc_on, v, B.agc_level); else snprintf(out, on, "OK %d %s nan", B.agc_on, v);
    } else if (!strcmp(c, "get_rate")) snprintf(out, on, "OK %d", B.fs);
    else if (!strcmp(c, "set_rate")) {
        if (nt < 2) { snprintf(out, on, "ERR usage"); return; }
        int fs = parse_rate(tok[1]);
        if (!fs) { snprintf(out, on, "ERR ValueError: unsupported sample rate %s", tok[1]); return; }
        pthread_mutex_lock(&B.lock);
        if (fs != B.fs) { int64_t lo = B.lo; B.fs = fs; B.lo = -1; retune(lo); }
        pthread_mutex_unlock(&B.lock);
        snprintf(out, on, "OK %d", B.fs);
    } else snprintf(out, on, "ERR unknown command %s", c);
}

static void *client(void *a)
{
    int fd = (int)(intptr_t)a; FILE *in = fdopen(fd, "r"); char line[512], out[512];
    while (fgets(line, sizeof line, in)) {
        char *s = line; while (*s == ' ' || *s == '\t') s++;
        if (!strncmp(s, "quit", 4) && (s[4] == '\n' || s[4] == '\r' || s[4] == 0)) break;
        handle(s, out, sizeof out);
        size_t n = strlen(out); out[n++] = '\n';
        if (send(fd, out, n, MSG_NOSIGNAL) != (ssize_t)n) break;
    }
    fclose(in); return NULL;
}

static void serve(int port)
{
    int s = socket(AF_INET, SOCK_STREAM, 0), one = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
    struct sockaddr_in a = {.sin_family = AF_INET, .sin_port = htons(port), .sin_addr.s_addr = INADDR_ANY};
    if (bind(s, (struct sockaddr *)&a, sizeof a) || listen(s, 4)) { perror("bind"); exit(1); }
    logf_("control on :%d\n", port);
    for (;;) {
        int c = accept(s, 0, 0); if (c < 0) { if (errno == EINTR) continue; perror("accept"); continue; }
        setsockopt(c, IPPROTO_TCP, 1 /* TCP_NODELAY */, &one, sizeof one);
        pthread_t t; pthread_create(&t, 0, client, (void *)(intptr_t)c); pthread_detach(t);
    }
}

// ---------------------------------------------------------------- main
int main(int argc, char **argv)
{
    signal(SIGPIPE, SIG_IGN);
    const char *bit = env("P501_BIT", "/home/xilinx/p501UHD/base_ts/system_top.bit");
    B.fs = parse_rate(env("P501_RATE", "30.72"));
    if (!B.fs) { fprintf(stderr, "unsupported P501_RATE\n"); return 1; }
    B.rx_gain[0] = B.rx_gain[1] = 40.0; B.tx_gain[0] = B.tx_gain[1] = TX_GAIN_MAX - 30.0; B.lo = -1;
    B.agc_on = !strcmp(env("P501_AGC", "0"), "1");
    B.agc_target = atof(env("P501_AGC_TARGET", "-17")); B.agc_min = atof(env("P501_AGC_MIN", "20")); B.agc_max = atof(env("P501_AGC_MAX", "65"));
    { char tmp[64]; snprintf(tmp, sizeof tmp, "%s", env("P501_AGC_UL_SLOTS", "7,8,9")); for (char *s = strtok(tmp, ","); s && B.agc_nul < 16; s = strtok(NULL, ",")) B.agc_ul[B.agc_nul++] = atoi(s); }

    // bitstream and addresses
    if (load_bitstream(bit)) return 1;
    char hwh[512]; snprintf(hwh, sizeof hwh, "%.*s.hwh", (int)(strrchr(bit, '.') ? strrchr(bit, '.') - bit : (long)strlen(bit)), bit);
    FILE *hf = fopen(hwh, "r"); if (!hf) { perror(hwh); return 1; }
    fseek(hf, 0, SEEK_END); long hn = ftell(hf); fseek(hf, 0, SEEK_SET);
    char *ht = malloc(hn + 1); if (fread(ht, 1, hn, hf) != (size_t)hn) { fclose(hf); return 1; } ht[hn] = 0; fclose(hf);
    uint64_t a_ad = hwh_base(ht, "axi_ad9361"), a_adc = hwh_base(ht, "axi_ad9361_adc_dma"), a_dac = hwh_base(ht, "axi_ad9361_dac_dma");
    free(ht);
    if (!a_ad || !a_adc || !a_dac) { fprintf(stderr, "%s: axi_ad9361 / DMA addresses not found\n", hwh); return 1; }
    B.time_phys = strstr(bit, "base_ts") ? strtoull(env("P501_TIME_PHYS", "0x99030000"), 0, 0) : 0;
    B.axi = map_mmio(a_ad, 0x10000); if (!B.axi) return 1;
    logf_("bitstream %s: axi_ad9361 0x%llx adc_dma 0x%llx dac_dma 0x%llx time 0x%llx\n", bit,
          (unsigned long long)a_ad, (unsigned long long)a_adc, (unsigned long long)a_dac, (unsigned long long)B.time_phys);

    if (argc > 2 && !strcmp(argv[1], "--bringup-only")) {
        retune(llround(atof(argv[2])));
        logf_("product id 0x%02x locked %d\n", ad9361_product_id(&B.ad), B.locked);
        return B.locked ? 0 : 2;
    }

    retune(700000000);
    dmabuf_t rx, tx;
    if (dmabuf_alloc(&rx, (size_t)NSLOTS * SLOT_BYTES) || dmabuf_alloc(&tx, (size_t)TX_NSLOTS * SLOT_BYTES)) return 1;
    logf_("rings: rx 0x%llx (%u MiB) tx 0x%llx (%u MiB)\n", (unsigned long long)rx.phys, NSLOTS * 4, (unsigned long long)tx.phys, TX_NSLOTS * 4);

    // streamer (same arguments as the Python daemon)
    char a1[32], a2[16], a3[16], a4[32], a6[32], a7[32], a8[32], a9[16], a10[32];
    snprintf(a1, sizeof a1, "0x%llx", (unsigned long long)rx.phys); snprintf(a2, sizeof a2, "%u", SLOT_BYTES);
    snprintf(a3, sizeof a3, "%d", NSLOTS); snprintf(a4, sizeof a4, "0x%llx", (unsigned long long)a_adc);
    snprintf(a6, sizeof a6, "0x%llx", (unsigned long long)a_ad); snprintf(a7, sizeof a7, "0x%llx", (unsigned long long)tx.phys);
    snprintf(a8, sizeof a8, "0x%llx", (unsigned long long)a_dac); snprintf(a9, sizeof a9, "%d", TX_NSLOTS);
    snprintf(a10, sizeof a10, "0x%llx", (unsigned long long)B.time_phys);
    const char *streamer = env("P501_STREAMER", "/home/xilinx/p501UHD/kernel/ffs/p501_stream");
    char *sargv[] = {(char *)streamer, "/dev/ffs", a1, a2, a3, a4, "0", a6, a7, a8, a9, B.time_phys ? a10 : NULL, NULL};
    pid_t pid = fork();
    if (pid == 0) { execv(streamer, sargv); perror("exec p501_stream"); _exit(127); }
    logf_("streamer pid %d\n", (int)pid);

    pthread_t at; pthread_create(&at, 0, agc_thread, 0); pthread_detach(at);
    serve(atoi(env("P501_PORT", "5555")));
    return 0;
}
