// Board side: FunctionFS bulk source (EP IN) + sink (EP OUT) for USB3 throughput tests.
#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <endian.h>
#include <linux/usb/ch9.h>
#include <linux/usb/functionfs.h>
#include <sys/mman.h>
#include <sys/uio.h>
#include <linux/aio_abi.h>
#include <sys/syscall.h>

#define PAYLOAD (512 * 1024)
#define RXP (256 * 1024)   /* RX message size (1.07 ms) */
static int rx_fb = 8, tx_fb = 8; static size_t rxp = 256 * 1024, txp = 512 * 1024;   /* frame bytes (4 per enabled channel pair) and message sizes, set per stream from the channel mask */
static double run_secs_g;
static char dir[256];

#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)

struct ep_desc { struct usb_endpoint_descriptor_no_audio d; } __attribute__((packed));

static const struct {
	struct usb_functionfs_descs_head_v2 header;
	__le32 fs_count, hs_count, ss_count;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio source, sink;
	} __attribute__((packed)) fs_descs, hs_descs;
	struct {
		struct usb_interface_descriptor intf;
		struct usb_endpoint_descriptor_no_audio source;
		struct usb_ss_ep_comp_descriptor source_comp;
		struct usb_endpoint_descriptor_no_audio sink;
		struct usb_ss_ep_comp_descriptor sink_comp;
	} __attribute__((packed)) ss_descs;
} __attribute__((packed)) descriptors = {
	.header = {
		.magic = cpu_to_le32(FUNCTIONFS_DESCRIPTORS_MAGIC_V2),
		.length = cpu_to_le32(sizeof descriptors),
		.flags = cpu_to_le32(FUNCTIONFS_HAS_FS_DESC | FUNCTIONFS_HAS_HS_DESC | FUNCTIONFS_HAS_SS_DESC),
	},
	.fs_count = cpu_to_le32(3), .hs_count = cpu_to_le32(3), .ss_count = cpu_to_le32(5),
	.fs_descs = {
		.intf = { .bLength = sizeof(descriptors.fs_descs.intf), .bDescriptorType = USB_DT_INTERFACE,
			  .bNumEndpoints = 2, .bInterfaceClass = USB_CLASS_VENDOR_SPEC, .iInterface = 1 },
		.source = { .bLength = sizeof(descriptors.fs_descs.source), .bDescriptorType = USB_DT_ENDPOINT,
			    .bEndpointAddress = 1 | USB_DIR_IN, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(64) },
		.sink = { .bLength = sizeof(descriptors.fs_descs.sink), .bDescriptorType = USB_DT_ENDPOINT,
			  .bEndpointAddress = 2 | USB_DIR_OUT, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(64) },
	},
	.hs_descs = {
		.intf = { .bLength = sizeof(descriptors.hs_descs.intf), .bDescriptorType = USB_DT_INTERFACE,
			  .bNumEndpoints = 2, .bInterfaceClass = USB_CLASS_VENDOR_SPEC, .iInterface = 1 },
		.source = { .bLength = sizeof(descriptors.hs_descs.source), .bDescriptorType = USB_DT_ENDPOINT,
			    .bEndpointAddress = 1 | USB_DIR_IN, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(512) },
		.sink = { .bLength = sizeof(descriptors.hs_descs.sink), .bDescriptorType = USB_DT_ENDPOINT,
			  .bEndpointAddress = 2 | USB_DIR_OUT, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(512) },
	},
	.ss_descs = {
		.intf = { .bLength = sizeof(descriptors.ss_descs.intf), .bDescriptorType = USB_DT_INTERFACE,
			  .bNumEndpoints = 2, .bInterfaceClass = USB_CLASS_VENDOR_SPEC, .iInterface = 1 },
		.source = { .bLength = sizeof(descriptors.ss_descs.source), .bDescriptorType = USB_DT_ENDPOINT,
			    .bEndpointAddress = 1 | USB_DIR_IN, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(1024) },
		.source_comp = { .bLength = sizeof(descriptors.ss_descs.source_comp), .bDescriptorType = USB_DT_SS_ENDPOINT_COMP, .bMaxBurst = 15 },
		.sink = { .bLength = sizeof(descriptors.ss_descs.sink), .bDescriptorType = USB_DT_ENDPOINT,
			  .bEndpointAddress = 2 | USB_DIR_OUT, .bmAttributes = USB_ENDPOINT_XFER_BULK, .wMaxPacketSize = cpu_to_le16(1024) },
		.sink_comp = { .bLength = sizeof(descriptors.ss_descs.sink_comp), .bDescriptorType = USB_DT_SS_ENDPOINT_COMP, .bMaxBurst = 15 },
	},
};

#define STR_INTERFACE "P501 bulk"
static const struct {
	struct usb_functionfs_strings_head header;
	struct { __le16 code; const char str1[sizeof STR_INTERFACE]; } __attribute__((packed)) lang0;
} __attribute__((packed)) strings = {
	.header = { .magic = cpu_to_le32(FUNCTIONFS_STRINGS_MAGIC), .length = cpu_to_le32(sizeof strings),
		    .str_count = cpu_to_le32(1), .lang_count = cpu_to_le32(1) },
	.lang0 = { cpu_to_le16(0x0409), STR_INTERFACE },
};


// ---- ADC/DAC DMA (axi_dmac) register access via /dev/mem; rings in cached CMA memory ----
#include <errno.h>
#define REG_CTRL 0x400
#define REG_START_TRANSFER 0x408
#define REG_FLAGS 0x40C
#define REG_DEST_ADDRESS 0x410
#define REG_SRC_ADDRESS 0x414
#define REG_X_LENGTH 0x418
#define REG_Y_LENGTH 0x41C
#define SENT 0x7777777777777777ULL
#define HF_OVR 1u
#define HF_HWOVF 2u
#define HF_END 0x80000000u
#define TX_PRIME 4
#define REQ_RX_START 1
#define REQ_RX_STOP 2
#define REQ_TX_START 3
#define REQ_TX_STOP 4
#define REQ_STATUS 5
#define REQ_TIME_GET 6
#define REQ_TIME_SET 7
#define REQ_RX_START_AT 8
#define HF_TX_TIMED 4u
#include <sched.h>
#include <sched.h>
static void pin_cpu(int cpu) { cpu_set_t m; CPU_ZERO(&m); CPU_SET(cpu, &m); if (sched_setaffinity(0, sizeof m, &m)) perror("affinity"); }   // CPU0 is left to the USB IRQ thread
static void rt_prio(int prio) { struct sched_param sp = { .sched_priority = prio }; if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp)) perror("SCHED_FIFO"); }
static volatile uint32_t *tm;                                  // p501_time registers (NULL if the bitstream has none)
static pthread_mutex_t tm_lock = PTHREAD_MUTEX_INITIALIZER;
static volatile int tx_timed_mode;
static void tcmd_l(uint32_t code, uint64_t a, uint32_t b) {
	tm[2] = (uint32_t)a; tm[3] = (uint32_t)(a >> 32); tm[4] = b; tm[5] = code;
	for (int i = 0; i < 1000000 && (tm[5] & 1); i++) ;
}
static void tcmd(uint32_t code, uint64_t a, uint32_t b) { if (!tm) return; pthread_mutex_lock(&tm_lock); tcmd_l(code, a, b); pthread_mutex_unlock(&tm_lock); }
static uint64_t tm_rd64(int lo_idx) { if (!tm) return 0; pthread_mutex_lock(&tm_lock); uint64_t lo = tm[lo_idx], hi = tm[lo_idx + 1]; pthread_mutex_unlock(&tm_lock); return (hi << 32) | lo; }
static uint64_t ts_now(void) { return tm_rd64(7); }
static uint64_t rx_first_ts(void) { return tm_rd64(9); }
static volatile uint32_t *adma, *ddma, *a9361;
static char *rx_ring, *tx_ring;
static uint64_t rx_phys, tx_phys; static size_t slot_bytes; static int nslots, tx_nslots;
static int ep_in = -1, ep_out = -1;
static double fs_hz = 30.72e6;
static void read_fs(void) { FILE *f = fopen("/tmp/p501_fs", "r"); double v; if (f) { if (fscanf(f, "%lf", &v) == 1 && v > 1e6) fs_hz = v; fclose(f); } }
static volatile int rx_run, rx_alive, tx_run, tx_alive;
static struct { uint64_t rx_slots, rx_hwovf, rx_msgs, tx_filled, tx_submitted, tx_unf, tx_bad, tx_msgs; uint64_t pl_ts, pl_done, pl_late, pl_unf; } st;
static volatile uint64_t tx_filled_v, tx_submitted_v;

static inline void civac(void *p, size_t n) {
	for (uintptr_t a = (uintptr_t)p & ~63UL; a < (uintptr_t)p + n; a += 64) asm volatile("dc civac, %0" :: "r"(a) : "memory");
	asm volatile("dsb sy" ::: "memory");
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
struct hdr { uint32_t magic, hdr_len; uint64_t seq; uint32_t payload_len, flags; uint64_t chunk; uint32_t overruns, slot_off; uint8_t pad[16]; } __attribute__((packed));

// queue one slot on a DMAC; -1 if its hardware queue is full. RX slots get a sentinel in their last cache line first.
static int submit(int is_tx, int slot)
{
	volatile uint32_t *d = is_tx ? ddma : adma;
	if (d[REG_START_TRANSFER / 4]) return -1;
	if (!is_tx) for (size_t j = 1; j <= slot_bytes / rxp; j++) { uint64_t *sp = (uint64_t *)(rx_ring + (size_t)slot * slot_bytes + j * rxp - 64); for (int i = 0; i < 8; i++) sp[i] = SENT; civac(sp, 64); }
	d[(is_tx ? REG_SRC_ADDRESS : REG_DEST_ADDRESS) / 4] = (uint32_t)((is_tx ? tx_phys : rx_phys) + (uint64_t)slot * slot_bytes);
	d[REG_X_LENGTH / 4] = slot_bytes - 1; d[REG_Y_LENGTH / 4] = 0; d[REG_FLAGS / 4] = 2;      // FLAG_LAST
	d[REG_START_TRANSFER / 4] = 1;
	return 0;
}

static volatile uint64_t rx_start_ts; static volatile int rx_timed_req;
static void *rx_thread(void *a)
{
	rt_prio(80); pin_cpu(3);
	uint64_t nsub = 0, nsend = 0, seq = 0, cs = 0; uint32_t flags = 0; double t0 = now();
	const uint64_t nch = slot_bytes / rxp;
	a9361[0x88 / 4] = 4;                                          // clear sticky ADC overflow
	if (tm) { tcmd(3, 0, 0); tcmd(4, 0, 1); tcmd(2, rx_timed_req ? rx_start_ts : 0, 0); }   // gate: timed start (0 = now)
	adma[REG_CTRL / 4] = 1; uint64_t t0ts = 0; int t0ok = 0;
	// RX USB writes go through FunctionFS AIO with up to rx_depth requests queued on the IN endpoint, so the endpoint never
	// idles between messages (a blocking writev per message used ~0.95 ms of every 1.07 ms in 2x2: no headroom to catch up
	// after a stall). A DMA slot is re-armed only once every write from it has completed.
	int rx_depth = getenv("P501_RX_AIO") ? atoi(getenv("P501_RX_AIO")) : 4; if (rx_depth < 1) rx_depth = 1; if (rx_depth > 16) rx_depth = 16;
	enum { Q = 64 };
	static struct hdr hq[Q]; static struct iovec ivq[Q][2]; static struct iocb cbq[Q]; static uint64_t tendq[Q], tsubq[Q]; static uint8_t doneq[Q];
	aio_context_t ctx = 0; if (syscall(SYS_io_setup, 32, &ctx)) { perror("io_setup rx"); goto out; }
	uint64_t cdone = 0;                                               // messages whose USB write has completed (contiguous)
	static int64_t dmx, wmx, amx, dsm, wsm, asm_; static int rn, wbig;
	while (rx_run) {
		while (nsub - nsend < (uint64_t)nslots) { if (submit(0, nsub % nslots) < 0) break; nsub++; }
		int progress = 0;
		while (cs < nsub * nch && cs - cdone < (uint64_t)rx_depth) {      // queue every finished message, up to rx_depth in flight
			uint64_t s = (cs / nch) % (uint64_t)nslots; size_t j = cs % nch;
			char *base = rx_ring + (size_t)s * slot_bytes + j * rxp;
			uint64_t *sp = (uint64_t *)(base + rxp - 64); civac(sp, 64);
			if (sp[0] == SENT && sp[7] == SENT) break;
			civac(base, rxp);
			if (tm && !t0ok) { t0ts = rx_first_ts(); t0ok = 1; }
			uint32_t o = a9361[0x88 / 4];
			if (o & 4) { a9361[0x88 / 4] = 4; if (cs > 0) { st.rx_hwovf++; flags |= HF_HWOVF; } if (cs > 0 && st.rx_hwovf <= 20) printf("HWOVF msg=%llu t=%.3f\n", (unsigned long long)cs, now() - t0); }
			int k = (int)(cs % Q); uint64_t sq = seq++;
			hq[k] = (struct hdr){ .magic = 0x31303550, .hdr_len = sizeof(struct hdr), .seq = sq, .payload_len = rxp, .flags = flags, .chunk = t0ts + sq * (uint64_t)(rxp / rx_fb), .slot_off = (uint32_t)(j * rxp) };
			flags = 0;
			ivq[k][0] = (struct iovec){ &hq[k], sizeof(struct hdr) }; ivq[k][1] = (struct iovec){ base, rxp };
			memset(&cbq[k], 0, sizeof cbq[k]);
			cbq[k].aio_lio_opcode = IOCB_CMD_PWRITEV; cbq[k].aio_fildes = ep_in; cbq[k].aio_buf = (uintptr_t)ivq[k]; cbq[k].aio_nbytes = 2; cbq[k].aio_data = cs;
			tendq[k] = hq[k].chunk + rxp / rx_fb; tsubq[k] = ts_now(); doneq[k] = 0;
			{ int64_t d = (int64_t)tsubq[k] - (int64_t)tendq[k]; if (d > dmx) dmx = d; dsm += d; }
			struct iocb *pc = &cbq[k];
			if (syscall(SYS_io_submit, ctx, 1, &pc) != 1) { perror("io_submit rx"); goto out_aio; }
			cs++; progress = 1;
		}
		if (cs > cdone) {                                             // reap completions (wait briefly only if nothing else to do)
			struct io_event ev[16]; struct timespec to = { 0, progress ? 0 : 200000 };
			int ne = syscall(SYS_io_getevents, ctx, progress ? 0 : 1, 16, ev, &to);
			if (ne < 0 && errno != EINTR) { perror("io_getevents rx"); goto out_aio; }
			for (int i = 0; i < ne; i++) {
				uint64_t m = ev[i].data; int k = (int)(m % Q);
				if ((long long)ev[i].res < 0) { errno = -(int)ev[i].res; perror("rx aio write"); goto out_aio; }
				doneq[k] = 1;
				uint64_t tq1 = ts_now(); int64_t w = (int64_t)(tq1 - tsubq[k]), ag = (int64_t)tq1 - (int64_t)tendq[k];
				if (w > wmx) wmx = w; if (ag > amx) amx = ag; wsm += w; asm_ += ag; if (w > (int64_t)(fs_hz * 4e-3)) wbig++;
				if (++rn == 2000) { printf("RXSTAT aio%d n=%d det avg %.2f max %.2f | wr avg %.2f max %.2f (>4ms %d) | age avg %.2f max %.2f ms\n", rx_depth, rn, dsm / (fs_hz / 1e3) / rn, dmx / (fs_hz / 1e3), wsm / (fs_hz / 1e3) / rn, wmx / (fs_hz / 1e3), wbig, asm_ / (fs_hz / 1e3) / rn, amx / (fs_hz / 1e3)); fflush(stdout); rn = 0; dmx = wmx = amx = dsm = wsm = asm_ = 0; wbig = 0; }
			}
			while (cdone < cs && doneq[cdone % Q]) {
				doneq[cdone % Q] = 0; cdone++; st.rx_msgs++;
				if (cdone % nch == 0) { nsend++; st.rx_slots++; }
			}
			if (ne > 0 || progress) continue;
		}
		if (!progress) usleep(50);
	}
	syscall(SYS_io_destroy, ctx);
	{ struct hdr h = { .magic = 0x31303550, .hdr_len = sizeof h, .seq = seq, .payload_len = 0, .flags = HF_END }; write(ep_in, &h, sizeof h); }
	goto out;
out_aio:
	syscall(SYS_io_destroy, ctx);
out:
	adma[REG_CTRL / 4] = 0; tcmd(3, 0, 0); tcmd(4, 0, 0);
	printf("rx stopped: %llu slots, hwovf %llu\n", (unsigned long long)st.rx_slots, (unsigned long long)st.rx_hwovf); fflush(stdout);
	rx_alive = 0; return NULL;
}

#define TX_PRIME_MSGS 16
#define TX_MAX_MSGS_PER_XFER 8
#define MAGIC 0x31303550
static int tx_ring_msgs;
struct txmeta { uint64_t start; uint32_t bytes; uint32_t timed; uint64_t t_rd; };
// TX latency histograms (ticks = 1/30.72 MHz). lead_* : 0.5 ms bins from -4 ms; lat/gap : 0.25 ms bins from 0.
#define HB 32
static uint64_t h_lead_rd[HB], h_lead_wr[HB], h_lat[HB], h_gap[HB], h_n_neg_wr, h_n_neg_rd;
static void hist_add(uint64_t *h, int64_t v, int64_t lo, int64_t bin) { int64_t b = (v - lo) / bin; if (v < lo) b = 0; if (b < 0) b = 0; if (b >= HB) b = HB - 1; h[b]++; }
static void hist_print(const char *nm, uint64_t *h, double lo_ms, double bin_ms) { printf("hist %s (lo %.2f ms, bin %.2f ms):", nm, lo_ms, bin_ms); for (int i = 0; i < HB; i++) printf(" %llu", (unsigned long long)h[i]); printf("\n"); }
static int dbg_d, dbg_r; static int dbg_on;
static struct txmeta *tx_meta; static uint64_t tx_next_ts;
static volatile uint64_t tx_released_v; static volatile double tx_last_fill;

static volatile uint64_t tx_rd_v;                              // messages read from USB, meta set (published in order)
// The USB read is kept free of cache maintenance: the next readv must be queued at once, or the OUT endpoint idles between
// messages and the stream cannot keep real time with short messages. tx_flusher does the cache flush and publishes tx_filled_v.
#include <linux/aio_abi.h>
#include <sys/syscall.h>
#ifndef TX_AIO_DEPTH
#define TX_AIO_DEPTH 6
#endif
// AIO keeps TX_AIO_DEPTH USB OUT requests queued on the endpoint at all times, so the copy-out of one message
// (done by FunctionFS before a sync readv returns) no longer leaves the endpoint idle.
static void *tx_reader(void *a)
{
	rt_prio(60); pin_cpu(2);
	aio_context_t ctx = 0; int R = tx_ring_msgs;
	if (syscall(SYS_io_setup, TX_AIO_DEPTH * 2, &ctx)) { perror("io_setup"); tx_run = 0; __atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL; }
	struct hdr *hs = calloc(R, sizeof *hs); struct iovec *iov = calloc(R, 2 * sizeof *iov); struct iocb *cb = calloc(R, sizeof *cb);
	ssize_t *res = calloc(R, sizeof *res); char *dn = calloc(R, 1);
	uint64_t sub = 0, rd = 0; int end = 0;
	while (tx_run && !end) {
		while (sub - rd < TX_AIO_DEPTH && sub < tx_released_v + (uint64_t)R) {           // keep the endpoint fed
			int k = sub % R; char *base = tx_ring + (size_t)k * txp;
			iov[2*k] = (struct iovec){ &hs[k], sizeof hs[k] }; iov[2*k+1] = (struct iovec){ base, txp };
			memset(&cb[k], 0, sizeof cb[k]);
			cb[k].aio_lio_opcode = IOCB_CMD_PREADV; cb[k].aio_fildes = ep_out; cb[k].aio_buf = (uintptr_t)&iov[2*k]; cb[k].aio_nbytes = 2; cb[k].aio_data = sub;
			struct iocb *p = &cb[k]; dn[k] = 0;
			if (syscall(SYS_io_submit, ctx, 1, &p) != 1) { perror("io_submit"); end = 1; break; }
			sub++;
		}
		if (end) break;
		struct io_event ev[TX_AIO_DEPTH * 2]; struct timespec to = { 0, 2000000 };
		int ne = syscall(SYS_io_getevents, ctx, 1, TX_AIO_DEPTH * 2, ev, &to);
		if (ne < 0) { if (errno == EINTR) continue; perror("io_getevents"); break; }
		for (int i = 0; i < ne; i++) { int k = ev[i].data % R; res[k] = ev[i].res; dn[k] = 1; }
		while (sub > rd && dn[rd % R]) {
			int k = rd % R; ssize_t n = res[k]; struct hdr h = hs[k]; char *base = tx_ring + (size_t)k * txp; dn[k] = 0;
			if (n < 0) { if (n == -EINTR) { end = 1; break; } fprintf(stderr, "aio read err %zd\n", n); end = 1; break; }
			if (n >= (ssize_t)sizeof h && h.magic == MAGIC && (h.flags & HF_END)) { end = 1; break; }
			if (n < (ssize_t)sizeof h || h.magic != MAGIC) { st.tx_bad++; fprintf(stderr, "tx bad hdr n=%zd\n", n); end = 1; break; }
			struct txmeta *tmeta = &tx_meta[rd % R];
			uint64_t t_rd = ts_now(); { static uint64_t prev_rd; if (prev_rd && t_rd > prev_rd) hist_add(h_gap, (int64_t)(t_rd - prev_rd), 0, 7680); prev_rd = t_rd; }
			if (h.flags & HF_TX_TIMED) {
				uint32_t bytes = (uint32_t)(n - sizeof h);
				if (!bytes || bytes > txp || (bytes % tx_fb) || bytes != h.payload_len) { st.tx_bad++; fprintf(stderr, "tx bad len\n"); end = 1; break; }
				if (bytes & 15) { uint32_t pad = 16 - (bytes & 15); memset(base + bytes, 0, pad); bytes += pad; }
				tmeta->start = h.chunk; tmeta->bytes = bytes; tmeta->timed = 1; tmeta->t_rd = t_rd;
			} else {
				if (n != (ssize_t)(sizeof h + txp)) { st.tx_bad++; fprintf(stderr, "tx bad untimed len\n"); end = 1; break; }
				tmeta->start = 0; tmeta->bytes = txp; tmeta->timed = 0;
			}
			__sync_synchronize();
			rd++; tx_rd_v = rd; st.tx_msgs++; tx_last_fill = now();
		}
	}
	syscall(SYS_io_destroy, ctx);                                   // cancels whatever is still queued on the endpoint
	free(hs); free(iov); free(cb); free(res); free(dn);
	tx_run = 0; __atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL;
}

static void *tx_flusher(void *a)
{
	rt_prio(65); pin_cpu(2);
	uint64_t filled = 0;
	while (tx_run) {
		if (filled < tx_rd_v) {
			struct txmeta *m = &tx_meta[filled % tx_ring_msgs];
			civac(tx_ring + (size_t)(filled % tx_ring_msgs) * txp, m->bytes);
			__sync_synchronize();
			filled++; tx_filled_v = filled; st.tx_filled = filled;
		} else usleep(20);
	}
	__atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL;
}

static int submit_tx(uint64_t msg, uint64_t bytes)
{
	if (ddma[REG_START_TRANSFER / 4]) return -1;
	ddma[REG_SRC_ADDRESS / 4] = (uint32_t)(tx_phys + msg * txp);
	ddma[REG_X_LENGTH / 4] = bytes - 1; ddma[REG_Y_LENGTH / 4] = 0; ddma[REG_FLAGS / 4] = 2;
	ddma[REG_START_TRANSFER / 4] = 1;
	return 0;
}

// Late-burst recovery. A burst the PL has to discard (it started later than the tolerance) drains at the DAC rate, so
// every following contiguous burst is late by the same amount and the PL never recovers (done freezes, late climbs).
// Fix on the host side: (1) never hand the PL a burst that is already (nearly) due, (2) when the PL reports a discarded
// burst, abort the DAC DMA, flush the PL descriptors and drop the queued bursts that are no longer playable.
// Tick limits are kept in time: p501d writes the sample rate to /tmp/p501_fs at every bring-up and the streams read it
// when they start (ticks = 1/fs). TX_MIN_LEAD 0.2 ms: a burst starting sooner cannot be fed in time (DMA latency);
// TX_RESYNC_LEAD 0.5 ms: after a resync only bursts starting later than this are kept.
#define TX_MIN_LEAD ((uint64_t)(fs_hz * 0.2e-3))
#define TX_RESYNC_LEAD ((uint64_t)(fs_hz * 0.5e-3))
static uint64_t n_resync, n_drop;

// Feeds the DAC DMAC with whatever contiguous messages are available (up to a slot per transfer). Playback starts once
// TX_PRIME_MSGS are queued, or when data has been waiting 20 ms (a short burst).
static void *tx_submitter(void *a)
{
	rt_prio(70); pin_cpu(1);
	uint64_t sub = 0, xfers = 0, ends[8] = {0}; int primed = 0, cnt = 0;
	uint32_t last_late = tm ? tm[0x30 / 4] : 0; double last_rs = 0;
	a9361[0x4088 / 4] = 1;                                        // clear sticky DAC underflow
	ddma[REG_CTRL / 4] = 1;
	while (tx_run) {
		uint64_t f = tx_filled_v;
		if (tx_timed_mode) {                                      // descriptor to the PL, then its DMA transfer (contiguous messages are merged)
			{ uint32_t lt = tm[0x30 / 4];
			  if (lt != last_late && now() - last_rs > 0.003) {
				uint64_t lim = ts_now() + TX_RESYNC_LEAD, dropped = 0, f2 = tx_filled_v;
				ddma[REG_CTRL / 4] = 0; tcmd(8, 0, 0); ddma[REG_CTRL / 4] = 1;     // abort DAC DMA, flush PL descriptors/pend
				while (sub < f2) { struct txmeta mm = tx_meta[sub % tx_ring_msgs]; if (mm.timed && mm.start < lim) { sub++; dropped++; } else break; }
				for (int i = 0; i < 8; i++) ends[i] = sub;
				xfers = 4; tx_submitted_v = sub; tx_released_v = sub; st.tx_submitted = sub; n_resync++; n_drop += dropped;
				last_late = tm[0x30 / 4]; last_rs = now();
				if (n_resync <= 50) { printf("RESYNC #%llu late=%u dropped_msgs=%llu sub=%llu filled=%llu\n", (unsigned long long)n_resync, lt, (unsigned long long)dropped, (unsigned long long)sub, (unsigned long long)f2); fflush(stdout); }
			  } }
			if (f > sub && !ddma[REG_START_TRANSFER / 4]) {
				uint64_t room = tx_ring_msgs - (sub % tx_ring_msgs);
				struct txmeta m = tx_meta[sub % tx_ring_msgs];
				uint64_t n = 1, bytes = m.bytes, nexts = m.start + m.bytes / tx_fb;
				if (!m.timed) {                                       // untimed data in timed mode: continue the previous burst, else start shortly from now
					uint64_t t = ts_now();
					m.start = (tx_next_ts >= t + 2000) ? tx_next_ts : t + 92160;
					nexts = m.start + m.bytes / tx_fb;
				}
				uint64_t first_start = m.start;
				while (n < TX_MAX_MSGS_PER_XFER && n < room && f > sub + n && bytes == (uint64_t)n * txp) {
					struct txmeta m2 = tx_meta[(sub + n) % tx_ring_msgs];
					if (m2.timed != m.timed || (m2.timed && m2.start != nexts)) break;
					bytes += m2.bytes; nexts += m2.bytes / tx_fb; n++;
				}
				tx_next_ts = first_start + bytes / tx_fb;
				if (m.timed && first_start < ts_now() + TX_MIN_LEAD) {      // already due: playing it would only make the PL discard it (and then stay late)
					sub += n; n_drop += n; xfers++; ends[xfers % 8] = sub; tx_submitted_v = sub; st.tx_submitted = sub;
					if (xfers >= 4) tx_released_v = ends[(xfers - 4) % 8];
					continue;
				}
				{ uint64_t tw = ts_now(); if (m.timed) { int64_t lr = (int64_t)first_start - (int64_t)m.t_rd, lw = (int64_t)first_start - (int64_t)tw; hist_add(h_lead_rd, lr, -61440, 15360); hist_add(h_lead_wr, lw, -61440, 15360); hist_add(h_lat, (int64_t)(tw - m.t_rd), 0, 7680); if (lr < 0) h_n_neg_rd++; if (lw < 0) h_n_neg_wr++; } }
tcmd(6, first_start, bytes / tx_fb);
				{ if (dbg_on && (dbg_d < 8 || (dbg_d % 64) == 0)) { printf("desc#%d msgs=%llu start=%llu frames=%llu now=%llu timed=%u\n", dbg_d, (unsigned long long)n, (unsigned long long)first_start, (unsigned long long)(bytes / tx_fb), (unsigned long long)ts_now(), m.timed); fflush(stdout); } dbg_d++; }
				if (submit_tx(sub % tx_ring_msgs, bytes) == 0) {
					sub += n; xfers++; ends[xfers % 8] = sub; tx_submitted_v = sub; st.tx_submitted = sub;
					if (xfers >= 4) tx_released_v = ends[(xfers - 4) % 8];
				}
			} else usleep(20);
			{ static double lastp; double tn = dbg_on ? now() : 0; if (dbg_on && tn - lastp > 0.5) { lastp = tn; printf("mon f=%llu sub=%llu rel=%llu startreg=%u ts=%llu st=%08x done=%u late=%u unf=%u ctrl=%x\n", (unsigned long long)f, (unsigned long long)sub, (unsigned long long)tx_released_v, ddma[REG_START_TRANSFER / 4], (unsigned long long)ts_now(), tm[6], tm[0x2C/4], tm[0x30/4], tm[0x34/4], ddma[REG_CTRL/4]); fflush(stdout); } }
			if ((++cnt & 15) == 0 && (a9361[0x4088 / 4] & 1)) { a9361[0x4088 / 4] = 1; st.tx_unf++; }
			continue;
		}
		if (!primed) {
			if (f - sub >= TX_PRIME_MSGS || (f > sub && now() - tx_last_fill > 0.02)) { primed = 1; a9361[0x4088 / 4] = 1; }
			else { usleep(200); continue; }
		}
		if (f > sub) {
			uint64_t n = f - sub, room = tx_ring_msgs - (sub % tx_ring_msgs);
			if (n > room) n = room; if (n > TX_MAX_MSGS_PER_XFER) n = TX_MAX_MSGS_PER_XFER;
			if (submit_tx(sub % tx_ring_msgs, n * txp) == 0) {
				sub += n; xfers++; ends[xfers % 8] = sub; tx_submitted_v = sub; st.tx_submitted = sub;
				if (xfers >= 4) tx_released_v = ends[(xfers - 4) % 8];        // transfers older than the 4-deep queue are done
			} else usleep(50);
		} else usleep(50);
		if ((++cnt & 15) == 0 && (a9361[0x4088 / 4] & 1)) { a9361[0x4088 / 4] = 1; st.tx_unf++; }
	}
	ddma[REG_CTRL / 4] = 0; tcmd(5, 0, 0);
	tx_submitted_v = 0; tx_filled_v = 0; tx_released_v = 0;
	hist_print("lead_at_read", h_lead_rd, -4.0, 0.5); hist_print("lead_at_desc_write", h_lead_wr, -4.0, 0.5); hist_print("read_to_desc_write_latency", h_lat, 0.0, 0.25); hist_print("readv_interarrival", h_gap, 0.0, 0.25); printf("neg_lead_at_read %llu neg_lead_at_write %llu\n", (unsigned long long)h_n_neg_rd, (unsigned long long)h_n_neg_wr);
	printf("resync %llu dropped_msgs %llu\n", (unsigned long long)n_resync, (unsigned long long)n_drop); printf("tx stopped: pl_done %u pl_late %u pl_unf %u st=%08x msgs",tm?tm[0x2C/4]:0,tm?tm[0x30/4]:0,tm?tm[0x34/4]:0,tm?tm[6]:0); printf(" %llu",0ULL); printf("|msgs %llu submitted %llu unf %llu bad %llu\n", (unsigned long long)st.tx_msgs, (unsigned long long)st.tx_submitted, (unsigned long long)st.tx_unf, (unsigned long long)st.tx_bad); fflush(stdout);
	__atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL;
}

static void *watchdog(void *a) { sleep((unsigned)run_secs_g); printf("watchdog exit\n"); fflush(stdout); exit(0); return NULL; }

static void start_rx(void) { pthread_t t; if (rx_alive || ep_in < 0) return; read_fs(); rx_alive = 1; rx_run = 1; pthread_create(&t, NULL, rx_thread, NULL); pthread_detach(t); }
static void start_tx(void)
{
	pthread_t t1, t2; if (tx_alive || ep_out < 0) return; read_fs();
	tx_ring_msgs = tx_nslots * (int)(slot_bytes / txp);
	memset(h_lead_rd, 0, sizeof h_lead_rd); memset(h_lead_wr, 0, sizeof h_lead_wr); memset(h_lat, 0, sizeof h_lat); memset(h_gap, 0, sizeof h_gap); h_n_neg_rd = h_n_neg_wr = 0; memset(&st.tx_filled, 0, 5 * sizeof(uint64_t)); tx_filled_v = tx_submitted_v = tx_released_v = 0; tx_last_fill = now(); tx_next_ts = 0; dbg_d = 0; dbg_r = 0; tcmd(5, 0, tx_timed_mode ? 1 : 0);
	tx_rd_v = 0; tx_alive = 3; tx_run = 1; pthread_create(&t1, NULL, tx_reader, NULL); pthread_detach(t1); pthread_create(&t2, NULL, tx_submitter, NULL); pthread_detach(t2); pthread_t t3; pthread_create(&t3, NULL, tx_flusher, NULL); pthread_detach(t3);
}

// One channel pair per bit (bit0 = RX1/TX1, bit1 = RX2/TX2). The PL packers (util_cpack2 / util_upack2) pack only the
// enabled channels, so a single pair halves the frame size: 4 bytes instead of 8.
static void set_rx_mask(int mask)
{
	if (!(mask & 3)) mask = 3;
	for (int ch = 0; ch < 4; ch++) {
		uint32_t v = a9361[(0x400 + ch * 0x40) / 4];
		v = ((mask >> (ch / 2)) & 1) ? (v | 1u) : (v & ~1u);
		a9361[(0x400 + ch * 0x40) / 4] = v;
	}
	rx_fb = 4 * __builtin_popcount(mask & 3); rxp = (size_t)256 * 1024 * rx_fb / 8;
	printf("rx mask %d: frame %d B, message %zu B\n", mask & 3, rx_fb, rxp); fflush(stdout);
}
static void set_tx_mask(int mask)
{
	if (!(mask & 3)) mask = 3;
	for (int ch = 0; ch < 4; ch++) a9361[(0x4418 + ch * 0x40) / 4] = ((mask >> (ch / 2)) & 1) ? 2 : 3;   // DATA_SEL: 2 = DMA, 3 = zero
	tx_fb = 4 * __builtin_popcount(mask & 3); txp = tx_fb == 8 ? (size_t)512 * 1024 : (size_t)128 * 1024; if (getenv("P501_TXP_KIB")) { txp = (size_t)atoi(getenv("P501_TXP_KIB")) * 1024; tx_ring_msgs = tx_nslots * (int)(slot_bytes / txp); }
	printf("tx mask %d: frame %d B, message %zu B\n", mask & 3, tx_fb, txp); fflush(stdout);
}

int main(int argc, char **argv)
{
	dbg_on = getenv("P501_STREAM_DEBUG") != NULL;
	if (argc < 11) { fprintf(stderr, "usage: %s ffsdir rx_ring_phys slot_bytes nslots adc_dma_phys secs ad9361_phys tx_ring_phys dac_dma_phys tx_nslots\n", argv[0]); return 1; }
	snprintf(dir, sizeof dir, "%s", argv[1]); rx_phys = strtoull(argv[2], 0, 0); slot_bytes = strtoull(argv[3], 0, 0); nslots = atoi(argv[4]);
	uint64_t adc_dma = strtoull(argv[5], 0, 0); run_secs_g = atof(argv[6]); uint64_t a9361_phys = strtoull(argv[7], 0, 0);
	tx_phys = strtoull(argv[8], 0, 0); uint64_t dac_dma = strtoull(argv[9], 0, 0); tx_nslots = atoi(argv[10]); uint64_t tm_phys = argc > 11 ? strtoull(argv[11], 0, 0) : 0; tx_ring_msgs = tx_nslots * (int)(slot_bytes / txp);
	int mfd = open("/dev/mem", O_RDWR | O_SYNC); if (mfd < 0) { perror("/dev/mem"); return 1; }
	adma = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, adc_dma); ddma = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, dac_dma);
	a9361 = mmap(0, 0x8000, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, a9361_phys);
	if (tm_phys) { tm = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, tm_phys); if (tm == MAP_FAILED) { perror("mmap time"); tm = NULL; } else { printf("p501_time id %08x\n", tm[0]); { const char *lt = getenv("P501_LATE_TOL"); uint32_t tol = lt ? (uint32_t)strtoul(lt, NULL, 0) : 30720; tm[0x38 / 4] = tol; printf("late_tol %u ticks\n", tol); } } }   /* PL late tolerance: the PL default of 64 ticks (2 us) discards whole bursts after a tiny DMA starvation */
	if (adma == MAP_FAILED || ddma == MAP_FAILED || a9361 == MAP_FAILED) { perror("mmap regs"); return 1; }
	int cfd = open("/dev/mem", O_RDWR);                           // cached mappings of the CMA rings
	rx_ring = mmap(0, (size_t)nslots * slot_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, rx_phys);
	tx_ring = mmap(0, (size_t)tx_nslots * slot_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, tx_phys);
	tx_meta = calloc((size_t)tx_nslots * (slot_bytes / (128 * 1024)), sizeof *tx_meta);
	if (rx_ring == MAP_FAILED || tx_ring == MAP_FAILED) { perror("mmap ring"); return 1; }
	char p[256]; snprintf(p, sizeof p, "%s/ep0", dir);
	int ep0 = open(p, O_RDWR); if (ep0 < 0) { perror("open ep0"); return 1; }
	if (write(ep0, &descriptors, sizeof descriptors) < 0) { perror("write descriptors"); return 1; }
	if (write(ep0, &strings, sizeof strings) < 0) { perror("write strings"); return 1; }
	printf("descriptors written\n"); fflush(stdout);
	{ FILE *f = fopen("/tmp/stream_ready", "w"); if (f) fclose(f); }
	if (run_secs_g > 0) { pthread_t w; pthread_create(&w, NULL, watchdog, NULL); pthread_detach(w); }
	for (;;) {
		struct usb_functionfs_event ev; ssize_t n = read(ep0, &ev, sizeof ev); if (n <= 0) { perror("ep0 read"); break; }
		if (ev.type == FUNCTIONFS_ENABLE) {
			if (ep_in < 0) { snprintf(p, sizeof p, "%s/ep1", dir); ep_in = open(p, O_WRONLY); snprintf(p, sizeof p, "%s/ep2", dir); ep_out = open(p, O_RDONLY); printf("endpoints open: in=%d out=%d\n", ep_in, ep_out); fflush(stdout); }
		} else if (ev.type == FUNCTIONFS_DISABLE) { rx_run = 0; tx_run = 0; }
		else if (ev.type == FUNCTIONFS_SETUP) {
			int req = ev.u.setup.bRequest, in = ev.u.setup.bRequestType & USB_DIR_IN;
			if (!in) {
				uint64_t arg = 0; int wl = ev.u.setup.wLength;
				if (wl > 0) { char db[16] = {0}; ssize_t rn = read(ep0, db, wl < 16 ? wl : 16); if (rn >= 8) memcpy(&arg, db, 8); }
				switch (req) {
				case REQ_RX_START: rx_timed_req = 0; if (!rx_alive) set_rx_mask(ev.u.setup.wValue & 3); start_rx(); break;
				case REQ_RX_START_AT: rx_start_ts = arg; rx_timed_req = 1; if (!rx_alive) set_rx_mask(ev.u.setup.wValue & 3); start_rx(); break;
				case REQ_RX_STOP: rx_run = 0; break;
				case REQ_TX_START: tx_timed_mode = tm ? (ev.u.setup.wValue & 1) : 0; if (!tx_alive) set_tx_mask((ev.u.setup.wValue >> 4) & 3); start_tx(); break;
				case REQ_TX_STOP: tx_run = 0; break;
				case REQ_TIME_SET: printf("TIME_SET %llu (was %llu)\n", (unsigned long long)arg, (unsigned long long)ts_now()); fflush(stdout); tcmd(1, arg, 0); printf("  now %llu\n", (unsigned long long)ts_now()); fflush(stdout); break; }
				if (wl == 0 && read(ep0, p, 0) < 0) perror("ep0 ack");
			} else if (req == REQ_TIME_GET) { uint64_t t = ts_now(); size_t l = ev.u.setup.wLength < 8 ? ev.u.setup.wLength : 8; if (write(ep0, &t, l) < 0) perror("ep0 time"); }
			else if (req == REQ_STATUS) { if (tm) { st.pl_ts = ts_now(); pthread_mutex_lock(&tm_lock); st.pl_done = tm[0x2C / 4]; st.pl_late = tm[0x30 / 4]; st.pl_unf = tm[0x34 / 4]; pthread_mutex_unlock(&tm_lock); } size_t l = ev.u.setup.wLength < sizeof st ? ev.u.setup.wLength : sizeof st; if (write(ep0, &st, l) < 0) perror("ep0 status"); }
			else write(ep0, NULL, 0);                              // stall
		}
	}
	return 0;
}
