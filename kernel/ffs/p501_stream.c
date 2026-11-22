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

#define PAYLOAD (512 * 1024)
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
	if (!is_tx) { uint64_t *sp = (uint64_t *)(rx_ring + (size_t)slot * slot_bytes + slot_bytes - 64); for (int i = 0; i < 8; i++) sp[i] = SENT; civac(sp, 64); }
	d[(is_tx ? REG_SRC_ADDRESS : REG_DEST_ADDRESS) / 4] = (uint32_t)((is_tx ? tx_phys : rx_phys) + (uint64_t)slot * slot_bytes);
	d[REG_X_LENGTH / 4] = slot_bytes - 1; d[REG_Y_LENGTH / 4] = 0; d[REG_FLAGS / 4] = 2;      // FLAG_LAST
	d[REG_START_TRANSFER / 4] = 1;
	return 0;
}

static volatile uint64_t rx_start_ts; static volatile int rx_timed_req;
static void *rx_thread(void *a)
{
	rt_prio(60);
	char done[64] = {0}; uint64_t nsub = 0, nsend = 0, seq = 0; uint32_t flags = 0; double t0 = now();
	a9361[0x88 / 4] = 4;                                          // clear sticky ADC overflow
	if (tm) { tcmd(3, 0, 0); tcmd(4, 0, 1); tcmd(2, rx_timed_req ? rx_start_ts : 0, 0); }   // gate: timed start (0 = now)
	adma[REG_CTRL / 4] = 1; uint64_t t0ts = 0; int t0ok = 0;
	while (rx_run) {
		while (nsub - nsend < (uint64_t)nslots) { if (submit(0, nsub % nslots) < 0) break; done[nsub % nslots] = 0; nsub++; }
		for (uint64_t k = nsend; k < nsub; k++) {                 // completion = sentinel overwritten by the DMA
			int s = k % nslots; if (done[s]) continue;
			uint64_t *sp = (uint64_t *)(rx_ring + (size_t)s * slot_bytes + slot_bytes - 64); civac(sp, 64);
			if (sp[0] != SENT || sp[7] != SENT) done[s] = 1;
		}
		if (nsub > nsend && done[nsend % nslots]) {
			int s = nsend % nslots; char *base = rx_ring + (size_t)s * slot_bytes;
			civac(base, slot_bytes);
			if (tm && !t0ok) { t0ts = rx_first_ts(); t0ok = 1; }
			uint32_t o = a9361[0x88 / 4];
			if (o & 4) { a9361[0x88 / 4] = 4; if (nsend > 0) { st.rx_hwovf++; flags |= HF_HWOVF; } if (nsend > 0 && st.rx_hwovf <= 20) printf("HWOVF slot=%llu first_seq=%llu t=%.3f\n", (unsigned long long)nsend, (unsigned long long)seq, now() - t0); }
			for (size_t off = 0; off < slot_bytes; off += PAYLOAD) {
				uint64_t sq = seq++; struct hdr h = { .magic = 0x31303550, .hdr_len = sizeof h, .seq = sq, .payload_len = PAYLOAD, .flags = flags, .chunk = t0ts + sq * (uint64_t)(PAYLOAD / 8), .slot_off = (uint32_t)off };
				flags = 0;
				struct iovec iov[2] = { { &h, sizeof h }, { base + off, PAYLOAD } };
				if (writev(ep_in, iov, 2) < 0) { perror("writev ep1"); goto out; }
				st.rx_msgs++;
			}
			nsend++; st.rx_slots++;
		} else usleep(50);
	}
	{ struct hdr h = { .magic = 0x31303550, .hdr_len = sizeof h, .seq = seq, .payload_len = 0, .flags = HF_END }; write(ep_in, &h, sizeof h); }
out:
	adma[REG_CTRL / 4] = 0; tcmd(3, 0, 0); tcmd(4, 0, 0);
	printf("rx stopped: %llu slots, hwovf %llu\n", (unsigned long long)st.rx_slots, (unsigned long long)st.rx_hwovf); fflush(stdout);
	rx_alive = 0; return NULL;
}

#define TX_PRIME_MSGS 16
#define TX_MAX_MSGS_PER_XFER 8
#define MAGIC 0x31303550
static int tx_ring_msgs;
struct txmeta { uint64_t start; uint32_t bytes; uint32_t timed; };
static int dbg_d, dbg_r; static int dbg_on;
static struct txmeta *tx_meta; static uint64_t tx_next_ts;
static volatile uint64_t tx_released_v; static volatile double tx_last_fill;

static void *tx_reader(void *a)
{
	rt_prio(60);
	uint64_t filled = 0;
	while (tx_run) {
		while (tx_run && filled >= tx_released_v + (uint64_t)tx_ring_msgs) usleep(100);        // ring full
		if (!tx_run) break;
		char *base = tx_ring + (size_t)(filled % tx_ring_msgs) * PAYLOAD;
		struct hdr h; struct iovec iov[2] = { { &h, sizeof h }, { base, PAYLOAD } };
		ssize_t n = readv(ep_out, iov, 2);
		if (n < 0) { if (errno == EINTR) continue; perror("readv ep2"); break; }
		if (n >= (ssize_t)sizeof h && h.magic == MAGIC && (h.flags & HF_END)) break;
		if (n < (ssize_t)sizeof h || h.magic != MAGIC) { st.tx_bad++; continue; }
		struct txmeta *tmeta = &tx_meta[filled % tx_ring_msgs];
		{ if (dbg_on && dbg_r < 10) { dbg_r++; printf("rd filled=%llu seq=%llu chunk=%llu flags=%x n=%zd\n", (unsigned long long)filled, (unsigned long long)h.seq, (unsigned long long)h.chunk, h.flags, n); fflush(stdout); } }
		if (h.flags & HF_TX_TIMED) {
			uint32_t bytes = (uint32_t)(n - sizeof h);
			if (!bytes || bytes > PAYLOAD || (bytes & 7) || bytes != h.payload_len) { st.tx_bad++; continue; }
			if (bytes & 15) { memset(base + bytes, 0, 8); bytes += 8; }            // DMA length multiple of 16: pad one zero frame
			tmeta->start = h.chunk; tmeta->bytes = bytes; tmeta->timed = 1;
			civac(base, bytes);
		} else {
			if (n != (ssize_t)(sizeof h + PAYLOAD)) { st.tx_bad++; continue; }
			tmeta->start = 0; tmeta->bytes = PAYLOAD; tmeta->timed = 0;
			civac(base, PAYLOAD);
		}
		__sync_synchronize();
		filled++; tx_filled_v = filled; st.tx_filled = filled; st.tx_msgs++; tx_last_fill = now();
	}
	tx_run = 0; __atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL;
}

static int submit_tx(uint64_t msg, uint64_t bytes)
{
	if (ddma[REG_START_TRANSFER / 4]) return -1;
	ddma[REG_SRC_ADDRESS / 4] = (uint32_t)(tx_phys + msg * PAYLOAD);
	ddma[REG_X_LENGTH / 4] = bytes - 1; ddma[REG_Y_LENGTH / 4] = 0; ddma[REG_FLAGS / 4] = 2;
	ddma[REG_START_TRANSFER / 4] = 1;
	return 0;
}

// Feeds the DAC DMAC with whatever contiguous messages are available (up to a slot per transfer). Playback starts once
// TX_PRIME_MSGS are queued, or when data has been waiting 20 ms (a short burst).
static void *tx_submitter(void *a)
{
	rt_prio(70);
	uint64_t sub = 0, xfers = 0, ends[8] = {0}; int primed = 0, cnt = 0;
	a9361[0x4088 / 4] = 1;                                        // clear sticky DAC underflow
	ddma[REG_CTRL / 4] = 1;
	while (tx_run) {
		uint64_t f = tx_filled_v;
		if (tx_timed_mode) {                                      // descriptor to the PL, then its DMA transfer (contiguous messages are merged)
			if (f > sub && !ddma[REG_START_TRANSFER / 4]) {
				uint64_t room = tx_ring_msgs - (sub % tx_ring_msgs);
				struct txmeta m = tx_meta[sub % tx_ring_msgs];
				uint64_t n = 1, bytes = m.bytes, nexts = m.start + m.bytes / 8;
				if (!m.timed) {                                       // untimed data in timed mode: continue the previous burst, else start shortly from now
					uint64_t t = ts_now();
					m.start = (tx_next_ts >= t + 2000) ? tx_next_ts : t + 92160;
					nexts = m.start + m.bytes / 8;
				}
				uint64_t first_start = m.start;
				while (n < TX_MAX_MSGS_PER_XFER && n < room && f > sub + n && bytes == (uint64_t)n * PAYLOAD) {
					struct txmeta m2 = tx_meta[(sub + n) % tx_ring_msgs];
					if (m2.timed != m.timed || (m2.timed && m2.start != nexts)) break;
					bytes += m2.bytes; nexts += m2.bytes / 8; n++;
				}
				tx_next_ts = first_start + bytes / 8;
				tcmd(6, first_start, bytes / 8);
				{ if (dbg_on && (dbg_d < 8 || (dbg_d % 64) == 0)) { printf("desc#%d msgs=%llu start=%llu frames=%llu now=%llu timed=%u\n", dbg_d, (unsigned long long)n, (unsigned long long)first_start, (unsigned long long)(bytes / 8), (unsigned long long)ts_now(), m.timed); fflush(stdout); } dbg_d++; }
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
			if (submit_tx(sub % tx_ring_msgs, n * PAYLOAD) == 0) {
				sub += n; xfers++; ends[xfers % 8] = sub; tx_submitted_v = sub; st.tx_submitted = sub;
				if (xfers >= 4) tx_released_v = ends[(xfers - 4) % 8];        // transfers older than the 4-deep queue are done
			} else usleep(50);
		} else usleep(50);
		if ((++cnt & 15) == 0 && (a9361[0x4088 / 4] & 1)) { a9361[0x4088 / 4] = 1; st.tx_unf++; }
	}
	ddma[REG_CTRL / 4] = 0; tcmd(5, 0, 0);
	tx_submitted_v = 0; tx_filled_v = 0; tx_released_v = 0;
	printf("tx stopped: pl_done %u pl_late %u pl_unf %u st=%08x msgs",tm?tm[0x2C/4]:0,tm?tm[0x30/4]:0,tm?tm[0x34/4]:0,tm?tm[6]:0); printf(" %llu",0ULL); printf("|msgs %llu submitted %llu unf %llu bad %llu\n", (unsigned long long)st.tx_msgs, (unsigned long long)st.tx_submitted, (unsigned long long)st.tx_unf, (unsigned long long)st.tx_bad); fflush(stdout);
	__atomic_fetch_sub(&tx_alive, 1, __ATOMIC_SEQ_CST); return NULL;
}

static void *watchdog(void *a) { sleep((unsigned)run_secs_g); printf("watchdog exit\n"); fflush(stdout); exit(0); return NULL; }

static void start_rx(void) { pthread_t t; if (rx_alive || ep_in < 0) return; rx_alive = 1; rx_run = 1; pthread_create(&t, NULL, rx_thread, NULL); pthread_detach(t); }
static void start_tx(void)
{
	pthread_t t1, t2; if (tx_alive || ep_out < 0) return;
	memset(&st.tx_filled, 0, 5 * sizeof(uint64_t)); tx_filled_v = tx_submitted_v = tx_released_v = 0; tx_last_fill = now(); tx_next_ts = 0; dbg_d = 0; dbg_r = 0; tcmd(5, 0, tx_timed_mode ? 1 : 0);
	tx_alive = 2; tx_run = 1; pthread_create(&t1, NULL, tx_reader, NULL); pthread_detach(t1); pthread_create(&t2, NULL, tx_submitter, NULL); pthread_detach(t2);
}

int main(int argc, char **argv)
{
	dbg_on = getenv("P501_STREAM_DEBUG") != NULL;
	if (argc < 11) { fprintf(stderr, "usage: %s ffsdir rx_ring_phys slot_bytes nslots adc_dma_phys secs ad9361_phys tx_ring_phys dac_dma_phys tx_nslots\n", argv[0]); return 1; }
	snprintf(dir, sizeof dir, "%s", argv[1]); rx_phys = strtoull(argv[2], 0, 0); slot_bytes = strtoull(argv[3], 0, 0); nslots = atoi(argv[4]);
	uint64_t adc_dma = strtoull(argv[5], 0, 0); run_secs_g = atof(argv[6]); uint64_t a9361_phys = strtoull(argv[7], 0, 0);
	tx_phys = strtoull(argv[8], 0, 0); uint64_t dac_dma = strtoull(argv[9], 0, 0); tx_nslots = atoi(argv[10]); uint64_t tm_phys = argc > 11 ? strtoull(argv[11], 0, 0) : 0; tx_ring_msgs = tx_nslots * (int)(slot_bytes / PAYLOAD);
	int mfd = open("/dev/mem", O_RDWR | O_SYNC); if (mfd < 0) { perror("/dev/mem"); return 1; }
	adma = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, adc_dma); ddma = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, dac_dma);
	a9361 = mmap(0, 0x8000, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, a9361_phys);
	if (tm_phys) { tm = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, tm_phys); if (tm == MAP_FAILED) { perror("mmap time"); tm = NULL; } else printf("p501_time id %08x\n", tm[0]); }
	if (adma == MAP_FAILED || ddma == MAP_FAILED || a9361 == MAP_FAILED) { perror("mmap regs"); return 1; }
	int cfd = open("/dev/mem", O_RDWR);                           // cached mappings of the CMA rings
	rx_ring = mmap(0, (size_t)nslots * slot_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, rx_phys);
	tx_ring = mmap(0, (size_t)tx_nslots * slot_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, tx_phys);
	tx_meta = calloc(tx_ring_msgs, sizeof *tx_meta);
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
				case REQ_RX_START: rx_timed_req = 0; start_rx(); break;
				case REQ_RX_START_AT: rx_start_ts = arg; rx_timed_req = 1; start_rx(); break;
				case REQ_RX_STOP: rx_run = 0; break;
				case REQ_TX_START: tx_timed_mode = tm ? (ev.u.setup.wValue & 1) : 0; start_tx(); break;
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
