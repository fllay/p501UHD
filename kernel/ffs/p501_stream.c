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


// ---- ADC DMA (axi_dmac) register access via /dev/mem, ring in cached CMA memory ----
#define REG_CTRL 0x400
#define REG_TRANSFER_ID 0x404
#define REG_START_TRANSFER 0x408
#define REG_FLAGS 0x40C
#define REG_DEST_ADDRESS 0x410
#define REG_X_LENGTH 0x418
#define REG_Y_LENGTH 0x41C
#define REG_TRANSFER_DONE 0x428
static volatile uint32_t *dma;
static char *ring;
static uint64_t ring_phys; static size_t slot_bytes; int nslots; static uint64_t dma_phys; static double run_secs;

static inline void civac(void *p, size_t n) {
	for (uintptr_t a = (uintptr_t)p & ~63UL; a < (uintptr_t)p + n; a += 64) asm volatile("dc civac, %0" :: "r"(a) : "memory");
	asm volatile("dsb sy" ::: "memory");
}
static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t); return t.tv_sec + t.tv_nsec * 1e-9; }
#define SENT 0x7777777777777777ULL
static uint64_t premature = 0;
static int submit(int slot) {
	if (dma[REG_START_TRANSFER / 4]) return -1;                 // hardware queue full
	{ uint64_t *sp = (uint64_t *)(ring + (size_t)slot * slot_bytes + slot_bytes - 64); for (int i = 0; i < 8; i++) sp[i] = SENT; civac(sp, 64); }
	dma[REG_DEST_ADDRESS / 4] = (uint32_t)(ring_phys + (uint64_t)slot * slot_bytes);
	dma[REG_X_LENGTH / 4] = slot_bytes - 1; dma[REG_Y_LENGTH / 4] = 0; dma[REG_FLAGS / 4] = 2;   // FLAG_LAST
	int tid = dma[REG_TRANSFER_ID / 4] & 0x3;
	dma[REG_START_TRANSFER / 4] = 1;
	return tid;
}
struct hdr { uint32_t magic, hdr_len; uint64_t seq; uint32_t payload_len, flags; uint64_t chunk; uint32_t overruns, slot_off; uint8_t pad[16]; } __attribute__((packed));

static void *stream_thread(void *a)
{
	char p[256]; snprintf(p, sizeof p, "%s/ep1", dir);
	int fd = open(p, O_WRONLY); if (fd < 0) { perror("open ep1"); return NULL; }
	dma[REG_CTRL / 4] = 1;
	int tid_of[64] = {0}; char done[64] = {0};
	uint64_t nsub = 0, nsend = 0, seq = 0; int pending = 0; uint32_t overruns = 0, ovr_flag = 0;
	double t0 = now(), tl = t0; uint64_t bytes = 0, lb = 0;
	while (run_secs <= 0 || now() - t0 < run_secs) {
		while (nsub - nsend < (uint64_t)nslots) {
			int t = submit(nsub % nslots); if (t < 0) break;
			tid_of[nsub % nslots] = t; done[nsub % nslots] = 0; nsub++; pending++;
		}
		uint32_t m = dma[REG_TRANSFER_DONE / 4];
		for (uint64_t k = nsend; k < nsub; k++) { int s = k % nslots; if (!done[s]) { uint64_t *sp = (uint64_t *)(ring + (size_t)s * slot_bytes + slot_bytes - 64); civac(sp, 64); if (sp[0] != SENT || sp[7] != SENT) { done[s] = 1; pending--; } } }
		if (pending == 0 && nsub > nsend + 0 && done[nsend % nslots] == 0) { }
		int c1 = (nsub == nsend), c2 = (pending == 0 && nsub - nsend >= (uint64_t)nslots);
		if (c2) { overruns++; ovr_flag = 1; if (overruns <= 12) { printf("OVR#%u c1=%d c2=%d nsub=%llu nsend=%llu pending=%d mask=%x t=%.3f\n", overruns, c1, c2, (unsigned long long)nsub, (unsigned long long)nsend, pending, m, now() - t0); fflush(stdout); } }   // nothing queued while the ADC keeps running (or ring full)
		if (nsub > nsend && done[nsend % nslots]) {
			int s = nsend % nslots; char *base = ring + (size_t)s * slot_bytes;
			civac(base, slot_bytes);
			{ uint64_t *sp = (uint64_t *)(base + slot_bytes - 64); if (sp[0] == SENT && sp[7] == SENT) { premature++; if (premature <= 20) { printf("PREMATURE slot=%d nsend=%llu nsub=%llu pending=%d mask=%x t=%.3f\n", s, (unsigned long long)nsend, (unsigned long long)nsub, pending, m, now() - t0); fflush(stdout); } } }
			for (size_t off = 0; off < slot_bytes; off += PAYLOAD) {
				struct hdr h = { .magic = 0x31303550, .hdr_len = sizeof h, .seq = seq++, .payload_len = PAYLOAD, .flags = ovr_flag, .chunk = nsend, .overruns = overruns, .slot_off = (uint32_t)off };
				ovr_flag = 0;
				struct iovec iov[2] = { { &h, sizeof h }, { base + off, PAYLOAD } };
				ssize_t n = writev(fd, iov, 2); if (n < 0) { perror("writev ep1"); goto out; }
				bytes += n;
			}
			nsend++;
		} else usleep(50);
		double t = now(); if (t - tl >= 1.0) { printf("stream %.1f MB/s  chunks %llu  overruns %u  pending %d\n", (bytes - lb) / (t - tl) / 1e6, (unsigned long long)nsend, overruns, pending); fflush(stdout); tl = t; lb = bytes; }
	}
out:
	dma[REG_CTRL / 4] = 0;
	printf("premature total %llu\n", (unsigned long long)premature); printf("done: %llu chunks, %llu MB, overruns %u\n", (unsigned long long)nsend, (unsigned long long)(bytes / 1000000), overruns); fflush(stdout);
	exit(0);
	return NULL;
}

int main(int argc, char **argv)
{
	if (argc < 7) { fprintf(stderr, "usage: %s ffsdir ring_phys slot_bytes nslots dma_phys secs\n", argv[0]); return 1; }
	snprintf(dir, sizeof dir, "%s", argv[1]); ring_phys = strtoull(argv[2], 0, 0); slot_bytes = strtoull(argv[3], 0, 0);
	nslots = atoi(argv[4]); dma_phys = strtoull(argv[5], 0, 0); run_secs = atof(argv[6]);
	int mfd = open("/dev/mem", O_RDWR | O_SYNC); if (mfd < 0) { perror("/dev/mem"); return 1; }
	dma = mmap(0, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, mfd, dma_phys); if (dma == MAP_FAILED) { perror("mmap dma"); return 1; }
	int cfd = open("/dev/mem", O_RDWR);                         // cached mapping of the CMA ring
	ring = mmap(0, (size_t)nslots * slot_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, cfd, ring_phys); if (ring == MAP_FAILED) { perror("mmap ring"); return 1; }
	char p[256]; snprintf(p, sizeof p, "%s/ep0", dir);
	int ep0 = open(p, O_RDWR); if (ep0 < 0) { perror("open ep0"); return 1; }
	if (write(ep0, &descriptors, sizeof descriptors) < 0) { perror("write descriptors"); return 1; }
	if (write(ep0, &strings, sizeof strings) < 0) { perror("write strings"); return 1; }
	printf("descriptors written\n"); fflush(stdout);
	{ FILE *f = fopen("/tmp/stream_ready", "w"); if (f) fclose(f); }
	pthread_t t; int started = 0;
	for (;;) {
		struct usb_functionfs_event ev; ssize_t n = read(ep0, &ev, sizeof ev); if (n <= 0) { perror("ep0 read"); break; }
		printf("event %d\n", ev.type); fflush(stdout);
		if (ev.type == FUNCTIONFS_ENABLE && !started) { started = 1; pthread_create(&t, NULL, stream_thread, NULL); }
		else if (ev.type == FUNCTIONFS_SETUP) { if (ev.u.setup.bRequestType & USB_DIR_IN) read(ep0, NULL, 0); else write(ep0, NULL, 0); }
	}
	return 0;
}
