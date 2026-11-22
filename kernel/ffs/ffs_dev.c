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

#define BUFSZ (512 * 1024)
static volatile uint64_t in_bytes, out_bytes;
static char dir[256];

#define cpu_to_le16(x) (x)  /* little-endian target */
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

static void *src_thread(void *a)
{
	char p[256]; snprintf(p, sizeof p, "%s/ep1", dir);
	int fd = open(p, O_WRONLY); if (fd < 0) { perror("open ep1"); return NULL; }
	char *buf = aligned_alloc(4096, BUFSZ); memset(buf, 0x5A, BUFSZ);
	for (;;) { ssize_t n = write(fd, buf, BUFSZ); if (n < 0) { perror("write ep1"); break; } in_bytes += n; }
	return NULL;
}
static void *sink_thread(void *a)
{
	char p[256]; snprintf(p, sizeof p, "%s/ep2", dir);
	int fd = open(p, O_RDONLY); if (fd < 0) { perror("open ep2"); return NULL; }
	char *buf = aligned_alloc(4096, BUFSZ);
	for (;;) { ssize_t n = read(fd, buf, BUFSZ); if (n < 0) { perror("read ep2"); break; } out_bytes += n; }
	return NULL;
}
static void *stat_thread(void *a)
{
	uint64_t li = 0, lo = 0;
	for (;;) { sleep(1); uint64_t i = in_bytes, o = out_bytes;
		if (i != li || o != lo) printf("in(to host) %.1f MB/s  out(from host) %.1f MB/s\n", (i - li) / 1e6, (o - lo) / 1e6), fflush(stdout);
		li = i; lo = o; }
	return NULL;
}

int main(int argc, char **argv)
{
	snprintf(dir, sizeof dir, "%s", argc > 1 ? argv[1] : "/dev/ffs");
	char p[256]; snprintf(p, sizeof p, "%s/ep0", dir);
	int ep0 = open(p, O_RDWR); if (ep0 < 0) { perror("open ep0"); return 1; }
	if (write(ep0, &descriptors, sizeof descriptors) < 0) { perror("write descriptors"); return 1; }
	if (write(ep0, &strings, sizeof strings) < 0) { perror("write strings"); return 1; }
	printf("descriptors written\n"); fflush(stdout);
	pthread_t t; int started = 0;
	for (;;) {
		struct usb_functionfs_event ev;
		ssize_t n = read(ep0, &ev, sizeof ev); if (n <= 0) { perror("ep0 read"); break; }
		printf("event %d\n", ev.type); fflush(stdout);
		if (ev.type == FUNCTIONFS_ENABLE && !started) {
			started = 1;
			pthread_create(&t, NULL, src_thread, NULL); pthread_create(&t, NULL, sink_thread, NULL); pthread_create(&t, NULL, stat_thread, NULL);
		} else if (ev.type == FUNCTIONFS_SETUP) {
			if (ev.u.setup.bRequestType & USB_DIR_IN) read(ep0, NULL, 0); else write(ep0, NULL, 0);
		}
	}
	return 0;
}
