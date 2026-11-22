// Host side: libusb async bulk throughput test against the P501 FunctionFS interface.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <libusb-1.0/libusb.h>
#define NXFER 8
#define XSZ (512 * 1024)
static unsigned char EPIN, EPOUT;
static uint64_t bytes; static int running = 1, fails;
static void LIBUSB_CALL cb(struct libusb_transfer *t)
{
	if (t->status == LIBUSB_TRANSFER_COMPLETED) bytes += t->actual_length; else fails++;
	if (running && fails < 10) { if (libusb_submit_transfer(t)) fails++; }
}
static double now(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
int main(int argc, char **argv)
{
	int vid = strtol(argv[1], 0, 16), pid = strtol(argv[2], 0, 16); double secs = argc > 3 ? atof(argv[3]) : 5;
	libusb_context *c; libusb_init(&c);
	libusb_device_handle *h = libusb_open_device_with_vid_pid(c, vid, pid);
	if (!h) { fprintf(stderr, "device not found / no permission\n"); return 1; }
	struct libusb_config_descriptor *cfg; libusb_get_active_config_descriptor(libusb_get_device(h), &cfg);
	int ifn = -1; for (int i = 0; i < cfg->bNumInterfaces; i++) if (cfg->interface[i].altsetting[0].bInterfaceClass == 0xff) ifn = i;
	if (ifn < 0) { fprintf(stderr, "no vendor interface\n"); return 1; }
	const struct libusb_interface_descriptor *id = &cfg->interface[ifn].altsetting[0];
	printf("interface %d, speed=%d (3=high 4=super)\n", ifn, libusb_get_device_speed(libusb_get_device(h)));
	for (int e = 0; e < id->bNumEndpoints; e++) printf("  ep 0x%02x maxpkt %d\n", id->endpoint[e].bEndpointAddress, id->endpoint[e].wMaxPacketSize);
	for (int e = 0; e < id->bNumEndpoints; e++) { unsigned char a = id->endpoint[e].bEndpointAddress; if (a & 0x80) EPIN = a; else EPOUT = a; }
	if (libusb_claim_interface(h, ifn)) { fprintf(stderr, "claim failed\n"); return 1; }
	for (int dir = 0; dir < 2; dir++) {
		unsigned char ep = dir == 0 ? EPIN : EPOUT; bytes = 0; running = 1; fails = 0;
		struct libusb_transfer *t[NXFER]; unsigned char *buf[NXFER];
		for (int i = 0; i < NXFER; i++) { buf[i] = calloc(1, XSZ); t[i] = libusb_alloc_transfer(0);
			libusb_fill_bulk_transfer(t[i], h, ep, buf[i], XSZ, cb, NULL, 0); libusb_submit_transfer(t[i]); }
		double t0 = now(), tend = t0 + secs; uint64_t b0 = 0;
		while (now() < tend) { struct timeval tv = {0, 100000}; libusb_handle_events_timeout(c, &tv);
			if (now() - t0 > 1.0 && !b0) { b0 = bytes; t0 = now(); } }
		double dt = now() - t0; uint64_t b = bytes - b0; running = 0;
		printf("%s ep 0x%02x: %.1f MB/s (%.2f Gb/s) over %.1fs, fails=%d\n", dir == 0 ? "device->host" : "host->device", ep, b / dt / 1e6, b * 8 / dt / 1e9, dt, fails);
		for (int i = 0; i < 5; i++) { struct timeval tv = {0, 100000}; libusb_handle_events_timeout(c, &tv); }
	}
	libusb_release_interface(h, ifn); libusb_close(h); libusb_exit(c); return 0;
}
