// Host: P501 full-duplex test. Start RX+TX streams (vendor control requests), send a continuous tone on TX
// (ch1: fs/16, ch2: fs/8), read RX and check sequence + ch1 tone continuity. Use with the AD9361 digital loopback.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <libusb-1.0/libusb.h>
#define NX 8
#define XSZ (1024 * 1024)
#define PAYLOAD (512 * 1024)
#define NTX 6
#define HF_END 0x80000000u
struct hdr { uint32_t magic, hdr_len; uint64_t seq; uint32_t payload_len, flags; uint64_t chunk; uint32_t overruns, slot_off; uint8_t pad[16]; } __attribute__((packed));
static uint64_t msgs, bytes, txbytes, bad_hdr, lost_msgs, seq_next, have_seq, hwovf_msgs, tone_bad, tone_n, tx_seq;
static uint64_t dropouts; static int ndrop; static int nprint, running = 1, fails, have_prev, rx_end, txfails; static double pI, pQ, rr, ri; static int rot_known;
static unsigned char *txpay;                                      // one 512 KiB tone payload, identical for every message
static void LIBUSB_CALL rx_cb(struct libusb_transfer *t)
{
	if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length >= (int)sizeof(struct hdr)) {
		struct hdr *h = (struct hdr *)t->buffer; int len = t->actual_length; bytes += len;
		if (h->magic != 0x31303550) bad_hdr++;
		else if (h->flags & HF_END) rx_end = 1;
		else {
			msgs++;
			if (msgs == 200) { int16_t *q = (int16_t *)(t->buffer + sizeof *h); int mx = 0; for (int i = 0; i < 65536 * 4; i++) { int a = abs(q[i]); if (a > mx) mx = a; }
				printf("DEBUG msg200 maxabs=%d ch1: %d,%d %d,%d %d,%d %d,%d  ch2: %d,%d %d,%d\n", mx, q[0], q[1], q[4], q[5], q[8], q[9], q[12], q[13], q[2], q[3], q[6], q[7]); }
			if (have_seq && h->seq != seq_next) { lost_msgs += h->seq - seq_next; have_prev = 0; }
			have_seq = 1; seq_next = h->seq + 1;
			if (h->flags & 2) { hwovf_msgs++; have_prev = 0; }
			int16_t *s = (int16_t *)(t->buffer + sizeof *h); int ns = (len - sizeof *h) / 8;
			for (int i = 0; i < ns; i++) {
				double I = s[4 * i], Q = s[4 * i + 1];
				if (rot_known && I * I + Q * Q < 900) { dropouts++; if (ndrop < 20) { ndrop++; printf("DROPOUT seq=%llu slot_off=%u sample_in_msg=%d\n", (unsigned long long)h->seq, h->slot_off, i); } }
				if (have_prev && I * I + Q * Q > 400 && pI * pI + pQ * pQ > 400) {
					double xr = I * pI + Q * pQ, xi = Q * pI - I * pQ, m = sqrt(xr * xr + xi * xi);
					if (!rot_known && m > 0) { rr = xr / m; ri = xi / m; rot_known = 1; printf("rotation/sample = %.4f rad (TX tone fs/16 = +-0.3927)\n", atan2(ri, rr)); }
					else if (rot_known) {
						double er = I - (pI * rr - pQ * ri), ei = Q - (pI * ri + pQ * rr);
						tone_n++; if (er * er + ei * ei > 0.04 * (pI * pI + pQ * pQ)) { tone_bad++; if (nprint < 20) { nprint++; printf("BREAK seq=%llu slot_off=%u sample_in_msg=%d err=%.2f\n", (unsigned long long)h->seq, h->slot_off, i, sqrt((er*er+ei*ei)/(pI*pI+pQ*pQ))); } }
					}
				}
				pI = I; pQ = Q; have_prev = 1;
			}
		}
	} else if (t->status != LIBUSB_TRANSFER_COMPLETED && running) fails++;
	if (!rx_end && fails < 20) libusb_submit_transfer(t);
}
static void LIBUSB_CALL tx_cb(struct libusb_transfer *t)
{
	if (t->status == LIBUSB_TRANSFER_COMPLETED) txbytes += t->actual_length; else if (running) txfails++;
	if (running && txfails < 20) { struct hdr *h = (struct hdr *)t->buffer; h->seq = tx_seq++; libusb_submit_transfer(t); }
}
static double now(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
static void ctl(libusb_device_handle *h, int ifn, int req) { int r = libusb_control_transfer(h, 0x41, req, 0, ifn, NULL, 0, 2000); if (r < 0) printf("control %d failed: %s\n", req, libusb_error_name(r)); }
int main(int argc, char **argv)
{
	int vid = strtol(argv[1], 0, 16), pid = strtol(argv[2], 0, 16); double secs = argc > 3 ? atof(argv[3]) : 10; int rx_only = argc > 4 && atoi(argv[4]) == 1;
	libusb_context *c; libusb_init(&c); libusb_device_handle *h = libusb_open_device_with_vid_pid(c, vid, pid);
	if (!h) { fprintf(stderr, "no device\n"); return 1; }
	struct libusb_config_descriptor *cfg; libusb_get_active_config_descriptor(libusb_get_device(h), &cfg);
	int ifn = -1; unsigned char epin = 0, epout = 0;
	for (int i = 0; i < cfg->bNumInterfaces; i++) { const struct libusb_interface_descriptor *d = &cfg->interface[i].altsetting[0];
		if (d->bInterfaceClass == 0xff) { ifn = i; for (int e = 0; e < d->bNumEndpoints; e++) { if (d->endpoint[e].bEndpointAddress & 0x80) epin = d->endpoint[e].bEndpointAddress; else epout = d->endpoint[e].bEndpointAddress; } } }
	if (ifn < 0 || libusb_claim_interface(h, ifn)) { fprintf(stderr, "claim failed\n"); return 1; }
	txpay = malloc(PAYLOAD); { int16_t *p = (int16_t *)txpay; for (int i = 0; i < PAYLOAD / 8; i++) {
		double a1 = 2 * M_PI * i / 16.0, a2 = 2 * M_PI * i / 8.0; p[4*i] = (int16_t)lrint(1000 * cos(a1)); p[4*i+1] = (int16_t)lrint(1000 * sin(a1)); p[4*i+2] = (int16_t)lrint(500 * cos(a2)); p[4*i+3] = (int16_t)lrint(500 * sin(a2)); } }
	struct libusb_transfer *t[NX];
	for (int i = 0; i < NX; i++) { t[i] = libusb_alloc_transfer(0); libusb_fill_bulk_transfer(t[i], h, epin, malloc(XSZ), XSZ, rx_cb, NULL, 0); }
	struct libusb_transfer *tt[NTX];
	for (int i = 0; i < NTX; i++) { unsigned char *b = malloc(sizeof(struct hdr) + PAYLOAD); memset(b, 0, sizeof(struct hdr)); struct hdr *hh = (struct hdr *)b;
		hh->magic = 0x31303550; hh->hdr_len = sizeof *hh; hh->payload_len = PAYLOAD; memcpy(b + sizeof *hh, txpay, PAYLOAD);
		tt[i] = libusb_alloc_transfer(0); libusb_fill_bulk_transfer(tt[i], h, epout, b, sizeof(struct hdr) + PAYLOAD, tx_cb, NULL, 0); }
	if (!rx_only) { ctl(h, ifn, 3); for (int i = 0; i < NTX; i++) { ((struct hdr *)tt[i]->buffer)->seq = tx_seq++; libusb_submit_transfer(tt[i]); } }
	ctl(h, ifn, 1);
	for (int i = 0; i < NX; i++) libusb_submit_transfer(t[i]);
	double t0 = now(), tl = t0; uint64_t bl = 0, tb = 0;
	while (now() - t0 < secs) { struct timeval tv = {0, 100000}; libusb_handle_events_timeout(c, &tv);
		if (now() - tl >= 1.0) { printf("RX %.1f MB/s TX %.1f MB/s msgs=%llu lost=%llu hwovf_msgs=%llu tone_bad=%llu/%llu bad_hdr=%llu txfail=%d\n", (bytes - bl) / (now() - tl) / 1e6, (txbytes - tb) / (now() - tl) / 1e6, (unsigned long long)msgs, (unsigned long long)lost_msgs, (unsigned long long)hwovf_msgs, (unsigned long long)tone_bad, (unsigned long long)tone_n, (unsigned long long)bad_hdr, txfails); fflush(stdout); tl = now(); bl = bytes; tb = txbytes; } }
	double dt = now() - t0;
	running = 0;
	ctl(h, ifn, 4);                                               // TX stop + END message so the board's blocked read returns
	{ struct hdr e; memset(&e, 0, sizeof e); e.magic = 0x31303550; e.hdr_len = sizeof e; e.flags = HF_END; int n; libusb_bulk_transfer(h, epout, (unsigned char *)&e, sizeof e, &n, 1000); }
	ctl(h, ifn, 2);                                               // RX stop; drain until the board's END message
	for (double te = now(); !rx_end && now() - te < 3.0; ) { struct timeval tv = {0, 100000}; libusb_handle_events_timeout(c, &tv); }
	uint64_t stv[8] = {0}; int r = libusb_control_transfer(h, 0xC1, 5, 0, ifn, (unsigned char *)stv, sizeof stv, 2000);
	printf("TOTAL RX %.1f MB (%.1f MB/s) TX %.1f MB (%.1f MB/s) in %.1fs; msgs=%llu lost=%llu hwovf_msgs=%llu tone_bad=%llu of %llu; dropout_samples=%llu; bad_hdr=%llu fails=%d txfails=%d end=%d\n",
	       bytes / 1e6, bytes / dt / 1e6, txbytes / 1e6, txbytes / dt / 1e6, dt, (unsigned long long)msgs, (unsigned long long)lost_msgs, (unsigned long long)hwovf_msgs, (unsigned long long)tone_bad, (unsigned long long)tone_n, (unsigned long long)dropouts, (unsigned long long)bad_hdr, fails, txfails, rx_end);
	if (r >= 0) printf("BOARD rx_slots=%llu rx_hwovf=%llu rx_msgs=%llu tx_filled=%llu tx_submitted=%llu tx_underflows=%llu tx_bad=%llu tx_msgs=%llu\n", (unsigned long long)stv[0], (unsigned long long)stv[1], (unsigned long long)stv[2], (unsigned long long)stv[3], (unsigned long long)stv[4], (unsigned long long)stv[5], (unsigned long long)stv[6], (unsigned long long)stv[7]);
	else printf("status request failed: %s\n", libusb_error_name(r));
	return 0;
}
