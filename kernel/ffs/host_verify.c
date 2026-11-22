// Host: read the P501 stream over libusb, check message sequence, overrun counter and BIST-tone phase continuity (ch1).
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include <time.h>
#include <libusb-1.0/libusb.h>
#define NX 8
#define XSZ (1024 * 1024)
struct hdr { uint32_t magic, hdr_len; uint64_t seq; uint32_t payload_len, flags; uint64_t chunk; uint32_t overruns, slot_off; uint8_t pad[16]; } __attribute__((packed));
static uint64_t msgs, bytes, bad_hdr, lost_msgs, seq_next, have_seq, ovr_msgs, tone_bad, tone_n, first_bad = ~0ULL;
static int nprint; static uint32_t last_overruns; static int running = 1, fails, have_prev; static double pI, pQ, rr, ri; static int rot_known;
static void LIBUSB_CALL cb(struct libusb_transfer *t)
{
	if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length > (int)sizeof(struct hdr)) {
		struct hdr *h = (struct hdr *)t->buffer; int len = t->actual_length; bytes += len;
		if (h->magic != 0x31303550) bad_hdr++;
		else {
			msgs++;
			if (have_seq && h->seq != seq_next) { lost_msgs += h->seq - seq_next; have_prev = 0; }
			have_seq = 1; seq_next = h->seq + 1;
			if (h->overruns != last_overruns) { ovr_msgs++; last_overruns = h->overruns; have_prev = 0; }
			int16_t *s = (int16_t *)(t->buffer + sizeof *h); int ns = (len - sizeof *h) / 8;
			for (int i = 0; i < ns; i++) {
				double I = s[4 * i], Q = s[4 * i + 1];
				if (have_prev && I * I + Q * Q > 1e4) {
					double xr = I * pI + Q * pQ, xi = Q * pI - I * pQ;          // x1 * conj(x0)
					double m = sqrt(xr * xr + xi * xi);
					if (!rot_known && m > 0) { rr = xr / m; ri = xi / m; rot_known = 1; printf("rotation/sample = %.4f rad (expect +-0.1963 = 2*pi/32)\n", atan2(ri, rr)); }
					else if (rot_known) {
						double er = I - (pI * rr - pQ * ri), ei = Q - (pI * ri + pQ * rr);
						tone_n++; if (er * er + ei * ei > 0.04 * (pI * pI + pQ * pQ)) { tone_bad++; if (nprint < 20) { nprint++; printf("BREAK seq=%llu slot_off=%u sample_in_msg=%d err=%.2f\n", (unsigned long long)h->seq, h->slot_off, i, sqrt((er*er+ei*ei)/(pI*pI+pQ*pQ))); } }
					}
				}
				pI = I; pQ = Q; have_prev = 1;
			}
		}
	} else if (t->status != LIBUSB_TRANSFER_COMPLETED) fails++;
	if (running && fails < 20) libusb_submit_transfer(t);
}
static double now(void) { struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts); return ts.tv_sec + ts.tv_nsec * 1e-9; }
int main(int argc, char **argv)
{
	int vid = strtol(argv[1], 0, 16), pid = strtol(argv[2], 0, 16); double secs = argc > 3 ? atof(argv[3]) : 10;
	libusb_context *c; libusb_init(&c); libusb_device_handle *h = libusb_open_device_with_vid_pid(c, vid, pid);
	if (!h) { fprintf(stderr, "no device\n"); return 1; }
	struct libusb_config_descriptor *cfg; libusb_get_active_config_descriptor(libusb_get_device(h), &cfg);
	int ifn = -1; unsigned char epin = 0;
	for (int i = 0; i < cfg->bNumInterfaces; i++) { const struct libusb_interface_descriptor *d = &cfg->interface[i].altsetting[0];
		if (d->bInterfaceClass == 0xff) { ifn = i; for (int e = 0; e < d->bNumEndpoints; e++) if (d->endpoint[e].bEndpointAddress & 0x80) epin = d->endpoint[e].bEndpointAddress; } }
	if (ifn < 0 || libusb_claim_interface(h, ifn)) { fprintf(stderr, "claim failed\n"); return 1; }
	struct libusb_transfer *t[NX];
	for (int i = 0; i < NX; i++) { t[i] = libusb_alloc_transfer(0); libusb_fill_bulk_transfer(t[i], h, epin, malloc(XSZ), XSZ, cb, NULL, 0); libusb_submit_transfer(t[i]); }
	double t0 = now(), tl = t0; uint64_t bl = 0;
	while (now() - t0 < secs) { struct timeval tv = {0, 100000}; libusb_handle_events_timeout(c, &tv);
		if (now() - tl >= 1.0) { printf("%.1f MB/s msgs=%llu lost=%llu ovr_msgs=%llu tone_bad=%llu/%llu bad_hdr=%llu\n", (bytes - bl) / (now() - tl) / 1e6, (unsigned long long)msgs, (unsigned long long)lost_msgs, (unsigned long long)ovr_msgs, (unsigned long long)tone_bad, (unsigned long long)tone_n, (unsigned long long)bad_hdr); fflush(stdout); tl = now(); bl = bytes; } }
	running = 0;
	double dt = now() - t0;
	printf("TOTAL %.1f MB in %.1fs = %.1f MB/s; msgs=%llu lost_msgs=%llu overrun_events=%llu tone_bad=%llu of %llu samples checked; bad_hdr=%llu fails=%d\n",
	       bytes / 1e6, dt, bytes / dt / 1e6, (unsigned long long)msgs, (unsigned long long)lost_msgs, (unsigned long long)ovr_msgs, (unsigned long long)tone_bad, (unsigned long long)tone_n, (unsigned long long)bad_hdr, fails);
	return 0;
}
