// Standalone host->board USB OUT throughput test for the P501 stream gadget (timed TX messages with start=0 are dropped by the board, so only the USB/reader path is measured).
#include <libusb-1.0/libusb.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <chrono>
#include <vector>
struct hdr_t { uint32_t magic, hdr_len; uint64_t seq; uint32_t payload_len, flags; uint64_t chunk; uint32_t overruns, slot_off; uint8_t pad[16]; } __attribute__((packed));
static const uint32_t MAGIC = 0x31303550, HF_TX_TIMED = 4, HF_END = 0x80000000u;
static volatile bool stopping = false; static uint64_t bytes = 0, msgs = 0, seqn = 0; static int inflight = 0, fails = 0;
static unsigned char epout = 0; static int ifn = -1;
static void LIBUSB_CALL cb(libusb_transfer* t)
{
    inflight--;
    if (t->status != LIBUSB_TRANSFER_COMPLETED || t->actual_length != t->length) { fails++; if (fails < 5) fprintf(stderr, "xfer status %d actual %d/%d\n", t->status, t->actual_length, t->length); }
    else { bytes += t->length; msgs++; }
    if (!stopping && !fails) { ((hdr_t*)t->buffer)->seq = seqn++; if (libusb_submit_transfer(t)) fails++; else inflight++; }
}
int main(int argc, char** argv)
{
    size_t frames = argc > 1 ? atoi(argv[1]) : 32768; int depth = argc > 2 ? atoi(argv[2]) : 12; double secs = argc > 3 ? atof(argv[3]) : 4; int mask = argc > 4 ? atoi(argv[4]) : 3;
    size_t fb = mask == 3 ? 8 : 4, pl = frames * fb;
    libusb_context* ctx; libusb_init(&ctx);
    libusb_device_handle* h = libusb_open_device_with_vid_pid(ctx, 0x1209, 0x0002);
    if (!h) { fprintf(stderr, "no device\n"); return 1; }
    libusb_config_descriptor* cfg; libusb_get_active_config_descriptor(libusb_get_device(h), &cfg);
    for (int i = 0; i < cfg->bNumInterfaces; i++) { const libusb_interface_descriptor* d = &cfg->interface[i].altsetting[0]; if (d->bInterfaceClass != 0xff) continue; ifn = i; for (int e = 0; e < d->bNumEndpoints; e++) if (!(d->endpoint[e].bEndpointAddress & 0x80)) epout = d->endpoint[e].bEndpointAddress; }
    if (ifn < 0 || libusb_claim_interface(h, ifn)) { fprintf(stderr, "claim failed\n"); return 1; }
    if (libusb_control_transfer(h, 0x41, 3, 1 | (mask << 4), ifn, nullptr, 0, 2000) < 0) { fprintf(stderr, "TX_START failed\n"); return 1; }
    std::vector<libusb_transfer*> ts;
    for (int i = 0; i < depth; i++) {
        unsigned char* b = (unsigned char*)calloc(1, sizeof(hdr_t) + pl); hdr_t* hd = (hdr_t*)b;
        hd->magic = MAGIC; hd->hdr_len = sizeof *hd; hd->payload_len = pl; hd->flags = HF_TX_TIMED; hd->chunk = 0; hd->seq = seqn++;
        libusb_transfer* t = libusb_alloc_transfer(0); libusb_fill_bulk_transfer(t, h, epout, b, sizeof(hdr_t) + pl, cb, nullptr, 0); ts.push_back(t);
        if (libusb_submit_transfer(t)) { fprintf(stderr, "submit failed (usbfs limit?)\n"); return 1; } inflight++;
    }
    auto now = [] { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
    double t0 = now(), tw = t0 + 1.0, tend = t0 + 1.0 + secs; uint64_t b0 = 0, m0 = 0; bool armed = false;
    while (now() < tend && !fails) {
        timeval tv = { 0, 20000 }; libusb_handle_events_timeout(ctx, &tv);
        if (!armed && now() >= tw) { armed = true; b0 = bytes; m0 = msgs; tw = now(); }
    }
    double dt = now() - tw; uint64_t db = bytes - b0, dm = msgs - m0;
    stopping = true; while (inflight > 0) { timeval tv = { 0, 50000 }; libusb_handle_events_timeout(ctx, &tv); }
    hdr_t e; memset(&e, 0, sizeof e); e.magic = MAGIC; e.hdr_len = sizeof e; e.flags = HF_END; int n = 0;
    libusb_bulk_transfer(h, epout, (unsigned char*)&e, sizeof e, &n, 1000); libusb_control_transfer(h, 0x41, 4, 0, ifn, nullptr, 0, 2000);
    printf("frames %zu (%zu KiB) depth %d mask %d: %.1f MB/s  %.0f msg/s  %.3f ms/msg  fails %d\n", frames, (pl + 56) / 1024, depth, mask, db / dt / 1e6, dm / dt, dt / dm * 1e3, fails);
    libusb_release_interface(h, ifn); libusb_close(h); libusb_exit(ctx); return 0;
}
