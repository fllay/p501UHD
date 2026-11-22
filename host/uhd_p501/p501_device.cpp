//
// uhd_p501 -- UHD 4.6 out-of-tree device module for the PZSDR P501 (XCZU5EV + AD9361, 2R2T).
//
//   control : ASCII line protocol over TCP to the board daemon p501d (default 192.168.2.1:5555, tools/p501d.py)
//   samples : USB3 bulk through the board's FunctionFS gadget (libusb); message = 56 B header + 512 KiB payload of
//             65536 frames {I1,Q1,I2,Q2} int16 (kernel/ffs/p501_stream.c)
//   v1      : one rate (30.72 MS/s 2R2T); no hardware time (time = host clock, timed commands are not honoured).
//
#include <libusb-1.0/libusb.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
#include <uhd/device.hpp>
#include <uhd/exception.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/stream_cmd.hpp>
#include <uhd/types/time_spec.hpp>
#include <uhd/usrp/dboard_eeprom.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/usrp/mboard_eeprom.hpp>
#include <uhd/usrp/subdev_spec.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/static.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <sstream>
#include <thread>

using namespace uhd;

namespace p501 {

static const uint16_t USB_VID = 0x1209, USB_PID = 0x0002;
static const char* DEFAULT_ADDR = "192.168.2.1";
static const int DEFAULT_PORT   = 5555;
static const double FS          = 30.72e6;
static const size_t FRAMES      = 65536;                 // frames per USB message
static const size_t PAYLOAD     = FRAMES * 8;            // 512 KiB
static const uint32_t MAGIC     = 0x31303550;
static const uint32_t HF_OVR = 1, HF_HWOVF = 2, HF_END = 0x80000000u;
enum { REQ_RX_START = 1, REQ_RX_STOP = 2, REQ_TX_START = 3, REQ_TX_STOP = 4 };

#define LOGW(x) UHD_LOG_WARNING("P501", x)

struct hdr_t {
    uint32_t magic, hdr_len;
    uint64_t seq;
    uint32_t payload_len, flags;
    uint64_t chunk;
    uint32_t overruns, slot_off;
    uint8_t pad[16];
} __attribute__((packed));
static_assert(sizeof(hdr_t) == 56, "header size (as the board writes it)");

static double steady_s()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static std::vector<std::string> split_ws(const std::string& s)
{
    std::istringstream is(s);
    std::vector<std::string> v;
    std::string t;
    while (is >> t)
        v.push_back(t);
    return v;
}

// ---------------------------------------------------------------------------
// control connection: one request line -> "OK tokens" / "ERR message"
// ---------------------------------------------------------------------------
class ctrl_t
{
public:
    ctrl_t(const std::string& addr, int port, double timeout_s)
    {
        struct addrinfo hints = {}, *res = nullptr;
        hints.ai_family   = AF_INET;
        hints.ai_socktype = SOCK_STREAM;
        if (getaddrinfo(addr.c_str(), std::to_string(port).c_str(), &hints, &res) != 0 || !res)
            throw uhd::io_error("p501: cannot resolve " + addr);
        _fd = socket(AF_INET, SOCK_STREAM, 0);
        int one = 1;
        setsockopt(_fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof one);
        const int fl = ::fcntl(_fd, F_GETFL, 0);
        ::fcntl(_fd, F_SETFL, fl | O_NONBLOCK);
        int r = ::connect(_fd, res->ai_addr, res->ai_addrlen);
        freeaddrinfo(res);
        if (r < 0) {
            fd_set w;
            FD_ZERO(&w);
            FD_SET(_fd, &w);
            timeval tv = {(time_t)timeout_s, (suseconds_t)((timeout_s - (time_t)timeout_s) * 1e6)};
            int err = 0;
            socklen_t l = sizeof err;
            if (select(_fd + 1, nullptr, &w, nullptr, &tv) <= 0
                || getsockopt(_fd, SOL_SOCKET, SO_ERROR, &err, &l) != 0 || err) {
                ::close(_fd);
                throw uhd::io_error("p501: no daemon at " + addr + ":" + std::to_string(port));
            }
        }
        ::fcntl(_fd, F_SETFL, fl);
        timeval rt = {60, 0}; // a retune takes a few seconds
        setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &rt, sizeof rt);
    }
    ~ctrl_t() { ::close(_fd); }

    std::vector<std::string> call(const std::string& line)
    {
        std::lock_guard<std::mutex> l(_m);
        std::string out = line + "\n";
        if (::send(_fd, out.data(), out.size(), MSG_NOSIGNAL) != (ssize_t)out.size())
            throw uhd::io_error("p501: control connection lost");
        std::string reply;
        char c;
        while (true) {
            ssize_t n = ::recv(_fd, &c, 1, 0);
            if (n <= 0)
                throw uhd::io_error("p501: control connection lost (reading reply to '" + line + "')");
            if (c == '\n')
                break;
            reply += c;
        }
        auto t = split_ws(reply);
        if (t.empty() || t[0] != "OK")
            throw uhd::runtime_error("p501: '" + line + "' failed: " + reply);
        t.erase(t.begin());
        return t;
    }
    double call_double(const std::string& line)
    {
        auto t = call(line);
        if (t.empty())
            throw uhd::runtime_error("p501: empty reply to " + line);
        return std::stod(t[0]);
    }

private:
    int _fd;
    std::mutex _m;
};

// ---------------------------------------------------------------------------
// USB link
// ---------------------------------------------------------------------------
class usb_t
{
public:
    usb_t()
    {
        if (libusb_init(&_ctx))
            throw uhd::io_error("p501: libusb_init failed");
        _h = libusb_open_device_with_vid_pid(_ctx, USB_VID, USB_PID);
        if (!_h)
            throw uhd::io_error("p501: USB device 1209:0002 not found (is p501d running? permissions?)");
        libusb_config_descriptor* cfg = nullptr;
        libusb_get_active_config_descriptor(libusb_get_device(_h), &cfg);
        for (int i = 0; cfg && i < cfg->bNumInterfaces; i++) {
            const libusb_interface_descriptor* d = &cfg->interface[i].altsetting[0];
            if (d->bInterfaceClass != 0xff)
                continue;
            _ifn = i;
            for (int e = 0; e < d->bNumEndpoints; e++) {
                if (d->endpoint[e].bEndpointAddress & 0x80)
                    _epin = d->endpoint[e].bEndpointAddress;
                else
                    _epout = d->endpoint[e].bEndpointAddress;
            }
        }
        if (cfg)
            libusb_free_config_descriptor(cfg);
        if (_ifn < 0 || libusb_claim_interface(_h, _ifn))
            throw uhd::io_error("p501: cannot claim the streaming USB interface");
        _run = true;
        _thr = std::thread([this] {
            while (_run) {
                timeval tv = {0, 100000};
                libusb_handle_events_timeout(_ctx, &tv);
            }
        });
    }
    ~usb_t()
    {
        _run = false;
        _thr.join();
        libusb_release_interface(_h, _ifn);
        libusb_close(_h);
        libusb_exit(_ctx);
    }
    void ctl(int req)
    {
        int r = libusb_control_transfer(_h, 0x41, req, 0, _ifn, nullptr, 0, 2000);
        if (r < 0)
            throw uhd::io_error(std::string("p501: USB control request failed: ") + libusb_error_name(r));
    }
    libusb_device_handle* h() { return _h; }
    unsigned char epin() const { return _epin; }
    unsigned char epout() const { return _epout; }

private:
    libusb_context* _ctx = nullptr;
    libusb_device_handle* _h = nullptr;
    int _ifn = -1;
    unsigned char _epin = 0, _epout = 0;
    std::atomic<bool> _run{false};
    std::thread _thr;
};

// state shared by the device and its streamers
struct shared_t {
    std::unique_ptr<ctrl_t> ctrl;
    std::unique_ptr<usb_t> usb;
    std::mutex usb_m; // opened lazily by the first streamer
    usb_t& get_usb()
    {
        std::lock_guard<std::mutex> l(usb_m);
        if (!usb)
            usb.reset(new usb_t());
        return *usb;
    }
    std::vector<size_t> rx_map{0, 1}, tx_map{0, 1};
    std::atomic<double> time_offset{0.0};
    double now() const { return steady_s() + time_offset.load(); }
};

// ---------------------------------------------------------------------------
// RX streamer
// ---------------------------------------------------------------------------
class rx_streamer_impl : public uhd::rx_streamer
{
public:
    static constexpr int NX = 16;
    static constexpr size_t XSZ = 1024 * 1024;

    rx_streamer_impl(std::shared_ptr<shared_t> sh, const stream_args_t& a) : _sh(sh)
    {
        _fc32 = a.cpu_format == "fc32";
        if (!_fc32 && a.cpu_format != "sc16")
            throw uhd::value_error("p501: cpu format must be fc32 or sc16, got " + a.cpu_format);
        _chans = a.channels.empty() ? std::vector<size_t>{0} : a.channels;
        for (auto& c : _chans) {
            if (c >= _sh->rx_map.size())
                throw uhd::value_error("p501: RX channel out of range");
            c = _sh->rx_map[c];
        }
        usb_t& u = _sh->get_usb();
        for (int i = 0; i < NX; i++) {
            libusb_transfer* t = libusb_alloc_transfer(0);
            libusb_fill_bulk_transfer(t, u.h(), u.epin(), (unsigned char*)malloc(XSZ), (int)XSZ, &cb, this, 0);
            _xfers.push_back(t);
            submit(t);
        }
    }
    ~rx_streamer_impl() override { shutdown(); }

    size_t get_num_channels() const override { return _chans.size(); }
    size_t get_max_num_samps() const override { return FRAMES; }
    const std::vector<size_t>& chans() const { return _chans; }

    void shutdown()
    {
        if (_dead)
            return;
        try {
            stop_stream();
        } catch (...) {
        }
        _dead = true;
        for (auto* t : _xfers)
            libusb_cancel_transfer(t);
        for (int i = 0; i < 400 && _outstanding > 0; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        for (auto* t : _xfers) {
            free(t->buffer);
            libusb_free_transfer(t);
        }
        _xfers.clear();
    }

    void issue_stream_cmd(const stream_cmd_t& cmd) override
    {
        std::lock_guard<std::mutex> l(_cmd_m);
        if (!cmd.stream_now && !_warned) {
            _warned = true;
            LOGW("timed stream commands are not supported (no hardware time): streaming starts now");
        }
        switch (cmd.stream_mode) {
            case stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
                start_stream(0, false);
                break;
            case stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE:
                start_stream(cmd.num_samps, true);
                break;
            case stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE:
                start_stream(cmd.num_samps, false);
                break;
            case stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS:
                stop_stream();
                break;
        }
    }

    size_t recv(const buffs_type& buffs, const size_t nsamps, rx_metadata_t& md, const double timeout,
        const bool one_packet = false) override
    {
        md = rx_metadata_t();
        if (!_active) {
            md.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
            std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout, 0.01)));
            return 0;
        }
        size_t done = 0;
        const double tend = steady_s() + timeout;
        while (done < nsamps) {
            if (!_cur) {
                if (!next_msg(std::max(0.0, tend - steady_s()))) {
                    if (done == 0)
                        md.error_code = rx_metadata_t::ERROR_CODE_TIMEOUT;
                    break;
                }
                if (_ovf_pending) {
                    if (done == 0) {
                        _ovf_pending = false;
                        md.error_code = rx_metadata_t::ERROR_CODE_OVERFLOW;
                        md.has_time_spec = true;
                        md.time_spec = time_spec_t(_sh->now());
                        return 0;
                    }
                    break; // deliver what we have; the overflow is reported by the next call
                }
            }
            if (done == 0) {
                md.has_time_spec = true;
                md.time_spec = time_spec_t(_t0 + double(_samps_out) / FS);
            }
            size_t n = std::min(nsamps - done, _frames - _off);
            if (_finite)
                n = std::min<size_t>(n, _remaining);
            convert(buffs, done, n);
            done += n;
            _off += n;
            _samps_out += n;
            if (_off >= _frames)
                release_msg();
            if (_finite) {
                _remaining -= n;
                if (_remaining == 0) {
                    _finite = false;
                    md.end_of_burst = true;
                    if (_stop_when_done)
                        stop_stream_locked();
                    break;
                }
            }
            if (one_packet)
                break;
        }
        return done;
    }

private:
    static void LIBUSB_CALL cb(libusb_transfer* t)
    {
        auto* s = (rx_streamer_impl*)t->user_data;
        s->_outstanding--;
        if (s->_dead)
            return;
        if (t->status == LIBUSB_TRANSFER_COMPLETED && t->actual_length >= (int)sizeof(hdr_t)) {
            auto* h = (hdr_t*)t->buffer;
            if (h->magic == MAGIC && (h->flags & HF_END)) {
                {
                    std::lock_guard<std::mutex> l(s->_m);
                    s->_end = true;
                }
                s->_cv.notify_all();
                s->submit(t);
                return;
            }
            if (h->magic == MAGIC) {
                {
                    std::lock_guard<std::mutex> l(s->_m);
                    s->_q.push_back(t);
                }
                s->_cv.notify_all();
                return;
            }
        }
        if (t->status == LIBUSB_TRANSFER_CANCELLED)
            return;
        if (++s->_fails < 1000)
            s->submit(t);
    }
    void submit(libusb_transfer* t)
    {
        _outstanding++;
        if (libusb_submit_transfer(t))
            _outstanding--;
    }

    bool next_msg(double timeout)
    {
        std::unique_lock<std::mutex> l(_m);
        if (!_cv.wait_for(l, std::chrono::duration<double>(timeout), [&] { return !_q.empty(); }))
            return false;
        _cur = _q.front();
        _q.pop_front();
        l.unlock();
        auto* h = (hdr_t*)_cur->buffer;
        if (_have_seq && h->seq != _next_seq)
            _ovf_pending = true;
        if (h->flags & (HF_OVR | HF_HWOVF))
            _ovf_pending = true;
        _have_seq = true;
        _next_seq = h->seq + 1;
        _off      = 0;
        _frames   = std::min<size_t>(FRAMES, (_cur->actual_length - sizeof(hdr_t)) / 8);
        return true;
    }
    void release_msg()
    {
        libusb_transfer* t = _cur;
        _cur = nullptr;
        if (t)
            submit(t);
    }

    void convert(const buffs_type& buffs, size_t dst, size_t n)
    {
        const int16_t* src = (const int16_t*)(_cur->buffer + sizeof(hdr_t)) + 4 * _off;
        for (size_t c = 0; c < _chans.size(); c++) {
            const int16_t* s = src + 2 * _chans[c];
            if (_fc32) {
                float* d = (float*)buffs[c] + 2 * dst;
                for (size_t i = 0; i < n; i++) {
                    d[2 * i]     = float(s[4 * i]) * (1.0f / 2048.0f);
                    d[2 * i + 1] = float(s[4 * i + 1]) * (1.0f / 2048.0f);
                }
            } else {
                int16_t* d = (int16_t*)buffs[c] + 2 * dst;
                for (size_t i = 0; i < n; i++) {
                    d[2 * i]     = int16_t(s[4 * i] * 16); // the AD9361 port is 12 bit right-justified
                    d[2 * i + 1] = int16_t(s[4 * i + 1] * 16);
                }
            }
        }
    }

    void start_stream(uint64_t nsamps, bool stop_when_done)
    {
        if (_active)
            stop_stream_locked();
        drain();
        {
            std::lock_guard<std::mutex> l(_m);
            _end = false;
        }
        _have_seq = false;
        _ovf_pending = false;
        _samps_out = 0;
        _finite = nsamps > 0;
        _remaining = nsamps;
        _stop_when_done = stop_when_done;
        _t0 = _sh->now();
        _sh->get_usb().ctl(REQ_RX_START);
        _active = true;
    }
    void stop_stream()
    {
        std::lock_guard<std::mutex> l(_stop_m);
        stop_stream_locked();
    }
    void stop_stream_locked()
    {
        if (!_active)
            return;
        _active = false;
        _sh->get_usb().ctl(REQ_RX_STOP);
        // keep the transfers cycling until the board's END message arrives
        for (int i = 0; i < 400; i++) {
            drain();
            std::unique_lock<std::mutex> l(_m);
            if (_cv.wait_for(l, std::chrono::milliseconds(5), [&] { return _end; }))
                break;
        }
        drain();
    }
    void drain()
    {
        if (_cur)
            release_msg();
        std::deque<libusb_transfer*> q;
        {
            std::lock_guard<std::mutex> l(_m);
            q.swap(_q);
        }
        for (auto* t : q)
            submit(t);
    }

    std::shared_ptr<shared_t> _sh;
    bool _fc32 = true;
    std::vector<size_t> _chans;
    std::vector<libusb_transfer*> _xfers;
    std::mutex _m, _cmd_m, _stop_m;
    std::condition_variable _cv;
    std::deque<libusb_transfer*> _q;
    std::atomic<int> _outstanding{0};
    std::atomic<int> _fails{0};
    std::atomic<bool> _dead{false}, _active{false};
    bool _end = false, _warned = false;
    libusb_transfer* _cur = nullptr;
    size_t _off = 0, _frames = 0;
    bool _have_seq = false, _ovf_pending = false, _finite = false, _stop_when_done = false;
    uint64_t _next_seq = 0, _remaining = 0, _samps_out = 0;
    double _t0 = 0;
};

// ---------------------------------------------------------------------------
// TX streamer
// ---------------------------------------------------------------------------
class tx_streamer_impl : public uhd::tx_streamer
{
public:
    static constexpr int NX = 12;

    tx_streamer_impl(std::shared_ptr<shared_t> sh, const stream_args_t& a) : _sh(sh)
    {
        _fc32 = a.cpu_format == "fc32";
        if (!_fc32 && a.cpu_format != "sc16")
            throw uhd::value_error("p501: cpu format must be fc32 or sc16, got " + a.cpu_format);
        _chans = a.channels.empty() ? std::vector<size_t>{0} : a.channels;
        for (auto& c : _chans) {
            if (c >= _sh->tx_map.size())
                throw uhd::value_error("p501: TX channel out of range");
            c = _sh->tx_map[c];
        }
        usb_t& u = _sh->get_usb();
        for (int i = 0; i < NX; i++) {
            libusb_transfer* t = libusb_alloc_transfer(0);
            unsigned char* b = (unsigned char*)calloc(1, sizeof(hdr_t) + PAYLOAD);
            libusb_fill_bulk_transfer(t, u.h(), u.epout(), b, (int)(sizeof(hdr_t) + PAYLOAD), &cb, this, 0);
            _xfers.push_back(t);
            _free.push_back(t);
        }
    }
    ~tx_streamer_impl() override { shutdown(); }

    size_t get_num_channels() const override { return _chans.size(); }
    size_t get_max_num_samps() const override { return FRAMES; }
    bool recv_async_msg(async_metadata_t&, double timeout = 0.1) override
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout, 0.1)));
        return false;
    }

    void shutdown()
    {
        if (_dead)
            return;
        try {
            if (_cur && _fill > 0)
                flush();
            for (int i = 0; i < 400 && _inflight > 0; i++)
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
            if (_started) {
                hdr_t e;
                memset(&e, 0, sizeof e);
                e.magic   = MAGIC;
                e.hdr_len = sizeof e;
                e.flags   = HF_END;
                int n     = 0;
                usb_t& u  = _sh->get_usb();
                libusb_bulk_transfer(u.h(), u.epout(), (unsigned char*)&e, sizeof e, &n, 1000);
                u.ctl(REQ_TX_STOP);
            }
        } catch (...) {
        }
        _dead = true;
        for (auto* t : _xfers)
            libusb_cancel_transfer(t);
        for (int i = 0; i < 400 && _inflight > 0; i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        for (auto* t : _xfers) {
            free(t->buffer);
            libusb_free_transfer(t);
        }
        _xfers.clear();
    }

    size_t send(const buffs_type& buffs, const size_t nsamps, const tx_metadata_t& md, const double timeout = 0.1) override
    {
        if (md.has_time_spec && !_warned) {
            _warned = true;
            LOGW("timed TX bursts are not supported (no hardware time): samples are played as they arrive");
        }
        if (!_started) {
            _sh->get_usb().ctl(REQ_TX_START);
            _started = true;
        }
        const double tend = steady_s() + timeout;
        size_t done = 0;
        while (done < nsamps) {
            if (!_cur && !acquire(std::max(0.0, tend - steady_s())))
                return done;
            size_t n = std::min(nsamps - done, FRAMES - _fill);
            convert(buffs, done, n);
            done += n;
            _fill += n;
            if (_fill == FRAMES)
                flush();
        }
        if (md.end_of_burst && _cur && _fill > 0)
            flush(); // the rest of the message is already zero
        return done;
    }

private:
    static void LIBUSB_CALL cb(libusb_transfer* t)
    {
        auto* s = (tx_streamer_impl*)t->user_data;
        {
            std::lock_guard<std::mutex> l(s->_m);
            s->_inflight--;
            if (!s->_dead)
                s->_free.push_back(t);
        }
        s->_cv.notify_all();
    }
    bool acquire(double timeout)
    {
        std::unique_lock<std::mutex> l(_m);
        if (!_cv.wait_for(l, std::chrono::duration<double>(timeout), [&] { return !_free.empty(); }))
            return false;
        _cur = _free.front();
        _free.pop_front();
        l.unlock();
        memset(_cur->buffer + sizeof(hdr_t), 0, PAYLOAD);
        _fill = 0;
        return true;
    }
    void flush()
    {
        auto* h        = (hdr_t*)_cur->buffer;
        memset(h, 0, sizeof *h);
        h->magic       = MAGIC;
        h->hdr_len     = sizeof *h;
        h->seq         = _seq++;
        h->payload_len = PAYLOAD;
        libusb_transfer* t = _cur;
        _cur  = nullptr;
        _fill = 0;
        _inflight++;
        if (libusb_submit_transfer(t)) {
            _inflight--;
            std::lock_guard<std::mutex> l(_m);
            _free.push_back(t);
        }
    }
    static int16_t sat(float x)
    {
        float v = x * 32767.0f;
        return v > 32767.0f ? 32767 : (v < -32768.0f ? -32768 : int16_t(lrintf(v)));
    }
    void convert(const buffs_type& buffs, size_t src, size_t n)
    {
        int16_t* dst = (int16_t*)(_cur->buffer + sizeof(hdr_t)) + 4 * _fill;
        for (size_t c = 0; c < _chans.size(); c++) {
            int16_t* d = dst + 2 * _chans[c];
            if (_fc32) {
                const float* s = (const float*)buffs[c] + 2 * src;
                for (size_t i = 0; i < n; i++) {
                    d[4 * i]     = sat(s[2 * i]);
                    d[4 * i + 1] = sat(s[2 * i + 1]);
                }
            } else {
                const int16_t* s = (const int16_t*)buffs[c] + 2 * src;
                for (size_t i = 0; i < n; i++) {
                    d[4 * i]     = s[2 * i];
                    d[4 * i + 1] = s[2 * i + 1];
                }
            }
        }
    }

    std::shared_ptr<shared_t> _sh;
    bool _fc32 = true, _started = false, _warned = false;
    std::vector<size_t> _chans;
    std::vector<libusb_transfer*> _xfers;
    std::deque<libusb_transfer*> _free;
    std::mutex _m;
    std::condition_variable _cv;
    std::atomic<int> _inflight{0};
    std::atomic<bool> _dead{false};
    libusb_transfer* _cur = nullptr;
    size_t _fill = 0;
    uint64_t _seq = 0;
};

// ---------------------------------------------------------------------------
// device + legacy property tree
// ---------------------------------------------------------------------------
class p501_device : public uhd::device
{
public:
    explicit p501_device(const device_addr_t& args)
    {
        _sh = std::make_shared<shared_t>();
        _sh->ctrl.reset(new ctrl_t(args.get("addr", DEFAULT_ADDR), DEFAULT_PORT, 3.0));
        _info = _sh->ctrl->call("info");
        _build_tree();
    }
    ~p501_device() override
    {
        if (auto o = _rx.lock())
            o->shutdown();
        if (auto o = _tx.lock())
            o->shutdown();
    }

    rx_streamer::sptr get_rx_stream(const stream_args_t& a) override
    {
        std::lock_guard<std::mutex> l(_m);
        if (auto o = _rx.lock())
            o->shutdown();
        auto s = std::make_shared<rx_streamer_impl>(_sh, a);
        _rx    = s;
        return s;
    }
    tx_streamer::sptr get_tx_stream(const stream_args_t& a) override
    {
        std::lock_guard<std::mutex> l(_m);
        if (auto o = _tx.lock())
            o->shutdown();
        auto s = std::make_shared<tx_streamer_impl>(_sh, a);
        _tx    = s;
        return s;
    }
    bool recv_async_msg(async_metadata_t&, double timeout) override
    {
        std::this_thread::sleep_for(std::chrono::duration<double>(std::min(timeout, 0.1)));
        return false;
    }

private:
    void _build_tree();
    void _build_chan(bool rx, size_t ch);

    std::shared_ptr<shared_t> _sh;
    std::vector<std::string> _info;
    std::mutex _m;
    std::weak_ptr<rx_streamer_impl> _rx;
    std::weak_ptr<tx_streamer_impl> _tx;
};

void p501_device::_build_tree()
{
    _tree            = property_tree::make();
    auto sh          = _sh;
    const fs_path mb = "/mboards/0";
    std::string serial = "P501";
    for (auto& t : _info)
        if (t.rfind("serial=", 0) == 0)
            serial = t.substr(7);

    _tree->create<std::string>("/name").set("P501 (uhd_p501)");
    _tree->create<std::string>(mb / "name").set("P501");
    _tree->create<std::string>(mb / "codename").set("p501");
    usrp::mboard_eeprom_t ee;
    ee["name"]    = "p501";
    ee["serial"]  = serial;
    ee["product"] = "p501";
    _tree->create<usrp::mboard_eeprom_t>(mb / "eeprom").set(ee).add_coerced_subscriber(
        [](const usrp::mboard_eeprom_t&) {});
    _tree->create<std::string>(mb / "fpga_version").set("base_dmac32");
    _tree->create<std::string>(mb / "fw_version").set("p501d " + (_info.size() > 1 ? _info[1] : std::string("?")));
    _tree->create<double>(mb / "tick_rate")
        .set_coercer([](double r) {
            if (std::abs(r - FS) > 1.0)
                LOGW("master clock rate is fixed at 30.72 MHz; ignoring " << r / 1e6 << " MHz");
            return FS;
        })
        .set_publisher([] { return FS; });
    _tree->create<double>(mb / "link_max_rate").set(500e6);

    // time: the host clock (no hardware timekeeper in v1)
    _tree->create<time_spec_t>(mb / "time/now")
        .set_publisher([sh] { return time_spec_t(sh->now()); })
        .add_coerced_subscriber([sh](const time_spec_t& t) { sh->time_offset = t.get_real_secs() - steady_s(); });
    _tree->create<time_spec_t>(mb / "time/pps")
        .set_publisher([sh] { return time_spec_t(std::floor(sh->now())); })
        .add_coerced_subscriber([sh](const time_spec_t& t) { sh->time_offset = t.get_real_secs() - steady_s(); });
    _tree->create<time_spec_t>(mb / "time/cmd").set(time_spec_t(0.0));
    for (const std::string kind : {"clock", "time"}) {
        _tree->create<std::vector<std::string>>(mb / (kind + "_source") / "options")
            .set(std::vector<std::string>{"internal"});
        _tree->create<std::string>(mb / (kind + "_source") / "value")
            .set_coercer([](const std::string&) { return std::string("internal"); })
            .set("internal");
    }
    _tree->create<int>(mb / "sensors");
    _tree->create<std::string>(mb / "rx_codecs/A/name").set("AD9361 ADC");
    _tree->create<int>(mb / "rx_codecs/A/gains");
    _tree->create<std::string>(mb / "tx_codecs/A/name").set("AD9361 DAC");
    _tree->create<int>(mb / "tx_codecs/A/gains");
    _tree->create<usrp::dboard_eeprom_t>(mb / "dboards/A/rx_eeprom").set(usrp::dboard_eeprom_t());
    _tree->create<usrp::dboard_eeprom_t>(mb / "dboards/A/tx_eeprom").set(usrp::dboard_eeprom_t());
    _tree->create<usrp::dboard_eeprom_t>(mb / "dboards/A/gdb_eeprom").set(usrp::dboard_eeprom_t());
    _tree->create<usrp::dboard_iface::sptr>(mb / "dboards/A/iface")
        .set_publisher([]() -> usrp::dboard_iface::sptr {
            throw uhd::not_implemented_error("p501: no GPIO daughterboard interface");
        });
    for (size_t ch = 0; ch < 2; ch++) {
        _build_chan(true, ch);
        _build_chan(false, ch);
    }
    for (const bool rx : {true, false}) {
        _tree->create<usrp::subdev_spec_t>(mb / (rx ? "rx_subdev_spec" : "tx_subdev_spec"))
            .set_coercer([sh, rx](const usrp::subdev_spec_t& s) {
                if (s.empty())
                    throw uhd::value_error("p501: empty subdev spec");
                std::vector<size_t> m;
                for (size_t i = 0; i < s.size(); i++) {
                    long fe = (s[i].sd_name == "0") ? 0 : (s[i].sd_name == "1" ? 1 : -1);
                    if (s[i].db_name != "A" || fe < 0 || std::find(m.begin(), m.end(), size_t(fe)) != m.end())
                        throw uhd::value_error("p501: subdev spec '" + s.to_string() + "' must use distinct A:0 / A:1");
                    m.push_back(size_t(fe));
                }
                (rx ? sh->rx_map : sh->tx_map) = m;
                return s;
            })
            .set(usrp::subdev_spec_t("A:0 A:1"));
    }
}

void p501_device::_build_chan(bool rx, size_t ch)
{
    auto ctrl = _sh->ctrl.get();
    auto sh   = _sh;
    const std::string dir = rx ? "rx" : "tx";
    const std::string c   = std::to_string(ch);
    const fs_path mb      = "/mboards/0";
    const fs_path dsp     = mb / (dir + "_dsps") / c;
    const fs_path fe      = mb / "dboards/A" / (dir + "_frontends") / c;

    // ---- DSP: a fixed rate, no DDC/DUC
    _tree->create<meta_range_t>(dsp / "rate/range").set(meta_range_t(FS, FS));
    _tree->create<double>(dsp / "rate/value")
        .set_coercer([](double r) {
            if (std::abs(r - FS) > 1.0)
                LOGW("sample rate is fixed at 30.72 MS/s (2R2T); requested " << r / 1e6 << " MS/s ignored");
            return FS;
        })
        .set_publisher([] { return FS; });
    _tree->create<meta_range_t>(dsp / "freq/range").set(meta_range_t(0.0, 0.0));
    _tree->create<double>(dsp / "freq/value").set_coercer([](double) { return 0.0; }).set(0.0);
    if (rx) {
        _tree->create<stream_cmd_t>(dsp / "stream_cmd").add_coerced_subscriber([this, ch](const stream_cmd_t& cmd) {
            std::shared_ptr<rx_streamer_impl> s;
            {
                std::lock_guard<std::mutex> l(_m);
                s = _rx.lock();
            }
            if (s && !s->chans().empty() && s->chans().front() == _sh->rx_map.at(ch))
                s->issue_stream_cmd(cmd);
        });
    }

    // ---- frontend
    _tree->create<std::string>(fe / "name").set(std::string("AD9361 ") + (rx ? "RX" : "TX") + c);
    _tree->create<std::string>(fe / "connection").set("IQ");
    _tree->create<bool>(fe / "use_lo_offset").set(false);
    _tree->create<bool>(fe / "enabled").set(true);
    const meta_range_t frange(70e6, 6000e6);
    _tree->create<meta_range_t>(fe / "freq/range").set(frange);
    auto cache = std::make_shared<double>(ctrl->call_double("get_" + dir + "_freq " + c));
    auto set_rf = [ctrl, dir, c, cache](double f) {
        *cache = ctrl->call_double("set_" + dir + "_freq " + c + " " + std::to_string((long long)std::llround(f)));
        return *cache;
    };
    auto get_rf = [cache] { return *cache; };
    _tree->create<double>(fe / "freq/value").set_coercer(set_rf).set_publisher(get_rf);
    _tree->create<double>(fe / "freq" / " value").set_coercer(set_rf).set_publisher(get_rf); // UHD 4.6 quirk
    {
        const fs_path l = fe / "los" / "all";
        _tree->create<std::vector<std::string>>(l / "source/options").set(std::vector<std::string>{"internal"});
        _tree->create<std::string>(l / "source/value")
            .set_coercer([](const std::string&) { return std::string("internal"); })
            .set("internal");
        _tree->create<bool>(l / "export").set_coercer([](bool) { return false; }).set(false);
        _tree->create<double>(l / "freq/value").set_coercer(set_rf).set_publisher(get_rf);
        _tree->create<meta_range_t>(l / "freq/range").set(frange);
    }
    _tree->create<double>(fe / "bandwidth/value").set_coercer([](double) { return FS; }).set(FS);
    _tree->create<meta_range_t>(fe / "bandwidth/range").set(meta_range_t(FS, FS));

    // ---- gain
    auto gr = ctrl->call("get_" + dir + "_gain_range " + c);
    if (gr.size() < 3)
        throw uhd::runtime_error("p501: bad gain range reply");
    _tree->create<meta_range_t>(fe / "gains/PGA/range")
        .set(meta_range_t(std::stod(gr[0]), std::stod(gr[1]), std::stod(gr[2])));
    auto gcache = std::make_shared<double>(ctrl->call_double("get_" + dir + "_gain " + c));
    _tree->create<double>(fe / "gains/PGA/value")
        .set_coercer([ctrl, dir, c, gcache](double g) {
            *gcache = ctrl->call_double("set_" + dir + "_gain " + c + " " + std::to_string(g));
            return *gcache;
        })
        .set_publisher([gcache] { return *gcache; });

    // ---- antenna: one fixed port; the usual names are accepted
    const std::string ant = rx ? "RX2" : "TX/RX";
    _tree->create<std::vector<std::string>>(fe / "antenna/options")
        .set(rx ? std::vector<std::string>{"RX2", "TX/RX"} : std::vector<std::string>{"TX/RX"});
    _tree->create<std::string>(fe / "antenna/value").set_coercer([ant](const std::string&) { return ant; }).set(ant);
    _tree->create<sensor_value_t>(fe / "sensors/lo_locked").set_publisher([ctrl] {
        return sensor_value_t("LO", ctrl->call("lo_locked").at(0) == "1", "locked", "unlocked");
    });
}

// ---------------------------------------------------------------------------
// discovery
// ---------------------------------------------------------------------------
static device_addrs_t p501_find(const device_addr_t& hint)
{
    device_addrs_t found;
    const std::string type = hint.get("type", "");
    if (!type.empty() && type != "p501")
        return found;
    const std::string addr = hint.get("addr", DEFAULT_ADDR);
    try {
        ctrl_t c(addr, DEFAULT_PORT, 0.5);
        auto info = c.call("info");
        if (info.empty() || info[0] != "p501")
            return found;
        device_addr_t dev;
        dev["type"]    = "p501";
        dev["addr"]    = addr;
        dev["name"]    = "p501";
        dev["product"] = "p501";
        for (auto& t : info)
            if (t.rfind("serial=", 0) == 0)
                dev["serial"] = t.substr(7);
        if (hint.has_key("serial") && hint["serial"] != dev.get("serial", ""))
            return found;
        found.push_back(dev);
    } catch (const std::exception&) {
    }
    return found;
}

static device::sptr p501_make(const device_addr_t& args)
{
    return std::make_shared<p501_device>(args);
}

UHD_STATIC_BLOCK(register_p501_device)
{
    device::register_device(&p501_find, &p501_make, device::USRP);
}

} // namespace p501
