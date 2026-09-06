// audio_sim.cc — "-x sim" software-channel audio backend for Iris.
//
// Mirrors Mercury's -x sim passband-socket backend (mercury/source/audioio/
// audioio.c: sim_connect_once :1611, sim_tx_bridge_thread :1690,
// sim_rx_bridge_thread :1767, SIM_CHUNK_SAMPLES :1586) so BOTH modems can run
// end-to-end through Mercury's channel relay (tools/sim_channel_relay.py) with
// ZERO relay edits. This file is PLUMBING ONLY — no FM channel model here (the
// faithful FM model is a later increment that replaces Channel.process() in the
// relay, not anything in Iris).
//
// PROTOCOL (must match audioio.c / sim_channel_relay.py exactly):
//   - Transport:   TCP over 127.0.0.1:MERCURY_SIM_PORT (default 52100),
//                  TCP_NODELAY. ONE connection per modem carries BOTH
//                  directions (this peer's TX out + its RX in).
//   - WHO connects: Iris is the CLIENT; the relay is the server that accepts
//                  exactly two peers keyed by a 1-byte role tag.
//   - Handshake:   immediately after connect, send ONE ASCII role byte
//                  'A' (commander) / 'B' (responder) from MERCURY_SIM_ROLE
//                  (default 'A').  audioio.c:1649-1653, relay :739-750.
//   - Wire format: BARE fixed chunks (default --wire-stamp 0) — 1024 mono
//                  little-endian float64 samples = 8192 bytes/chunk, no header.
//                  relay CHUNK_SAMPLES=1024 / CHUNK_BYTES=8192 / FS=48000.
//
// KEY FORMAT BRIDGE: Iris audio is float32 in-memory (AudioCallback, audio.h:24);
// the Mercury wire is float64. So this backend converts float32->float64 on TX
// and float64->float32 on RX at the socket boundary (per-sample), which lets the
// relay be reused unmodified.
//
// RE-BLOCKING: Iris opens the stream at 4800 frames/buffer (main.cc:762-765) but
// the wire chunk is 1024 samples. Each thread keeps a small staging buffer that
// re-blocks between the 4800-frame callback and the 1024-sample wire packet.
//
// SHARED SOCKET: SimCapture (RX) and SimPlayback (TX) are separate objects but
// share ONE TCP connection and do the role handshake exactly once, via the
// idempotent sim_connect_once() below (mirrors audioio.c:1611).
//
// PACING/CLOCK: Iris has no sim virtual clock (sim_clock is Mercury-only). This
// backend runs wall-clock (real-time) paced like the loopback backend. That is
// fine for BARE mode + a plumbing smoke test, but it is a KNOWN fidelity gap:
// turnaround timing is host-speed-dependent, unlike Mercury's --wire-stamp
// virtual-clock path. Do NOT enable --wire-stamp against this backend (it has no
// stamp coupling). Idle silence is emitted naturally: modem.process_tx fills the
// TX buffer with silence when there is nothing to send, so the relay's
// per-direction clock keeps advancing (mirrors the audioio.c:1720-1750 intent).

#include "audio/audio.h"

#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>

#if defined(_WIN32)
  #include <winsock2.h>
  #include <ws2tcpip.h>
  typedef SOCKET sim_sock_t;
  #define SIM_BAD_SOCK INVALID_SOCKET
  #define SIM_CLOSESOCK closesocket
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <netinet/tcp.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  typedef int sim_sock_t;
  #define SIM_BAD_SOCK (-1)
  #define SIM_CLOSESOCK ::close
#endif

namespace iris {

// Wire chunk geometry — MUST equal the relay's CHUNK_SAMPLES=1024 / float64
// (sim_channel_relay.py:103-104). 1024 doubles = 8192 bytes ~= 21.3 ms @48 kHz.
static constexpr int SIM_CHUNK_SAMPLES = 1024;
static constexpr int SIM_CHUNK_BYTES   = SIM_CHUNK_SAMPLES * (int)sizeof(double);

static void sim_store_f64_le(uint8_t* dst, double value) {
    uint64_t bits;
    std::memcpy(&bits, &value, sizeof(bits));
    for (int i = 0; i < 8; i++)
        dst[i] = (uint8_t)(bits >> (i * 8));
}

static double sim_load_f64_le(const uint8_t* src) {
    uint64_t bits = 0;
    for (int i = 0; i < 8; i++)
        bits |= (uint64_t)src[i] << (i * 8);
    double value;
    std::memcpy(&value, &bits, sizeof(value));
    return value;
}

// Compiled-in guard marker (mirrors audioio.c:97). A harness can grep the
// running binary's stdout for this to confirm it is a sim-capable build; a stale
// binary that does not understand "-x sim" never prints it (it falls through to
// real devices, which the guard below then refuses to open).
static const char* SIM_AUDIO_GUARD_MARKER = "[SIM-AUDIO-GUARD]";

// Latched true the moment a sim backend opens. Real device backends check this
// and refuse to open (mirrors the GUARD-1 abort at audioio.c:550/967), so a
// misconfigured run cannot leak modem tones to a real sound card.
static std::atomic<bool> g_sim_guard_active{false};
bool sim_audio_guard_active() { return g_sim_guard_active.load(); }

// ---------------------------------------------------------------------------
// Shared single TCP connection (both directions ride this one socket).
// ---------------------------------------------------------------------------
static std::mutex        g_sim_conn_mutex;
static sim_sock_t        g_sim_sock = SIM_BAD_SOCK;
static std::atomic<bool> g_sim_connected{false};
static std::atomic<bool> g_sim_failed{false};

static int sim_send_all(sim_sock_t s, const uint8_t* buf, int len) {
    int sent = 0;
    while (sent < len) {
        int n = (int)send(s, (const char*)(buf + sent), len - sent, 0);
        if (n <= 0) return -1;
        sent += n;
    }
    return 0;
}

static int sim_recv_all(sim_sock_t s, uint8_t* buf, int len) {
    int got = 0;
    while (got < len) {
        int n = (int)recv(s, (char*)(buf + got), len - got, 0);
        if (n <= 0) return -1;
        got += n;
    }
    return 0;
}

// Establish the single shared TCP connection + role handshake (idempotent,
// mirrors sim_connect_once, audioio.c:1611). Safe to call from either bridge
// thread; the first caller connects, the rest return the cached result.
static bool sim_connect_once() {
    std::lock_guard<std::mutex> lock(g_sim_conn_mutex);
    if (g_sim_connected.load()) return true;
    if (g_sim_failed.load())    return false;

#if defined(_WIN32)
    // main() already called WSAStartup; ensure it in case of standalone use.
    static bool wsa_started = false;
    if (!wsa_started) { WSADATA w; WSAStartup(MAKEWORD(2, 2), &w); wsa_started = true; }
#endif

    const char* port_s = getenv("MERCURY_SIM_PORT");
    const char* role_s = getenv("MERCURY_SIM_ROLE");
    int  port = port_s ? atoi(port_s) : 52100;
    char role = (role_s && role_s[0]) ? role_s[0] : 'A';

    sim_sock_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == SIM_BAD_SOCK) {
        printf("[SIM] socket() failed\n"); fflush(stdout);
        g_sim_failed.store(true);
        return false;
    }
    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));

    struct sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port   = htons((unsigned short)port);
    addr.sin_addr.s_addr = inet_addr("127.0.0.1");

    // Retry ~20 s: the relay or the peer may start a moment after us.
    int attempts = 0;
    while (connect(s, (struct sockaddr*)&addr, sizeof(addr)) != 0) {
        if (++attempts > 200) {
            printf("[SIM] connect to 127.0.0.1:%d failed after %d attempts\n",
                   port, attempts);
            fflush(stdout);
            SIM_CLOSESOCK(s);
            g_sim_failed.store(true);
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // Send the 1-byte role tag so the relay can cross-wire the two peers.
    if (sim_send_all(s, (const uint8_t*)&role, 1) != 0) {
        printf("[SIM] role handshake send failed\n"); fflush(stdout);
        SIM_CLOSESOCK(s);
        g_sim_failed.store(true);
        return false;
    }

    g_sim_sock = s;
    g_sim_connected.store(true);
    printf("[SIM] connected to channel relay 127.0.0.1:%d as role '%c'\n",
           port, role);
    fflush(stdout);
    return true;
}

// Unblock a bridge thread blocked in recv()/send() on stop() by shutting down
// the shared socket in both directions (idempotent).
static void sim_shutdown_socket() {
    std::lock_guard<std::mutex> lock(g_sim_conn_mutex);
    if (g_sim_sock != SIM_BAD_SOCK) {
#if defined(_WIN32)
        shutdown(g_sim_sock, SD_BOTH);
#else
        shutdown(g_sim_sock, SHUT_RDWR);
#endif
    }
}

// ---------------------------------------------------------------------------
// SimPlayback (TX bridge): drain modem.process_tx passband -> relay.
// ---------------------------------------------------------------------------
class SimPlayback : public AudioPlayback {
public:
    bool open(int, int sample_rate, int, int frames_per_buffer) override {
        sample_rate_ = sample_rate > 0 ? sample_rate : 48000;
        buf_size_    = frames_per_buffer > 0 ? frames_per_buffer : 4800;
        g_sim_guard_active.store(true);
        printf("[SIM] software channel backend active (no audio device) %s\n",
               SIM_AUDIO_GUARD_MARKER);
        fflush(stdout);
        return true;
    }
    bool start() override {
        running_ = true;
        thread_  = std::thread(&SimPlayback::run, this);
        return true;
    }
    bool stop() override {
        running_ = false;
        sim_shutdown_socket();
        if (thread_.joinable()) thread_.join();
        return true;
    }
    void close() override { stop(); }
    bool is_running() const override { return running_; }
    float volume() const override { return 1.0f; }
    void set_volume(float) override {}
    void set_callback(AudioCallback cb) override { cb_ = cb; }
    int write(const float* buffer, int frame_count) override {
        if (!buffer || frame_count <= 0 || !sim_connect_once()) return 0;

        write_stage_.insert(write_stage_.end(), buffer, buffer + frame_count);
        uint8_t chunk[SIM_CHUNK_BYTES];
        size_t off = 0;
        while (write_stage_.size() - off >= (size_t)SIM_CHUNK_SAMPLES) {
            for (int i = 0; i < SIM_CHUNK_SAMPLES; i++)
                sim_store_f64_le(chunk + i * sizeof(double),
                                 (double)write_stage_[off + i]);
            if (sim_send_all(g_sim_sock, chunk, SIM_CHUNK_BYTES) != 0) {
                write_stage_.clear();
                return 0;
            }
            off += SIM_CHUNK_SAMPLES;
        }
        if (off > 0)
            write_stage_.erase(write_stage_.begin(), write_stage_.begin() + off);
        return frame_count;
    }

private:
    void run() {
        if (!sim_connect_once()) { running_ = false; return; }

        std::vector<float>  buf(buf_size_);
        std::vector<double> stage;                 // f64 samples pending ship
        stage.reserve((size_t)buf_size_ + SIM_CHUNK_SAMPLES);
        uint8_t chunk[SIM_CHUNK_BYTES];

        const auto block_duration = std::chrono::microseconds(
            (long long)buf_size_ * 1000000 / sample_rate_);
        auto next_block = std::chrono::steady_clock::now();
        while (running_) {
            // Fill from the modem. process_tx writes silence when idle, which
            // keeps the relay's per-direction clock advancing (RF realism).
            std::memset(buf.data(), 0, (size_t)buf_size_ * sizeof(float));
            if (cb_) cb_(buf.data(), buf_size_, 1);

            // float32 -> float64, append to staging buffer.
            for (int i = 0; i < buf_size_; i++)
                stage.push_back((double)buf[i]);

            // Ship every full 1024-double chunk; keep the sub-chunk remainder.
            size_t off = 0;
            while (stage.size() - off >= (size_t)SIM_CHUNK_SAMPLES) {
                for (int i = 0; i < SIM_CHUNK_SAMPLES; i++)
                    sim_store_f64_le(chunk + i * sizeof(double),
                                     stage[off + i]);
                if (sim_send_all(g_sim_sock, chunk,
                                 SIM_CHUNK_BYTES) != 0) {
                    if (running_)
                        printf("[SIM] TX bridge send failed (relay closed?)\n");
                    running_ = false;
                    break;
                }
                off += SIM_CHUNK_SAMPLES;
            }
            if (off > 0)
                stage.erase(stage.begin(), stage.begin() + off);

            // Real-time self-pace (like loopback). ~100 ms per 4800-frame block.
            next_block += block_duration;
            std::this_thread::sleep_until(next_block);
        }
    }

    int sample_rate_ = 48000;
    int buf_size_    = 4800;
    std::atomic<bool> running_{false};
    AudioCallback cb_;
    std::thread thread_;
    std::vector<float> write_stage_;
};

// ---------------------------------------------------------------------------
// SimCapture (RX bridge): relay passband -> modem.process_rx.
// ---------------------------------------------------------------------------
class SimCapture : public AudioCapture {
public:
    bool open(int, int sample_rate, int, int frames_per_buffer) override {
        sample_rate_ = sample_rate > 0 ? sample_rate : 48000;
        buf_size_    = frames_per_buffer > 0 ? frames_per_buffer : 4800;
        g_sim_guard_active.store(true);
        return true;
    }
    bool start() override {
        running_ = true;
        thread_  = std::thread(&SimCapture::run, this);
        return true;
    }
    bool stop() override {
        running_ = false;
        sim_shutdown_socket();
        if (thread_.joinable()) thread_.join();
        return true;
    }
    void close() override { stop(); }
    bool is_running() const override { return running_; }
    float volume() const override { return 1.0f; }
    void set_volume(float) override {}
    void set_callback(AudioCallback cb) override { cb_ = cb; }
    int read(float* buffer, int frame_count) override {
        if (!buffer || frame_count <= 0 || !sim_connect_once()) return 0;

        uint8_t chunk[SIM_CHUNK_BYTES];
        while (read_stage_.size() < (size_t)frame_count) {
            if (sim_recv_all(g_sim_sock, chunk, SIM_CHUNK_BYTES) != 0)
                return 0;
            for (int i = 0; i < SIM_CHUNK_SAMPLES; i++)
                read_stage_.push_back((float)sim_load_f64_le(
                    chunk + i * sizeof(double)));
        }
        std::memcpy(buffer, read_stage_.data(),
                    (size_t)frame_count * sizeof(float));
        read_stage_.erase(read_stage_.begin(),
                          read_stage_.begin() + frame_count);
        return frame_count;
    }

private:
    void run() {
        if (!sim_connect_once()) { running_ = false; return; }

        std::vector<float> out(buf_size_);
        std::vector<float> stage;                  // f32 samples pending deliver
        stage.reserve((size_t)buf_size_ + SIM_CHUNK_SAMPLES);
        uint8_t chunk[SIM_CHUNK_BYTES];

        while (running_) {
            // recv blocks -> the relay's forwarding rate paces us naturally.
            if (sim_recv_all(g_sim_sock, (uint8_t*)chunk, SIM_CHUNK_BYTES) != 0) {
                if (running_)
                    printf("[SIM] RX bridge recv failed (relay closed?)\n");
                running_ = false;
                break;
            }
            // float64 -> float32, append to staging buffer.
            for (int i = 0; i < SIM_CHUNK_SAMPLES; i++)
                stage.push_back((float)sim_load_f64_le(
                    chunk + i * sizeof(double)));

            // Deliver in Iris's native 4800-frame callback blocks.
            while (stage.size() >= (size_t)buf_size_) {
                std::memcpy(out.data(), stage.data(),
                            (size_t)buf_size_ * sizeof(float));
                if (cb_) cb_(out.data(), buf_size_, 1);
                stage.erase(stage.begin(), stage.begin() + buf_size_);
            }
        }
    }

    int sample_rate_ = 48000;
    int buf_size_    = 4800;
    std::atomic<bool> running_{false};
    AudioCallback cb_;
    std::thread thread_;
    std::vector<float> read_stage_;
};

std::unique_ptr<AudioCapture> create_sim_capture() {
    return std::make_unique<SimCapture>();
}

std::unique_ptr<AudioPlayback> create_sim_playback() {
    return std::make_unique<SimPlayback>();
}

} // namespace iris
