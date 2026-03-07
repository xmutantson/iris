#include "audio/audio.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstring>

namespace iris {

// Shared loopback buffer: playback writes here, capture reads from here.
// A delay is inserted so TX audio arrives at RX after TX finishes,
// bypassing the half-duplex mute (process_rx discards audio during TX).
static std::mutex g_loop_mutex;
static std::vector<float> g_loop_buffer;
static size_t g_loop_read_pos = 0;
static size_t g_loop_write_pos = 0;
static constexpr size_t LOOP_BUF_SIZE = 48000 * 8;   // 8 seconds at 48kHz mono
static constexpr size_t DELAY_SAMPLES = 48000 * 2;    // 2 second delay

void loopback_reset() {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    g_loop_buffer.assign(LOOP_BUF_SIZE, 0.0f);
    g_loop_read_pos = 0;
    g_loop_write_pos = DELAY_SAMPLES;  // Write ahead by delay amount
}

class LoopbackCapture : public AudioCapture {
public:
    bool open(int, int sample_rate, int, int frames_per_buffer) override {
        sample_rate_ = sample_rate;
        buf_size_ = frames_per_buffer;
        return true;
    }
    bool start() override {
        running_ = true;
        thread_ = std::thread(&LoopbackCapture::run, this);
        return true;
    }
    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }
    void close() override { stop(); }
    bool is_running() const override { return running_; }
    float volume() const override { return 1.0f; }
    void set_volume(float) override {}
    void set_callback(AudioCallback cb) override { cb_ = cb; }
    int read(float*, int) override { return 0; }

private:
    void run() {
        std::vector<float> buf(buf_size_);
        while (running_) {
            {
                std::lock_guard<std::mutex> lock(g_loop_mutex);
                size_t avail = (g_loop_write_pos >= g_loop_read_pos)
                    ? g_loop_write_pos - g_loop_read_pos
                    : LOOP_BUF_SIZE - g_loop_read_pos + g_loop_write_pos;
                if (avail >= (size_t)buf_size_) {
                    for (int i = 0; i < buf_size_; i++) {
                        buf[i] = g_loop_buffer[g_loop_read_pos];
                        g_loop_read_pos = (g_loop_read_pos + 1) % LOOP_BUF_SIZE;
                    }
                } else {
                    std::memset(buf.data(), 0, buf_size_ * sizeof(float));
                }
            }
            if (cb_) cb_(buf.data(), buf_size_, 1);
            // Sleep to match real-time rate (~10ms chunks)
            std::this_thread::sleep_for(
                std::chrono::microseconds(buf_size_ * 1000000 / sample_rate_));
        }
    }

    int sample_rate_ = 48000;
    int buf_size_ = 480;
    std::atomic<bool> running_{false};
    AudioCallback cb_;
    std::thread thread_;
};

class LoopbackPlayback : public AudioPlayback {
public:
    bool open(int, int sample_rate, int, int frames_per_buffer) override {
        sample_rate_ = sample_rate;
        buf_size_ = frames_per_buffer;
        return true;
    }
    bool start() override {
        running_ = true;
        thread_ = std::thread(&LoopbackPlayback::run, this);
        return true;
    }
    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }
    void close() override { stop(); }
    bool is_running() const override { return running_; }
    float volume() const override { return 1.0f; }
    void set_volume(float) override {}
    void set_callback(AudioCallback cb) override { cb_ = cb; }
    int write(const float*, int) override { return 0; }

private:
    void run() {
        std::vector<float> buf(buf_size_);
        while (running_) {
            if (cb_) cb_(buf.data(), buf_size_, 1);
            {
                std::lock_guard<std::mutex> lock(g_loop_mutex);
                for (int i = 0; i < buf_size_; i++) {
                    g_loop_buffer[g_loop_write_pos] = buf[i];
                    g_loop_write_pos = (g_loop_write_pos + 1) % LOOP_BUF_SIZE;
                }
            }
            std::this_thread::sleep_for(
                std::chrono::microseconds(buf_size_ * 1000000 / sample_rate_));
        }
    }

    int sample_rate_ = 48000;
    int buf_size_ = 480;
    std::atomic<bool> running_{false};
    AudioCallback cb_;
    std::thread thread_;
};

std::unique_ptr<AudioCapture> create_loopback_capture() {
    return std::make_unique<LoopbackCapture>();
}

std::unique_ptr<AudioPlayback> create_loopback_playback() {
    return std::make_unique<LoopbackPlayback>();
}

} // namespace iris
