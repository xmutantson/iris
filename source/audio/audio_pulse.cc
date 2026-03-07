/*
 * PulseAudio backend for Iris.
 * Compiled on Linux when PulseAudio headers are available.
 * Falls back to ALSA if PulseAudio is not present.
 */
#include "audio/audio.h"

#if defined(__linux__) && defined(IRIS_USE_PULSE)

#include <pulse/pulseaudio.h>
#include <pulse/simple.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>

namespace iris {

class PulseCapture : public AudioCapture {
public:
    PulseCapture() = default;
    ~PulseCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        pa_sample_spec ss;
        ss.format = PA_SAMPLE_FLOAT32LE;
        ss.rate = (uint32_t)sample_rate;
        ss.channels = (uint8_t)channels;

        int error = 0;
        simple_ = pa_simple_new(nullptr, "Iris", PA_STREAM_RECORD,
                                nullptr, "capture", &ss, nullptr, nullptr, &error);
        return simple_ != nullptr;
    }

    bool start() override {
        if (!simple_ || running_) return false;
        running_ = true;
        thread_ = std::thread(&PulseCapture::capture_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (simple_) { pa_simple_free(simple_); simple_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float* buffer, int frame_count) override {
        if (!simple_) return 0;
        int error = 0;
        int bytes = frame_count * channels_ * (int)sizeof(float);
        if (pa_simple_read(simple_, buffer, bytes, &error) < 0) return 0;
        return frame_count;
    }

private:
    void capture_thread() {
        std::vector<float> buf(buffer_frames_ * channels_);
        int bytes = (int)(buf.size() * sizeof(float));
        while (running_) {
            int error = 0;
            if (pa_simple_read(simple_, buf.data(), bytes, &error) < 0) continue;
            if (volume_ != 1.0f) {
                for (size_t i = 0; i < buf.size(); i++)
                    buf[i] *= volume_;
            }
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
        }
    }

    pa_simple* simple_ = nullptr;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

class PulsePlayback : public AudioPlayback {
public:
    PulsePlayback() = default;
    ~PulsePlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        pa_sample_spec ss;
        ss.format = PA_SAMPLE_FLOAT32LE;
        ss.rate = (uint32_t)sample_rate;
        ss.channels = (uint8_t)channels;

        int error = 0;
        simple_ = pa_simple_new(nullptr, "Iris", PA_STREAM_PLAYBACK,
                                nullptr, "playback", &ss, nullptr, nullptr, &error);
        return simple_ != nullptr;
    }

    bool start() override {
        if (!simple_ || running_) return false;
        running_ = true;
        thread_ = std::thread(&PulsePlayback::playback_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (simple_) { pa_simple_free(simple_); simple_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float* buffer, int frame_count) override {
        if (!simple_) return 0;
        int error = 0;
        int bytes = frame_count * channels_ * (int)sizeof(float);
        if (pa_simple_write(simple_, buffer, bytes, &error) < 0) return 0;
        return frame_count;
    }

private:
    void playback_thread() {
        std::vector<float> buf(buffer_frames_ * channels_, 0.0f);
        int bytes = (int)(buf.size() * sizeof(float));
        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            if (volume_ != 1.0f) {
                for (size_t i = 0; i < buf.size(); i++)
                    buf[i] *= volume_;
            }
            int error = 0;
            pa_simple_write(simple_, buf.data(), bytes, &error);
        }
    }

    pa_simple* simple_ = nullptr;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

std::vector<AudioDevice> enumerate_audio_devices() {
    // PulseAudio simple API doesn't enumerate — return default device
    std::vector<AudioDevice> devices;
    AudioDevice dev;
    dev.id = 0;
    dev.name = "PulseAudio Default";
    dev.max_input_channels = 1;
    dev.max_output_channels = 1;
    dev.default_sample_rate = 48000;
    devices.push_back(dev);
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<PulseCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<PulsePlayback>();
}

} // namespace iris

#endif // __linux__ && IRIS_USE_PULSE
