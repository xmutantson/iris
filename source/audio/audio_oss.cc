/*
 * OSS (Open Sound System) backend for Iris.
 * Fallback for systems without ALSA or PulseAudio (FreeBSD, older Linux).
 * Compile with IRIS_USE_OSS defined.
 */
#include "audio/audio.h"

#if (defined(__linux__) || defined(__FreeBSD__)) && defined(IRIS_USE_OSS)

#include <sys/ioctl.h>
#include <sys/soundcard.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>
#include <cmath>

namespace iris {

class OssCapture : public AudioCapture {
public:
    OssCapture() = default;
    ~OssCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        fd_ = ::open("/dev/dsp", O_RDONLY);
        if (fd_ < 0) return false;

        int fmt = AFMT_S16_LE;
        ioctl(fd_, SNDCTL_DSP_SETFMT, &fmt);
        ioctl(fd_, SNDCTL_DSP_CHANNELS, &channels);
        ioctl(fd_, SNDCTL_DSP_SPEED, &sample_rate);
        sample_rate_ = sample_rate;
        return true;
    }

    bool start() override {
        if (fd_ < 0 || running_) return false;
        running_ = true;
        thread_ = std::thread(&OssCapture::capture_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float* buffer, int frame_count) override {
        if (fd_ < 0) return 0;
        std::vector<int16_t> s16(frame_count * channels_);
        int bytes = (int)(s16.size() * sizeof(int16_t));
        int ret = ::read(fd_, s16.data(), bytes);
        if (ret <= 0) return 0;
        int frames = ret / (int)(channels_ * sizeof(int16_t));
        for (int i = 0; i < frames * channels_; i++)
            buffer[i] = s16[i] / 32768.0f;
        return frames;
    }

private:
    void capture_thread() {
        int n_samples = buffer_frames_ * channels_;
        std::vector<int16_t> s16(n_samples);
        std::vector<float> buf(n_samples);
        int bytes = n_samples * (int)sizeof(int16_t);
        while (running_) {
            int ret = ::read(fd_, s16.data(), bytes);
            if (ret <= 0) continue;
            int frames = ret / (int)(channels_ * sizeof(int16_t));
            for (int i = 0; i < frames * channels_; i++)
                buf[i] = s16[i] / 32768.0f * volume_;
            if (callback_) callback_(buf.data(), frames, channels_);
        }
    }

    int fd_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

class OssPlayback : public AudioPlayback {
public:
    OssPlayback() = default;
    ~OssPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        fd_ = ::open("/dev/dsp", O_WRONLY);
        if (fd_ < 0) return false;

        int fmt = AFMT_S16_LE;
        ioctl(fd_, SNDCTL_DSP_SETFMT, &fmt);
        ioctl(fd_, SNDCTL_DSP_CHANNELS, &channels);
        ioctl(fd_, SNDCTL_DSP_SPEED, &sample_rate);
        sample_rate_ = sample_rate;
        return true;
    }

    bool start() override {
        if (fd_ < 0 || running_) return false;
        running_ = true;
        thread_ = std::thread(&OssPlayback::playback_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (fd_ >= 0) { ::close(fd_); fd_ = -1; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float* buffer, int frame_count) override {
        if (fd_ < 0) return 0;
        std::vector<int16_t> s16(frame_count * channels_);
        for (int i = 0; i < frame_count * channels_; i++) {
            float s = buffer[i] * 32767.0f;
            if (s > 32767.0f) s = 32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            s16[i] = (int16_t)s;
        }
        int bytes = (int)(s16.size() * sizeof(int16_t));
        int ret = ::write(fd_, s16.data(), bytes);
        if (ret <= 0) return 0;
        return ret / (int)(channels_ * sizeof(int16_t));
    }

private:
    void playback_thread() {
        int n_samples = buffer_frames_ * channels_;
        std::vector<float> buf(n_samples, 0.0f);
        std::vector<int16_t> s16(n_samples);
        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            for (int i = 0; i < n_samples; i++) {
                float s = buf[i] * volume_ * 32767.0f;
                if (s > 32767.0f) s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                s16[i] = (int16_t)s;
            }
            ::write(fd_, s16.data(), n_samples * (int)sizeof(int16_t));
        }
    }

    int fd_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

std::vector<AudioDevice> enumerate_audio_devices() {
    std::vector<AudioDevice> devices;
    AudioDevice dev;
    dev.id = 0;
    dev.name = "OSS Default (/dev/dsp)";
    dev.max_input_channels = 1;
    dev.max_output_channels = 1;
    dev.default_sample_rate = 48000;
    devices.push_back(dev);
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<OssCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<OssPlayback>();
}

} // namespace iris

#endif // OSS
