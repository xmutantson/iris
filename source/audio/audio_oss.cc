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
#include <cerrno>
#include <cstring>
#include <vector>
#include <cmath>

namespace iris {

using OssOpenHook = int (*)(const char*, int);
using OssCloseHook = int (*)(int);
using OssIoctlHook = int (*)(int, unsigned long, void*);
using OssWriteHook = ssize_t (*)(int, const void*, size_t);

static int oss_real_open(const char* path, int flags) { return ::open(path, flags); }
static int oss_real_close(int fd) { return ::close(fd); }
static int oss_real_ioctl(int fd, unsigned long request, void* value) {
    return ::ioctl(fd, request, value);
}
static ssize_t oss_real_write(int fd, const void* buffer, size_t bytes) {
    return ::write(fd, buffer, bytes);
}

static OssOpenHook oss_open = oss_real_open;
static OssCloseHook oss_close = oss_real_close;
static OssIoctlHook oss_ioctl = oss_real_ioctl;
static OssWriteHook oss_write = oss_real_write;

void oss_set_test_io(OssOpenHook open_hook, OssCloseHook close_hook,
                     OssIoctlHook ioctl_hook, OssWriteHook write_hook) {
    oss_open = open_hook;
    oss_close = close_hook;
    oss_ioctl = ioctl_hook;
    oss_write = write_hook;
}

void oss_reset_test_io() {
    oss_set_test_io(oss_real_open, oss_real_close, oss_real_ioctl, oss_real_write);
}

class OssCapture : public AudioCapture {
public:
    OssCapture() = default;
    ~OssCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        close();
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
        close();
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        fd_ = oss_open("/dev/dsp", O_WRONLY);
        if (fd_ < 0) return false;

        int fmt = AFMT_S16_LE;
        oss_ioctl(fd_, SNDCTL_DSP_SETFMT, &fmt);
        oss_ioctl(fd_, SNDCTL_DSP_CHANNELS, &channels);
        oss_ioctl(fd_, SNDCTL_DSP_SPEED, &sample_rate);
        sample_rate_ = sample_rate;
        bytes_written_ = 0;
        drain_mark_ = -1;
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
        if (fd_ >= 0) { oss_close(fd_); fd_ = -1; }
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
        int ret = write_fully(s16.data(), bytes);
        if (ret < 0) return -1;
        return ret / (int)(channels_ * sizeof(int16_t));
    }

    void mark_drain() override {
        drain_mark_ = bytes_written_.load();
    }

    bool is_drained() const override {
        int64_t mark = drain_mark_.load();
        if (mark < 0) return true;
        int64_t submitted = bytes_written_.load();
        int delay = 0;
        if (oss_ioctl(fd_, SNDCTL_DSP_GETODELAY, &delay) < 0) return false;
        return submitted - delay >= mark;
    }

private:
    int write_fully(const void* buffer, int bytes) {
        int total = 0;
        const char* p = (const char*)buffer;
        while (total < bytes) {
            ssize_t n = oss_write(fd_, p + total, bytes - total);
            if (n < 0) {
                if (errno == EINTR) continue;
                return total > 0 ? total : -1;
            }
            if (n == 0) break;
            total += (int)n;
            bytes_written_.fetch_add(n);
        }
        return total;
    }

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
            int bytes = n_samples * (int)sizeof(int16_t);
            if (write_fully(s16.data(), bytes) < bytes)
                running_ = false;
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
    std::atomic<int64_t> bytes_written_{0};
    std::atomic<int64_t> drain_mark_{-1};
};

static AudioBackend g_audio_backend = AudioBackend::AUTO;

std::vector<AudioDevice> enumerate_audio_devices() {
    std::vector<AudioDevice> devices;
    if (g_audio_backend != AudioBackend::AUTO) return devices;

    int fd = ::open("/dev/dsp", O_RDWR);
    if (fd < 0) return devices;

    int fmt = AFMT_S16_LE;
    int channels = 2;
    int sample_rate = 48000;
    if (::ioctl(fd, SNDCTL_DSP_SETFMT, &fmt) < 0 || fmt != AFMT_S16_LE ||
        ::ioctl(fd, SNDCTL_DSP_CHANNELS, &channels) < 0 || channels <= 0 ||
        ::ioctl(fd, SNDCTL_DSP_SPEED, &sample_rate) < 0 || sample_rate <= 0) {
        ::close(fd);
        return devices;
    }
    ::close(fd);

    AudioDevice dev;
    dev.id = 0;
    dev.name = "OSS Default (/dev/dsp)";
    dev.max_input_channels = channels;
    dev.max_output_channels = channels;
    dev.default_sample_rate = sample_rate;
    devices.push_back(dev);
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    if (g_audio_backend != AudioBackend::AUTO) return nullptr;
    return std::make_unique<OssCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    if (g_audio_backend != AudioBackend::AUTO) return nullptr;
    return std::make_unique<OssPlayback>();
}

void set_audio_backend(AudioBackend backend) { g_audio_backend = backend; }
AudioBackend get_audio_backend() { return g_audio_backend; }
void set_alsa_device(const std::string&) {}
const std::string& get_alsa_device() { static std::string s; return s; }

} // namespace iris

#endif // OSS
