#include "audio/audio.h"

#if defined(__linux__) && !defined(_WIN32)

#include <unistd.h>
#include <fcntl.h>
#include <signal.h>
#include <sys/wait.h>
#include <sys/stat.h>
#include <sys/inotify.h>
#include <poll.h>
#include <thread>
#include <atomic>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <vector>
#include <string>

namespace iris {

// ── Backend selection state ──────────────────────────────────────────
static AudioBackend g_audio_backend = AudioBackend::AUTO;
static std::string  g_alsa_device;

void set_audio_backend(AudioBackend b) { g_audio_backend = b; }
AudioBackend get_audio_backend() { return g_audio_backend; }
void set_alsa_device(const std::string& d) { g_alsa_device = d; }
const std::string& get_alsa_device() { return g_alsa_device; }

static const char* alsa_dev_name() {
    return g_alsa_device.empty() ? "default" : g_alsa_device.c_str();
}

// Hardware always opens stereo (RPi I2S constraint). Mono<->stereo in software.
static const int HW_CHANNELS = 2;

// ── File-based capture using arecord ────────────────────────────────
// On RPi I2S / Fe-Pi SGTL5000, arecord only produces real audio when
// writing WAV format to a seekable file. Pipe/stdout/raw modes all
// produce noise (confirmed identical DAPM states, same kernel device).
// Workaround: arecord writes to a tmpfs file; we tail-read it via
// inotify for ~25ms latency with zero audio gaps.

class AlsaCapture : public AudioCapture {
public:
    AlsaCapture() = default;
    ~AlsaCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        char rate_str[16], ch_str[4];
        snprintf(rate_str, sizeof(rate_str), "%d", sample_rate);
        snprintf(ch_str, sizeof(ch_str), "%d", HW_CHANNELS);

        // Generate temp file path on tmpfs (RAM-backed, no disk I/O)
        snprintf(wav_path_, sizeof(wav_path_),
                 "/tmp/iris_capture_%d.wav", (int)getpid());

        // Remove stale file from previous run
        unlink(wav_path_);

        // Launch arecord writing WAV to a real file (the only mode that
        // works on RPi I2S / Fe-Pi hardware)
        pid_ = fork();
        if (pid_ < 0) return false;

        if (pid_ == 0) {
            // Child: redirect stderr to /dev/null
            int devnull = ::open("/dev/null", O_WRONLY);
            if (devnull >= 0) { dup2(devnull, STDERR_FILENO); ::close(devnull); }

            execlp("arecord", "arecord",
                   "-D", alsa_dev_name(),
                   "-f", "S16_LE",
                   "-r", rate_str,
                   "-c", ch_str,
                   "-t", "wav",
                   wav_path_,
                   (char*)nullptr);
            _exit(127);
        }

        // Wait for the WAV file to appear and have at least the header
        if (!wait_for_header(3000)) {
            fprintf(stderr, "[ALSA] arecord failed to create WAV file\n");
            kill(pid_, SIGTERM);
            waitpid(pid_, nullptr, 0);
            pid_ = -1;
            return false;
        }

        // Open the file for reading
        read_fd_ = ::open(wav_path_, O_RDONLY);
        if (read_fd_ < 0) return false;

        // Set up inotify to wake when arecord writes new data
        inotify_fd_ = inotify_init1(IN_NONBLOCK);
        if (inotify_fd_ >= 0)
            inotify_wd_ = inotify_add_watch(inotify_fd_, wav_path_, IN_MODIFY);

        // Skip the 44-byte WAV header
        char hdr[44];
        int got = 0;
        while (got < 44) {
            ssize_t n = ::read(read_fd_, hdr + got, 44 - got);
            if (n > 0) { got += (int)n; continue; }
            usleep(5000);
        }

        fprintf(stderr, "[ALSA] file-based capture ready: %s\n", wav_path_);
        return true;
    }

    bool start() override {
        if (read_fd_ < 0 || running_) return false;
        running_ = true;
        thread_ = std::thread(&AlsaCapture::capture_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (read_fd_ >= 0) { ::close(read_fd_); read_fd_ = -1; }
        if (inotify_fd_ >= 0) {
            if (inotify_wd_ >= 0)
                inotify_rm_watch(inotify_fd_, inotify_wd_);
            ::close(inotify_fd_);
            inotify_fd_ = -1;
            inotify_wd_ = -1;
        }
        if (pid_ > 0) {
            kill(pid_, SIGTERM);
            waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
        if (wav_path_[0]) unlink(wav_path_);
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float* buffer, int frame_count) override {
        if (read_fd_ < 0) return 0;
        std::vector<int16_t> s16(frame_count * HW_CHANNELS);
        int bytes_needed = frame_count * HW_CHANNELS * (int)sizeof(int16_t);
        int bytes_read = read_new_data(s16.data(), bytes_needed);
        int frames = bytes_read / (HW_CHANNELS * (int)sizeof(int16_t));
        // Fe-Pi LINE_IN routes to right channel (index 1) on RPi I2S
        for (int i = 0; i < frames; i++)
            buffer[i] = s16[i * HW_CHANNELS + 1] / 32768.0f;
        return frames;
    }

private:
    bool wait_for_header(int timeout_ms) {
        for (int elapsed = 0; elapsed < timeout_ms; elapsed += 10) {
            struct stat st;
            if (stat(wav_path_, &st) == 0 && st.st_size >= 44)
                return true;
            usleep(10000);
        }
        return false;
    }

    // Read new data from the growing WAV file. Blocks (with poll) until
    // data is available or running_ becomes false.
    int read_new_data(void* buf, int bytes_wanted) {
        int total = 0;
        char* p = (char*)buf;

        while (total < bytes_wanted && running_) {
            ssize_t n = ::read(read_fd_, p + total, bytes_wanted - total);
            if (n > 0) {
                total += (int)n;
                continue;
            }
            // At EOF of currently-written data. Wait for arecord to write more.
            if (inotify_fd_ >= 0) {
                struct pollfd pfd = { inotify_fd_, POLLIN, 0 };
                poll(&pfd, 1, 100);  // 100ms timeout for stop-flag check
                if (pfd.revents & POLLIN) drain_inotify();
            } else {
                usleep(5000);  // fallback: 5ms poll
            }
            // Check if arecord died
            if (pid_ > 0) {
                int status;
                if (waitpid(pid_, &status, WNOHANG) > 0) {
                    fprintf(stderr, "[ALSA] arecord exited unexpectedly\n");
                    pid_ = -1;
                    break;
                }
            }
        }
        return total;
    }

    void drain_inotify() {
        char buf[256];
        while (::read(inotify_fd_, buf, sizeof(buf)) > 0) {}
    }

    void capture_thread() {
        int s16_samples = buffer_frames_ * HW_CHANNELS;
        int bytes_per_read = s16_samples * (int)sizeof(int16_t);
        std::vector<int16_t> s16(s16_samples);
        std::vector<float> buf(buffer_frames_ * channels_);
        int diag_count = 0;

        while (running_) {
            int bytes = read_new_data(s16.data(), bytes_per_read);
            int frames = bytes / (HW_CHANNELS * (int)sizeof(int16_t));
            if (frames <= 0) {
                if (running_) usleep(10000);
                continue;
            }
            // Diagnostic: log peak every ~5s
            if (++diag_count % (5 * sample_rate_ / buffer_frames_) == 1) {
                int16_t peak = 0;
                for (int i = 0; i < frames * HW_CHANNELS; i++) {
                    int16_t v = s16[i] < 0 ? -s16[i] : s16[i];
                    if (v > peak) peak = v;
                }
                float fpeak = 0;
                for (int i = 0; i < frames; i++) {
                    float fv = s16[i * HW_CHANNELS] / 32768.0f;
                    if (fv < 0) fv = -fv;
                    if (fv > fpeak) fpeak = fv;
                }
                // Per-channel peaks
                int16_t peakL = 0, peakR = 0;
                for (int i = 0; i < frames; i++) {
                    int16_t vL = s16[i*2] < 0 ? -s16[i*2] : s16[i*2];
                    int16_t vR = s16[i*2+1] < 0 ? -s16[i*2+1] : s16[i*2+1];
                    if (vL > peakL) peakL = vL;
                    if (vR > peakR) peakR = vR;
                }
                fprintf(stderr, "[ALSA-DIAG] frames=%d L=%d R=%d float=%.4f cb=%s\n",
                        frames, peakL, peakR, fpeak, callback_ ? "yes" : "NO");
            }
            float vol = volume_;
            // Fe-Pi LINE_IN routes to right channel (index 1) on RPi I2S
            for (int i = 0; i < frames; i++)
                buf[i] = s16[i * HW_CHANNELS + 1] / 32768.0f * vol;
            if (callback_) callback_(buf.data(), frames, channels_);
        }
    }

    char wav_path_[128] = {};
    int read_fd_ = -1;
    int inotify_fd_ = -1;
    int inotify_wd_ = -1;
    pid_t pid_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

// ── Subprocess-based playback using aplay ───────────────────────────

class AlsaPlayback : public AudioPlayback {
public:
    AlsaPlayback() = default;
    ~AlsaPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        char rate_str[16], ch_str[4];
        snprintf(rate_str, sizeof(rate_str), "%d", sample_rate);
        snprintf(ch_str, sizeof(ch_str), "%d", HW_CHANNELS);

        int pipefd[2];
        if (pipe(pipefd) < 0) return false;

        pid_ = fork();
        if (pid_ < 0) {
            ::close(pipefd[0]);
            ::close(pipefd[1]);
            return false;
        }

        if (pid_ == 0) {
            // Child: redirect stdin from pipe read end
            ::close(pipefd[1]);
            dup2(pipefd[0], STDIN_FILENO);
            ::close(pipefd[0]);
            int devnull = ::open("/dev/null", O_WRONLY);
            if (devnull >= 0) { dup2(devnull, STDERR_FILENO); ::close(devnull); }

            execlp("aplay", "aplay",
                   "-D", alsa_dev_name(),
                   "-f", "S16_LE",
                   "-r", rate_str,
                   "-c", ch_str,
                   "-t", "raw",
                   "-q",
                   "-", (char*)nullptr);
            _exit(127);
        }

        ::close(pipefd[0]);
        pipe_fd_ = pipefd[1];
        return true;
    }

    bool start() override {
        if (pipe_fd_ < 0 || running_) return false;
        running_ = true;
        thread_ = std::thread(&AlsaPlayback::playback_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        return true;
    }

    void close() override {
        stop();
        if (pipe_fd_ >= 0) { ::close(pipe_fd_); pipe_fd_ = -1; }
        if (pid_ > 0) {
            kill(pid_, SIGTERM);
            waitpid(pid_, nullptr, 0);
            pid_ = -1;
        }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float* buffer, int frame_count) override {
        if (pipe_fd_ < 0) return 0;
        std::vector<int16_t> s16(frame_count * HW_CHANNELS);
        for (int i = 0; i < frame_count; i++) {
            float s = buffer[i] * 32767.0f;
            if (s > 32767.0f) s = 32767.0f;
            if (s < -32768.0f) s = -32768.0f;
            int16_t v = (int16_t)s;
            s16[i * HW_CHANNELS]     = v;
            s16[i * HW_CHANNELS + 1] = v;
        }
        int bytes = frame_count * HW_CHANNELS * (int)sizeof(int16_t);
        return write_fully(s16.data(), bytes) / (HW_CHANNELS * (int)sizeof(int16_t));
    }

private:
    int write_fully(const void* buf, int bytes) {
        int total = 0;
        const char* p = (const char*)buf;
        while (total < bytes) {
            ssize_t n = ::write(pipe_fd_, p + total, bytes - total);
            if (n <= 0) break;
            total += (int)n;
        }
        return total;
    }

    void playback_thread() {
        int n_mono = buffer_frames_ * channels_;
        std::vector<float> buf(n_mono, 0.0f);
        std::vector<int16_t> s16(buffer_frames_ * HW_CHANNELS);

        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            float vol = volume_;
            for (int i = 0; i < buffer_frames_; i++) {
                float s = buf[i] * vol * 32767.0f;
                if (s > 32767.0f) s = 32767.0f;
                if (s < -32768.0f) s = -32768.0f;
                int16_t v = (int16_t)s;
                s16[i * HW_CHANNELS]     = v;
                s16[i * HW_CHANNELS + 1] = v;
            }
            int bytes = buffer_frames_ * HW_CHANNELS * (int)sizeof(int16_t);
            if (write_fully(s16.data(), bytes) <= 0) {
                if (running_) usleep(10000);
            }
        }
    }

    int pipe_fd_ = -1;
    pid_t pid_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

// ── ALSA factory functions ───────────────────────────────────────────
std::vector<AudioDevice> alsa_enumerate_devices() {
    std::vector<AudioDevice> devices;

    if (!g_alsa_device.empty()) {
        AudioDevice dev;
        dev.id = 0;
        dev.name = "ALSA: " + g_alsa_device;
        dev.max_input_channels = 1;
        dev.max_output_channels = 1;
        dev.default_sample_rate = 48000;
        devices.push_back(dev);
        return devices;
    }

    // Simple default device entry (full enumeration not needed for pipe backend)
    AudioDevice dev;
    dev.id = 0;
    dev.name = "ALSA Default";
    dev.max_input_channels = 1;
    dev.max_output_channels = 1;
    dev.default_sample_rate = 48000;
    devices.push_back(dev);
    return devices;
}

std::unique_ptr<AudioCapture> alsa_create_capture() {
    return std::make_unique<AlsaCapture>();
}

std::unique_ptr<AudioPlayback> alsa_create_playback() {
    return std::make_unique<AlsaPlayback>();
}

// ── Multiplexer ──────────────────────────────────────────────────────
#ifdef IRIS_USE_PULSE
std::vector<AudioDevice> pulse_enumerate_devices();
std::unique_ptr<AudioCapture>  pulse_create_capture();
std::unique_ptr<AudioPlayback> pulse_create_playback();
#endif

static bool use_alsa() {
    if (g_audio_backend == AudioBackend::ALSA) return true;
    if (g_audio_backend == AudioBackend::PULSE) return false;
    return true;
}

std::vector<AudioDevice> enumerate_audio_devices() {
#ifdef IRIS_USE_PULSE
    if (!use_alsa()) return pulse_enumerate_devices();
#endif
    return alsa_enumerate_devices();
}

std::unique_ptr<AudioCapture> create_capture() {
    if (g_audio_backend == AudioBackend::ALSA_RAW) return alsa_raw_create_capture();
#ifdef IRIS_USE_PULSE
    if (!use_alsa()) return pulse_create_capture();
#endif
    return alsa_create_capture();
}

std::unique_ptr<AudioPlayback> create_playback() {
    if (g_audio_backend == AudioBackend::ALSA_RAW) return alsa_raw_create_playback();
#ifdef IRIS_USE_PULSE
    if (!use_alsa()) return pulse_create_playback();
#endif
    return alsa_create_playback();
}

} // namespace iris

#endif // __linux__
