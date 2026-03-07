#include "audio/audio.h"

#if defined(__linux__) && !defined(_WIN32)

#include <alsa/asoundlib.h>
#include <thread>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

namespace iris {

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

        int err = snd_pcm_open(&pcm_, "default", SND_PCM_STREAM_CAPTURE, 0);
        if (err < 0) return false;

        snd_pcm_hw_params_t* params;
        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(pcm_, params);
        snd_pcm_hw_params_set_access(pcm_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_, params, SND_PCM_FORMAT_FLOAT_LE);
        snd_pcm_hw_params_set_channels(pcm_, params, (unsigned)channels);

        unsigned rate = (unsigned)sample_rate;
        snd_pcm_hw_params_set_rate_near(pcm_, params, &rate, nullptr);

        snd_pcm_uframes_t period = (snd_pcm_uframes_t)frames_per_buffer;
        snd_pcm_hw_params_set_period_size_near(pcm_, params, &period, nullptr);

        err = snd_pcm_hw_params(pcm_, params);
        if (err < 0) { snd_pcm_close(pcm_); pcm_ = nullptr; return false; }

        return true;
    }

    bool start() override {
        if (!pcm_ || running_) return false;
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
        if (pcm_) { snd_pcm_close(pcm_); pcm_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float* buffer, int frame_count) override {
        if (!pcm_) return 0;
        int ret = snd_pcm_readi(pcm_, buffer, frame_count);
        if (ret < 0) {
            snd_pcm_recover(pcm_, ret, 0);
            return 0;
        }
        return ret;
    }

private:
    void capture_thread() {
        std::vector<float> buf(buffer_frames_ * channels_);
        while (running_) {
            int frames = snd_pcm_readi(pcm_, buf.data(), buffer_frames_);
            if (frames < 0) {
                snd_pcm_recover(pcm_, frames, 0);
                continue;
            }
            if (volume_ != 1.0f) {
                for (int i = 0; i < frames * channels_; i++)
                    buf[i] *= volume_;
            }
            if (callback_) callback_(buf.data(), frames, channels_);
        }
    }

    snd_pcm_t* pcm_ = nullptr;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

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

        int err = snd_pcm_open(&pcm_, "default", SND_PCM_STREAM_PLAYBACK, 0);
        if (err < 0) return false;

        snd_pcm_hw_params_t* params;
        snd_pcm_hw_params_alloca(&params);
        snd_pcm_hw_params_any(pcm_, params);
        snd_pcm_hw_params_set_access(pcm_, params, SND_PCM_ACCESS_RW_INTERLEAVED);
        snd_pcm_hw_params_set_format(pcm_, params, SND_PCM_FORMAT_FLOAT_LE);
        snd_pcm_hw_params_set_channels(pcm_, params, (unsigned)channels);

        unsigned rate = (unsigned)sample_rate;
        snd_pcm_hw_params_set_rate_near(pcm_, params, &rate, nullptr);

        snd_pcm_uframes_t period = (snd_pcm_uframes_t)frames_per_buffer;
        snd_pcm_hw_params_set_period_size_near(pcm_, params, &period, nullptr);

        err = snd_pcm_hw_params(pcm_, params);
        if (err < 0) { snd_pcm_close(pcm_); pcm_ = nullptr; return false; }

        return true;
    }

    bool start() override {
        if (!pcm_ || running_) return false;
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
        if (pcm_) { snd_pcm_close(pcm_); pcm_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float* buffer, int frame_count) override {
        if (!pcm_) return 0;
        int ret = snd_pcm_writei(pcm_, buffer, frame_count);
        if (ret < 0) {
            snd_pcm_recover(pcm_, ret, 0);
            return 0;
        }
        return ret;
    }

private:
    void playback_thread() {
        std::vector<float> buf(buffer_frames_ * channels_, 0.0f);
        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            if (volume_ != 1.0f) {
                for (int i = 0; i < buffer_frames_ * channels_; i++)
                    buf[i] *= volume_;
            }
            int ret = snd_pcm_writei(pcm_, buf.data(), buffer_frames_);
            if (ret < 0) snd_pcm_recover(pcm_, ret, 0);
        }
    }

    snd_pcm_t* pcm_ = nullptr;
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

    // List ALSA PCM devices
    void** hints = nullptr;
    if (snd_device_name_hint(-1, "pcm", &hints) < 0) return devices;

    int id = 0;
    for (void** h = hints; *h; h++) {
        char* name = snd_device_name_get_hint(*h, "NAME");
        char* desc = snd_device_name_get_hint(*h, "DESC");
        char* ioid = snd_device_name_get_hint(*h, "IOID");

        if (name) {
            AudioDevice dev;
            dev.id = id++;
            dev.name = desc ? desc : name;
            // Remove newlines from description
            for (auto& c : dev.name) if (c == '\n') c = ' ';
            dev.default_sample_rate = 48000;

            if (!ioid || strcmp(ioid, "Input") == 0)
                dev.max_input_channels = 1;
            else
                dev.max_input_channels = 0;

            if (!ioid || strcmp(ioid, "Output") == 0)
                dev.max_output_channels = 1;
            else
                dev.max_output_channels = 0;

            devices.push_back(dev);
        }

        if (name) free(name);
        if (desc) free(desc);
        if (ioid) free(ioid);
    }

    snd_device_name_free_hint(hints);
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<AlsaCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<AlsaPlayback>();
}

} // namespace iris

#endif // __linux__
