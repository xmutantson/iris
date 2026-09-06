// audio_alsa_raw.cc — bit-exact raw S32_LE snd_pcm ALSA backend.
//
// Unlike audio_alsa.cc (which shells out to arecord/aplay for the RPi I2S /
// Fe-Pi hardware quirk), this backend talks to libasound directly with NO
// "plug" plugin and NO rate-resampling, so samples are delivered bit-exact.
// It is the backend the HW-clocked fleet snd-aloop faithful-sim path uses:
// two Iris stacks bridged through snd-aloop (hw:Loopback) driven by the real
// ALSA HW timer (no virtual-clock probe-timing skew, unlike "-x sim").
//
// Scaling mirrors Mercury's audioio.c: full-scale float [-1,1] maps to the
// full S32 range via IRIS_RAW_INT_MAX = 2147483647.0. Capture deinterleaves
// stereo and takes channel 0; playback duplicates mono into both channels.

#include "audio/audio.h"

#if defined(__linux__)

#include <alsa/asoundlib.h>
#include <atomic>
#include <thread>
#include <vector>
#include <string>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Full-scale S32 magnitude (mirrors Mercury audioio.c INT32 scaling).
static const double IRIS_RAW_INT_MAX = 2147483647.0;

// Raw ALSA always opens stereo interleaved; mono<->stereo handled in software.
static const int RAW_HW_CHANNELS = 2;

// ── Per-direction device resolution ──────────────────────────────────
// env override → --alsa-raw <hwdev> (get_alsa_device) → documented default.
static std::string raw_capture_dev() {
    const char* e = getenv("IRIS_ALSA_RAW_CAPTURE");
    if (e && *e) return e;
    const std::string& d = get_alsa_device();
    if (!d.empty()) return d;
    return "hw:Loopback,1,0";
}
static std::string raw_playback_dev() {
    const char* e = getenv("IRIS_ALSA_RAW_PLAYBACK");
    if (e && *e) return e;
    const std::string& d = get_alsa_device();
    if (!d.empty()) return d;
    return "hw:Loopback,0,0";
}

// ── Shared hw-params setup ────────────────────────────────────────────
// any -> resample(0) [NO plug] -> RW_INTERLEAVED -> S32_LE -> 2ch ->
// 48000 exact -> period_size_near(frames)/periods_near(4) -> prepare.
static bool raw_set_params(snd_pcm_t* pcm, const char* tag,
                           unsigned int sample_rate,
                           snd_pcm_uframes_t frames) {
    snd_pcm_hw_params_t* hw = nullptr;
    snd_pcm_hw_params_alloca(&hw);

    int e;
    if ((e = snd_pcm_hw_params_any(pcm, hw)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s hw_params_any: %s\n", tag, snd_strerror(e));
        return false;
    }
    // No software rate conversion — bit-exact HW rate only.
    if ((e = snd_pcm_hw_params_set_rate_resample(pcm, hw, 0)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_rate_resample(0): %s\n", tag, snd_strerror(e));
        return false;
    }
    if ((e = snd_pcm_hw_params_set_access(pcm, hw, SND_PCM_ACCESS_RW_INTERLEAVED)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_access(RW_INTERLEAVED): %s\n", tag, snd_strerror(e));
        return false;
    }
    if ((e = snd_pcm_hw_params_set_format(pcm, hw, SND_PCM_FORMAT_S32_LE)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_format(S32_LE): %s\n", tag, snd_strerror(e));
        return false;
    }
    if ((e = snd_pcm_hw_params_set_channels(pcm, hw, RAW_HW_CHANNELS)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_channels(2): %s\n", tag, snd_strerror(e));
        return false;
    }
    unsigned int rate = sample_rate;
    // Exact rate: dir=0, and verify the driver did not adjust it.
    if ((e = snd_pcm_hw_params_set_rate(pcm, hw, rate, 0)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_rate(%u exact): %s\n", tag, sample_rate, snd_strerror(e));
        return false;
    }
    if (rate != sample_rate) {
        fprintf(stderr, "[ALSA-RAW] %s rate %u not exact (got %u)\n", tag, sample_rate, rate);
        return false;
    }
    snd_pcm_uframes_t period = frames;
    int dir = 0;
    if ((e = snd_pcm_hw_params_set_period_size_near(pcm, hw, &period, &dir)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_period_size_near: %s\n", tag, snd_strerror(e));
        return false;
    }
    unsigned int periods = 4;
    dir = 0;
    if ((e = snd_pcm_hw_params_set_periods_near(pcm, hw, &periods, &dir)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s set_periods_near(4): %s\n", tag, snd_strerror(e));
        return false;
    }
    if ((e = snd_pcm_hw_params(pcm, hw)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s hw_params apply: %s\n", tag, snd_strerror(e));
        return false;
    }
    if ((e = snd_pcm_prepare(pcm)) < 0) {
        fprintf(stderr, "[ALSA-RAW] %s prepare: %s\n", tag, snd_strerror(e));
        return false;
    }
    return true;
}

// ── Raw S32 capture ──────────────────────────────────────────────────
class AlsaRawCapture : public AudioCapture {
public:
    AlsaRawCapture() = default;
    ~AlsaRawCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        dev_ = raw_capture_dev();

        int e = snd_pcm_open(&pcm_, dev_.c_str(), SND_PCM_STREAM_CAPTURE, 0);
        if (e < 0) {
            fprintf(stderr, "[ALSA-RAW] capture open '%s': %s\n", dev_.c_str(), snd_strerror(e));
            pcm_ = nullptr;
            return false;
        }
        if (!raw_set_params(pcm_, "capture", (unsigned)sample_rate_,
                            (snd_pcm_uframes_t)buffer_frames_)) {
            snd_pcm_close(pcm_);
            pcm_ = nullptr;
            return false;
        }
        fprintf(stderr, "[ALSA-RAW] capture ready: %s (S32_LE %dHz)\n",
                dev_.c_str(), sample_rate_);
        return true;
    }

    bool start() override {
        if (!pcm_ || running_) return false;
        running_ = true;
        thread_ = std::thread(&AlsaRawCapture::capture_thread, this);
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
        std::vector<int32_t> s32(frame_count * RAW_HW_CHANNELS);
        int frames = read_frames(s32.data(), frame_count);
        if (frames <= 0) return frames < 0 ? 0 : 0;
        for (int i = 0; i < frames; i++)
            buffer[i] = (float)(s32[i * RAW_HW_CHANNELS] / IRIS_RAW_INT_MAX);
        return frames;
    }

private:
    // Blocking readi with xrun recovery. Returns frames read (0 on transient).
    int read_frames(int32_t* s32, int frame_count) {
        snd_pcm_sframes_t n = snd_pcm_readi(pcm_, s32, frame_count);
        if (n < 0) {
            int r = snd_pcm_recover(pcm_, (int)n, 1);
            if (r < 0) {
                fprintf(stderr, "[ALSA-RAW] capture recover: %s\n", snd_strerror(r));
                return 0;
            }
            n = snd_pcm_readi(pcm_, s32, frame_count);
            if (n < 0) return 0;
        }
        return (int)n;
    }

    void capture_thread() {
        std::vector<int32_t> s32(buffer_frames_ * RAW_HW_CHANNELS);
        std::vector<float> buf(buffer_frames_ * channels_);
        while (running_) {
            int frames = read_frames(s32.data(), buffer_frames_);
            if (frames <= 0) {
                if (running_) std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            float vol = volume_;
            for (int i = 0; i < frames; i++)
                buf[i] = (float)(s32[i * RAW_HW_CHANNELS] / IRIS_RAW_INT_MAX) * vol;
            if (callback_) callback_(buf.data(), frames, channels_);
        }
    }

    snd_pcm_t* pcm_ = nullptr;
    std::string dev_;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

// ── Raw S32 playback ─────────────────────────────────────────────────
class AlsaRawPlayback : public AudioPlayback {
public:
    AlsaRawPlayback() = default;
    ~AlsaRawPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        dev_ = raw_playback_dev();

        int e = snd_pcm_open(&pcm_, dev_.c_str(), SND_PCM_STREAM_PLAYBACK, 0);
        if (e < 0) {
            fprintf(stderr, "[ALSA-RAW] playback open '%s': %s\n", dev_.c_str(), snd_strerror(e));
            pcm_ = nullptr;
            return false;
        }
        if (!raw_set_params(pcm_, "playback", (unsigned)sample_rate_,
                            (snd_pcm_uframes_t)buffer_frames_)) {
            snd_pcm_close(pcm_);
            pcm_ = nullptr;
            return false;
        }
        fprintf(stderr, "[ALSA-RAW] playback ready: %s (S32_LE %dHz)\n",
                dev_.c_str(), sample_rate_);
        return true;
    }

    bool start() override {
        if (!pcm_ || running_) return false;
        running_ = true;
        thread_ = std::thread(&AlsaRawPlayback::playback_thread, this);
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
        std::vector<int32_t> s32(frame_count * RAW_HW_CHANNELS);
        float vol = volume_;
        for (int i = 0; i < frame_count; i++) {
            int32_t v = to_s32(buffer[i] * vol);
            s32[i * RAW_HW_CHANNELS]     = v;
            s32[i * RAW_HW_CHANNELS + 1] = v;
        }
        return write_frames(s32.data(), frame_count);
    }

private:
    static int32_t to_s32(float f) {
        double s = (double)f * IRIS_RAW_INT_MAX;
        if (s >  IRIS_RAW_INT_MAX) s =  IRIS_RAW_INT_MAX;
        if (s < -IRIS_RAW_INT_MAX) s = -IRIS_RAW_INT_MAX;
        return (int32_t)s;
    }

    // Blocking writei with xrun recovery. Returns frames written.
    int write_frames(const int32_t* s32, int frame_count) {
        snd_pcm_sframes_t n = snd_pcm_writei(pcm_, s32, frame_count);
        if (n < 0) {
            int r = snd_pcm_recover(pcm_, (int)n, 1);
            if (r < 0) {
                fprintf(stderr, "[ALSA-RAW] playback recover: %s\n", snd_strerror(r));
                return 0;
            }
            n = snd_pcm_writei(pcm_, s32, frame_count);
            if (n < 0) return 0;
        }
        return (int)n;
    }

    void playback_thread() {
        std::vector<float> buf(buffer_frames_ * channels_, 0.0f);
        std::vector<int32_t> s32(buffer_frames_ * RAW_HW_CHANNELS);
        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            float vol = volume_;
            for (int i = 0; i < buffer_frames_; i++) {
                int32_t v = to_s32(buf[i] * vol);
                s32[i * RAW_HW_CHANNELS]     = v;
                s32[i * RAW_HW_CHANNELS + 1] = v;
            }
            if (write_frames(s32.data(), buffer_frames_) <= 0) {
                if (running_) std::this_thread::sleep_for(std::chrono::milliseconds(5));
            }
        }
    }

    snd_pcm_t* pcm_ = nullptr;
    std::string dev_;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

// ── Factories ─────────────────────────────────────────────────────────
std::unique_ptr<AudioCapture> alsa_raw_create_capture() {
    return std::make_unique<AlsaRawCapture>();
}
std::unique_ptr<AudioPlayback> alsa_raw_create_playback() {
    return std::make_unique<AlsaRawPlayback>();
}

// ── Smoke test ────────────────────────────────────────────────────────
// Open playback + capture on the resolved raw devices and round-trip a
// single 1 kHz buffer. Returns 0 on success, non-zero on failure.
int alsa_raw_smoke_test() {
    const int rate = 48000;
    const int frames = 1024;

    fprintf(stderr, "[ALSA-RAW] smoke: capture=%s playback=%s\n",
            raw_capture_dev().c_str(), raw_playback_dev().c_str());

    AlsaRawPlayback pb;
    AlsaRawCapture cap;

    if (!pb.open(0, rate, 1, frames)) {
        fprintf(stderr, "[ALSA-RAW] smoke: playback open FAILED\n");
        return 1;
    }
    if (!cap.open(0, rate, 1, frames)) {
        fprintf(stderr, "[ALSA-RAW] smoke: capture open FAILED\n");
        return 2;
    }

    // Build a 1 kHz test tone at half scale.
    std::vector<float> tx(frames);
    for (int i = 0; i < frames; i++)
        tx[i] = 0.5f * (float)std::sin(2.0 * M_PI * 1000.0 * i / rate);

    int wrote = pb.write(tx.data(), frames);
    if (wrote != frames) {
        fprintf(stderr, "[ALSA-RAW] smoke: write returned %d (want %d)\n", wrote, frames);
        return 3;
    }

    std::vector<float> rx(frames, 0.0f);
    int got = cap.read(rx.data(), frames);
    if (got <= 0) {
        fprintf(stderr, "[ALSA-RAW] smoke: read returned %d\n", got);
        return 4;
    }

    float peak = 0.0f;
    for (int i = 0; i < got; i++) {
        float a = rx[i] < 0 ? -rx[i] : rx[i];
        if (a > peak) peak = a;
    }
    fprintf(stderr, "[ALSA-RAW] smoke: OK — wrote %d, read %d frames, rx peak=%.4f\n",
            wrote, got, peak);
    return 0;
}

} // namespace iris

#endif // __linux__
