#include "audio/audio.h"
#include <thread>
#include <atomic>
#include <mutex>
#include <vector>
#include <cstring>
#include <cmath>
#include <random>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// Shared loopback buffer: playback writes here, capture reads from here.
// Zero-delay passthrough — process_rx already skips TX mute in loopback mode.
// Real TX/RX turnaround delays come from ptt_pre_delay_ms / ptt_post_delay_ms.
static std::mutex g_loop_mutex;
static std::vector<float> g_loop_buffer;
static size_t g_loop_read_pos = 0;
static size_t g_loop_write_pos = 0;
static constexpr size_t LOOP_BUF_SIZE = 48000 * 8;   // 8 seconds at 48kHz mono

// AWGN noise injection: amplitude of white Gaussian noise added to loopback
static std::atomic<float> g_noise_amplitude{0.0f};

// ============================================================================
//  FM Channel Simulator
//
//  Models the audio path through an FM radio pair:
//    TX mic input → pre-emphasis → deviation limiter → [RF] → de-emphasis
//                 → audio bandpass filter → frequency offset → AWGN
//
//  Enabled globally via loopback_set_fm_channel(). When disabled (default),
//  the loopback is a clean passthrough (same as before).
// ============================================================================

struct FmChannelState {
    bool enabled = false;
    float sample_rate = 48000.0f;

    // Pre-emphasis: first-order highpass, H(s) = 1+sτ (6 dB/octave boost above corner)
    // Standard NBFM: τ=530µs → corner=300 Hz (TIA/EIA-603)
    float preemph_tau_us = 530.0f;
    float preemph_alpha = 0.0f;     // coefficient: 1-pole IIR
    float preemph_prev = 0.0f;      // filter state

    // Deviation limiter: hard-clips audio to simulate FM deviation limiting
    // Real FM radios limit to ±5 kHz deviation. The limiter clips the
    // pre-emphasized audio, which is the dominant source of OFDM degradation.
    float deviation_limit = 0.95f;  // clip threshold (after pre-emphasis)

    // De-emphasis: first-order lowpass, H(s) = 1/(1+sτ)
    // Cancels pre-emphasis to restore flat frequency response for voice.
    // For OFDM data, the TX pre-emphasis compensation should cancel this.
    float deemph_b0 = 0.0f, deemph_b1 = 0.0f;
    float deemph_a1 = 0.0f;
    float deemph_z1 = 0.0f;     // previous output (IIR feedback)
    float deemph_prev = 0.0f;   // previous input

    // Audio bandpass filter: models radio's audio passband (typically 300-3000 Hz)
    // 4th-order Butterworth (2 cascaded biquad sections per HP/LP edge)
    struct Biquad {
        float b0=1,b1=0,b2=0,a1=0,a2=0,z1=0,z2=0;
        float process(float x) {
            // Direct Form II: w = x - a1*z1 - a2*z2, y = b0*w + b1*z1 + b2*z2
            float w = x - a1*z1 - a2*z2;
            float y = b0*w + b1*z1 + b2*z2;
            z2 = z1; z1 = w;
            return y;
        }
    };
    Biquad bp_hi[2], bp_lo[2];     // 2-stage HP + 2-stage LP = 4th order BP
    bool bp_enabled = false;
    float bp_low_hz = 300.0f;
    float bp_high_hz = 3000.0f;

    // Frequency offset: simulates VCO drift between TX and RX radios
    float cfo_hz = 0.0f;           // Hz (typically ±10-50 Hz for FM)
    float cfo_phase = 0.0f;        // accumulator (radians)

    // Noise (AWGN is already separate, but we track FM-specific SNR here)
    float snr_db = 30.0f;          // FM audio SNR after demod (30 dB = clean, 10 dB = noisy)

    void init(float fs) {
        sample_rate = fs;

        // Pre-emphasis: y[n] = x[n] - α*x[n-1] (first-order highpass)
        // α = exp(-1/(τ*fs)) ≈ 1 - 1/(τ*fs) for small 1/(τ*fs)
        float tau_s = preemph_tau_us * 1e-6f;
        preemph_alpha = std::exp(-1.0f / (tau_s * fs));
        preemph_prev = 0.0f;

        // De-emphasis: H(s) = 1/(1+sτ) → bilinear transform
        float wc = 1.0f / tau_s;
        float K = 2.0f * fs;
        float a = K + wc;
        deemph_b0 = wc / a;
        deemph_b1 = wc / a;
        deemph_a1 = (wc - K) / a;
        deemph_z1 = 0.0f;
        deemph_prev = 0.0f;

        // Bandpass filter (4th-order Butterworth = 2 biquad sections per edge)
        if (bp_low_hz > 0 && bp_high_hz > bp_low_hz) {
            bp_enabled = true;
            const float Q4[2] = {0.5412f, 1.3066f};  // 4th-order Butterworth Q values

            // Highpass sections
            for (int i = 0; i < 2; i++) {
                float f0 = bp_low_hz;
                float w0 = 2.0f * (float)M_PI * f0 / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q4[i]);
                float a0 = 1.0f + alpha;
                bp_hi[i] = {};
                bp_hi[i].b0 = ((1.0f + c) / 2.0f) / a0;
                bp_hi[i].b1 = -(1.0f + c) / a0;
                bp_hi[i].b2 = ((1.0f + c) / 2.0f) / a0;
                bp_hi[i].a1 = (-2.0f * c) / a0;
                bp_hi[i].a2 = (1.0f - alpha) / a0;
            }
            // Lowpass sections
            for (int i = 0; i < 2; i++) {
                float f0 = bp_high_hz;
                float w0 = 2.0f * (float)M_PI * f0 / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q4[i]);
                float a0 = 1.0f + alpha;
                bp_lo[i] = {};
                bp_lo[i].b0 = ((1.0f - c) / 2.0f) / a0;
                bp_lo[i].b1 = (1.0f - c) / a0;
                bp_lo[i].b2 = ((1.0f - c) / 2.0f) / a0;
                bp_lo[i].a1 = (-2.0f * c) / a0;
                bp_lo[i].a2 = (1.0f - alpha) / a0;
            }
        }

        cfo_phase = 0.0f;
    }

    // Process one sample through the full FM channel model
    float process(float x) {
        // 1. Pre-emphasis (TX radio mic input processing)
        float pe = x - preemph_alpha * preemph_prev;
        preemph_prev = x;

        // 2. Deviation limiter (FM radio clips to max deviation)
        if (pe > deviation_limit) pe = deviation_limit;
        if (pe < -deviation_limit) pe = -deviation_limit;

        // 3. De-emphasis (RX radio audio output processing)
        // Proper IIR: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
        // deemph_z1 stores previous OUTPUT (feedback), deemph_prev stores previous input
        float de = deemph_b0 * pe + deemph_b1 * deemph_prev - deemph_a1 * deemph_z1;
        deemph_prev = pe;
        deemph_z1 = de;

        // 4. Audio bandpass filter (radio's audio passband)
        float bp = de;
        if (bp_enabled) {
            for (int i = 0; i < 2; i++) bp = bp_hi[i].process(bp);
            for (int i = 0; i < 2; i++) bp = bp_lo[i].process(bp);
        }

        // 5. Frequency offset (VCO drift between radios)
        // For real-valued audio, frequency offset shifts the entire spectrum.
        // Model as: multiply by cos(2π*cfo*t) which shifts energy by ±cfo Hz.
        // This is a simplification — true FM offset shifts the demodulated
        // audio spectrum linearly, which for narrowband is well-approximated
        // by this multiplication.
        float out = bp;
        if (std::abs(cfo_hz) > 0.01f) {
            cfo_phase += 2.0f * (float)M_PI * cfo_hz / sample_rate;
            if (cfo_phase > (float)M_PI) cfo_phase -= 2.0f * (float)M_PI;
            if (cfo_phase < -(float)M_PI) cfo_phase += 2.0f * (float)M_PI;
            out = bp * std::cos(cfo_phase);
        }

        return out;
    }
};

static FmChannelState g_fm_channel;

void loopback_reset() {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    g_loop_buffer.assign(LOOP_BUF_SIZE, 0.0f);
    g_loop_read_pos = 0;
    g_loop_write_pos = 0;
}

void loopback_set_noise(float amplitude) {
    g_noise_amplitude.store(amplitude);
}

// Configure the FM channel simulator.
// preemph_us: pre-emphasis time constant in microseconds (530 = NBFM, 0 = disabled)
// bp_low/bp_high: audio bandpass in Hz (300-3000 typical, 0 = disabled)
// cfo_hz: frequency offset in Hz (0 = none)
// deviation_limit: hard clip threshold (0.95 typical)
void loopback_set_fm_channel(float preemph_us, float bp_low, float bp_high,
                              float cfo_hz, float deviation_limit) {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    g_fm_channel.preemph_tau_us = preemph_us;
    g_fm_channel.bp_low_hz = bp_low;
    g_fm_channel.bp_high_hz = bp_high;
    g_fm_channel.cfo_hz = cfo_hz;
    g_fm_channel.deviation_limit = deviation_limit;
    g_fm_channel.enabled = (preemph_us > 0 || bp_low > 0 || cfo_hz != 0.0f);
    if (g_fm_channel.enabled) {
        g_fm_channel.init(48000.0f);
    }
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
        std::mt19937 rng(42);
        std::normal_distribution<float> gauss(0.0f, 1.0f);

        while (running_) {
            float noise_amp = g_noise_amplitude.load();
            {
                std::lock_guard<std::mutex> lock(g_loop_mutex);
                size_t avail = (g_loop_write_pos >= g_loop_read_pos)
                    ? g_loop_write_pos - g_loop_read_pos
                    : LOOP_BUF_SIZE - g_loop_read_pos + g_loop_write_pos;
                if (avail >= (size_t)buf_size_) {
                    for (int i = 0; i < buf_size_; i++) {
                        float sample = g_loop_buffer[g_loop_read_pos];
                        g_loop_read_pos = (g_loop_read_pos + 1) % LOOP_BUF_SIZE;

                        // Apply FM channel model if enabled
                        if (g_fm_channel.enabled) {
                            sample = g_fm_channel.process(sample);
                        }

                        buf[i] = sample;
                    }
                } else {
                    std::memset(buf.data(), 0, buf_size_ * sizeof(float));
                }
            }
            // Inject AWGN if configured
            if (noise_amp > 0.0f) {
                for (int i = 0; i < buf_size_; i++)
                    buf[i] += noise_amp * gauss(rng);
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
