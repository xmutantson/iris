#include "audio/audio.h"
#include "common/logging.h"
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

    // Pre-emphasis: first-order shelving boost, H(s) = (1+s·τ₁)/(1+s·τ₂)
    // Standard NBFM: τ₁=530µs → corner=300 Hz (TIA/EIA-603)
    // τ₂ adds a stabilizing pole at ~15 kHz (well above audio band)
    float preemph_tau_us = 530.0f;
    float preemph_b0 = 1.0f, preemph_b1 = 0.0f;  // IIR numerator
    float preemph_a1 = 0.0f;                       // IIR denominator (feedback)
    float preemph_x1 = 0.0f;    // previous input
    float preemph_y1 = 0.0f;    // previous output (IIR feedback)

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
    // 2nd-order Butterworth (1 biquad section per HP/LP edge).
    // Reduced from 4th-order: the IIR group delay at low carriers (44 samples
    // for 4th-order at 468 Hz) exceeded CP=32, creating an ISI/ICI floor at
    // ~22 dB that broke 64QAM+. 2nd-order has ~20 sample GD, fits within CP.
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
    Biquad bp_hi[2], bp_lo[2];     // Only [0] used; [1] reserved for future
    bool bp_enabled = false;
    float bp_low_hz = 300.0f;
    float bp_high_hz = 3000.0f;

    // Frequency offset: simulates VCO drift between TX and RX radios
    float cfo_hz = 0.0f;           // Hz (typically ±10-50 Hz for FM)
    float cfo_phase = 0.0f;        // accumulator (radians)

    // f²-shaped FM discriminator noise
    // Real FM discriminator output noise PSD ∝ f². We generate white noise,
    // filter through a differentiator (y[n] = x[n] - x[n-1]) for f² shaping,
    // and inject BEFORE de-emphasis so the combined PSD ∝ f²/(1+(2πfτ)²).
    float noise_amplitude = 0.0f;  // 0 = no noise
    float noise_prev = 0.0f;       // differentiator state
    std::mt19937 noise_rng{42};
    std::normal_distribution<float> noise_dist{0.0f, 1.0f};

    // Multipath: 2-tap delay line (direct + delayed reflection)
    // Models RF multipath causing time-domain echoes in the audio.
    // delay_samples: reflection delay (e.g., 10 samples = 0.21ms at 48kHz)
    // reflection_gain: amplitude of reflection (e.g., 0.3 = -10.5 dB)
    int multipath_delay = 0;         // 0 = disabled
    float multipath_gain = 0.0f;     // reflection amplitude (0-1)
    static constexpr int MP_MAX_DELAY = 480;  // max 10ms at 48kHz
    float mp_delay_line[480] = {};
    int mp_write_pos = 0;

    // Frequency drift: Wiener process (random walk in frequency).
    // Models slowly-varying VCO frequency offset between TX and RX radios.
    // OTA logs show residual CFO of 1-7 Hz with drift over the frame.
    // σ_f = 3 Hz/√s → over a 1.3s frame, frequency wanders ~3.4 Hz RMS.
    // Ported from tests.cc batch model to per-sample real-time.
    float freq_drift_rate = 0.0f;    // Hz/√s (0 = disabled, 3.0 typical)
    float drift_freq_offset = 0.0f;  // current accumulated frequency drift (Hz)
    float drift_step_std = 0.0f;     // pre-computed: freq_drift_rate * sqrt(1/fs)
    std::mt19937 drift_rng{77};
    std::normal_distribution<float> drift_dist{0.0f, 1.0f};

    // Rayleigh fading: time-varying complex channel gain (Watterson model).
    // Ported from Ionos HF simulator. Models VHF multipath fading from
    // vehicle motion or atmospheric scintillation.
    // The RF Rayleigh envelope maps to audio-domain amplitude variation:
    //   fade_gain = |H(t)| = sqrt(I² + Q²), normalized to unit mean power.
    // I and Q are independent Gaussian processes filtered to Doppler bandwidth
    // via IIR lowpass (equivalent to Ionos's 128-tap Gaussian FIR at decimated rate).
    float doppler_hz = 0.0f;         // Doppler spread (0 = disabled, 0.1-2 Hz typical)
    float fade_I = 1.0f;             // in-phase channel gain (Gaussian filtered)
    float fade_Q = 0.0f;             // quadrature channel gain
    float fade_alpha = 0.0f;         // IIR coefficient: exp(-2π·f_doppler/fs)
    float fade_scale = 1.0f;         // normalization: sqrt((1+α)/(1-α))
    std::mt19937 fade_rng{99};

    // Flat fading: sinusoidal amplitude modulation.
    // Ported from Ionos Fade() function (modes 6-7).
    // Models signal strength variation from moving platforms.
    // SNR(t) = SNR_max - depth/2 × (1 - cos(2π·rate·t))
    float flat_fade_depth_db = 0.0f; // 0 = disabled, 0-40 dB
    float flat_fade_rate_hz = 0.0f;  // 0.1-20 Hz typical
    float flat_fade_phase = 0.0f;    // accumulator (radians)

    void init(float fs) {
        sample_rate = fs;

        // Pre-emphasis: H(s) = (1+s·τ₁)/(1+s·τ₂) via bilinear transform
        // τ₁ = audio pre-emphasis time constant (530µs for NBFM)
        // τ₂ = stabilizing pole at ~15 kHz (prevents infinite gain at Nyquist)
        // Bilinear: H(z) = [(1+c₁)+(1-c₁)z⁻¹] / [(1+c₂)+(1-c₂)z⁻¹]
        float tau1_s = preemph_tau_us * 1e-6f;
        float tau2_s = 1.0f / (2.0f * (float)M_PI * 15000.0f);  // 10.61µs
        float c1 = 2.0f * fs * tau1_s;  // ~50.88
        float c2 = 2.0f * fs * tau2_s;  // ~1.02
        float pe_a0 = 1.0f + c2;
        preemph_b0 = (1.0f + c1) / pe_a0;
        preemph_b1 = (1.0f - c1) / pe_a0;
        preemph_a1 = (1.0f - c2) / pe_a0;
        preemph_x1 = 0.0f;
        preemph_y1 = 0.0f;

        // De-emphasis: H(s) = 1/(1+sτ) → bilinear transform
        float wc = 1.0f / tau1_s;
        float K = 2.0f * fs;
        float a = K + wc;
        deemph_b0 = wc / a;
        deemph_b1 = wc / a;
        deemph_a1 = (wc - K) / a;
        deemph_z1 = 0.0f;
        deemph_prev = 0.0f;

        // Bandpass filter (2nd-order Butterworth = 1 biquad section per edge)
        if (bp_low_hz > 0 && bp_high_hz > bp_low_hz) {
            bp_enabled = true;
            const float Q = 0.7071f;  // 2nd-order Butterworth Q

            // Highpass (1 section)
            {
                float f0 = bp_low_hz;
                float w0 = 2.0f * (float)M_PI * f0 / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q);
                float a0 = 1.0f + alpha;
                bp_hi[0] = {};
                bp_hi[0].b0 = ((1.0f + c) / 2.0f) / a0;
                bp_hi[0].b1 = -(1.0f + c) / a0;
                bp_hi[0].b2 = ((1.0f + c) / 2.0f) / a0;
                bp_hi[0].a1 = (-2.0f * c) / a0;
                bp_hi[0].a2 = (1.0f - alpha) / a0;
            }
            // Lowpass (1 section)
            {
                float f0 = bp_high_hz;
                float w0 = 2.0f * (float)M_PI * f0 / fs;
                float c = std::cos(w0), s = std::sin(w0);
                float alpha = s / (2.0f * Q);
                float a0 = 1.0f + alpha;
                bp_lo[0] = {};
                bp_lo[0].b0 = ((1.0f - c) / 2.0f) / a0;
                bp_lo[0].b1 = (1.0f - c) / a0;
                bp_lo[0].b2 = ((1.0f - c) / 2.0f) / a0;
                bp_lo[0].a1 = (-2.0f * c) / a0;
                bp_lo[0].a2 = (1.0f - alpha) / a0;
            }
        }

        cfo_phase = 0.0f;

        // Reset multipath delay line
        memset(mp_delay_line, 0, sizeof(mp_delay_line));
        mp_write_pos = 0;

        // Frequency drift
        drift_freq_offset = 0.0f;
        if (freq_drift_rate > 0.0f) {
            float dt = 1.0f / fs;
            drift_step_std = freq_drift_rate * std::sqrt(dt);
        } else {
            drift_step_std = 0.0f;
        }

        // Rayleigh fading: IIR lowpass for Doppler filtering
        if (doppler_hz > 0.0f) {
            fade_alpha = std::exp(-2.0f * (float)M_PI * doppler_hz / fs);
            // Scale so filtered output has unit variance per component.
            // IIR output variance = (1-α)²/(1-α²) × input_variance
            // = (1-α)/(1+α). Scale factor = sqrt((1+α)/(1-α)).
            fade_scale = std::sqrt((1.0f + fade_alpha) / (1.0f - fade_alpha));
            fade_I = 1.0f;
            fade_Q = 0.0f;
        }

        // Flat fading
        flat_fade_phase = 0.0f;
    }

    // Process one sample through the full FM channel model
    float process(float x) {
        // 1. Pre-emphasis (TX radio mic input processing)
        // IIR: y[n] = b0·x[n] + b1·x[n-1] - a1·y[n-1]
        float pe = preemph_b0 * x + preemph_b1 * preemph_x1 - preemph_a1 * preemph_y1;
        preemph_x1 = x;
        preemph_y1 = pe;

        // 2. Deviation limiter (FM radio clips to max deviation)
        if (pe > deviation_limit) pe = deviation_limit;
        if (pe < -deviation_limit) pe = -deviation_limit;

        // 2.3. Rayleigh fading: time-varying amplitude from RF envelope.
        // The FM discriminator SNR is proportional to carrier power.
        // When carrier fades, signal drops and noise rises relative to signal.
        if (doppler_hz > 0.0f) {
            // Update complex channel gain via IIR-filtered Gaussian noise
            float wn_i = drift_dist(fade_rng);
            float wn_q = drift_dist(fade_rng);
            fade_I = fade_alpha * fade_I + (1.0f - fade_alpha) * wn_i * fade_scale;
            fade_Q = fade_alpha * fade_Q + (1.0f - fade_alpha) * wn_q * fade_scale;
            // Rayleigh envelope, normalized so E[gain²] = 1
            // E[I²+Q²] = 2σ² = 2 (unit variance per component), so divide by sqrt(2)
            float rayleigh_env = std::sqrt(fade_I * fade_I + fade_Q * fade_Q);
            float fade_gain = rayleigh_env * 0.7071f;  // 1/sqrt(2)
            pe *= fade_gain;
        }

        // 2.4. Flat fading: sinusoidal amplitude modulation.
        // Ported from Ionos Fade(): gain = 10^(-depth/2 × (1-cos(2π·rate·t))/20)
        if (flat_fade_depth_db > 0.0f && flat_fade_rate_hz > 0.0f) {
            flat_fade_phase += 2.0f * (float)M_PI * flat_fade_rate_hz / sample_rate;
            if (flat_fade_phase > 2.0f * (float)M_PI)
                flat_fade_phase -= 2.0f * (float)M_PI;
            float fade_db = flat_fade_depth_db * 0.5f * (1.0f - std::cos(flat_fade_phase));
            pe *= std::pow(10.0f, -fade_db / 20.0f);
        }

        // 2.5. f²-shaped FM discriminator noise (before de-emphasis)
        if (noise_amplitude > 0.0f) {
            float wn = noise_amplitude * noise_dist(noise_rng);
            float shaped = wn - noise_prev;  // differentiator: +6 dB/octave → f² PSD
            noise_prev = wn;
            pe += shaped;
        }

        // 3. De-emphasis (RX radio audio output processing)
        // Proper IIR: y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
        // deemph_z1 stores previous OUTPUT (feedback), deemph_prev stores previous input
        float de = deemph_b0 * pe + deemph_b1 * deemph_prev - deemph_a1 * deemph_z1;
        deemph_prev = pe;
        deemph_z1 = de;

        // 4. Audio bandpass filter (radio's audio passband, 2nd-order)
        float bp = de;
        if (bp_enabled) {
            bp = bp_hi[0].process(bp);
            bp = bp_lo[0].process(bp);
        }

        // 5. Multipath: direct + delayed reflection
        if (multipath_delay > 0 && multipath_delay < MP_MAX_DELAY) {
            mp_delay_line[mp_write_pos] = bp;
            int read_pos = (mp_write_pos - multipath_delay + MP_MAX_DELAY) % MP_MAX_DELAY;
            bp = bp + multipath_gain * mp_delay_line[read_pos];
            mp_write_pos = (mp_write_pos + 1) % MP_MAX_DELAY;
        }

        // 6. Frequency offset + drift (VCO between radios)
        // Fixed CFO + Wiener-process drift (ported from tests.cc batch model).
        // For real-valued audio, multiply by cos(2π*f(t)*t) shifts spectrum.
        // The drift models VCO crystal aging/temperature variation.
        float out = bp;
        {
            // Update drift: random walk in frequency
            if (drift_step_std > 0.0f) {
                drift_freq_offset += drift_step_std * drift_dist(drift_rng);
            }
            float total_cfo = cfo_hz + drift_freq_offset;
            if (std::abs(total_cfo) > 0.01f) {
                cfo_phase += 2.0f * (float)M_PI * total_cfo / sample_rate;
                if (cfo_phase > (float)M_PI) cfo_phase -= 2.0f * (float)M_PI;
                if (cfo_phase < -(float)M_PI) cfo_phase += 2.0f * (float)M_PI;
                out = bp * std::cos(cfo_phase);
            }
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

void loopback_set_fm_fading(float doppler_hz, float flat_depth_db,
                             float flat_rate_hz) {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    g_fm_channel.doppler_hz = std::max(0.0f, doppler_hz);
    g_fm_channel.flat_fade_depth_db = std::max(0.0f, flat_depth_db);
    g_fm_channel.flat_fade_rate_hz = std::max(0.0f, flat_rate_hz);
    if (g_fm_channel.enabled)
        g_fm_channel.init(48000.0f);
    IRIS_LOG("[FM-FADING] Rayleigh doppler=%.2f Hz, flat depth=%.1f dB rate=%.2f Hz",
             doppler_hz, flat_depth_db, flat_rate_hz);
}

void loopback_set_fm_drift(float drift_rate) {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    g_fm_channel.freq_drift_rate = std::max(0.0f, drift_rate);
    if (g_fm_channel.enabled)
        g_fm_channel.init(48000.0f);
    IRIS_LOG("[FM-DRIFT] rate=%.2f Hz/sqrt(s)", drift_rate);
}

void loopback_set_fm_multipath(float delay_ms, float gain) {
    std::lock_guard<std::mutex> lock(g_loop_mutex);
    int delay_samples = (int)(delay_ms * 48000.0f / 1000.0f + 0.5f);
    if (delay_samples < 0) delay_samples = 0;
    if (delay_samples >= FmChannelState::MP_MAX_DELAY) delay_samples = FmChannelState::MP_MAX_DELAY - 1;
    g_fm_channel.multipath_delay = delay_samples;
    g_fm_channel.multipath_gain = std::max(0.0f, std::min(1.0f, gain));
    if (delay_samples > 0) {
        memset(g_fm_channel.mp_delay_line, 0, sizeof(g_fm_channel.mp_delay_line));
        g_fm_channel.mp_write_pos = 0;
    }
    IRIS_LOG("[FM-MULTIPATH] delay=%d samples (%.2f ms), gain=%.2f (%.1f dB)",
             delay_samples, delay_ms, g_fm_channel.multipath_gain,
             g_fm_channel.multipath_gain > 0 ? 20.0f * std::log10(g_fm_channel.multipath_gain) : -999.0f);
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
                            g_fm_channel.noise_amplitude = noise_amp;
                            sample = g_fm_channel.process(sample);
                        }

                        buf[i] = sample;
                    }
                } else {
                    std::memset(buf.data(), 0, buf_size_ * sizeof(float));
                }
            }
            // Inject noise: f²-shaped if FM channel enabled, flat AWGN otherwise
            if (noise_amp > 0.0f) {
                if (g_fm_channel.enabled) {
                    // Noise already injected inside FM channel process() with f² shaping.
                    // Just update the amplitude (process() reads it each sample).
                } else {
                    for (int i = 0; i < buf_size_; i++)
                        buf[i] += noise_amp * gauss(rng);
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
