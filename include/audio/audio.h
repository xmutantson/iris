#ifndef IRIS_AUDIO_H
#define IRIS_AUDIO_H

#include <cstdint>
#include <cstddef>
#include <string>
#include <vector>
#include <functional>
#include <memory>

namespace iris {

struct AudioDevice {
    int id;
    std::string name;
    int max_input_channels;
    int max_output_channels;
    int default_sample_rate;
};

// Audio callback: called from audio thread with interleaved float samples
// For capture: samples contains mic input
// For playback: fill samples with output
using AudioCallback = std::function<void(float* samples, int frame_count, int channels)>;

class AudioStream {
public:
    virtual ~AudioStream() = default;

    virtual bool open(int device_id, int sample_rate, int channels,
                      int frames_per_buffer) = 0;
    virtual bool start() = 0;
    virtual bool stop() = 0;
    virtual void close() = 0;
    virtual bool is_running() const = 0;

    // Get/set volume (0.0 to 1.0)
    virtual float volume() const = 0;
    virtual void set_volume(float v) = 0;
};

class AudioCapture : public AudioStream {
public:
    virtual ~AudioCapture() = default;
    virtual void set_callback(AudioCallback cb) = 0;

    // Blocking read (alternative to callback)
    virtual int read(float* buffer, int frame_count) = 0;
};

class AudioPlayback : public AudioStream {
public:
    virtual ~AudioPlayback() = default;
    virtual void set_callback(AudioCallback cb) = 0;

    // Blocking write (alternative to callback)
    virtual int write(const float* buffer, int frame_count) = 0;

    // Mark the current point in the audio pipeline. All audio produced before
    // this call must play out before is_drained() returns true.
    virtual void mark_drain() {}

    // Returns true when all audio produced before mark_drain() has been rendered.
    // Default: always true (no pipeline tracking).
    virtual bool is_drained() const { return true; }
};

// Audio backend selection (Linux only — Windows always uses WASAPI).
// NOTE: append new backends at the END. AudioBackend is a runtime-only
// selection (set via CLI flags in main.cc, never serialized to disk), so
// appending ALSA_RAW keeps AUTO=0/ALSA=1/PULSE=2 and adds ALSA_RAW=3 with
// no persisted value shift.
enum class AudioBackend { AUTO, ALSA, PULSE, ALSA_RAW };
void set_audio_backend(AudioBackend backend);
AudioBackend get_audio_backend();

#if defined(__linux__)
// Bit-exact raw S32_LE snd_pcm ALSA backend (no plug/resample). See
// source/audio/audio_alsa_raw.cc. Per-direction device via env
// IRIS_ALSA_RAW_CAPTURE (default hw:Loopback,1,0) / IRIS_ALSA_RAW_PLAYBACK
// (default hw:Loopback,0,0), or --alsa-raw <hwdev>.
std::unique_ptr<AudioCapture>  alsa_raw_create_capture();
std::unique_ptr<AudioPlayback> alsa_raw_create_playback();
int alsa_raw_smoke_test();  // open + round-trip one buffer; 0 = ok
#endif

// ALSA device name (e.g., "plughw:Audio", "default"). Empty = "default".
void set_alsa_device(const std::string& dev);
const std::string& get_alsa_device();

// Platform-specific factory (respects backend selection on Linux)
std::vector<AudioDevice> enumerate_audio_devices();
std::unique_ptr<AudioCapture> create_capture();
std::unique_ptr<AudioPlayback> create_playback();

// Internal loopback (TX output -> RX input, no audio hardware)
void loopback_reset();
void loopback_set_noise(float amplitude);  // AWGN noise amplitude (0 = off)

// FM channel simulator for loopback testing.
// preemph_us: pre-emphasis time constant (530 = NBFM standard, 0 = off)
// bp_low/bp_high: audio bandpass Hz (300-3000 typical, 0 = off)
// cfo_hz: frequency offset Hz (0 = none)
// deviation_limit: hard-clip threshold (0.95 typical)
void loopback_set_fm_channel(float preemph_us, float bp_low, float bp_high,
                              float cfo_hz, float deviation_limit);

// Configure FM multipath simulation (2-tap: direct + delayed reflection).
// delay_ms: reflection delay in ms (0 = disabled, typical 0.2-2.0 ms)
// gain: reflection amplitude (0-1, e.g., 0.3 = -10.5 dB reflection)
void loopback_set_fm_multipath(float delay_ms, float gain);

// Configure VHF fading (ported from Ionos HF/VHF simulator).
// doppler_hz: Rayleigh fading Doppler spread (0 = disabled, 0.1-2.0 typical)
// flat_depth_db: sinusoidal fade depth in dB (0 = disabled, 0-40)
// flat_rate_hz: sinusoidal fade rate (0.1-20 Hz typical)
void loopback_set_fm_fading(float doppler_hz, float flat_depth_db,
                             float flat_rate_hz);

// Configure VCO frequency drift (Wiener process).
// drift_rate: Hz/sqrt(s) — 3.0 typical for FM radios (0 = disabled)
void loopback_set_fm_drift(float drift_rate);

std::unique_ptr<AudioCapture> create_loopback_capture();
std::unique_ptr<AudioPlayback> create_loopback_playback();

// "-x sim" software-channel backend: connects to Mercury's sim_channel_relay.py
// over TCP and bridges passband audio to a second Iris stack through an external
// channel. See source/audio/audio_sim.cc.
std::unique_ptr<AudioCapture> create_sim_capture();
std::unique_ptr<AudioPlayback> create_sim_playback();

// True once a "-x sim" backend has opened. Real device backends refuse to open
// when this is set, so a stale/misconfigured run cannot leak modem tones to a
// real sound card (mirrors Mercury's GUARD-1 abort, audioio.c:550/967).
bool sim_audio_guard_active();

// RMS level measurement (for calibration)
float measure_rms(const float* samples, int count);

// Peak level measurement
float measure_peak(const float* samples, int count);

} // namespace iris

#endif
