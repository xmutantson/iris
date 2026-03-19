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

// Platform-specific factory
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

std::unique_ptr<AudioCapture> create_loopback_capture();
std::unique_ptr<AudioPlayback> create_loopback_playback();

// RMS level measurement (for calibration)
float measure_rms(const float* samples, int count);

// Peak level measurement
float measure_peak(const float* samples, int count);

} // namespace iris

#endif
