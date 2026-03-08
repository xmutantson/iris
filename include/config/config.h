#ifndef IRIS_CONFIG_H
#define IRIS_CONFIG_H

#include "native/constellation.h"
#include <string>
#include <map>

namespace iris {

struct IrisConfig {
    // Station
    std::string callsign = "N0CALL";
    int ssid = 0;

    // Audio
    int capture_device = -1;   // -1 = default
    int playback_device = -1;
    int sample_rate = 48000;

    // Mode
    std::string mode = "A";    // A, B, C
    bool ax25_only = false;    // true = no native upgrade
    int ax25_baud = 1200;      // 1200 or 9600

    // Native PHY
    Modulation max_modulation = Modulation::QAM256;
    float tx_level = 0.5f;     // 0.0-1.0, controls FM deviation
    float rx_gain = 1.0f;

    // Band plan (Hz) — configurable for different radio passband characteristics
    // Default: 300-3500 Hz (typical FM audio passband, above CTCSS max 254 Hz)
    float band_low_hz = 300.0f;    // Low edge (must be > 254 Hz to avoid CTCSS)
    float band_high_hz = 3500.0f;  // High edge
    float center_freq_hz = 0.0f;   // 0 = auto (midpoint of band_low/band_high)

    // Radio / PTT
    std::string ptt_method = "none";   // none, rigctl, vox, cm108, serial
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    int rigctld_model = 0;          // Hamlib model number (0 = external rigctld, user manages it)
    std::string rigctld_device;     // Serial port for rigctld to open (e.g., COM3)
    std::string serial_port;
    int serial_baud = 9600;
    int serial_ptt_line = 0;    // 0=RTS, 1=DTR
    int cm108_gpio = 3;         // CM108 GPIO pin for PTT (typically 3)
    int ptt_pre_delay_ms = 100;   // TXDelay: PTT key-up settle time (FT-60 ~100ms)
    int ptt_post_delay_ms = 50;   // TXTail: PTT release tail (FT-60 ~50ms)

    // CSMA/CA channel access (AX.25 standard)
    int slottime_ms = 100;        // CSMA slot time (direwolf default 100ms)
    int persist = 63;             // p-persistence 0-255 (63 = 25%, direwolf default)

    // Network
    int kiss_port = 8001;
    int agw_port = 8000;

    // Security
    int encryption_mode = 0;     // 0=off, 1=strict (hybrid KX before data), 2=fast (classical-first)
    std::string psk_hex;         // Pre-shared key in hex (up to 128 chars = 64 bytes)

    // Native hail: use native PHY (BPSK+LDPC) for HAIL/CONNECT instead of AFSK
    // ~10 dB better sensitivity than 1200 baud AFSK. Both sides must enable.
    bool native_hail = false;

    // B2F
    bool b2f_unroll = true;      // Enable B2F LZHUF unroll/reroll for Winlink

    // Calibration
    float calibrated_tx_level = -1.0f;  // -1 = not calibrated

    // Test: simulated radio bandpass filter (--bandpass low-high)
    // When set, all TX audio is bandpass-filtered to simulate radio passband.
    float sim_bandpass_low = 0.0f;   // 0 = disabled
    float sim_bandpass_high = 0.0f;

    // Logging
    bool log_enabled = false;    // Auto-log to AppData/Iris/logs/

    // GUI
    bool show_constellation = true;
    bool show_waterfall = true;
};

// Load config from INI file. Missing keys get defaults.
IrisConfig load_config(const std::string& path);

// Save config to INI file
bool save_config(const std::string& path, const IrisConfig& cfg);

// Simple INI parser
class IniFile {
public:
    bool load(const std::string& path);
    bool save(const std::string& path) const;

    std::string get(const std::string& section, const std::string& key,
                    const std::string& default_val = "") const;
    int get_int(const std::string& section, const std::string& key,
                int default_val = 0) const;
    float get_float(const std::string& section, const std::string& key,
                    float default_val = 0.0f) const;
    bool get_bool(const std::string& section, const std::string& key,
                  bool default_val = false) const;

    void set(const std::string& section, const std::string& key,
             const std::string& value);
    void set_int(const std::string& section, const std::string& key, int value);
    void set_float(const std::string& section, const std::string& key, float value);
    void set_bool(const std::string& section, const std::string& key, bool value);

private:
    // section -> (key -> value)
    std::map<std::string, std::map<std::string, std::string>> data_;
};

} // namespace iris

#endif
