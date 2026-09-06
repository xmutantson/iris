#ifndef IRIS_CONFIG_H
#define IRIS_CONFIG_H

#include "native/constellation.h"
#include <string>
#include <map>
#include <cstdint>

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
    bool skip_tune = false;    // true = skip TUNE phase (use current tx_level as-is)
    bool force_ofdm = false;   // true = skip probe, activate OFDM immediately on connect
    int ax25_baud = 1200;      // 1200 or 9600

    // Native PHY
    Modulation max_modulation = Modulation::QAM256;
    float tx_level = 0.5f;     // 0.0-1.0, controls FM deviation
    float rx_gain = 1.0f;

    // Band plan (Hz) — initial native PHY band edges before probe discovery.
    // Default: 1200-2200 Hz (AFSK mark/space width, guaranteed to work if AX.25 does).
    // The comb probe discovers the full usable passband and auto-widens.
    float band_low_hz = 1200.0f;   // Low edge (AFSK mark tone)
    float band_high_hz = 2200.0f;  // High edge (AFSK space tone)
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

    // TCP bind address for the KISS and AGW servers. Default 127.0.0.1
    // (loopback only). These servers grant a connected client UNAUTHENTICATED
    // control of the transmitter (inject AX.25 frames, key the radio under the
    // operator's callsign), so on a licensed radio service they must not be
    // reachable from the LAN unless the operator explicitly opts in. Set to
    // "0.0.0.0" (or "any") to listen on all interfaces. A malformed value is
    // rejected at startup — never silently widened to all interfaces.
    std::string bind_address = "127.0.0.1";

    // Security
    int encryption_mode = 0;     // 0=off, 1=strict (hybrid KX before data), 2=fast (classical-first)
    std::string psk_hex;         // Pre-shared key in hex (up to 128 chars = 64 bytes)

    // Native hail: use native PHY (BPSK+LDPC) for HAIL/CONNECT instead of AFSK
    // ~10 dB better sensitivity than 1200 baud AFSK. Both sides must enable.
    bool native_hail = false;

    // FX.25 Forward Error Correction for AX.25
    // 0 = off (plain AX.25), 16/32/64 = number of RS check bytes
    // RX always decodes FX.25 regardless of this setting (backwards compatible)
    int fx25_mode = 0;

    // AFSK pre-emphasis compensation (for FM de-emphasis)
    // 0.95 = standard FM (compensate ~6dB/octave de-emphasis)
    // 0.0 = flat (no compensation, use for discriminator taps or flat audio)
    float preemph_alpha = 0.95f;

    // B2F
    bool b2f_unroll = true;      // Enable B2F LZHUF unroll/reroll for Winlink

    // Calibration
    float calibrated_tx_level = -1.0f;  // -1 = not calibrated

    // Test: simulated radio bandpass filter (--bandpass low-high)
    // When set, all TX audio is bandpass-filtered to simulate radio passband.
    float sim_bandpass_low = 0.0f;   // 0 = disabled
    float sim_bandpass_high = 0.0f;

    // Test: simulated FM de-emphasis (--deemphasis <us>)
    // Standard values: 75 (US/Japan), 50 (Europe/Australia). 0 = disabled.
    float sim_deemph_us = 0.0f;

    // DCD (Data Carrier Detect) — defer TX when channel is busy
    // 0 = disabled, typical values: 0.05-0.20 (adjustable in GUI)
    float dcd_threshold = 0.05f;

    // DCD holdoff: how long to keep TX suppressed after carrier drops (ms).
    // Must exceed FM squelch tail and inter-frame gaps within a burst.
    // 500ms covers squelch tail (~100-300ms) plus AX.25 inter-frame gap.
    int dcd_holdoff_ms = 500;

    // Auto-DCD: automatically calibrate DCD threshold and detect inverted-squelch
    // radios where the noise floor drops when a signal is present.
    // When enabled, DCD measures the noise baseline at startup and adjusts polarity.
    bool dcd_auto = true;

    // Data directory for caches (speed level cache, etc.)
    // Set to %APPDATA%/Iris (Windows) or ~/.config/iris (Linux) by main.cc
    std::string data_dir;

    // Logging
    bool log_enabled = false;    // Auto-log to AppData/Iris/logs/

    // OFDM PHY settings
    bool ofdm_enable = true;           // Master OFDM enable/disable
    int ofdm_nfft = 1024;             // FFT size (256, 512, or 1024) — 46.875 Hz spacing at 48k (~44 baud)
    int ofdm_cp_samples = 64;         // Cyclic prefix samples (1.33ms). FM ~0 delay spread; minimal CP.
    bool ofdm_auto_spacing = false;   // Auto-train subcarrier spacing
    bool ofdm_waterfill = true;       // Per-subcarrier adaptive bit loading
    bool ofdm_nuc = true;             // Non-uniform constellations
    float ofdm_preemph_corner_hz = 300.0f;  // FM pre-emphasis corner (0=flat/data port)
    // Clean-channel decode mode: use broadcast AWGN noise floor (measured
    // from guard FFT bins) for MMSE equalization and LLR scaling, instead
    // of per-carrier H-smoothness residual. On flat channels the residual
    // confuses signal shape for noise and inflates nv by 10-15 dB (fact
    // doc §5.3/§6.5a). Safe on AWGN-dominant paths; hurts on strongly
    // frequency-selective channels (FM sim). Default false.
    bool ofdm_clean_channel = false;
    // PAPR clipper disable: skip the OFDM PAPR hard clipper. On FM paths
    // this is mandatory (deviation limiter would clip anything above 7 dB
    // PAPR). On clean cable it adds 15-20 dB of distortion on QPSK but —
    // counter-intuitively — the Fe-Pi ADC has its own saturation behavior
    // that the TX clipper pre-shapes for. Testing on OTA showed disabling
    // the clipper HURT throughput (see fact doc §7.3). Kept as a separate
    // flag from clean_channel so the NV-broadcast and clipper-off behaviors
    // can be tested independently. Default false.
    bool ofdm_skip_papr_clip = false;
    // LLR-scalar source: when true, dft_sigma_sq for LLR scaling is computed
    // from guard-bin nv_frame (frame-wide AWGN floor) instead of per-carrier
    // H-smoothness residual. Equalizer's per-carrier noise_var is unchanged.
    // Codec2/FreeDV-700 uses an analogous frame-wide scalar (mpdecode_core.c
    // Demod2D, EsNo=3.0 hardcoded). On flat channels the per-carrier path
    // is poisoned by deep-fade carriers (fact doc §6.5a/§9). Default false.
    bool ofdm_llr_use_frame_nv = false;

    // Digipeater role ([Digipeat] section) — an operator DEPLOYMENT choice,
    // so config-gated and DEFAULT OFF: a modem must never start repeating
    // other stations' traffic unless the operator opts in.  Shape ported from
    // Direwolf's config surface (DIGIPEAT line, ../direwolf/src/config.c:2717;
    // CDIGIPEAT line, :2956), collapsed to Iris's single AFSK channel.
    // AFSK->AFSK only; no OFDM cross-mode relay.
    bool digipeat_enable = false;
    // Comma-separated callsign-SSIDs we digipeat for (exact match on the next
    // un-repeated via hop).  Empty = the station Callsign-SSID.
    std::string digipeat_mycall;
    bool digipeat_connected = true;   // connected-mode I/S/U path (cdigipeater.c)
    bool digipeat_ui = true;          // UI/APRS path (digipeater.c + dedupe.c)
    std::string digipeat_ui_alias;    // UI alias regex (empty = none)
    std::string digipeat_ui_wide = "^WIDE[1-7]-[1-7]$";  // UI WIDEn-N regex
    int digipeat_dedupe_ttl_s = 30;   // UI duplicate window (dedupe.c default)

    // GUI
    bool show_constellation = true;
    bool show_waterfall = true;
};

// Load config from INI file. Missing keys get defaults.
IrisConfig load_config(const std::string& path);

// Save config to INI file.
bool save_config(const std::string& path, const IrisConfig& cfg);

// Parse an IPv4 bind-address string into a host-byte-order 32-bit address.
// Accepts a strict dotted-quad literal ("127.0.0.1", "0.0.0.0") or the
// case-insensitive alias "any" (== 0.0.0.0 == all interfaces). Returns false on
// malformed input WITHOUT modifying out_host_order; callers MUST treat false as
// fatal and MUST NOT fall back to INADDR_ANY (that would silently expose the
// transmitter to the LAN).
bool parse_bind_address(const std::string& s, uint32_t& out_host_order);

// Effective bind address given the config value and an optional CLI override.
// The CLI value wins when non-empty (mirrors the --port / --agw-port rule).
inline std::string effective_bind_address(const std::string& cfg_value,
                                          const std::string& cli_value) {
    return cli_value.empty() ? cfg_value : cli_value;
}

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
