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

    // Radio / PTT
    std::string ptt_method = "none";   // none, rigctl, vox, cm108
    std::string rigctl_host = "localhost";
    int rigctl_port = 4532;
    std::string serial_port;
    int serial_baud = 9600;

    // Network
    int kiss_port = 8001;

    // Calibration
    float calibrated_tx_level = -1.0f;  // -1 = not calibrated

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
