#include "config/config.h"
#include <fstream>
#include <sstream>
#include <algorithm>
#include <set>

namespace iris {

// --- IniFile ---

bool IniFile::load(const std::string& path) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line, section;
    while (std::getline(f, line)) {
        // Trim
        while (!line.empty() && (line.back() == '\r' || line.back() == '\n' || line.back() == ' '))
            line.pop_back();
        while (!line.empty() && line.front() == ' ')
            line.erase(line.begin());

        if (line.empty() || line[0] == '#' || line[0] == ';')
            continue;

        if (line[0] == '[') {
            auto end = line.find(']');
            if (end != std::string::npos)
                section = line.substr(1, end - 1);
            continue;
        }

        auto eq = line.find('=');
        if (eq == std::string::npos) continue;

        std::string key = line.substr(0, eq);
        std::string val = line.substr(eq + 1);

        // Trim key and value
        while (!key.empty() && key.back() == ' ') key.pop_back();
        while (!val.empty() && val.front() == ' ') val.erase(val.begin());

        data_[section][key] = val;
    }
    return true;
}

bool IniFile::save(const std::string& path) const {
    std::ofstream f(path);
    if (!f.is_open()) return false;

    for (auto& [section, kvs] : data_) {
        if (!section.empty())
            f << "[" << section << "]\n";
        for (auto& [key, val] : kvs)
            f << key << " = " << val << "\n";
        f << "\n";
    }
    return true;
}

std::string IniFile::get(const std::string& section, const std::string& key,
                          const std::string& default_val) const {
    auto sit = data_.find(section);
    if (sit == data_.end()) return default_val;
    auto kit = sit->second.find(key);
    if (kit == sit->second.end()) return default_val;
    return kit->second;
}

int IniFile::get_int(const std::string& section, const std::string& key, int default_val) const {
    std::string s = get(section, key);
    if (s.empty()) return default_val;
    try { return std::stoi(s); } catch (...) { return default_val; }
}

float IniFile::get_float(const std::string& section, const std::string& key, float default_val) const {
    std::string s = get(section, key);
    if (s.empty()) return default_val;
    try { return std::stof(s); } catch (...) { return default_val; }
}

bool IniFile::get_bool(const std::string& section, const std::string& key, bool default_val) const {
    std::string s = get(section, key);
    if (s.empty()) return default_val;
    return (s == "true" || s == "1" || s == "yes");
}

void IniFile::set(const std::string& section, const std::string& key, const std::string& value) {
    data_[section][key] = value;
}

void IniFile::set_int(const std::string& section, const std::string& key, int value) {
    set(section, key, std::to_string(value));
}

void IniFile::set_float(const std::string& section, const std::string& key, float value) {
    std::ostringstream oss;
    oss << value;
    set(section, key, oss.str());
}

void IniFile::set_bool(const std::string& section, const std::string& key, bool value) {
    set(section, key, value ? "true" : "false");
}

// --- Config load/save ---

// Check which parameters need default-reset due to version bump.
// Returns a set of "Section.Key" strings that should use code defaults.
static std::set<std::string> find_stale_params(const IniFile& ini) {
    std::set<std::string> stale;
    for (int i = 0; i < NUM_PARAM_VERSIONS; i++) {
        const auto& pv = PARAM_VERSIONS[i];
        std::string ver_key = std::string(pv.section) + "_" + pv.key + "_ver";
        int ini_ver = ini.get_int("Version", ver_key, 0);
        if (ini_ver < pv.version) {
            stale.insert(std::string(pv.section) + "." + pv.key);
        }
    }
    return stale;
}

IrisConfig load_config(const std::string& path) {
    IrisConfig cfg;
    IniFile ini;
    if (!ini.load(path)) return cfg;

    auto stale = find_stale_params(ini);

    cfg.callsign = ini.get("Station", "Callsign", cfg.callsign);
    cfg.ssid = ini.get_int("Station", "SSID", cfg.ssid);

    cfg.capture_device = ini.get_int("Audio", "CaptureDevice", cfg.capture_device);
    cfg.playback_device = ini.get_int("Audio", "PlaybackDevice", cfg.playback_device);
    cfg.sample_rate = ini.get_int("Audio", "SampleRate", cfg.sample_rate);

    cfg.mode = ini.get("Modem", "Mode", cfg.mode);
    cfg.ax25_only = ini.get_bool("Modem", "AX25Only", cfg.ax25_only);
    cfg.ax25_baud = ini.get_int("Modem", "AX25Baud", cfg.ax25_baud);

    int max_mod = ini.get_int("Modem", "MaxModulation", (int)cfg.max_modulation);
    if (max_mod >= 0 && max_mod <= (int)Modulation::QAM256)
        cfg.max_modulation = (Modulation)max_mod;

    cfg.tx_level = ini.get_float("Modem", "TxLevel", cfg.tx_level);
    cfg.rx_gain = ini.get_float("Modem", "RxGain", cfg.rx_gain);

    cfg.ptt_method = ini.get("Radio", "PTTMethod", cfg.ptt_method);
    cfg.rigctl_host = ini.get("Radio", "RigctlHost", cfg.rigctl_host);
    cfg.rigctl_port = ini.get_int("Radio", "RigctlPort", cfg.rigctl_port);
    cfg.rigctld_model = ini.get_int("Radio", "RigctldModel", cfg.rigctld_model);
    cfg.rigctld_device = ini.get("Radio", "RigctldDevice", cfg.rigctld_device);
    cfg.serial_port = ini.get("Radio", "SerialPort", cfg.serial_port);
    cfg.serial_baud = ini.get_int("Radio", "SerialBaud", cfg.serial_baud);
    cfg.serial_ptt_line = ini.get_int("Radio", "SerialPTTLine", cfg.serial_ptt_line);
    cfg.cm108_gpio = ini.get_int("Radio", "CM108GPIO", cfg.cm108_gpio);
    cfg.ptt_pre_delay_ms = ini.get_int("Radio", "TXDelay", cfg.ptt_pre_delay_ms);
    cfg.ptt_post_delay_ms = ini.get_int("Radio", "TXTail", cfg.ptt_post_delay_ms);
    cfg.slottime_ms = ini.get_int("Radio", "SlotTime", cfg.slottime_ms);
    cfg.persist = ini.get_int("Radio", "Persist", cfg.persist);

    cfg.kiss_port = ini.get_int("Network", "KISSPort", cfg.kiss_port);
    cfg.agw_port = ini.get_int("Network", "AGWPort", cfg.agw_port);

    cfg.encryption_mode = ini.get_int("Security", "EncryptionMode", cfg.encryption_mode);
    cfg.psk_hex = ini.get("Security", "PSK", cfg.psk_hex);

    if (!stale.count("Modem.BandLowHz"))
        cfg.band_low_hz = ini.get_float("Modem", "BandLowHz", cfg.band_low_hz);
    if (!stale.count("Modem.BandHighHz"))
        cfg.band_high_hz = ini.get_float("Modem", "BandHighHz", cfg.band_high_hz);
    cfg.center_freq_hz = ini.get_float("Modem", "CenterFreqHz", cfg.center_freq_hz);

    cfg.b2f_unroll = ini.get_bool("Modem", "B2FUnroll", cfg.b2f_unroll);
    cfg.native_hail = ini.get_bool("Modem", "NativeHail", cfg.native_hail);
    if (!stale.count("Modem.FX25Mode"))
        cfg.fx25_mode = ini.get_int("Modem", "FX25Mode", cfg.fx25_mode);
    cfg.preemph_alpha = ini.get_float("Modem", "PreemphAlpha", cfg.preemph_alpha);

    cfg.calibrated_tx_level = ini.get_float("Calibration", "TxLevel", cfg.calibrated_tx_level);

    cfg.log_enabled = ini.get_bool("Logging", "LogEnabled", cfg.log_enabled);

    if (!stale.count("Modem.DcdThreshold"))
        cfg.dcd_threshold = ini.get_float("Modem", "DcdThreshold", cfg.dcd_threshold);
    cfg.dcd_holdoff_ms = ini.get_int("Modem", "DcdHoldoffMs", cfg.dcd_holdoff_ms);
    cfg.dcd_auto = ini.get_bool("Modem", "DcdAuto", cfg.dcd_auto);

    cfg.sim_bandpass_low = ini.get_float("Test", "SimBandpassLow", cfg.sim_bandpass_low);
    cfg.sim_bandpass_high = ini.get_float("Test", "SimBandpassHigh", cfg.sim_bandpass_high);

    cfg.ofdm_enable = ini.get_bool("OFDM", "Enable", cfg.ofdm_enable);
    cfg.ofdm_nfft = ini.get_int("OFDM", "NFFT", cfg.ofdm_nfft);
    cfg.ofdm_auto_spacing = ini.get_bool("OFDM", "AutoSpacing", cfg.ofdm_auto_spacing);
    cfg.ofdm_waterfill = ini.get_bool("OFDM", "Waterfill", cfg.ofdm_waterfill);
    cfg.ofdm_nuc = ini.get_bool("OFDM", "NUC", cfg.ofdm_nuc);
    cfg.ofdm_preemph_corner_hz = ini.get_float("OFDM", "PreemphCornerHz", cfg.ofdm_preemph_corner_hz);

    cfg.show_constellation = ini.get_bool("GUI", "ShowConstellation", cfg.show_constellation);
    cfg.show_waterfall = ini.get_bool("GUI", "ShowWaterfall", cfg.show_waterfall);

    return cfg;
}

bool save_config(const std::string& path, const IrisConfig& cfg) {
    IniFile ini;

    ini.set("Station", "Callsign", cfg.callsign);
    ini.set_int("Station", "SSID", cfg.ssid);

    ini.set_int("Audio", "CaptureDevice", cfg.capture_device);
    ini.set_int("Audio", "PlaybackDevice", cfg.playback_device);
    ini.set_int("Audio", "SampleRate", cfg.sample_rate);

    ini.set("Modem", "Mode", cfg.mode);
    ini.set_bool("Modem", "AX25Only", cfg.ax25_only);
    ini.set_int("Modem", "AX25Baud", cfg.ax25_baud);
    ini.set_int("Modem", "MaxModulation", (int)cfg.max_modulation);
    ini.set_float("Modem", "TxLevel", cfg.tx_level);
    ini.set_float("Modem", "RxGain", cfg.rx_gain);

    ini.set("Radio", "PTTMethod", cfg.ptt_method);
    ini.set("Radio", "RigctlHost", cfg.rigctl_host);
    ini.set_int("Radio", "RigctlPort", cfg.rigctl_port);
    ini.set_int("Radio", "RigctldModel", cfg.rigctld_model);
    ini.set("Radio", "RigctldDevice", cfg.rigctld_device);
    ini.set("Radio", "SerialPort", cfg.serial_port);
    ini.set_int("Radio", "SerialBaud", cfg.serial_baud);
    ini.set_int("Radio", "SerialPTTLine", cfg.serial_ptt_line);
    ini.set_int("Radio", "CM108GPIO", cfg.cm108_gpio);
    ini.set_int("Radio", "TXDelay", cfg.ptt_pre_delay_ms);
    ini.set_int("Radio", "TXTail", cfg.ptt_post_delay_ms);
    ini.set_int("Radio", "SlotTime", cfg.slottime_ms);
    ini.set_int("Radio", "Persist", cfg.persist);

    ini.set_int("Network", "KISSPort", cfg.kiss_port);
    ini.set_int("Network", "AGWPort", cfg.agw_port);

    ini.set_int("Security", "EncryptionMode", cfg.encryption_mode);
    ini.set("Security", "PSK", cfg.psk_hex);

    ini.set_float("Modem", "BandLowHz", cfg.band_low_hz);
    ini.set_float("Modem", "BandHighHz", cfg.band_high_hz);
    ini.set_float("Modem", "CenterFreqHz", cfg.center_freq_hz);

    ini.set_bool("Modem", "B2FUnroll", cfg.b2f_unroll);
    ini.set_bool("Modem", "NativeHail", cfg.native_hail);
    ini.set_int("Modem", "FX25Mode", cfg.fx25_mode);
    ini.set_float("Modem", "PreemphAlpha", cfg.preemph_alpha);

    ini.set_float("Calibration", "TxLevel", cfg.calibrated_tx_level);

    ini.set_bool("Logging", "LogEnabled", cfg.log_enabled);

    ini.set_float("Modem", "DcdThreshold", cfg.dcd_threshold);
    ini.set_int("Modem", "DcdHoldoffMs", cfg.dcd_holdoff_ms);
    ini.set_bool("Modem", "DcdAuto", cfg.dcd_auto);

    ini.set_float("Test", "SimBandpassLow", cfg.sim_bandpass_low);
    ini.set_float("Test", "SimBandpassHigh", cfg.sim_bandpass_high);

    ini.set_bool("OFDM", "Enable", cfg.ofdm_enable);
    ini.set_int("OFDM", "NFFT", cfg.ofdm_nfft);
    ini.set_bool("OFDM", "AutoSpacing", cfg.ofdm_auto_spacing);
    ini.set_bool("OFDM", "Waterfill", cfg.ofdm_waterfill);
    ini.set_bool("OFDM", "NUC", cfg.ofdm_nuc);
    ini.set_float("OFDM", "PreemphCornerHz", cfg.ofdm_preemph_corner_hz);

    ini.set_bool("GUI", "ShowConstellation", cfg.show_constellation);
    ini.set_bool("GUI", "ShowWaterfall", cfg.show_waterfall);

    // Write current parameter versions so future upgrades know what's stale
    for (int i = 0; i < NUM_PARAM_VERSIONS; i++) {
        const auto& pv = PARAM_VERSIONS[i];
        std::string ver_key = std::string(pv.section) + "_" + pv.key + "_ver";
        ini.set_int("Version", ver_key, pv.version);
    }

    return ini.save(path);
}

} // namespace iris
