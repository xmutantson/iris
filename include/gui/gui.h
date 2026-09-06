#ifndef IRIS_GUI_H
#define IRIS_GUI_H

#include "engine/modem.h"
#include "config/config.h"
#include "audio/audio.h"
#include <functional>
#include <string>
#include <vector>
#include <mutex>

namespace iris {

// GUI event callbacks
struct GuiCallbacks {
    std::function<void()> on_connect;
    std::function<void()> on_disconnect;
    std::function<void()> on_calibrate;
    std::function<void()> on_probe;
    std::function<void(const std::string&)> on_autotune;  // callsign arg
    std::function<void(const IrisConfig&)> on_config_changed;
    std::function<void()> on_quit;
};

// Iris GUI using Dear ImGui
// When Dear ImGui is vendored, this renders the full interface.
// Without it, this is a no-op stub that compiles cleanly.
class IrisGui {
public:
    IrisGui();
    ~IrisGui();

    // Initialize the GUI window
    bool init(const std::string& title, int width = 800, int height = 600);

    // Set callbacks
    void set_callbacks(const GuiCallbacks& cb) { callbacks_ = cb; }

    // Update with current state (call each frame)
    void update(const ModemDiag& diag, IrisConfig& config,
                const std::vector<AudioDevice>& audio_devices);

    // Returns true while GUI window is open
    bool is_open() const { return open_; }

    // Render one frame. Returns false if window closed.
    bool render_frame();

    // Shutdown
    void shutdown();

    // Log a message to the GUI log panel
    void log(const std::string& msg);

    // Log a packet to the packet log panel
    // direction: true=TX (red), false=RX (white)
    // protocol: e.g. "AX.25", "FX.25", "Iris-A3"
    void log_packet(bool is_tx, const std::string& protocol,
                    const std::string& description);

private:
    bool open_ = false;
    GuiCallbacks callbacks_;
    std::vector<std::string> log_lines_;
    ModemDiag last_diag_{};

    // GUI state
    int selected_capture_ = 0;
    int selected_playback_ = 0;
    int selected_mode_ = 0;  // 0=A, 1=B, 2=C
    bool show_config_ = false;
    bool show_log_ = false;
    bool show_packet_log_ = false;
    bool show_kalman_ = false;
    int settings_tab_ = 0;

    // Packet log entries
    struct PacketLogEntry {
        bool is_tx;              // true=TX (red), false=RX (white)
        std::string timestamp;   // HH:MM:SS
        std::string protocol;    // AX.25, FX.25-16, Iris-A3, etc.
        std::string description; // Frame description
    };
    std::vector<PacketLogEntry> packet_log_;
    std::mutex packet_log_mutex_;

    // Waterfall scrolling history (rows x bins)
    static constexpr int WF_HISTORY = 100;  // rows of history
    std::vector<std::vector<float>> wf_history_;

    // Config snapshot for dirty detection
    IrisConfig config_snapshot_;
    bool config_dirty_ = false;
};

} // namespace iris

#endif
