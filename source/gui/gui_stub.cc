#include "gui/gui.h"
#include <cstdio>

// Stub GUI implementation
// When Dear ImGui is vendored (third_party/imgui/), replace this file with
// gui_imgui.cc that uses the real ImGui API. The interface is the same.

namespace iris {

IrisGui::IrisGui() = default;
IrisGui::~IrisGui() { shutdown(); }

bool IrisGui::init(const std::string& title, int width, int height) {
    (void)width; (void)height;
    printf("[GUI] Iris Modem - %s (text mode, Dear ImGui not available)\n", title.c_str());
    open_ = true;
    return true;
}

void IrisGui::update(const ModemDiag& diag, IrisConfig& config,
                     const std::vector<AudioDevice>& audio_devices) {
    (void)config;
    (void)audio_devices;
    last_diag_ = diag;
}

bool IrisGui::render_frame() {
    // In text mode, just print periodic status
    return open_;
}

void IrisGui::shutdown() {
    open_ = false;
}

void IrisGui::log(const std::string& msg) {
    printf("[LOG] %s\n", msg.c_str());
    log_lines_.push_back(msg);
    if (log_lines_.size() > 1000)
        log_lines_.erase(log_lines_.begin());
}

} // namespace iris
