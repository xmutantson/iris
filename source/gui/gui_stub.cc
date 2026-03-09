#include "gui/gui.h"

// This file is only compiled when IRIS_HAS_IMGUI is NOT defined.
// When building with Dear ImGui, gui_imgui.cc is compiled instead.
#ifndef IRIS_HAS_IMGUI

#include <cstdio>

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

void IrisGui::log_packet(bool is_tx, const std::string& protocol,
                          const std::string& description) {
    printf("[%s] [%s] %s\n", is_tx ? "TX" : "RX", protocol.c_str(), description.c_str());
}

} // namespace iris

#endif // !IRIS_HAS_IMGUI
