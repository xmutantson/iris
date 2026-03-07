#include "gui/gui.h"

#ifdef IRIS_HAS_IMGUI

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "engine/speed_level.h"
#include "arq/arq.h"

#include <cstdio>
#include <cmath>
#include <cstring>

// === Platform backend selection ===
// SDL2 is preferred (cross-platform). Falls back to Win32 on Windows.
#ifdef IRIS_USE_SDL2
  #include "imgui_impl_sdl2.h"
  #include <SDL.h>
  #include <SDL_opengl.h>
#elif defined(_WIN32)
  #include "imgui_impl_win32.h"
  #include <windows.h>
  #include <GL/gl.h>
  extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);
#else
  #error "IRIS_HAS_IMGUI requires either IRIS_USE_SDL2 or _WIN32"
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// ============================================================
// Platform abstraction for window management
// ============================================================

#ifdef IRIS_USE_SDL2

// --- SDL2 backend ---
static SDL_Window* g_sdl_window = nullptr;
static SDL_GLContext g_sdl_gl_ctx = nullptr;
static bool g_want_close = false;

static bool platform_init(const char* title, int w, int h) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        fprintf(stderr, "[GUI] SDL_Init error: %s\n", SDL_GetError());
        return false;
    }

    SDL_GL_SetAttribute(SDL_GL_CONTEXT_FLAGS, 0);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);

    g_sdl_window = SDL_CreateWindow(title,
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        w, h, SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!g_sdl_window) return false;

    g_sdl_gl_ctx = SDL_GL_CreateContext(g_sdl_window);
    SDL_GL_MakeCurrent(g_sdl_window, g_sdl_gl_ctx);
    SDL_GL_SetSwapInterval(1); // vsync

    ImGui_ImplSDL2_InitForOpenGL(g_sdl_window, g_sdl_gl_ctx);
    ImGui_ImplOpenGL3_Init("#version 130");
    g_want_close = false;
    return true;
}

static bool platform_pump_events() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        ImGui_ImplSDL2_ProcessEvent(&event);
        if (event.type == SDL_QUIT) g_want_close = true;
        if (event.type == SDL_WINDOWEVENT &&
            event.window.event == SDL_WINDOWEVENT_CLOSE &&
            event.window.windowID == SDL_GetWindowID(g_sdl_window))
            g_want_close = true;
    }
    return !g_want_close;
}

static void platform_new_frame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();
}

static void platform_render() {
    ImGui::Render();
    int w, h;
    SDL_GL_GetDrawableSize(g_sdl_window, &w, &h);
    glViewport(0, 0, w, h);
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(g_sdl_window);
}

static void platform_shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    if (g_sdl_gl_ctx) { SDL_GL_DeleteContext(g_sdl_gl_ctx); g_sdl_gl_ctx = nullptr; }
    if (g_sdl_window) { SDL_DestroyWindow(g_sdl_window); g_sdl_window = nullptr; }
    SDL_Quit();
}

#elif defined(_WIN32)

// --- Win32 backend ---
static HWND g_hwnd = nullptr;
static HDC g_hdc = nullptr;
static HGLRC g_hglrc = nullptr;
static bool g_want_close = false;

static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam))
        return true;
    switch (msg) {
    case WM_SIZE:
        if (wParam != SIZE_MINIMIZED) glViewport(0, 0, LOWORD(lParam), HIWORD(lParam));
        return 0;
    case WM_CLOSE:
        g_want_close = true;
        return 0;
    case WM_DESTROY:
        PostQuitMessage(0);
        return 0;
    }
    return DefWindowProcW(hWnd, msg, wParam, lParam);
}

static bool platform_init(const char* title, int w, int h) {
    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.style = CS_OWNDC;
    wc.lpfnWndProc = WndProc;
    wc.hInstance = GetModuleHandleW(nullptr);
    wc.lpszClassName = L"IrisModem";
    RegisterClassExW(&wc);

    wchar_t wtitle[256];
    MultiByteToWideChar(CP_UTF8, 0, title, -1, wtitle, 256);
    g_hwnd = CreateWindowExW(0, L"IrisModem", wtitle, WS_OVERLAPPEDWINDOW,
                              CW_USEDEFAULT, CW_USEDEFAULT, w, h,
                              nullptr, nullptr, wc.hInstance, nullptr);
    if (!g_hwnd) return false;

    g_hdc = GetDC(g_hwnd);
    PIXELFORMATDESCRIPTOR pfd = {};
    pfd.nSize = sizeof(pfd); pfd.nVersion = 1;
    pfd.dwFlags = PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER;
    pfd.iPixelType = PFD_TYPE_RGBA; pfd.cColorBits = 32; pfd.cDepthBits = 24;
    SetPixelFormat(g_hdc, ChoosePixelFormat(g_hdc, &pfd), &pfd);

    g_hglrc = wglCreateContext(g_hdc);
    wglMakeCurrent(g_hdc, g_hglrc);
    ShowWindow(g_hwnd, SW_SHOWDEFAULT);
    UpdateWindow(g_hwnd);

    ImGui_ImplWin32_InitForOpenGL(g_hwnd);
    ImGui_ImplOpenGL3_Init("#version 130");
    g_want_close = false;
    return true;
}

static bool platform_pump_events() {
    MSG msg;
    while (PeekMessageW(&msg, nullptr, 0, 0, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessageW(&msg);
        if (msg.message == WM_QUIT) g_want_close = true;
    }
    return !g_want_close;
}

static void platform_new_frame() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplWin32_NewFrame();
    ImGui::NewFrame();
}

static void platform_render() {
    ImGui::Render();
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SwapBuffers(g_hdc);
}

static void platform_shutdown() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    if (g_hglrc) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(g_hglrc); g_hglrc = nullptr; }
    if (g_hdc && g_hwnd) { ReleaseDC(g_hwnd, g_hdc); g_hdc = nullptr; }
    if (g_hwnd) { DestroyWindow(g_hwnd); g_hwnd = nullptr; }
    UnregisterClassW(L"IrisModem", GetModuleHandleW(nullptr));
}

#endif // platform backends

// ============================================================
// State name helper
// ============================================================
static const char* state_name(ModemState s) {
    switch (s) {
        case ModemState::IDLE: return "IDLE";
        case ModemState::RX_AX25: return "RX AX.25";
        case ModemState::RX_NATIVE: return "RX Native";
        case ModemState::TX_AX25: return "TX AX.25";
        case ModemState::TX_NATIVE: return "TX Native";
        case ModemState::CALIBRATING: return "CALIBRATING";
    }
    return "?";
}

// ============================================================
// IrisGui implementation (platform-independent ImGui code)
// ============================================================

IrisGui::IrisGui() = default;
IrisGui::~IrisGui() { shutdown(); }

bool IrisGui::init(const std::string& title, int width, int height) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    if (!platform_init(title.c_str(), width, height)) {
        ImGui::DestroyContext();
        return false;
    }

    open_ = true;
    return true;
}

void IrisGui::update(const ModemDiag& diag, IrisConfig& config,
                     const std::vector<AudioDevice>& audio_devices) {
    last_diag_ = diag;

    if (!platform_pump_events()) {
        open_ = false;
        if (callbacks_.on_quit) callbacks_.on_quit();
        return;
    }

    platform_new_frame();

    // === Main Panel ===
    ImGui::Begin("Iris FM Data Modem", nullptr, ImGuiWindowFlags_MenuBar);

    if (ImGui::BeginMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Settings")) show_config_ = true;
            if (ImGui::MenuItem("Quit")) {
                if (callbacks_.on_quit) callbacks_.on_quit();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMenuBar();
    }

    // Status line
    ImGui::Text("State: %s", state_name(diag.state));
    ImGui::SameLine(200);
    ImGui::Text("Speed: %s", SPEED_LEVELS[diag.speed_level].name);
    ImGui::SameLine(350);
    ImGui::Text("SNR: %.1f dB", diag.snr_db);
    ImGui::SameLine(500);
    ImGui::Text("KISS: %d clients", diag.kiss_clients);

    ImGui::Separator();

    // Controls
    if (ImGui::Button("Connect")) {
        if (callbacks_.on_connect) callbacks_.on_connect();
    }
    ImGui::SameLine();
    if (ImGui::Button("Disconnect")) {
        if (callbacks_.on_disconnect) callbacks_.on_disconnect();
    }
    ImGui::SameLine();
    if (ImGui::Button("Auto Level Cal")) {
        if (callbacks_.on_calibrate) callbacks_.on_calibrate();
    }

    ImGui::Separator();

    // Stats row
    ImGui::Columns(4, "stats");
    ImGui::Text("RX: %d", diag.frames_rx); ImGui::NextColumn();
    ImGui::Text("TX: %d", diag.frames_tx); ImGui::NextColumn();
    ImGui::Text("CRC Err: %d", diag.crc_errors); ImGui::NextColumn();
    ImGui::Text("RX RMS: %.3f", diag.rx_rms); ImGui::Columns(1);

    // ARQ status
    {
        const char* arq_str = "IDLE";
        ImVec4 arq_color(0.5f, 0.5f, 0.5f, 1.0f);
        if (diag.arq_state == ArqState::CONNECTING) { arq_str = "CONNECTING"; arq_color = ImVec4(1,1,0,1); }
        else if (diag.arq_state == ArqState::CONNECTED) { arq_str = "CONNECTED"; arq_color = ImVec4(0,1,0,1); }
        else if (diag.arq_state == ArqState::DISCONNECTING) { arq_str = "DISCONNECTING"; arq_color = ImVec4(1,0.5f,0,1); }
        ImGui::PushStyleColor(ImGuiCol_Text, arq_color);
        ImGui::Text("ARQ: %s", arq_str);
        ImGui::PopStyleColor();
        if (diag.arq_state != ArqState::IDLE) {
            ImGui::SameLine();
            ImGui::Text("  Role: %s  Retx: %d",
                diag.arq_role == ArqRole::COMMANDER ? "CMD" : "RSP",
                diag.retransmits);
        }
    }

    // PTT indicator
    if (diag.ptt_active) {
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1, 0, 0, 1));
        ImGui::Text("*** TX ***");
        ImGui::PopStyleColor();
    }

    ImGui::Separator();

    // === SNR Meter ===
    {
        float snr_frac = (diag.snr_db + 5.0f) / 40.0f;
        snr_frac = snr_frac < 0 ? 0 : snr_frac > 1 ? 1 : snr_frac;
        ImVec4 color = snr_frac < 0.3f ? ImVec4(1,0,0,1) :
                       snr_frac < 0.6f ? ImVec4(1,1,0,1) : ImVec4(0,1,0,1);
        ImGui::PushStyleColor(ImGuiCol_PlotHistogram, color);
        char overlay[32];
        snprintf(overlay, sizeof(overlay), "%.1f dB", diag.snr_db);
        ImGui::ProgressBar(snr_frac, ImVec2(-1, 20), overlay);
        ImGui::PopStyleColor();

        // Speed level buttons
        ImGui::Text("Speed levels:");
        for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
            if (i > 0) ImGui::SameLine();
            bool active = (i == diag.speed_level);
            if (active) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.8f, 0.2f, 1));
            ImGui::SmallButton(SPEED_LEVELS[i].name);
            if (active) ImGui::PopStyleColor();
        }

        int tp = net_throughput(diag.speed_level,
            mode_baud_rate(config.mode.empty() ? 'A' : config.mode[0]));
        ImGui::Text("Net throughput: %d bps", tp);
    }

    ImGui::Separator();

    // === Constellation Plot ===
    if (!diag.constellation.empty()) {
        ImGui::Text("Constellation (%zu symbols)", diag.constellation.size());
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size(200, 200);
        ImDrawList* dl = ImGui::GetWindowDrawList();

        dl->AddRectFilled(canvas_pos,
            ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
            IM_COL32(20, 20, 30, 255));

        float cx = canvas_pos.x + canvas_size.x * 0.5f;
        float cy = canvas_pos.y + canvas_size.y * 0.5f;
        dl->AddLine(ImVec2(canvas_pos.x, cy),
                    ImVec2(canvas_pos.x + canvas_size.x, cy), IM_COL32(60,60,60,255));
        dl->AddLine(ImVec2(cx, canvas_pos.y),
                    ImVec2(cx, canvas_pos.y + canvas_size.y), IM_COL32(60,60,60,255));

        float scale = canvas_size.x * 0.3f;
        for (const auto& sym : diag.constellation) {
            float px = cx + sym.real() * scale;
            float py = cy - sym.imag() * scale;
            dl->AddCircleFilled(ImVec2(px, py), 2.0f, IM_COL32(0, 255, 100, 200));
        }
        ImGui::Dummy(canvas_size);
    }

    // === Spectrum / Waterfall ===
    if (!diag.spectrum.empty()) {
        ImGui::Separator();
        ImGui::Text("Spectrum (%zu bins)", diag.spectrum.size());
        ImVec2 spec_pos = ImGui::GetCursorScreenPos();
        ImVec2 spec_size(ImGui::GetContentRegionAvail().x, 100);
        ImDrawList* dl = ImGui::GetWindowDrawList();

        dl->AddRectFilled(spec_pos,
            ImVec2(spec_pos.x + spec_size.x, spec_pos.y + spec_size.y),
            IM_COL32(10, 10, 20, 255));

        int n_bins = (int)diag.spectrum.size();
        float bin_w = spec_size.x / n_bins;
        float db_min = -80.0f, db_max = 0.0f;

        for (int i = 0; i < n_bins; i++) {
            float db = diag.spectrum[i];
            float frac = (db - db_min) / (db_max - db_min);
            frac = frac < 0.0f ? 0.0f : frac > 1.0f ? 1.0f : frac;

            float x0 = spec_pos.x + i * bin_w;
            float y0 = spec_pos.y + spec_size.y * (1.0f - frac);
            float x1 = x0 + bin_w;
            float y1 = spec_pos.y + spec_size.y;

            // Color gradient: blue -> cyan -> green -> yellow
            uint8_t r, g, b;
            if (frac < 0.33f) {
                float t = frac / 0.33f;
                r = 0; g = (uint8_t)(t * 200); b = (uint8_t)(100 + t * 155);
            } else if (frac < 0.66f) {
                float t = (frac - 0.33f) / 0.33f;
                r = 0; g = (uint8_t)(200 + t * 55); b = (uint8_t)(255 * (1 - t));
            } else {
                float t = (frac - 0.66f) / 0.34f;
                r = (uint8_t)(t * 255); g = 255; b = 0;
            }

            dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), IM_COL32(r, g, b, 200));
        }

        ImGui::Dummy(spec_size);
    }

    ImGui::End(); // Main panel

    // === Settings Window ===
    if (show_config_) {
        ImGui::Begin("Settings", &show_config_);

        char cs_buf[16];
        strncpy(cs_buf, config.callsign.c_str(), sizeof(cs_buf) - 1);
        cs_buf[sizeof(cs_buf) - 1] = '\0';
        if (ImGui::InputText("Callsign", cs_buf, sizeof(cs_buf)))
            config.callsign = cs_buf;

        const char* modes[] = {"A (Audio)", "B (9600 port)", "C (SDR)"};
        int mi = config.mode == "B" ? 1 : config.mode == "C" ? 2 : 0;
        if (ImGui::Combo("Channel Mode", &mi, modes, 3))
            config.mode = mi == 0 ? "A" : mi == 1 ? "B" : "C";

        const char* bauds[] = {"1200", "9600"};
        int bi = config.ax25_baud == 9600 ? 1 : 0;
        if (ImGui::Combo("AX.25 Baud", &bi, bauds, 2))
            config.ax25_baud = bi ? 9600 : 1200;

        ImGui::SliderFloat("TX Level", &config.tx_level, 0.0f, 1.0f, "%.2f");

        if (config.calibrated_tx_level > 0)
            ImGui::Text("Calibrated TX: %.3f", config.calibrated_tx_level);

        // Audio devices
        if (!audio_devices.empty()) {
            ImGui::Separator();
            ImGui::Text("Audio Devices:");
            std::vector<std::string> cap_names, play_names;
            for (const auto& d : audio_devices) {
                if (d.max_input_channels > 0) cap_names.push_back(d.name);
                if (d.max_output_channels > 0) play_names.push_back(d.name);
            }
            if (!cap_names.empty()) {
                if (selected_capture_ >= (int)cap_names.size()) selected_capture_ = 0;
                if (ImGui::BeginCombo("Capture", cap_names[selected_capture_].c_str())) {
                    for (int i = 0; i < (int)cap_names.size(); i++)
                        if (ImGui::Selectable(cap_names[i].c_str(), i == selected_capture_))
                            selected_capture_ = i;
                    ImGui::EndCombo();
                }
            }
            if (!play_names.empty()) {
                if (selected_playback_ >= (int)play_names.size()) selected_playback_ = 0;
                if (ImGui::BeginCombo("Playback", play_names[selected_playback_].c_str())) {
                    for (int i = 0; i < (int)play_names.size(); i++)
                        if (ImGui::Selectable(play_names[i].c_str(), i == selected_playback_))
                            selected_playback_ = i;
                    ImGui::EndCombo();
                }
            }
        }

        ImGui::Separator();
        const char* ptt_methods[] = {"none", "rigctl", "vox"};
        int pi = config.ptt_method == "rigctl" ? 1 : config.ptt_method == "vox" ? 2 : 0;
        if (ImGui::Combo("PTT Method", &pi, ptt_methods, 3))
            config.ptt_method = ptt_methods[pi];

        if (pi == 1) {
            char hb[64];
            strncpy(hb, config.rigctl_host.c_str(), sizeof(hb)-1); hb[sizeof(hb)-1]='\0';
            if (ImGui::InputText("rigctl Host", hb, sizeof(hb)))
                config.rigctl_host = hb;
            ImGui::InputInt("rigctl Port", &config.rigctl_port);
        }

        ImGui::Separator();
        if (ImGui::Button("Save")) {
            if (callbacks_.on_config_changed) callbacks_.on_config_changed(config);
        }
        ImGui::End();
    }

    // === Log Window ===
    ImGui::Begin("Log");
    for (const auto& line : log_lines_)
        ImGui::TextUnformatted(line.c_str());
    if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
        ImGui::SetScrollHereY(1.0f);
    ImGui::End();
}

bool IrisGui::render_frame() {
    if (!open_) return false;
    platform_render();
    return open_;
}

void IrisGui::shutdown() {
    if (!open_) return;
    open_ = false;
    platform_shutdown();
    ImGui::DestroyContext();
}

void IrisGui::log(const std::string& msg) {
    log_lines_.push_back(msg);
    if (log_lines_.size() > 1000)
        log_lines_.erase(log_lines_.begin());
}

} // namespace iris

#endif // IRIS_HAS_IMGUI
