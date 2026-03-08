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
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
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
    glClearColor(0.06f, 0.06f, 0.08f, 1.0f);
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
// Custom widgets (Mercury-style meters and indicators)
// ============================================================

static void DrawMeter(const char* label, float value, float min_val, float max_val,
                      float yellow_threshold, float red_threshold,
                      const char* format = "%.1f dB", bool invert_colors = false) {
    ImGui::PushID(label);

    float normalized = (value - min_val) / (max_val - min_val);
    if (normalized < 0.0f) normalized = 0.0f;
    if (normalized > 1.0f) normalized = 1.0f;

    float yellow_norm = (yellow_threshold - min_val) / (max_val - min_val);
    float red_norm = (red_threshold - min_val) / (max_val - min_val);

    ImGui::Text("%s", label);

    ImVec2 pos = ImGui::GetCursorScreenPos();
    float avail_w = ImGui::GetContentRegionAvail().x;
    float bar_w = avail_w > 250.0f ? avail_w - 80.0f : avail_w * 0.7f;
    if (bar_w < 80.0f) bar_w = 80.0f;
    ImVec2 size(bar_w, 18);
    ImDrawList* dl = ImGui::GetWindowDrawList();

    ImU32 left_dim   = invert_colors ? IM_COL32(60,0,0,255) : IM_COL32(0,60,0,255);
    ImU32 mid_dim    = IM_COL32(60,60,0,255);
    ImU32 right_dim  = invert_colors ? IM_COL32(0,60,0,255) : IM_COL32(60,0,0,255);
    ImU32 left_bright  = invert_colors ? IM_COL32(220,0,0,255) : IM_COL32(0,220,0,255);
    ImU32 mid_bright   = IM_COL32(220,220,0,255);
    ImU32 right_bright = invert_colors ? IM_COL32(0,220,0,255) : IM_COL32(220,0,0,255);

    float zone1_end = pos.x + yellow_norm * size.x;
    float zone2_end = pos.x + red_norm * size.x;

    dl->AddRectFilled(pos, ImVec2(zone1_end, pos.y + size.y), left_dim);
    dl->AddRectFilled(ImVec2(zone1_end, pos.y), ImVec2(zone2_end, pos.y + size.y), mid_dim);
    dl->AddRectFilled(ImVec2(zone2_end, pos.y), ImVec2(pos.x + size.x, pos.y + size.y), right_dim);

    float bar_end = pos.x + normalized * size.x;
    if (bar_end > pos.x) {
        float seg_end = (bar_end < zone1_end) ? bar_end : zone1_end;
        if (seg_end > pos.x)
            dl->AddRectFilled(pos, ImVec2(seg_end, pos.y + size.y), left_bright);
    }
    if (bar_end > zone1_end) {
        float seg_end = (bar_end < zone2_end) ? bar_end : zone2_end;
        if (seg_end > zone1_end)
            dl->AddRectFilled(ImVec2(zone1_end, pos.y), ImVec2(seg_end, pos.y + size.y), mid_bright);
    }
    if (bar_end > zone2_end)
        dl->AddRectFilled(ImVec2(zone2_end, pos.y), ImVec2(bar_end, pos.y + size.y), right_bright);

    dl->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(128, 128, 128, 255));
    ImGui::Dummy(size);

    ImGui::SameLine();
    ImGui::Text(format, value);

    ImGui::PopID();
}

static void DrawConstellation(const std::vector<std::complex<float>>& points, float width, float height) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(15, 15, 20, 255));

    float cx = pos.x + width * 0.5f;
    float cy = pos.y + height * 0.5f;
    dl->AddLine(ImVec2(pos.x, cy), ImVec2(pos.x + width, cy), IM_COL32(40, 40, 50, 255));
    dl->AddLine(ImVec2(cx, pos.y), ImVec2(cx, pos.y + height), IM_COL32(40, 40, 50, 255));

    float dim = (width < height) ? width : height;
    float scale = dim / 4.5f;

    for (const auto& sym : points) {
        float px = cx + sym.real() * scale;
        float py = cy - sym.imag() * scale;
        if (px >= pos.x && px <= pos.x + width && py >= pos.y && py <= pos.y + height)
            dl->AddCircleFilled(ImVec2(px, py), 2.0f, IM_COL32(0, 200, 255, 180));
    }

    if (points.empty()) {
        const char* txt = "Waiting for data...";
        ImVec2 ts = ImGui::CalcTextSize(txt);
        dl->AddText(ImVec2(cx - ts.x * 0.5f, cy - 6), IM_COL32(80, 80, 80, 200), txt);
    }

    dl->AddRect(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(80, 80, 80, 255));
    ImGui::Dummy(ImVec2(width, height));
}

static void DrawSpectrum(const std::vector<float>& spectrum, float width, float height) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(10, 10, 20, 255));

    int n_bins = (int)spectrum.size();
    if (n_bins <= 0) { ImGui::Dummy(ImVec2(width, height)); return; }
    float bin_w = width / n_bins;
    float db_min = -80.0f, db_max = 0.0f;

    for (int i = 0; i < n_bins; i++) {
        float frac = (spectrum[i] - db_min) / (db_max - db_min);
        frac = frac < 0.0f ? 0.0f : frac > 1.0f ? 1.0f : frac;

        float x0 = pos.x + i * bin_w;
        float y0 = pos.y + height * (1.0f - frac);
        float x1 = x0 + bin_w;
        float y1 = pos.y + height;

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

    dl->AddRect(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(80, 80, 80, 255));
    ImGui::Dummy(ImVec2(width, height));
}

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
// IrisGui implementation
// ============================================================

IrisGui::IrisGui() = default;
IrisGui::~IrisGui() { shutdown(); }

bool IrisGui::init(const std::string& title, int width, int height) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    ImGui::StyleColorsDark();
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowRounding = 2.0f;
    style.FrameRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.WindowPadding = ImVec2(6, 6);
    style.ItemSpacing = ImVec2(6, 4);
    style.FramePadding = ImVec2(4, 2);

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
    ImGuiIO& io = ImGui::GetIO();

    // === Main Menu Bar ===
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("Settings")) {
            if (ImGui::MenuItem("Setup...")) show_config_ = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Quit")) {
                if (callbacks_.on_quit) callbacks_.on_quit();
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // === Full-window main panel (Mercury style) ===
    float menu_h = ImGui::GetFrameHeight();
    ImGui::SetNextWindowPos(ImVec2(0, menu_h));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, io.DisplaySize.y - menu_h));
    ImGui::Begin("##Main", nullptr,
                 ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
                 ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
                 ImGuiWindowFlags_NoBringToFrontOnFocus);

    // ========== Top Row: Meters ==========
    ImGui::BeginChild("Meters", ImVec2(0, 110), true);
    ImGui::Columns(3, "meter_cols", false);

    // Column 1: Audio / Signal meters
    {
        float vu_db = 20.0f * std::log10(diag.rx_rms + 1e-10f);
        DrawMeter("Audio Input", vu_db, -60.0f, 0.0f, -12.0f, -3.0f, "%.1f dB");
        DrawMeter("Signal (SNR)", diag.snr_db, -5.0f, 40.0f, 10.0f, 25.0f, "%.1f dB", true);
    }

    ImGui::NextColumn();

    // Column 2: Frame counters
    {
        ImGui::Text("RX Frames: %d", diag.frames_rx);
        ImGui::Text("TX Frames: %d", diag.frames_tx);
        ImGui::Text("CRC Errors: %d", diag.crc_errors);
        ImGui::Text("Retransmits: %d", diag.retransmits);
    }

    ImGui::NextColumn();

    // Column 3: Throughput & Mode info
    {
        int tp = net_throughput(diag.speed_level,
            mode_baud_rate(config.mode.empty() ? 'A' : config.mode[0]));
        ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f), "Speed: %s",
                           SPEED_LEVELS[diag.speed_level].name);
        if (tp >= 1000)
            ImGui::Text("%.1f kbps", tp / 1000.0f);
        else
            ImGui::Text("%d bps", tp);
        ImGui::Text("Mode %s  |  %d baud",
                     config.mode.c_str(),
                     mode_baud_rate(config.mode.empty() ? 'A' : config.mode[0]));
        ImGui::Text("KISS: %d clients", diag.kiss_clients);
    }

    ImGui::Columns(1);
    ImGui::EndChild();

    // ========== Signal Quality Row: Constellation + Indicators + Status + Controls ==========
    ImGui::BeginChild("SignalQuality", ImVec2(0, 150), true, ImGuiWindowFlags_NoScrollbar);
    {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float diag_sz = avail.y;
        if (diag_sz < 80) diag_sz = 80;

        // Left: Constellation diagram
        DrawConstellation(diag.constellation, diag_sz, diag_sz);

        ImGui::SameLine();

        // TX/RX/PTT/CAL indicators
        ImGui::BeginGroup();
        {
            bool is_tx = diag.ptt_active ||
                         diag.state == ModemState::TX_AX25 ||
                         diag.state == ModemState::TX_NATIVE;
            bool is_rx = diag.state == ModemState::RX_AX25 ||
                         diag.state == ModemState::RX_NATIVE;

            if (is_tx) ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), " TX");
            else       ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), " TX");
            if (is_rx) ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), " RX");
            else       ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), " RX");

            if (diag.ptt_active) {
                float t = (float)fmod(ImGui::GetTime() * 4.0, 1.0);
                float b = (t < 0.5f) ? 1.0f : 0.3f;
                ImGui::TextColored(ImVec4(b, 0.0f, 0.0f, 1.0f), " PTT");
            } else {
                ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), " PTT");
            }

            if (diag.cal_state != CalState::IDLE)
                ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.0f, 1.0f), " CAL");
            else
                ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), " CAL");
        }
        ImGui::EndGroup();

        ImGui::SameLine();

        // ARQ status + encryption
        ImGui::BeginGroup();
        {
            const char* arq_str = "IDLE";
            ImVec4 arq_color(0.5f, 0.5f, 0.5f, 1.0f);
            if (diag.arq_state == ArqState::CONNECTING) {
                arq_str = "CONNECTING"; arq_color = ImVec4(1, 1, 0, 1);
            } else if (diag.arq_state == ArqState::CONNECTED) {
                arq_str = "CONNECTED"; arq_color = ImVec4(0, 1, 0, 1);
            } else if (diag.arq_state == ArqState::DISCONNECTING) {
                arq_str = "DISCONNECTING"; arq_color = ImVec4(1, 0.5f, 0, 1);
            }

            ImGui::TextColored(arq_color, "%s", arq_str);
            if (diag.arq_state != ArqState::IDLE) {
                ImGui::Text("Role: %s",
                    diag.arq_role == ArqRole::COMMANDER ? "Commander" : "Responder");
            }

            ImGui::Spacing();

            if (config.encryption_mode == 1)
                ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "E2E Strict");
            else if (config.encryption_mode == 2)
                ImGui::TextColored(ImVec4(0.4f, 0.9f, 1.0f, 1.0f), "E2E Fast");
            else
                ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "No Encryption");

            ImGui::Text("Callsign: %s", config.callsign.c_str());
        }
        ImGui::EndGroup();

        ImGui::SameLine();

        // Speed level ladder
        ImGui::BeginGroup();
        {
            ImGui::Text("Speed Levels:");
            for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
                bool active = (i == diag.speed_level);
                if (active)
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.7f, 0.2f, 1.0f));
                else
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.15f, 0.18f, 1.0f));
                ImGui::SmallButton(SPEED_LEVELS[i].name);
                ImGui::PopStyleColor();
                if (i < NUM_SPEED_LEVELS - 1) ImGui::SameLine();
            }
        }
        ImGui::EndGroup();

        ImGui::SameLine();

        // Connect / Disconnect / Cal buttons
        ImGui::BeginGroup();
        {
            bool connected = (diag.arq_state == ArqState::CONNECTED);
            if (connected) ImGui::BeginDisabled();
            if (ImGui::Button("Connect", ImVec2(90, 28))) {
                if (callbacks_.on_connect) callbacks_.on_connect();
            }
            if (connected) ImGui::EndDisabled();

            if (!connected) ImGui::BeginDisabled();
            if (ImGui::Button("Disconnect", ImVec2(90, 28))) {
                if (callbacks_.on_disconnect) callbacks_.on_disconnect();
            }
            if (!connected) ImGui::EndDisabled();

            if (ImGui::Button("Auto Cal", ImVec2(90, 28))) {
                if (callbacks_.on_calibrate) callbacks_.on_calibrate();
            }
            if (config.calibrated_tx_level > 0)
                ImGui::Text("Cal: %.3f", config.calibrated_tx_level);
        }
        ImGui::EndGroup();
    }
    ImGui::EndChild();

    // ========== Controls Row: TX Level + RX Gain ==========
    ImGui::BeginChild("Controls", ImVec2(0, 45), true);
    {
        ImGui::Columns(3, "ctrl_cols", false);

        ImGui::Text("TX Level");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetColumnWidth() - 70);
        ImGui::SliderFloat("##txlevel", &config.tx_level, 0.0f, 1.0f, "%.2f");

        ImGui::NextColumn();

        ImGui::Text("RX Gain");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetColumnWidth() - 70);
        ImGui::SliderFloat("##rxgain", &config.rx_gain, 0.1f, 10.0f, "%.1f");

        ImGui::NextColumn();

        ImGui::Text("AX.25: %d baud  |  %s",
                     config.ax25_baud, config.ax25_only ? "AX.25 Only" : "Native OK");

        ImGui::Columns(1);
    }
    ImGui::EndChild();

    // ========== Spectrum / Waterfall ==========
    if (!diag.spectrum.empty()) {
        float spec_h = 80.0f;
        ImGui::BeginChild("Spectrum", ImVec2(0, spec_h + 8), true,
                           ImGuiWindowFlags_NoScrollbar);
        DrawSpectrum(diag.spectrum, ImGui::GetContentRegionAvail().x, spec_h);
        ImGui::EndChild();
    }

    // ========== Log Panel (fills remaining space) ==========
    {
        float log_h = ImGui::GetContentRegionAvail().y;
        if (log_h < 60) log_h = 60;
        ImGui::BeginChild("Log", ImVec2(0, log_h), true);
        for (const auto& line : log_lines_)
            ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
        ImGui::EndChild();
    }

    ImGui::End(); // ##Main

    // ========== Settings Window (popup) ==========
    if (show_config_) {
        ImGui::SetNextWindowSize(ImVec2(420, 520), ImGuiCond_FirstUseEver);
        ImGui::Begin("Setup", &show_config_);

        if (ImGui::CollapsingHeader("Station", ImGuiTreeNodeFlags_DefaultOpen)) {
            char cs_buf[16];
            strncpy(cs_buf, config.callsign.c_str(), sizeof(cs_buf) - 1);
            cs_buf[sizeof(cs_buf) - 1] = '\0';
            if (ImGui::InputText("Callsign", cs_buf, sizeof(cs_buf)))
                config.callsign = cs_buf;

            const char* modes[] = {"A (Standard FM)", "B (9600 baud FM)"};
            int mi = config.mode == "B" ? 1 : 0;
            if (ImGui::Combo("Channel Mode", &mi, modes, 2))
                config.mode = mi == 0 ? "A" : "B";

            const char* bauds[] = {"1200", "9600"};
            int bi = config.ax25_baud == 9600 ? 1 : 0;
            if (ImGui::Combo("AX.25 Baud", &bi, bauds, 2))
                config.ax25_baud = bi ? 9600 : 1200;

            ImGui::Checkbox("AX.25 Only (no native upgrade)", &config.ax25_only);
        }

        if (ImGui::CollapsingHeader("Modem")) {
            const char* mod_names[] = {"BPSK", "QPSK", "QAM16", "QAM64", "QAM256"};
            int mod_idx = (int)config.max_modulation;
            if (mod_idx < 0) mod_idx = 0;
            if (mod_idx > 4) mod_idx = 4;
            if (ImGui::Combo("Max Modulation", &mod_idx, mod_names, 5))
                config.max_modulation = (Modulation)mod_idx;

            ImGui::SliderFloat("TX Level", &config.tx_level, 0.0f, 1.0f, "%.2f");
            ImGui::SliderFloat("RX Gain", &config.rx_gain, 0.1f, 10.0f, "%.1f");

            ImGui::Separator();
            ImGui::Text("Band Plan (Hz)");
            ImGui::SliderFloat("Low Edge", &config.band_low_hz, 200.0f, 1000.0f, "%.0f Hz");
            ImGui::SliderFloat("High Edge", &config.band_high_hz, 2000.0f, 5000.0f, "%.0f Hz");
            ImGui::SliderFloat("Center Freq", &config.center_freq_hz, 0.0f, 4000.0f, "%.0f Hz");
            if (config.center_freq_hz < 1.0f)
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "Center: auto (%.0f Hz)", (config.band_low_hz + config.band_high_hz) / 2.0f);
            float bw = (config.band_high_hz - config.band_low_hz);
            ImGui::TextColored(ImVec4(0.5f, 0.7f, 1.0f, 1.0f),
                "Band: %.0f Hz wide (%.0f - %.0f)", bw, config.band_low_hz, config.band_high_hz);
            if (config.band_low_hz < 255.0f)
                ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                    "Warning: Low edge below CTCSS max (254 Hz)!");
        }

        if (ImGui::CollapsingHeader("Audio Devices")) {
            if (!audio_devices.empty()) {
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
            } else {
                ImGui::TextColored(ImVec4(1, 1, 0, 1), "No audio devices found");
            }
        }

        if (ImGui::CollapsingHeader("Radio / PTT")) {
            const char* ptt_methods[] = {"none", "rigctl", "vox", "cm108", "serial"};
            int pi = 0;
            if (config.ptt_method == "rigctl") pi = 1;
            else if (config.ptt_method == "vox") pi = 2;
            else if (config.ptt_method == "cm108") pi = 3;
            else if (config.ptt_method == "serial") pi = 4;
            if (ImGui::Combo("PTT Method", &pi, ptt_methods, 5))
                config.ptt_method = ptt_methods[pi];

            if (pi == 1) {
                char hb[64];
                strncpy(hb, config.rigctl_host.c_str(), sizeof(hb) - 1);
                hb[sizeof(hb) - 1] = '\0';
                if (ImGui::InputText("rigctl Host", hb, sizeof(hb)))
                    config.rigctl_host = hb;
                ImGui::InputInt("rigctl Port", &config.rigctl_port);
            }
            if (pi == 4) {
                char sp[64];
                strncpy(sp, config.serial_port.c_str(), sizeof(sp) - 1);
                sp[sizeof(sp) - 1] = '\0';
                if (ImGui::InputText("Serial Port", sp, sizeof(sp)))
                    config.serial_port = sp;
                const char* ser_bauds[] = {"9600", "19200", "38400", "57600", "115200"};
                int sb = 0;
                if (config.serial_baud == 19200) sb = 1;
                else if (config.serial_baud == 38400) sb = 2;
                else if (config.serial_baud == 57600) sb = 3;
                else if (config.serial_baud == 115200) sb = 4;
                if (ImGui::Combo("Serial Baud", &sb, ser_bauds, 5)) {
                    const int baud_vals[] = {9600, 19200, 38400, 57600, 115200};
                    config.serial_baud = baud_vals[sb];
                }
            }

            ImGui::Separator();
            ImGui::Text("Timing (Direwolf-compatible)");
            ImGui::SliderInt("TXDelay (ms)", &config.ptt_pre_delay_ms, 0, 1000);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Delay after keying before data (default 300ms)");
            ImGui::SliderInt("TXTail (ms)", &config.ptt_post_delay_ms, 0, 500);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Extra TX time after data (default 100ms)");
            ImGui::SliderInt("SlotTime (ms)", &config.slottime_ms, 10, 500);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("CSMA slot time (default 100ms)");
            ImGui::SliderInt("Persist", &config.persist, 0, 255);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("p-persistence 0-255 (63=25%%, default 63)");
        }

        if (ImGui::CollapsingHeader("Network")) {
            ImGui::InputInt("KISS Port", &config.kiss_port);
            ImGui::InputInt("AGW Port", &config.agw_port);
        }

        if (ImGui::CollapsingHeader("Security")) {
            const char* enc_modes[] = {"Off", "Strict", "Fast"};
            if (ImGui::Combo("Encryption", &config.encryption_mode, enc_modes, 3)) {}
            if (config.encryption_mode > 0) {
                char psk_buf[130];
                strncpy(psk_buf, config.psk_hex.c_str(), sizeof(psk_buf) - 1);
                psk_buf[sizeof(psk_buf) - 1] = '\0';
                if (ImGui::InputText("PSK (hex)", psk_buf, sizeof(psk_buf),
                                      ImGuiInputTextFlags_Password))
                    config.psk_hex = psk_buf;
                ImGui::TextWrapped("Pre-shared key in hex. Both sides must match.");
            }
        }

        if (ImGui::CollapsingHeader("Logging")) {
            ImGui::Checkbox("Enable log file output", &config.log_enabled);
            ImGui::TextWrapped("Logs to AppData/Iris/logs/ (Win) or ~/.config/iris/logs/ (Linux).");
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Save & Close", ImVec2(120, 0))) {
            if (callbacks_.on_config_changed) callbacks_.on_config_changed(config);
            show_config_ = false;
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            show_config_ = false;
        }
        ImGui::End();
    }
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
