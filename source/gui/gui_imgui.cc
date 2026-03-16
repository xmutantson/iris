#include "gui/gui.h"

#ifdef IRIS_HAS_IMGUI

#include "imgui.h"
#include "imgui_impl_opengl3.h"
#include "engine/speed_level.h"
#include "native/frame.h"
#include "arq/arq.h"
#include "ax25/ax25_session.h"
#include "probe/passband_probe.h"

#include <cstdio>
#include <cmath>
#include <cstring>
#include <ctime>

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

    // Hide the console window when running with GUI
    HWND console = GetConsoleWindow();
    if (console) ShowWindow(console, SW_HIDE);

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
    // Show console again on shutdown so error messages are visible
    HWND console = GetConsoleWindow();
    if (console) ShowWindow(console, SW_SHOW);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplWin32_Shutdown();
    if (g_hglrc) { wglMakeCurrent(nullptr, nullptr); wglDeleteContext(g_hglrc); g_hglrc = nullptr; }
    if (g_hdc && g_hwnd) { ReleaseDC(g_hwnd, g_hdc); g_hdc = nullptr; }
    if (g_hwnd) { DestroyWindow(g_hwnd); g_hwnd = nullptr; }
    UnregisterClassW(L"IrisModem", GetModuleHandleW(nullptr));
}

#endif // platform backends

// ============================================================
// Custom widgets
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

// 3D Kalman state viewer — shows phase/freq/accel trajectory in projected 3D space
// Replaces constellation diagram. Auto-rotates, forward Kalman in cyan, RTS smoothed in green.
static float g_kalman_yaw = 0.4f;    // rotation around vertical axis (radians)
static float g_kalman_pitch = 0.3f;  // tilt angle
static bool  g_kalman_auto_rotate = true;
static bool  g_kalman_show_axes = true;

// Simple 3D->2D isometric projection
struct Vec3 { float x, y, z; };

static ImVec2 project3d(Vec3 v, float yaw, float pitch, float cx, float cy, float scale) {
    float c1 = cosf(yaw), s1 = sinf(yaw);
    float rx = v.x * c1 + v.z * s1;
    float rz = -v.x * s1 + v.z * c1;
    float ry = v.y;
    float c2 = cosf(pitch), s2 = sinf(pitch);
    float py = ry * c2 - rz * s2;
    return ImVec2(cx + rx * scale, cy - py * scale);
}

static void DrawKalman3D(const KalmanTrace& trace, float width, float height, bool show_controls = false) {
    if (show_controls) {
        ImGui::Checkbox("Axes", &g_kalman_show_axes);
        ImGui::SameLine();
        ImGui::Checkbox("Rotate", &g_kalman_auto_rotate);
    }

    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(10, 10, 15, 255));
    // Clip drawing to widget bounds
    dl->PushClipRect(pos, ImVec2(pos.x + width, pos.y + height), true);

    float cx = pos.x + width * 0.5f;
    float cy = pos.y + height * 0.5f;
    float dim = (width < height) ? width : height;
    float scale = dim * 0.35f;

    // Auto-rotate
    if (g_kalman_auto_rotate)
        g_kalman_yaw += 0.005f;

    float yaw = g_kalman_yaw;
    float pitch = g_kalman_pitch;

    // Draw axes
    float axis_len = 0.9f;
    Vec3 origin = {0, 0, 0};
    Vec3 ax_x = {axis_len, 0, 0};
    Vec3 ax_y = {0, axis_len, 0};
    Vec3 ax_z = {0, 0, axis_len};

    ImVec2 o2 = project3d(origin, yaw, pitch, cx, cy, scale);
    ImVec2 xp = project3d(ax_x, yaw, pitch, cx, cy, scale);
    ImVec2 yp = project3d(ax_y, yaw, pitch, cx, cy, scale);
    ImVec2 zp = project3d(ax_z, yaw, pitch, cx, cy, scale);

    if (g_kalman_show_axes) {
        dl->AddLine(o2, xp, IM_COL32(200, 60, 60, 150), 1.0f);
        dl->AddLine(o2, yp, IM_COL32(60, 200, 60, 150), 1.0f);
        dl->AddLine(o2, zp, IM_COL32(60, 60, 200, 150), 1.0f);
        dl->AddText(xp, IM_COL32(200, 80, 80, 200), "Ph");
        dl->AddText(yp, IM_COL32(80, 200, 80, 200), "Fr");
        dl->AddText(zp, IM_COL32(80, 80, 200, 200), "Ac");
    }

    if (trace.fwd.empty()) {
        const char* txt = "No Kalman data";
        ImVec2 ts = ImGui::CalcTextSize(txt);
        dl->AddText(ImVec2(cx - ts.x * 0.5f, cy - 6), IM_COL32(80, 80, 80, 200), txt);
    } else {
        // Compute data ranges for normalization to [-1, 1]
        float ph_min = 1e9f, ph_max = -1e9f;
        float fr_min = 1e9f, fr_max = -1e9f;
        float ac_min = 1e9f, ac_max = -1e9f;
        for (const auto& p : trace.smoothed) {
            ph_min = std::min(ph_min, p.phase); ph_max = std::max(ph_max, p.phase);
            fr_min = std::min(fr_min, p.freq);  fr_max = std::max(fr_max, p.freq);
            ac_min = std::min(ac_min, p.accel); ac_max = std::max(ac_max, p.accel);
        }
        for (const auto& p : trace.fwd) {
            ph_min = std::min(ph_min, p.phase); ph_max = std::max(ph_max, p.phase);
            fr_min = std::min(fr_min, p.freq);  fr_max = std::max(fr_max, p.freq);
            ac_min = std::min(ac_min, p.accel); ac_max = std::max(ac_max, p.accel);
        }
        auto safe_range = [](float& lo, float& hi) {
            if (hi - lo < 1e-9f) { lo -= 0.5f; hi += 0.5f; }
            float m = (hi - lo) * 0.05f;
            lo -= m; hi += m;
        };
        safe_range(ph_min, ph_max);
        safe_range(fr_min, fr_max);
        safe_range(ac_min, ac_max);

        auto normalize = [](float v, float lo, float hi) -> float {
            return 2.0f * (v - lo) / (hi - lo) - 1.0f;
        };

        auto to3d = [&](const KalmanTracePoint& p) -> Vec3 {
            return { normalize(p.phase, ph_min, ph_max) * 0.8f,
                     normalize(p.freq, fr_min, fr_max) * 0.8f,
                     normalize(p.accel, ac_min, ac_max) * 0.8f };
        };

        // Forward Kalman trajectory (cyan, thin)
        int n = (int)trace.fwd.size();
        for (int i = 1; i < n; i++) {
            ImVec2 a = project3d(to3d(trace.fwd[i-1]), yaw, pitch, cx, cy, scale);
            ImVec2 b = project3d(to3d(trace.fwd[i]), yaw, pitch, cx, cy, scale);
            dl->AddLine(a, b, IM_COL32(0, 160, 200, 100), 1.0f);
        }

        // Smoothed trajectory (green->yellow gradient, thicker)
        n = (int)trace.smoothed.size();
        for (int i = 1; i < n; i++) {
            float t = (float)i / (float)(n - 1);
            uint8_t r = (uint8_t)(t * 200);
            uint8_t g = (uint8_t)(200 + t * 55);
            ImVec2 a = project3d(to3d(trace.smoothed[i-1]), yaw, pitch, cx, cy, scale);
            ImVec2 b = project3d(to3d(trace.smoothed[i]), yaw, pitch, cx, cy, scale);
            dl->AddLine(a, b, IM_COL32(r, g, 0, 220), 1.5f);
        }

        // Pilot markers (white dots)
        for (int i = 0; i < (int)trace.smoothed.size(); i++) {
            if (trace.smoothed[i].is_pilot) {
                ImVec2 p = project3d(to3d(trace.smoothed[i]), yaw, pitch, cx, cy, scale);
                dl->AddCircleFilled(p, 2.0f, IM_COL32(255, 255, 255, 200));
            }
        }

        // Start (green) / end (red) markers
        if (!trace.smoothed.empty()) {
            ImVec2 sp = project3d(to3d(trace.smoothed.front()), yaw, pitch, cx, cy, scale);
            ImVec2 ep = project3d(to3d(trace.smoothed.back()), yaw, pitch, cx, cy, scale);
            dl->AddCircleFilled(sp, 4.0f, IM_COL32(0, 255, 0, 255));
            dl->AddCircleFilled(ep, 4.0f, IM_COL32(255, 60, 60, 255));
        }

        // Info text
        char info[128];
        snprintf(info, sizeof(info), "%d sym  Ph:%.0f%c  Fr:%.1e",
                 trace.total_symbols,
                 (ph_max - ph_min) * 180.0f / (float)M_PI, 0xB0,  // degree sign
                 fr_max - fr_min);
        dl->AddText(ImVec2(pos.x + 4, pos.y + 2), IM_COL32(160, 160, 160, 200), info);
    }

    dl->PopClipRect();
    dl->AddRect(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(80, 80, 80, 255));
    ImGui::Dummy(ImVec2(width, height));
}

// Waterfall color map: dB value -> color (black -> blue -> cyan -> green -> yellow -> red -> white)
static ImU32 waterfall_color(float db, float db_min = -80.0f, float db_max = 0.0f) {
    float frac = (db - db_min) / (db_max - db_min);
    if (frac < 0.0f) frac = 0.0f;
    if (frac > 1.0f) frac = 1.0f;

    uint8_t r, g, b;
    if (frac < 0.2f) {
        float t = frac / 0.2f;
        r = 0; g = 0; b = (uint8_t)(t * 180);
    } else if (frac < 0.4f) {
        float t = (frac - 0.2f) / 0.2f;
        r = 0; g = (uint8_t)(t * 200); b = (uint8_t)(180 + t * 75);
    } else if (frac < 0.6f) {
        float t = (frac - 0.4f) / 0.2f;
        r = 0; g = (uint8_t)(200 + t * 55); b = (uint8_t)(255 * (1 - t));
    } else if (frac < 0.8f) {
        float t = (frac - 0.6f) / 0.2f;
        r = (uint8_t)(t * 255); g = 255; b = 0;
    } else {
        float t = (frac - 0.8f) / 0.2f;
        r = 255; g = (uint8_t)(255 * (1 - t * 0.5f)); b = (uint8_t)(t * 255);
    }
    return IM_COL32(r, g, b, 255);
}

static void DrawWaterfall(std::vector<std::vector<float>>& history,
                          const std::vector<float>& current_spectrum,
                          float width, float height,
                          float band_low_hz, float band_high_hz,
                          int max_history) {
    ImVec2 pos = ImGui::GetCursorScreenPos();
    ImDrawList* dl = ImGui::GetWindowDrawList();

    // Push current spectrum into history
    if (!current_spectrum.empty()) {
        history.push_back(current_spectrum);
        while ((int)history.size() > max_history)
            history.erase(history.begin());
    }

    // Draw background
    dl->AddRectFilled(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(0, 0, 0, 255));

    if (history.empty()) {
        ImGui::Dummy(ImVec2(width, height + 16));
        return;
    }

    int n_rows = (int)history.size();
    float row_h = height / max_history;
    if (row_h < 1.0f) row_h = 1.0f;

    for (int row = 0; row < n_rows; row++) {
        const auto& spec = history[row];
        int n_bins = (int)spec.size();
        if (n_bins <= 0) continue;
        float bin_w = width / n_bins;

        // Draw this row from bottom (newest at bottom)
        float y0 = pos.y + height - (n_rows - row) * row_h;
        float y1 = y0 + row_h;
        if (y0 < pos.y) y0 = pos.y;
        if (y1 > pos.y + height) y1 = pos.y + height;

        for (int i = 0; i < n_bins; i++) {
            float x0 = pos.x + i * bin_w;
            float x1 = x0 + bin_w;
            if (x1 > pos.x + width) x1 = pos.x + width;
            dl->AddRectFilled(ImVec2(x0, y0), ImVec2(x1, y1), waterfall_color(spec[i]));
        }
    }

    dl->AddRect(pos, ImVec2(pos.x + width, pos.y + height), IM_COL32(80, 80, 80, 255));

    // Frequency axis labels
    float label_y = pos.y + height + 2;
    int n_labels = 5;
    for (int i = 0; i <= n_labels; i++) {
        float frac_x = (float)i / n_labels;
        float freq = band_low_hz + frac_x * (band_high_hz - band_low_hz);
        float lx = pos.x + frac_x * width;
        char lbl[32];
        if (freq >= 1000.0f)
            snprintf(lbl, sizeof(lbl), "%.1fk", freq / 1000.0f);
        else
            snprintf(lbl, sizeof(lbl), "%.0f", freq);
        ImVec2 ts = ImGui::CalcTextSize(lbl);
        float tx = lx - ts.x * 0.5f;
        if (tx < pos.x) tx = pos.x;
        if (tx + ts.x > pos.x + width) tx = pos.x + width - ts.x;
        dl->AddText(ImVec2(tx, label_y), IM_COL32(140, 140, 140, 200), lbl);
        dl->AddLine(ImVec2(lx, pos.y + height - 3), ImVec2(lx, pos.y + height),
                     IM_COL32(100, 100, 100, 200));
    }

    ImGui::Dummy(ImVec2(width, height + 16));
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
    style.TabRounding = 2.0f;
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
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Setup...")) show_config_ = true;
            ImGui::Separator();
            if (ImGui::MenuItem("Quit")) {
                if (callbacks_.on_quit) callbacks_.on_quit();
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Event Log...", nullptr, show_log_))
                show_log_ = !show_log_;
            if (ImGui::MenuItem("Packet Log...", nullptr, show_packet_log_))
                show_packet_log_ = !show_packet_log_;
            if (ImGui::MenuItem("Kalman 3D...", nullptr, show_kalman_))
                show_kalman_ = !show_kalman_;
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    // === Full-window main panel ===
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

    // ========== Signal Quality Row ==========
    ImGui::BeginChild("SignalQuality", ImVec2(0, 150), true, ImGuiWindowFlags_NoScrollbar);
    {
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float diag_sz = avail.y;
        if (diag_sz < 80) diag_sz = 80;

        // Left: 3D Kalman state viewer (phase/freq/accel trajectory)
        DrawKalman3D(diag.kalman_trace, diag_sz, diag_sz);

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

            // AX.25 session state takes priority for display when active
            if (diag.ax25_state == Ax25SessionState::CONNECTED ||
                diag.ax25_state == Ax25SessionState::TIMER_RECOVERY) {
                arq_str = "CONNECTED"; arq_color = ImVec4(0, 1, 0, 1);
            } else if (diag.ax25_state == Ax25SessionState::AWAITING_CONNECTION) {
                arq_str = "CONNECTING"; arq_color = ImVec4(1, 1, 0, 1);
            } else if (diag.ax25_state == Ax25SessionState::AWAITING_RELEASE) {
                arq_str = "DISCONNECTING"; arq_color = ImVec4(1, 0.5f, 0, 1);
            } else if (diag.arq_state == ArqState::CONNECTING) {
                arq_str = "CONNECTING"; arq_color = ImVec4(1, 1, 0, 1);
            } else if (diag.arq_state == ArqState::CONNECTED) {
                arq_str = "CONNECTED"; arq_color = ImVec4(0, 1, 0, 1);
            } else if (diag.arq_state == ArqState::DISCONNECTING) {
                arq_str = "DISCONNECTING"; arq_color = ImVec4(1, 0.5f, 0, 1);
            } else if (diag.arq_state == ArqState::HAILING) {
                arq_str = "HAILING"; arq_color = ImVec4(1, 1, 0, 1);
            } else if (diag.arq_state == ArqState::LISTENING) {
                arq_str = "LISTENING"; arq_color = ImVec4(0.3f, 0.6f, 0.9f, 1.0f);
            }

            ImGui::TextColored(arq_color, "%s", arq_str);
            if (diag.arq_state != ArqState::IDLE &&
                diag.arq_state != ArqState::LISTENING) {
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

        // Speed level indicators (read-only, not clickable)
        ImGui::BeginGroup();
        {
            ImGui::Text("Speed Level:");
            for (int i = 0; i < NUM_SPEED_LEVELS; i++) {
                bool active = (i == diag.speed_level);
                ImVec4 col = active ? ImVec4(0.2f, 0.7f, 0.2f, 1.0f)
                                    : ImVec4(0.22f, 0.22f, 0.25f, 1.0f);
                ImVec4 text_col = active ? ImVec4(1, 1, 1, 1) : ImVec4(0.5f, 0.5f, 0.5f, 1);
                ImGui::PushStyleColor(ImGuiCol_Button, col);
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, col);  // No hover effect
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, col);   // No click effect
                ImGui::PushStyleColor(ImGuiCol_Text, text_col);
                ImGui::SmallButton(SPEED_LEVELS[i].name);
                ImGui::PopStyleColor(4);
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

            ImGui::Checkbox("Native Hail", &config.native_hail);
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Use native PHY (BPSK+LDPC) for hailing.\n~10 dB better sensitivity than AFSK.\nBoth sides must enable this.");

            // Auto Tune: callsign input + button
            {
                static char tune_call[10] = "";
                ImGui::SetNextItemWidth(70);
                ImGui::InputText("##tunecall", tune_call, sizeof(tune_call),
                                 ImGuiInputTextFlags_CharsUppercase | ImGuiInputTextFlags_CharsNoBlank);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Remote callsign for auto-tune");
                ImGui::SameLine();
                bool can_tune = (tune_call[0] != '\0');
                if (!can_tune) ImGui::BeginDisabled();
                if (ImGui::Button("Auto Tune", ImVec2(90, 28))) {
                    if (callbacks_.on_autotune) callbacks_.on_autotune(std::string(tune_call));
                }
                if (!can_tune) ImGui::EndDisabled();
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Bilateral gain calibration using native frames.\nBoth sides adjust TX level so peer receives at gain ~1.0.");
            }

            if (ImGui::Button("Auto Cal", ImVec2(90, 28))) {
                if (callbacks_.on_calibrate) callbacks_.on_calibrate();
            }
            ImGui::SameLine();
            if (ImGui::Button("Probe", ImVec2(90, 28))) {
                if (callbacks_.on_probe) callbacks_.on_probe();
            }
            if (config.calibrated_tx_level > 0)
                ImGui::Text("Cal: %.3f", config.calibrated_tx_level);
            if (diag.native_rx_gain != 1.0f) {
                ImGui::SameLine();
                ImGui::Text("RxG: %.2f", diag.native_rx_gain);
            }
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

        int baud = config.mode == "B" ? 9600 : 1200;
        ImGui::Text("AX.25: %d baud  |  %s", baud,
                     config.ax25_only ? "AX.25 Only" : "Native OK");

        ImGui::Columns(1);
    }
    ImGui::EndChild();

    // ========== Passband Probe Visualization (always visible) ==========
    {
        float probe_h = diag.probe_has_results ? 80.0f : 24.0f;
        ImGui::BeginChild("ProbeVis", ImVec2(0, probe_h), true, ImGuiWindowFlags_NoScrollbar);
        if (diag.probe_has_results) {
            ImVec2 avail = ImGui::GetContentRegionAvail();
            ImDrawList* dl = ImGui::GetWindowDrawList();
            float w = avail.x;
            float row_h = 14.0f;
            float bar_h = row_h - 2.0f;
            float label_w = 70.0f;
            float tone_w = (w - label_w - 10.0f) / PassbandProbeConfig::N_TONES;
            if (tone_w < 2.0f) tone_w = 2.0f;

            if (diag.probe_negotiated.valid) {
                ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f),
                    "Passband: %.0f - %.0f Hz (%.0f Hz usable)",
                    diag.probe_negotiated.low_hz, diag.probe_negotiated.high_hz,
                    diag.probe_negotiated.bandwidth_hz);
            } else {
                ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.2f, 1.0f), "Passband: probing...");
            }

            // Row 1: Our TX -> Their RX
            ImVec2 r1 = ImGui::GetCursorScreenPos();
            dl->AddText(ImVec2(r1.x, r1.y + 1), IM_COL32(180, 180, 180, 255), "They hear");
            for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                float x0 = r1.x + label_w + k * tone_w;
                ImU32 col = diag.probe_my_tx.tone_detected[k]
                    ? IM_COL32(0, 200, 100, 220) : IM_COL32(60, 20, 20, 180);
                dl->AddRectFilled(ImVec2(x0, r1.y), ImVec2(x0 + tone_w - 1, r1.y + bar_h), col);
            }
            ImGui::Dummy(ImVec2(w, row_h));

            // Row 2: Their TX -> Our RX
            ImVec2 r2 = ImGui::GetCursorScreenPos();
            dl->AddText(ImVec2(r2.x, r2.y + 1), IM_COL32(180, 180, 180, 255), "We hear");
            for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                float x0 = r2.x + label_w + k * tone_w;
                ImU32 col = diag.probe_their_tx.tone_detected[k]
                    ? IM_COL32(100, 150, 255, 220) : IM_COL32(20, 20, 60, 180);
                dl->AddRectFilled(ImVec2(x0, r2.y), ImVec2(x0 + tone_w - 1, r2.y + bar_h), col);
            }
            ImGui::Dummy(ImVec2(w, row_h));

            // Row 3: Negotiated overlap
            ImVec2 r3 = ImGui::GetCursorScreenPos();
            dl->AddText(ImVec2(r3.x, r3.y + 1), IM_COL32(180, 180, 180, 255), "Usable");
            for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
                float x0 = r3.x + label_w + k * tone_w;
                bool both = diag.probe_my_tx.tone_detected[k] &&
                            diag.probe_their_tx.tone_detected[k];
                ImU32 col;
                if (both) {
                    float freq = PassbandProbeConfig::TONE_LOW_HZ +
                                 k * PassbandProbeConfig::TONE_SPACING_HZ;
                    if (diag.probe_negotiated.valid &&
                        freq >= diag.probe_negotiated.low_hz &&
                        freq <= diag.probe_negotiated.high_hz)
                        col = IM_COL32(255, 220, 50, 240);
                    else
                        col = IM_COL32(120, 100, 30, 180);
                } else {
                    col = IM_COL32(30, 30, 30, 150);
                }
                dl->AddRectFilled(ImVec2(x0, r3.y), ImVec2(x0 + tone_w - 1, r3.y + bar_h), col);
            }
            ImGui::Dummy(ImVec2(w, row_h));
        } else {
            ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f),
                "Passband: %.0f - %.0f Hz (config default)",
                diag.band_low_hz, diag.band_high_hz);
        }
        ImGui::EndChild();
    }

    // ========== DCD Threshold (inline, above waterfall) ==========
    {
        ImGui::Checkbox("Auto DCD", &config.dcd_auto);
        ImGui::SameLine();
        ImGui::PushItemWidth(200);
        float thr = config.dcd_threshold;
        if (ImGui::SliderFloat("DCD threshold", &thr, 0.0f, 0.30f, "%.3f"))
            config.dcd_threshold = thr;
        ImGui::PopItemWidth();
        ImGui::SameLine();
        ImGui::PushItemWidth(120);
        int holdoff = config.dcd_holdoff_ms;
        if (ImGui::SliderInt("Holdoff ms", &holdoff, 50, 500))
            config.dcd_holdoff_ms = holdoff;
        ImGui::PopItemWidth();
        ImGui::SameLine();
        if (diag.dcd_busy)
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "BUSY  tone:%.0f  rms:%.4f", diag.dcd_tone_energy, diag.rx_raw_rms);
        else
            ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "clear  tone:%.0f  rms:%.4f", diag.dcd_tone_energy, diag.rx_raw_rms);
    }

    // ========== Waterfall (fills remaining space above status bar) ==========
    {
        float status_bar_h = 27.0f;
        float remaining = ImGui::GetContentRegionAvail().y - status_bar_h - 4.0f;
        float wf_h = remaining > 60.0f ? remaining - 18.0f : 60.0f;
        ImGui::BeginChild("Waterfall", ImVec2(0, remaining), true,
                           ImGuiWindowFlags_NoScrollbar);
        DrawWaterfall(wf_history_, diag.spectrum,
                      ImGui::GetContentRegionAvail().x, wf_h,
                      diag.spectrum_low_hz, diag.spectrum_high_hz, WF_HISTORY);
        ImGui::EndChild();
    }

    // ========== Bottom Status Bar (pinned to bottom) ==========
    {
        ImGui::BeginChild("StatusBar", ImVec2(0, 0), true, ImGuiWindowFlags_NoScrollbar);
        ImDrawList* dl = ImGui::GetWindowDrawList();
        ImVec2 p = ImGui::GetCursorScreenPos();

        // Status LED — AX.25 session takes priority when active
        ImU32 led_col;
        const char* status_text;
        if (diag.ax25_state == Ax25SessionState::CONNECTED ||
            diag.ax25_state == Ax25SessionState::TIMER_RECOVERY) {
            led_col = IM_COL32(0, 220, 0, 255); status_text = "CONNECTED";
        } else if (diag.ax25_state == Ax25SessionState::AWAITING_CONNECTION) {
            led_col = IM_COL32(220, 220, 0, 255); status_text = "CONNECTING";
        } else if (diag.ax25_state == Ax25SessionState::AWAITING_RELEASE) {
            led_col = IM_COL32(220, 120, 0, 255); status_text = "DISCONNECTING";
        } else if (diag.arq_state == ArqState::CONNECTED) {
            led_col = IM_COL32(0, 220, 0, 255); status_text = "CONNECTED";
        } else if (diag.arq_state == ArqState::CONNECTING) {
            led_col = IM_COL32(220, 220, 0, 255); status_text = "CONNECTING";
        } else if (diag.arq_state == ArqState::HAILING) {
            led_col = IM_COL32(220, 220, 0, 255); status_text = "HAILING";
        } else if (diag.arq_state == ArqState::LISTENING) {
            led_col = IM_COL32(80, 160, 220, 255); status_text = "LISTENING";
        } else {
            led_col = IM_COL32(80, 80, 80, 255); status_text = "IDLE";
        }
        dl->AddCircleFilled(ImVec2(p.x + 6, p.y + 8), 5, led_col);
        ImGui::SetCursorScreenPos(ImVec2(p.x + 16, p.y));
        ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 1.0f), "%s", status_text);

        ImGui::SameLine(0, 6);
        ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), "|");
        ImGui::SameLine(0, 6);

        // Mode + baud
        if (diag.native_mode)
            ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.3f, 1.0f), "Native %d baud", diag.baud_rate);
        else
            ImGui::TextColored(ImVec4(0.7f, 0.7f, 0.7f, 1.0f), "AX.25");

        ImGui::SameLine(0, 6);
        ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), "|");
        ImGui::SameLine(0, 6);

        // TX/RX/DCD indicator
        bool is_tx = diag.ptt_active;
        if (is_tx)
            ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "TX");
        else if (diag.dcd_busy)
            ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "DCD");
        else
            ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.2f, 1.0f), "RX");

        ImGui::SameLine(0, 6);
        ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), "|");
        ImGui::SameLine(0, 6);

        // PHY bitrate + app bitrate with efficiency
        if (diag.compression_ratio > 1.0f)
            ImGui::Text("%d/%d bps (%.1fx)", diag.app_bps, diag.phy_bps, diag.compression_ratio);
        else if (diag.phy_bps >= 1000)
            ImGui::Text("%.1f kbps", diag.phy_bps / 1000.0f);
        else
            ImGui::Text("%d bps", diag.phy_bps);

        ImGui::SameLine(0, 6);
        ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), "|");
        ImGui::SameLine(0, 6);

        // Bytes transferred
        ImGui::Text("TX:%lluB RX:%lluB",
                     (unsigned long long)diag.bytes_tx,
                     (unsigned long long)diag.bytes_rx);

        ImGui::SameLine(0, 6);
        ImGui::TextColored(ImVec4(0.3f, 0.3f, 0.3f, 1.0f), "|");
        ImGui::SameLine(0, 6);

        // Encryption state
        switch (diag.encryption_state) {
            case 0: ImGui::TextColored(ImVec4(0.4f, 0.4f, 0.4f, 1.0f), "No Encryption"); break;
            case 1: ImGui::TextColored(ImVec4(1.0f, 1.0f, 0.2f, 1.0f), "KEY EXCHANGE"); break;
            case 2: ImGui::TextColored(ImVec4(0.2f, 1.0f, 0.4f, 1.0f), "ENCRYPTED"); break;
            case 3: ImGui::TextColored(ImVec4(1.0f, 0.2f, 0.2f, 1.0f), "PSK MISMATCH"); break;
        }

        // Level overflow indicator
        ImGui::SameLine(0, 6);
        if (diag.rx_peak > 0.95f) {
            ImGui::TextColored(ImVec4(1.0f, 0.1f, 0.1f, 1.0f), "OVL");
        }

        ImGui::EndChild();
    }

    ImGui::End(); // ##Main

    // ========== Event Log Window (separate, toggled from View menu) ==========
    if (show_log_) {
        ImGui::SetNextWindowSize(ImVec2(500, 300), ImGuiCond_FirstUseEver);
        ImGui::Begin("Event Log", &show_log_);
        for (const auto& line : log_lines_)
            ImGui::TextUnformatted(line.c_str());
        if (ImGui::GetScrollY() >= ImGui::GetScrollMaxY())
            ImGui::SetScrollHereY(1.0f);
        ImGui::End();
    }

    // ========== Packet Log Window (separate, toggled from View menu) ==========
    if (show_packet_log_) {
        ImGui::SetNextWindowSize(ImVec2(600, 350), ImGuiCond_FirstUseEver);
        ImGui::Begin("Packet Log", &show_packet_log_);

        // Clear button
        if (ImGui::Button("Clear")) {
            std::lock_guard<std::mutex> lock(packet_log_mutex_);
            packet_log_.clear();
        }
        ImGui::SameLine();
        {
            std::lock_guard<std::mutex> lock(packet_log_mutex_);
            ImGui::Text("%zu packets", packet_log_.size());
        }
        ImGui::Separator();

        // Build text buffer from packet log for selectable/copyable display
        static std::string pkt_text_buf;
        bool auto_scroll = false;
        {
            std::lock_guard<std::mutex> lock(packet_log_mutex_);
            pkt_text_buf.clear();
            pkt_text_buf.reserve(packet_log_.size() * 80);
            for (const auto& entry : packet_log_) {
                pkt_text_buf += entry.timestamp;
                pkt_text_buf += entry.is_tx ? " TX [" : " RX [";
                pkt_text_buf += entry.protocol;
                pkt_text_buf += "] ";
                pkt_text_buf += entry.description;
                pkt_text_buf += '\n';
            }
            static size_t prev_count = 0;
            if (packet_log_.size() != prev_count) {
                auto_scroll = true;
                prev_count = packet_log_.size();
            }
        }
        // Read-only multiline input — supports text selection and Ctrl+C
        ImVec2 avail = ImGui::GetContentRegionAvail();
        // Need writable buffer for InputTextMultiline
        pkt_text_buf.push_back('\0');  // ensure null terminator space
        ImGui::InputTextMultiline("##pktlog", pkt_text_buf.data(),
                                  pkt_text_buf.size(),
                                  avail, ImGuiInputTextFlags_ReadOnly);

        ImGui::End();
    }

    // ========== Kalman 3D Window (floating, draggable, resizable) ==========
    if (show_kalman_) {
        ImGui::SetNextWindowSize(ImVec2(400, 420), ImGuiCond_FirstUseEver);
        ImGui::Begin("Kalman 3D", &show_kalman_);
        ImVec2 avail = ImGui::GetContentRegionAvail();
        float sz = (avail.x < avail.y) ? avail.x : avail.y;
        if (sz < 100) sz = 100;
        DrawKalman3D(last_diag_.kalman_trace, sz, sz, true);
        ImGui::End();
    }

    // ========== Settings Window (tabs, not collapsible) ==========
    if (show_config_) {
        ImGui::SetNextWindowSize(ImVec2(440, 480), ImGuiCond_FirstUseEver);
        ImGui::Begin("Setup", &show_config_, ImGuiWindowFlags_NoCollapse);

        if (ImGui::BeginTabBar("SettingsTabs")) {

            // --- Station Tab ---
            if (ImGui::BeginTabItem("Station")) {
                ImGui::Spacing();
                char cs_buf[16];
                strncpy(cs_buf, config.callsign.c_str(), sizeof(cs_buf) - 1);
                cs_buf[sizeof(cs_buf) - 1] = '\0';
                if (ImGui::InputText("Callsign", cs_buf, sizeof(cs_buf)))
                    config.callsign = cs_buf;

                const char* modes[] = {"A - 1200 baud (Standard FM)", "B - 9600 baud (9600 FM)"};
                int mi = config.mode == "B" ? 1 : 0;
                if (ImGui::Combo("Channel Mode", &mi, modes, 2)) {
                    config.mode = mi == 0 ? "A" : "B";
                    config.ax25_baud = mi == 0 ? 1200 : 9600;
                }

                ImGui::Checkbox("AX.25 Only (no native upgrade)", &config.ax25_only);

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                const char* mod_names[] = {"BPSK", "QPSK", "QAM16", "QAM64", "QAM256"};
                int mod_idx = (int)config.max_modulation;
                if (mod_idx < 0) mod_idx = 0;
                if (mod_idx > 4) mod_idx = 4;
                if (ImGui::Combo("Max Modulation", &mod_idx, mod_names, 5))
                    config.max_modulation = (Modulation)mod_idx;

                ImGui::SliderFloat("TX Level", &config.tx_level, 0.0f, 1.0f, "%.2f");
                ImGui::SliderFloat("RX Gain", &config.rx_gain, 0.1f, 10.0f, "%.1f");

                ImGui::EndTabItem();
            }

            // --- Audio Tab ---
            if (ImGui::BeginTabItem("Audio")) {
                ImGui::Spacing();
                if (!audio_devices.empty()) {
                    // Build filtered lists with device IDs
                    struct DevEntry { int id; std::string name; };
                    std::vector<DevEntry> cap_devs, play_devs;
                    for (const auto& d : audio_devices) {
                        if (d.max_input_channels > 0) cap_devs.push_back({d.id, d.name});
                        if (d.max_output_channels > 0) play_devs.push_back({d.id, d.name});
                    }

                    // Initialize selection from config (once)
                    static bool audio_sel_init = false;
                    if (!audio_sel_init) {
                        // Find matching device, or default to first in list
                        selected_capture_ = 0;
                        for (int i = 0; i < (int)cap_devs.size(); i++)
                            if (cap_devs[i].id == config.capture_device) { selected_capture_ = i; break; }
                        selected_playback_ = 0;
                        for (int i = 0; i < (int)play_devs.size(); i++)
                            if (play_devs[i].id == config.playback_device) { selected_playback_ = i; break; }
                        // Resolve -1 (default) to actual device ID so Save persists it
                        if (!cap_devs.empty()) config.capture_device = cap_devs[selected_capture_].id;
                        if (!play_devs.empty()) config.playback_device = play_devs[selected_playback_].id;
                        audio_sel_init = true;
                    }

                    if (!cap_devs.empty()) {
                        if (selected_capture_ >= (int)cap_devs.size()) selected_capture_ = 0;
                        if (ImGui::BeginCombo("Capture Device", cap_devs[selected_capture_].name.c_str())) {
                            for (int i = 0; i < (int)cap_devs.size(); i++)
                                if (ImGui::Selectable(cap_devs[i].name.c_str(), i == selected_capture_)) {
                                    selected_capture_ = i;
                                    config.capture_device = cap_devs[i].id;
                                }
                            ImGui::EndCombo();
                        }
                    }
                    if (!play_devs.empty()) {
                        if (selected_playback_ >= (int)play_devs.size()) selected_playback_ = 0;
                        if (ImGui::BeginCombo("Playback Device", play_devs[selected_playback_].name.c_str())) {
                            for (int i = 0; i < (int)play_devs.size(); i++)
                                if (ImGui::Selectable(play_devs[i].name.c_str(), i == selected_playback_)) {
                                    selected_playback_ = i;
                                    config.playback_device = play_devs[i].id;
                                }
                            ImGui::EndCombo();
                        }
                    }
                    ImGui::Spacing();
                    ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                        "Changing audio devices requires a modem restart.");
                } else {
                    ImGui::TextColored(ImVec4(1, 1, 0, 1), "No audio devices found");
                }
                ImGui::EndTabItem();
            }

            // --- Radio / PTT Tab ---
            if (ImGui::BeginTabItem("Radio")) {
                ImGui::Spacing();
                const char* ptt_methods[] = {"None", "Rig Control (rigctld)", "VOX", "CM108 GPIO", "Serial Port"};
                int pi = 0;
                if (config.ptt_method == "rigctl") pi = 1;
                else if (config.ptt_method == "vox") pi = 2;
                else if (config.ptt_method == "cm108") pi = 3;
                else if (config.ptt_method == "serial") pi = 4;
                if (ImGui::Combo("PTT Method", &pi, ptt_methods, 5)) {
                    const char* method_ids[] = {"none", "rigctl", "vox", "cm108", "serial"};
                    config.ptt_method = method_ids[pi];
                }

                ImGui::Spacing();

                if (pi == 1) {
                    // Rig Control — radio picker
                    const auto& models = get_radio_models();

                    // Find current selection index
                    static int radio_sel = -1;
                    static char radio_filter[64] = "";
                    if (radio_sel < 0) {
                        for (int j = 0; j < (int)models.size(); j++) {
                            if (models[j].id == config.rigctld_model) { radio_sel = j; break; }
                        }
                        if (radio_sel < 0) radio_sel = 0;
                    }

                    const char* preview = (radio_sel >= 0 && radio_sel < (int)models.size())
                        ? models[radio_sel].display.c_str() : "Select radio...";

                    ImGui::Text("Radio:");
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                    if (ImGui::BeginCombo("##RadioModel", preview)) {
                        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                        ImGui::InputTextWithHint("##filter", "Type to filter...",
                                                  radio_filter, sizeof(radio_filter));
                        ImGui::Separator();

                        for (int j = 0; j < (int)models.size(); j++) {
                            if (radio_filter[0] != '\0') {
                                std::string dl = models[j].display;
                                std::string fl = radio_filter;
                                for (auto& c : dl) c = (char)tolower(c);
                                for (auto& c : fl) c = (char)tolower(c);
                                if (dl.find(fl) == std::string::npos) continue;
                            }
                            bool selected = (j == radio_sel);
                            if (ImGui::Selectable(models[j].display.c_str(), selected)) {
                                radio_sel = j;
                                config.rigctld_model = models[j].id;
                                radio_filter[0] = '\0';
                            }
                            if (selected) ImGui::SetItemDefaultFocus();
                        }
                        ImGui::EndCombo();
                    }

                    if (config.rigctld_model > 0) {
                        // COM port dropdown for radio
                        static auto rig_ports = enumerate_serial_ports();
                        static bool rig_ports_init = false;
                        if (!rig_ports_init) { rig_ports = enumerate_serial_ports(); rig_ports_init = true; }

                        const char* cur_port = config.rigctld_device.empty()
                            ? "(select port)" : config.rigctld_device.c_str();
                        if (ImGui::BeginCombo("Radio Port", cur_port)) {
                            for (const auto& p : rig_ports) {
                                bool sel = (p == config.rigctld_device);
                                if (ImGui::Selectable(p.c_str(), sel))
                                    config.rigctld_device = p;
                            }
                            ImGui::EndCombo();
                        }
                        if (rig_ports.empty())
                            ImGui::TextColored(ImVec4(1, 1, 0, 1), "No COM ports detected");
                        ImGui::TextColored(ImVec4(0.5f, 0.7f, 1.0f, 1.0f),
                            "Iris will auto-launch rigctld for this radio.");
                    } else {
                        char hb[64];
                        strncpy(hb, config.rigctl_host.c_str(), sizeof(hb) - 1);
                        hb[sizeof(hb) - 1] = '\0';
                        if (ImGui::InputText("rigctld Host", hb, sizeof(hb)))
                            config.rigctl_host = hb;
                        ImGui::InputInt("rigctld Port", &config.rigctl_port);
                        ImGui::TextWrapped("You manage rigctld externally.");
                    }
                }

                if (pi == 3) {
                    // CM108 GPIO
                    const char* gpio_pins[] = {"GPIO 1", "GPIO 2", "GPIO 3", "GPIO 4",
                                               "GPIO 5", "GPIO 6", "GPIO 7", "GPIO 8"};
                    int gpi = config.cm108_gpio - 1;
                    if (gpi < 0) gpi = 2;  // Default GPIO 3
                    if (gpi > 7) gpi = 7;
                    if (ImGui::Combo("GPIO Pin", &gpi, gpio_pins, 8))
                        config.cm108_gpio = gpi + 1;
                    ImGui::TextWrapped("CM108/CM119 USB audio GPIO PTT (Digirig, Masters, etc.).");
                }

                if (pi == 4) {
                    // Serial port — use shared port enumeration
                    static auto ser_ports = enumerate_serial_ports();
                    static bool ser_ports_init = false;
                    if (!ser_ports_init) { ser_ports = enumerate_serial_ports(); ser_ports_init = true; }

                    if (!ser_ports.empty()) {
                        const char* cur = config.serial_port.empty()
                            ? "(select port)" : config.serial_port.c_str();
                        if (ImGui::BeginCombo("Serial Port", cur)) {
                            for (const auto& p : ser_ports) {
                                bool sel = (p == config.serial_port);
                                if (ImGui::Selectable(p.c_str(), sel))
                                    config.serial_port = p;
                            }
                            ImGui::EndCombo();
                        }
                    } else {
                        char sp[64];
                        strncpy(sp, config.serial_port.c_str(), sizeof(sp) - 1);
                        sp[sizeof(sp) - 1] = '\0';
                        if (ImGui::InputText("Serial Port", sp, sizeof(sp)))
                            config.serial_port = sp;
                        ImGui::TextColored(ImVec4(1, 1, 0, 1), "No serial ports detected");
                    }

                    const char* ptt_lines[] = {"RTS", "DTR"};
                    if (ImGui::Combo("PTT Line", &config.serial_ptt_line, ptt_lines, 2)) {}

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

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                ImGui::Text("AX.25 Timing");
                ImGui::Text("TXDelay:  %d ms", config.ptt_pre_delay_ms);
                ImGui::Text("TXTail:   %d ms", config.ptt_post_delay_ms);
                ImGui::Text("SlotTime: %d ms", config.slottime_ms);
                ImGui::Text("Persist:  %d", config.persist);
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "These values are set by your KISS host (Winlink, etc).");

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                ImGui::Text("DCD (Carrier Detect)");
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "DCD threshold is adjustable on the main screen.");

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();
                ImGui::Text("FX.25 Forward Error Correction");
                {
                    int fx25_idx = 0;
                    if (config.fx25_mode == 16) fx25_idx = 1;
                    else if (config.fx25_mode == 32) fx25_idx = 2;
                    else if (config.fx25_mode == 64) fx25_idx = 3;
                    const char* fx25_items[] = {"Off", "16 check bytes", "32 check bytes", "64 check bytes"};
                    if (ImGui::Combo("FX.25 TX Mode", &fx25_idx, fx25_items, 4)) {
                        const int fx25_vals[] = {0, 16, 32, 64};
                        config.fx25_mode = fx25_vals[fx25_idx];
                    }
                }
                ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                    "Reed-Solomon FEC for AX.25. RX decodes FX.25 automatically.");

                ImGui::EndTabItem();
            }

            // --- Network Tab ---
            if (ImGui::BeginTabItem("Network")) {
                ImGui::Spacing();
                ImGui::InputInt("KISS Port", &config.kiss_port);
                ImGui::InputInt("AGW Port", &config.agw_port);
                ImGui::Spacing();
                ImGui::TextWrapped("KISS: Used by Winlink Express, APRS clients.");
                ImGui::TextWrapped("AGW: Used by Pat and AGW-compatible software.");
                ImGui::EndTabItem();
            }

            // --- Security Tab ---
            if (ImGui::BeginTabItem("Security")) {
                ImGui::Spacing();
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
                ImGui::EndTabItem();
            }

            // --- Advanced Tab ---
            if (ImGui::BeginTabItem("Advanced")) {
                ImGui::Spacing();
                ImGui::PushTextWrapPos(ImGui::GetContentRegionAvail().x);
                ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.3f, 1.0f),
                    "These are development settings. Changing them may cause "
                    "incompatibility with other modems. All stations in a "
                    "session must use identical settings.");
                ImGui::PopTextWrapPos();
                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                // Band edges — auto-negotiated by probe, with manual override option
                ImGui::Text("Frequency Band");
                float bw = (config.band_high_hz - config.band_low_hz);
                ImGui::TextColored(ImVec4(0.5f, 0.7f, 1.0f, 1.0f),
                    "Current: %.0f - %.0f Hz (%.0f Hz BW)",
                    config.band_low_hz, config.band_high_hz, bw);

                static bool manual_override = false;
                ImGui::Checkbox("Manual band edge override", &manual_override);
                if (manual_override) {
                    ImGui::SliderFloat("Low Edge (Hz)", &config.band_low_hz, 200.0f, 1000.0f, "%.0f Hz");
                    ImGui::SliderFloat("High Edge (Hz)", &config.band_high_hz, 2000.0f, 5000.0f, "%.0f Hz");
                    if (config.band_low_hz < 255.0f)
                        ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f),
                            "Warning: Low edge below CTCSS max (254 Hz)!");
                } else {
                    ImGui::TextColored(ImVec4(0.6f, 0.6f, 0.6f, 1.0f),
                        "Auto-negotiated by passband probe during connection.");
                }

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                ImGui::Text("AFSK De-emphasis Compensation");
                ImGui::SliderFloat("Pre-emph alpha", &config.preemph_alpha, 0.0f, 0.99f, "%.2f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Compensates FM de-emphasis. 0.95 = standard FM, 0.0 = flat (no compensation).");

                ImGui::Spacing();
                ImGui::Separator();
                ImGui::Spacing();

                ImGui::Text("Logging");
                ImGui::Checkbox("Enable log file output", &config.log_enabled);
                ImGui::TextWrapped("Logs to AppData/Iris/logs/ (Win) or ~/.config/iris/logs/ (Linux).");

                ImGui::EndTabItem();
            }

            ImGui::EndTabBar();
        }

        ImGui::Spacing();
        ImGui::Separator();
        if (ImGui::Button("Save & Restart", ImVec2(130, 0))) {
            if (callbacks_.on_config_changed) callbacks_.on_config_changed(config);
            config_dirty_ = false;
            show_config_ = false;
        }
        if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Save settings and restart the modem");
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(80, 0))) {
            show_config_ = false;
        }

        // Detect if user closed via X button (show_config_ was set false by ImGui)
        if (!show_config_ && config_dirty_) {
            // Re-open to show save prompt
            show_config_ = true;
            ImGui::OpenPopup("Unsaved Changes");
        }
        if (ImGui::BeginPopupModal("Unsaved Changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
            ImGui::Text("You have unsaved changes. Save before closing?");
            ImGui::Spacing();
            if (ImGui::Button("Save & Restart", ImVec2(130, 0))) {
                if (callbacks_.on_config_changed) callbacks_.on_config_changed(config);
                config_dirty_ = false;
                show_config_ = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Discard", ImVec2(80, 0))) {
                config_dirty_ = false;
                show_config_ = false;
                ImGui::CloseCurrentPopup();
            }
            ImGui::SameLine();
            if (ImGui::Button("Cancel", ImVec2(80, 0))) {
                ImGui::CloseCurrentPopup();
            }
            ImGui::EndPopup();
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

void IrisGui::log_packet(bool is_tx, const std::string& protocol,
                          const std::string& description) {
    // Get current time as HH:MM:SS
    char ts[16];
    time_t now = time(nullptr);
    struct tm* t = localtime(&now);
    snprintf(ts, sizeof(ts), "%02d:%02d:%02d", t->tm_hour, t->tm_min, t->tm_sec);

    std::lock_guard<std::mutex> lock(packet_log_mutex_);
    packet_log_.push_back({is_tx, ts, protocol, description});
    if (packet_log_.size() > 2000)
        packet_log_.erase(packet_log_.begin());
}

} // namespace iris

#endif // IRIS_HAS_IMGUI
