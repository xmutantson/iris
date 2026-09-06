#include "radio/rigctl.h"
#include <memory>
#include <vector>
#include <cstring>
#include <cstdio>
#include <cstdarg>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <setupapi.h>
extern "C" {
#include <hidsdi.h>
}
#pragma comment(lib, "ws2_32.lib")
#pragma comment(lib, "setupapi.lib")
#pragma comment(lib, "hid.lib")
using socket_t = SOCKET;
#define CLOSE_SOCKET closesocket
#define SOCKET_INVALID INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
using socket_t = int;
#define CLOSE_SOCKET close
#define SOCKET_INVALID -1
#endif

namespace iris {

static std::function<void(const std::string&)> g_rigctl_log_cb;

void rigctl_set_log_callback(std::function<void(const std::string&)> cb) {
    g_rigctl_log_cb = cb;
}

static void rigctl_log(const char* fmt, ...) {
    char buf[2048];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    printf("%s\n", buf);
    if (g_rigctl_log_cb) g_rigctl_log_cb(buf);
}

#ifdef _WIN32
struct WsaInit {
    WsaInit() {
        WSADATA wsa;
        WSAStartup(MAKEWORD(2, 2), &wsa);
    }
    ~WsaInit() { WSACleanup(); }
};
static WsaInit wsa_init;
#endif

RigCtl::~RigCtl() { disconnect(); }

bool RigCtl::connect(const std::string& host, int port) {
    disconnect();

    struct addrinfo hints = {}, *result = nullptr;
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;

    std::string port_str = std::to_string(port);
    if (getaddrinfo(host.c_str(), port_str.c_str(), &hints, &result) != 0)
        return false;

    socket_t s = socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (s == SOCKET_INVALID) {
        freeaddrinfo(result);
        return false;
    }

    if (::connect(s, result->ai_addr, (int)result->ai_addrlen) != 0) {
        CLOSE_SOCKET(s);
        freeaddrinfo(result);
        return false;
    }

    freeaddrinfo(result);
    sock_ = (int)s;

    // Set recv timeout to 5 seconds (rigctld CI-V commands can take a moment)
#ifdef _WIN32
    DWORD tv = 5000;
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, (const char*)&tv, sizeof(tv));
#else
    struct timeval tv = {5, 0};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif
    return true;
}

void RigCtl::disconnect() {
    if (sock_ >= 0) {
        CLOSE_SOCKET((socket_t)sock_);
        sock_ = -1;
    }
}

std::string RigCtl::command(const std::string& cmd) {
    if (sock_ < 0) {
        rigctl_log("[RIGCTL] command('%s') — socket not connected", cmd.c_str());
        return "";
    }

    std::string msg = cmd + "\n";
    int sent = send((socket_t)sock_, msg.c_str(), (int)msg.size(), 0);
    if (sent <= 0) {
        rigctl_log("[RIGCTL] send('%s') failed (err=%d)", cmd.c_str(),
#ifdef _WIN32
                   WSAGetLastError()
#else
                   errno
#endif
        );
        return "";
    }

    char buf[1024];
    int n = recv((socket_t)sock_, buf, sizeof(buf) - 1, 0);
    if (n <= 0) {
        rigctl_log("[RIGCTL] recv('%s') returned %d (err=%d)", cmd.c_str(), n,
#ifdef _WIN32
                   WSAGetLastError()
#else
                   errno
#endif
        );
        // Connection lost — close socket so we don't keep trying
        CLOSE_SOCKET((socket_t)sock_);
        sock_ = -1;
        return "";
    }
    buf[n] = '\0';

    // Strip trailing newline
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r'))
        buf[--n] = '\0';

    return std::string(buf);
}

bool RigCtl::set_ptt(bool tx) {
    std::string cmd_str = tx ? "T 1" : "T 0";
    std::string resp = command(cmd_str);
    if (resp.find("RPRT 0") != std::string::npos || resp == "0") {
        ptt_state_ = tx;
        printf("[RIGCTL] PTT %s OK\n", tx ? "ON" : "OFF");
        return true;
    }
    rigctl_log("[RIGCTL] PTT %s FAILED — response: \"%s\"", tx ? "ON" : "OFF", resp.c_str());
    return false;
}

bool RigCtl::get_ptt() const {
    return ptt_state_;
}

bool RigCtl::set_frequency(uint64_t freq_hz) {
    std::string resp = command("F " + std::to_string(freq_hz));
    return resp.find("RPRT 0") != std::string::npos || resp == "0";
}

uint64_t RigCtl::get_frequency() {
    std::string resp = command("f");
    try { return std::stoull(resp); } catch (...) { return 0; }
}

bool RigCtl::set_mode(const std::string& mode, int bandwidth) {
    std::string cmd = "M " + mode;
    if (bandwidth > 0) cmd += " " + std::to_string(bandwidth);
    std::string resp = command(cmd);
    return resp.find("RPRT 0") != std::string::npos || resp == "0";
}

// --- Auto-launch rigctld ---

#ifdef _WIN32
static HANDLE g_rigctld_process = nullptr;
static HANDLE g_rigctld_pipe = nullptr;  // Keep pipe open so rigctld doesn't crash on write

// Try to spawn rigctld.exe from the same directory as iris.exe
static void auto_launch_rigctld(int port, int model, const std::string& device) {
    if (model <= 0) return;  // 0 = user manages rigctld externally

    // Kill any stale rigctld from a previous crash
    system("taskkill /F /IM rigctld.exe >nul 2>&1");
    Sleep(200);

    // Find our own exe directory
    char exe_path[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
    std::string dir(exe_path);
    size_t slash = dir.find_last_of("\\/");
    if (slash != std::string::npos) dir = dir.substr(0, slash + 1);

    std::string rigctld_path = dir + "rigctld.exe";

    // Check if rigctld.exe exists alongside iris.exe
    DWORD attrs = GetFileAttributesA(rigctld_path.c_str());
    if (attrs == INVALID_FILE_ATTRIBUTES) {
        rigctl_log("[RIGCTLD] rigctld.exe not found at %s", rigctld_path.c_str());
        return;
    }

    // Build command line
    std::string cmdline = "\"" + rigctld_path + "\" -m " + std::to_string(model)
                        + " -t " + std::to_string(port);
    if (!device.empty())
        cmdline += " -r " + device;

    rigctl_log("[RIGCTLD] Launching: %s", cmdline.c_str());

    // Create pipe to capture rigctld stderr for error diagnostics
    SECURITY_ATTRIBUTES pipe_sa = {};
    pipe_sa.nLength = sizeof(pipe_sa);
    pipe_sa.bInheritHandle = TRUE;
    HANDLE hErrRead = nullptr, hErrWrite = nullptr;
    CreatePipe(&hErrRead, &hErrWrite, &pipe_sa, 0);
    SetHandleInformation(hErrRead, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
    si.wShowWindow = SW_HIDE;  // Hidden window
    si.hStdInput = GetStdHandle(STD_INPUT_HANDLE);
    si.hStdError = hErrWrite;
    si.hStdOutput = hErrWrite;

    PROCESS_INFORMATION pi = {};
    if (CreateProcessA(nullptr, (char*)cmdline.c_str(), nullptr, nullptr, TRUE,
                       CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
        CloseHandle(hErrWrite);  // Close write end so reads can detect EOF
        g_rigctld_process = pi.hProcess;
        CloseHandle(pi.hThread);
        // Give rigctld time to start listening
        Sleep(1000);
        // Check if rigctld is still running (it exits immediately on errors like bad COM port)
        DWORD exit_code = 0;
        if (GetExitCodeProcess(g_rigctld_process, &exit_code) && exit_code != STILL_ACTIVE) {
            rigctl_log("[RIGCTLD] ERROR: rigctld exited immediately (code %lu) — check COM port and radio", exit_code);
            // Read any error output from rigctld
            char errbuf[2048];
            DWORD bytes_read = 0;
            if (ReadFile(hErrRead, errbuf, sizeof(errbuf) - 1, &bytes_read, nullptr) && bytes_read > 0) {
                errbuf[bytes_read] = '\0';
                rigctl_log("[RIGCTLD] rigctld output: %s", errbuf);
            }
            CloseHandle(g_rigctld_process);
            g_rigctld_process = nullptr;
        } else {
            rigctl_log("[RIGCTLD] Started (PID %lu)", pi.dwProcessId);
        }
        g_rigctld_pipe = hErrRead;  // Keep open so rigctld doesn't crash on verbose writes
    } else {
        CloseHandle(hErrWrite);
        CloseHandle(hErrRead);
        rigctl_log("[RIGCTLD] Failed to launch (error %lu)", GetLastError());
    }
}

void rigctld_shutdown() {
    if (g_rigctld_process) {
        TerminateProcess(g_rigctld_process, 0);
        CloseHandle(g_rigctld_process);
        g_rigctld_process = nullptr;
        rigctl_log("[RIGCTLD] Stopped");
    }
    if (g_rigctld_pipe) {
        CloseHandle(g_rigctld_pipe);
        g_rigctld_pipe = nullptr;
    }
}
#else
static void auto_launch_rigctld(int, int, const std::string&) {}
void rigctld_shutdown() {}
#endif

// --- Radio model list from rigctld --list ---

static std::vector<RadioModel> g_radio_models;
static bool g_models_loaded = false;

const std::vector<RadioModel>& get_radio_models() {
    if (g_models_loaded) return g_radio_models;
    g_models_loaded = true;

    // Add "None / External" as first entry
    g_radio_models.push_back({0, "", "", "None (external rigctld)"});

#ifdef _WIN32
    // Find rigctld.exe alongside iris.exe
    char exe_path[MAX_PATH] = {};
    GetModuleFileNameA(nullptr, exe_path, MAX_PATH);
    std::string dir(exe_path);
    size_t slash = dir.find_last_of("\\/");
    if (slash != std::string::npos) dir = dir.substr(0, slash + 1);
    std::string rigctld_path = dir + "rigctld.exe";

    DWORD attrs = GetFileAttributesA(rigctld_path.c_str());
    if (attrs == INVALID_FILE_ATTRIBUTES) {
        rigctl_log("[RIGCTLD] rigctld.exe not found, radio list unavailable");
        return g_radio_models;
    }

    // Run rigctld --list and capture output
    std::string cmdline = "\"" + rigctld_path + "\" --list";

    SECURITY_ATTRIBUTES sa = {};
    sa.nLength = sizeof(sa);
    sa.bInheritHandle = TRUE;
    HANDLE hReadPipe, hWritePipe;
    CreatePipe(&hReadPipe, &hWritePipe, &sa, 0);
    SetHandleInformation(hReadPipe, HANDLE_FLAG_INHERIT, 0);

    STARTUPINFOA si = {};
    si.cb = sizeof(si);
    si.dwFlags = STARTF_USESTDHANDLES | STARTF_USESHOWWINDOW;
    si.hStdOutput = hWritePipe;
    si.hStdError = hWritePipe;
    si.wShowWindow = SW_HIDE;

    PROCESS_INFORMATION pi = {};
    if (!CreateProcessA(nullptr, (char*)cmdline.c_str(), nullptr, nullptr, TRUE,
                        CREATE_NO_WINDOW, nullptr, nullptr, &si, &pi)) {
        CloseHandle(hReadPipe);
        CloseHandle(hWritePipe);
        return g_radio_models;
    }
    CloseHandle(hWritePipe);

    // Read output
    std::string output;
    char buf[4096];
    DWORD n;
    while (ReadFile(hReadPipe, buf, sizeof(buf), &n, nullptr) && n > 0)
        output.append(buf, n);
    CloseHandle(hReadPipe);
    WaitForSingleObject(pi.hProcess, 3000);
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);

    // Parse lines: "  1234  Manufacturer   Model Name   Version   Status   Macro"
    // Fields are whitespace-separated but Manufacturer and Model can have spaces
    // Format: number, then mfg (one word), then model (rest until version pattern)
    size_t pos = 0;
    while (pos < output.size()) {
        size_t eol = output.find('\n', pos);
        if (eol == std::string::npos) eol = output.size();
        std::string line = output.substr(pos, eol - pos);
        pos = eol + 1;

        // Skip header and empty lines
        if (line.size() < 10) continue;

        // Parse model ID (leading spaces + number)
        size_t i = 0;
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
        if (i >= line.size() || !isdigit(line[i])) continue;

        int model_id = 0;
        while (i < line.size() && isdigit(line[i]))
            model_id = model_id * 10 + (line[i++] - '0');
        if (model_id <= 0) continue;

        // Skip spaces to manufacturer
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
        size_t mfg_start = i;
        while (i < line.size() && line[i] != ' ' && line[i] != '\t') i++;
        // Allow multi-word mfg: check if next word is still clearly mfg (not model-like)
        std::string mfg = line.substr(mfg_start, i - mfg_start);

        // Skip spaces to model name
        while (i < line.size() && (line[i] == ' ' || line[i] == '\t')) i++;
        size_t model_start = i;

        // Model name extends until we hit a version-like pattern (YYYYMMDD.N)
        // or "Stable"/"Beta"/"Untested"/"Alpha"
        size_t model_end = model_start;
        while (i < line.size()) {
            // Check for version pattern: 8 digits
            if (isdigit(line[i]) && i + 8 < line.size()) {
                bool is_ver = true;
                for (int d = 0; d < 8; d++)
                    if (!isdigit(line[i + d])) { is_ver = false; break; }
                if (is_ver) break;
            }
            // Check for status keywords
            if (line.substr(i, 6) == "Stable" || line.substr(i, 4) == "Beta" ||
                line.substr(i, 8) == "Untested" || line.substr(i, 5) == "Alpha")
                break;
            if (line[i] != ' ' && line[i] != '\t') model_end = i + 1;
            i++;
        }

        std::string model = line.substr(model_start, model_end - model_start);
        // Trim trailing spaces
        while (!model.empty() && (model.back() == ' ' || model.back() == '\t'))
            model.pop_back();

        if (model.empty()) continue;

        // Only include "Stable" status radios
        if (line.find("Stable") == std::string::npos) continue;

        RadioModel rm;
        rm.id = model_id;
        rm.manufacturer = mfg;
        rm.model = model;
        rm.display = mfg + " " + model;
        g_radio_models.push_back(rm);
    }

    rigctl_log("[RIGCTLD] Loaded %d radio models", (int)g_radio_models.size() - 1);
#endif
    return g_radio_models;
}

// --- Serial port enumeration ---

std::vector<std::string> enumerate_serial_ports() {
    std::vector<std::string> ports;
#ifdef _WIN32
    // Try opening COM1-COM256, keep the ones that exist
    for (int i = 1; i <= 256; i++) {
        char name[16];
        snprintf(name, sizeof(name), "COM%d", i);
        // Use CreateFile to test if port exists (faster than QueryDosDevice for small ranges)
        char path[32];
        snprintf(path, sizeof(path), "\\\\.\\COM%d", i);
        HANDLE h = CreateFileA(path, GENERIC_READ | GENERIC_WRITE, 0, nullptr,
                                OPEN_EXISTING, 0, nullptr);
        if (h != INVALID_HANDLE_VALUE) {
            CloseHandle(h);
            ports.push_back(name);
        } else if (GetLastError() == ERROR_ACCESS_DENIED) {
            // Port exists but is in use by another app
            ports.push_back(name);
        }
    }
#else
    // Linux/macOS: check common serial device patterns
    const char* patterns[] = {
        "/dev/ttyUSB", "/dev/ttyACM", "/dev/ttyS", "/dev/tty.usbserial",
        "/dev/tty.usbmodem", "/dev/tty.wchusbserial", nullptr
    };
    for (int p = 0; patterns[p]; p++) {
        for (int i = 0; i < 16; i++) {
            std::string path = std::string(patterns[p]) + std::to_string(i);
            if (access(path.c_str(), F_OK) == 0)
                ports.push_back(path);
        }
    }
#endif
    return ports;
}

// --- PTT implementations ---

RigctlPtt::RigctlPtt(const std::string& host, int port, int model,
                       const std::string& device) {
    auto_launch_rigctld(port, model, device);
    // Retry connection — rigctld may take a moment to initialize
    for (int attempt = 0; attempt < 5; attempt++) {
        if (rig_.connect(host, port)) {
            rigctl_log("[RIGCTL] Connected to rigctld at %s:%d (attempt %d)", host.c_str(), port, attempt + 1);
            return;
        }
#ifdef _WIN32
        Sleep(500);
#else
        usleep(500000);
#endif
    }
    rigctl_log("[RIGCTL] ERROR: Failed to connect to rigctld at %s:%d after 5 attempts", host.c_str(), port);
}

RigctlPtt::~RigctlPtt() {
    rig_.set_ptt(false);
    rig_.disconnect();
}

bool RigctlPtt::set_ptt(bool tx) { return rig_.set_ptt(tx); }
bool RigctlPtt::get_ptt() const { return rig_.get_ptt(); }

// --- Serial PTT ---

#ifdef _WIN32

SerialPtt::SerialPtt(const std::string& port, int baud) {
    std::string path = port;
    // Prefix \\.\  for COM ports above COM9 or always for safety
    if (path.find("\\\\.\\") != 0 && path.find("COM") == 0)
        path = "\\\\.\\" + path;

    handle_ = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE,
                           0, nullptr, OPEN_EXISTING, 0, nullptr);
    if (handle_ == INVALID_HANDLE_VALUE) {
        rigctl_log("[PTT-SERIAL] Failed to open %s", port.c_str());
        return;
    }

    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(handle_, &dcb);
    dcb.BaudRate = (DWORD)baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    SetCommState(handle_, &dcb);
    rigctl_log("[PTT-SERIAL] Opened %s at %d baud", port.c_str(), baud);
}

SerialPtt::~SerialPtt() {
    set_ptt(false);
    if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_);
}

bool SerialPtt::set_ptt(bool tx) {
    if (handle_ == INVALID_HANDLE_VALUE) return false;
    EscapeCommFunction(handle_, tx ? SETRTS : CLRRTS);
    EscapeCommFunction(handle_, tx ? SETDTR : CLRDTR);
    state_ = tx;
    return true;
}

#else // POSIX

SerialPtt::SerialPtt(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        rigctl_log("[PTT-SERIAL] Failed to open %s", port.c_str());
        return;
    }

    struct termios tio = {};
    tcgetattr(fd_, &tio);
    speed_t speed = B9600;
    if (baud == 19200) speed = B19200;
    else if (baud == 38400) speed = B38400;
    else if (baud == 57600) speed = B57600;
    else if (baud == 115200) speed = B115200;
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    tio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    tcsetattr(fd_, TCSANOW, &tio);

    // Start with RTS/DTR low
    int flags = 0;
    ioctl(fd_, TIOCMGET, &flags);
    flags &= ~(TIOCM_RTS | TIOCM_DTR);
    ioctl(fd_, TIOCMSET, &flags);
    rigctl_log("[PTT-SERIAL] Opened %s at %d baud", port.c_str(), baud);
}

SerialPtt::~SerialPtt() {
    set_ptt(false);
    if (fd_ >= 0) ::close(fd_);
}

bool SerialPtt::set_ptt(bool tx) {
    if (fd_ < 0) return false;
    int flags = 0;
    ioctl(fd_, TIOCMGET, &flags);
    if (tx) {
        flags |= (TIOCM_RTS | TIOCM_DTR);
    } else {
        flags &= ~(TIOCM_RTS | TIOCM_DTR);
    }
    ioctl(fd_, TIOCMSET, &flags);
    state_ = tx;
    return true;
}

#endif // Serial PTT

// --- CM108 HID PTT ---

#ifdef _WIN32

Cm108Ptt::Cm108Ptt(const std::string& device) {
    // Find first CM108-compatible HID device with GPIO
    GUID hid_guid;
    HidD_GetHidGuid(&hid_guid);
    HDEVINFO dev_info = SetupDiGetClassDevsA(&hid_guid, nullptr, nullptr,
                                              DIGCF_PRESENT | DIGCF_DEVICEINTERFACE);
    if (dev_info == INVALID_HANDLE_VALUE) {
        rigctl_log("[PTT-CM108] Failed to enumerate HID devices");
        return;
    }

    SP_DEVICE_INTERFACE_DATA iface_data = {};
    iface_data.cbSize = sizeof(iface_data);

    for (DWORD idx = 0; SetupDiEnumDeviceInterfaces(dev_info, nullptr, &hid_guid,
                                                      idx, &iface_data); idx++) {
        DWORD needed = 0;
        SetupDiGetDeviceInterfaceDetailA(dev_info, &iface_data, nullptr, 0, &needed, nullptr);
        if (needed == 0) continue;

        std::vector<uint8_t> buf(needed);
        auto* detail = (SP_DEVICE_INTERFACE_DETAIL_DATA_A*)buf.data();
        detail->cbSize = sizeof(SP_DEVICE_INTERFACE_DETAIL_DATA_A);

        if (!SetupDiGetDeviceInterfaceDetailA(dev_info, &iface_data, detail, needed, nullptr, nullptr))
            continue;

        // If user specified a device path, match it
        if (!device.empty() && std::string(detail->DevicePath).find(device) == std::string::npos)
            continue;

        HANDLE h = CreateFileA(detail->DevicePath, GENERIC_WRITE | GENERIC_READ,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                nullptr, OPEN_EXISTING, 0, nullptr);
        if (h == INVALID_HANDLE_VALUE) continue;

        // Check if this is a CM108-type device (VID 0x0d8c is C-Media)
        HIDD_ATTRIBUTES attrs = {};
        attrs.Size = sizeof(attrs);
        HidD_GetAttributes(h, &attrs);

        // Accept C-Media (0x0d8c) or any device if user specified a path
        if (attrs.VendorID == 0x0d8c || !device.empty()) {
            handle_ = h;
            rigctl_log("[PTT-CM108] Opened HID device: %s (VID=%04x PID=%04x)",
                   detail->DevicePath, attrs.VendorID, attrs.ProductID);
            break;
        }
        CloseHandle(h);
    }

    SetupDiDestroyDeviceInfoList(dev_info);
    if (handle_ == INVALID_HANDLE_VALUE)
        rigctl_log("[PTT-CM108] No CM108 HID device found");
}

Cm108Ptt::~Cm108Ptt() {
    set_ptt(false);
    if (handle_ != INVALID_HANDLE_VALUE) CloseHandle(handle_);
}

bool Cm108Ptt::set_gpio(bool on) {
    if (handle_ == INVALID_HANDLE_VALUE) return false;

    // CM108 HID output report: 5 bytes
    // Byte 0: Report ID (0x00)
    // Byte 1: GPIO direction mask (bit 3 = GPIO3 output)
    // Byte 2: GPIO value (bit 3 = GPIO3 state)
    // Bytes 3-4: unused
    uint8_t report[5] = {0x00, 0x08, (uint8_t)(on ? 0x08 : 0x00), 0x00, 0x00};
    DWORD written = 0;
    return WriteFile(handle_, report, sizeof(report), &written, nullptr) != 0;
}

bool Cm108Ptt::set_ptt(bool tx) {
    if (!set_gpio(tx)) return false;
    state_ = tx;
    return true;
}

#else // POSIX CM108

Cm108Ptt::Cm108Ptt(const std::string& device) {
    std::string path = device;
    if (path.empty()) path = "/dev/hidraw0";

    fd_ = ::open(path.c_str(), O_WRONLY);
    if (fd_ < 0) {
        rigctl_log("[PTT-CM108] Failed to open %s", path.c_str());
        return;
    }
    rigctl_log("[PTT-CM108] Opened %s", path.c_str());
}

Cm108Ptt::~Cm108Ptt() {
    set_ptt(false);
    if (fd_ >= 0) ::close(fd_);
}

bool Cm108Ptt::set_gpio(bool on) {
    if (fd_ < 0) return false;

    // CM108 HID output report: 5 bytes
    uint8_t report[5] = {0x00, 0x08, (uint8_t)(on ? 0x08 : 0x00), 0x00, 0x00};
    return ::write(fd_, report, sizeof(report)) == sizeof(report);
}

bool Cm108Ptt::set_ptt(bool tx) {
    if (!set_gpio(tx)) return false;
    state_ = tx;
    return true;
}

#endif // CM108 PTT

// Factory
std::unique_ptr<PttController> create_ptt(const std::string& method,
                                           const std::string& host, int port,
                                           const std::string& serial_port,
                                           int serial_baud,
                                           int rigctld_model,
                                           const std::string& rigctld_device) {
    if (method == "rigctl")
        return std::make_unique<RigctlPtt>(host, port, rigctld_model, rigctld_device);
    else if (method == "vox")
        return std::make_unique<VoxPtt>();
    else if (method == "serial")
        return std::make_unique<SerialPtt>(serial_port, serial_baud);
    else if (method == "cm108")
        return std::make_unique<Cm108Ptt>(serial_port);
    else
        return std::make_unique<NullPtt>();
}

} // namespace iris
