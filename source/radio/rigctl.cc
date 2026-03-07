#include "radio/rigctl.h"
#include <memory>
#include <vector>
#include <cstring>
#include <cstdio>

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
    return true;
}

void RigCtl::disconnect() {
    if (sock_ >= 0) {
        CLOSE_SOCKET((socket_t)sock_);
        sock_ = -1;
    }
}

std::string RigCtl::command(const std::string& cmd) {
    if (sock_ < 0) return "";

    std::string msg = cmd + "\n";
    send((socket_t)sock_, msg.c_str(), (int)msg.size(), 0);

    char buf[1024];
    int n = recv((socket_t)sock_, buf, sizeof(buf) - 1, 0);
    if (n <= 0) return "";
    buf[n] = '\0';

    // Strip trailing newline
    while (n > 0 && (buf[n-1] == '\n' || buf[n-1] == '\r'))
        buf[--n] = '\0';

    return std::string(buf);
}

bool RigCtl::set_ptt(bool tx) {
    std::string resp = command(tx ? "T 1" : "T 0");
    if (resp.find("RPRT 0") != std::string::npos || resp == "0") {
        ptt_state_ = tx;
        return true;
    }
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

// --- PTT implementations ---

RigctlPtt::RigctlPtt(const std::string& host, int port) {
    rig_.connect(host, port);
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
        printf("[PTT-SERIAL] Failed to open %s\n", port.c_str());
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
    printf("[PTT-SERIAL] Opened %s at %d baud\n", port.c_str(), baud);
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
        printf("[PTT-SERIAL] Failed to open %s\n", port.c_str());
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
    printf("[PTT-SERIAL] Opened %s at %d baud\n", port.c_str(), baud);
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
        printf("[PTT-CM108] Failed to enumerate HID devices\n");
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
            printf("[PTT-CM108] Opened HID device: %s (VID=%04x PID=%04x)\n",
                   detail->DevicePath, attrs.VendorID, attrs.ProductID);
            break;
        }
        CloseHandle(h);
    }

    SetupDiDestroyDeviceInfoList(dev_info);
    if (handle_ == INVALID_HANDLE_VALUE)
        printf("[PTT-CM108] No CM108 HID device found\n");
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
        printf("[PTT-CM108] Failed to open %s\n", path.c_str());
        return;
    }
    printf("[PTT-CM108] Opened %s\n", path.c_str());
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
                                           int serial_baud) {
    if (method == "rigctl")
        return std::make_unique<RigctlPtt>(host, port);
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
