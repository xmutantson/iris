#include "radio/rigctl.h"
#include <memory>
#include <cstring>
#include <cstdio>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
using socket_t = SOCKET;
#define CLOSE_SOCKET closesocket
#define SOCKET_INVALID INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <netdb.h>
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

// Factory
std::unique_ptr<PttController> create_ptt(const std::string& method,
                                           const std::string& host, int port) {
    if (method == "rigctl")
        return std::make_unique<RigctlPtt>(host, port);
    else if (method == "vox")
        return std::make_unique<VoxPtt>();
    else
        return std::make_unique<NullPtt>();
}

} // namespace iris
