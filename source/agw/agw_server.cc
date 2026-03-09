#include "agw/agw_server.h"
#include "common/logging.h"
#include <cstring>
#include <cstdio>
#include <algorithm>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
using socket_t = SOCKET;
#define CLOSE_SOCKET closesocket
#define SOCKET_ERROR_VAL SOCKET_ERROR
#define SOCKET_INVALID INVALID_SOCKET
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
using socket_t = int;
#define CLOSE_SOCKET close
#define SOCKET_ERROR_VAL -1
#define SOCKET_INVALID -1
#endif

namespace iris {

// AGW frame type constants
static constexpr uint8_t AGW_VERSION      = 'R';  // Version query
static constexpr uint8_t AGW_PORT_INFO    = 'G';  // Port info query
static constexpr uint8_t AGW_PORT_CAP     = 'g';  // Port capabilities query
static constexpr uint8_t AGW_REGISTER     = 'X';  // Register callsign
static constexpr uint8_t AGW_UNREGISTER   = 'x';  // Unregister callsign
static constexpr uint8_t AGW_CONNECT      = 'C';  // Connect request
static constexpr uint8_t AGW_DATA         = 'D';  // Connected data
static constexpr uint8_t AGW_DISCONNECT   = 'd';  // Disconnect request
static constexpr uint8_t AGW_TX_DATA_UI   = 'M';  // Unproto (UI) data
static constexpr uint8_t AGW_OUTSTANDING  = 'Y';  // Outstanding frames query
static constexpr uint8_t AGW_HEARD        = 'H';  // Heard stations query
static constexpr uint8_t AGW_MON_ON       = 'k';  // Monitor on
static constexpr uint8_t AGW_RAW          = 'K';  // Raw AX.25 frame
static constexpr uint8_t AGW_PORT_OUTSTANDING = 'y'; // Outstanding frames (port-level)

static void set_call(char* dst, const std::string& call) {
    memset(dst, 0, 10);
    size_t n = call.size();
    if (n > 9) n = 9;
    memcpy(dst, call.c_str(), n);
}

static std::string get_call(const char* src) {
    char buf[11] = {};
    memcpy(buf, src, 10);
    // Trim trailing nulls and spaces
    int len = 9;
    while (len >= 0 && (buf[len] == '\0' || buf[len] == ' ')) len--;
    return std::string(buf, len + 1);
}

AgwServer::AgwServer() = default;

AgwServer::~AgwServer() { stop(); }

bool AgwServer::start(int port) {
    if (running_) return false;

    socket_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == SOCKET_INVALID) return false;

    int opt = 1;
    setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&opt, sizeof(opt));

    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons((uint16_t)port);

    if (bind(s, (struct sockaddr*)&addr, sizeof(addr)) == SOCKET_ERROR_VAL) {
        CLOSE_SOCKET(s);
        return false;
    }

    if (listen(s, 4) == SOCKET_ERROR_VAL) {
        CLOSE_SOCKET(s);
        return false;
    }

    listen_sock_ = (int)s;
    running_ = true;
    accept_thread_ = std::thread(&AgwServer::accept_thread, this);

    printf("[AGW] Server listening on port %d\n", port);
    return true;
}

void AgwServer::stop() {
    running_ = false;

    if (listen_sock_ >= 0) {
        CLOSE_SOCKET((socket_t)listen_sock_);
        listen_sock_ = -1;
    }

    if (accept_thread_.joinable())
        accept_thread_.join();

    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int s : client_sockets_)
        CLOSE_SOCKET((socket_t)s);
    client_sockets_.clear();
    connected_client_ = -1;
}

void AgwServer::accept_thread() {
    while (running_) {
        // Use select() with timeout so we can check running_ periodically
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET((socket_t)listen_sock_, &fds);
        struct timeval tv = {1, 0};  // 1 second timeout
        int sel = select((int)listen_sock_ + 1, &fds, nullptr, nullptr, &tv);
        if (sel <= 0) continue;  // timeout or error — recheck running_

        struct sockaddr_in client_addr = {};
        int addr_len = sizeof(client_addr);

        socket_t cs = accept((socket_t)listen_sock_,
                              (struct sockaddr*)&client_addr, (socklen_t*)&addr_len);

        if (cs == SOCKET_INVALID) continue;

        printf("[AGW] Client connected\n");

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_sockets_.push_back((int)cs);
        }

        std::thread(&AgwServer::client_thread, this, (int)cs).detach();
    }
}

void AgwServer::client_thread(int sock) {
    while (running_) {
        // Read 36-byte header
        AgwHeader hdr = {};
        int total = 0;
        while (total < (int)sizeof(hdr)) {
            int n = recv((socket_t)sock, (char*)&hdr + total,
                         (int)sizeof(hdr) - total, 0);
            if (n <= 0) goto done;
            total += n;
        }

        // Read data payload
        std::vector<uint8_t> data;
        if (hdr.data_len > 0) {
            if (hdr.data_len > 65536) goto done;  // Sanity limit
            data.resize(hdr.data_len);
            total = 0;
            while (total < (int)hdr.data_len) {
                int n = recv((socket_t)sock, (char*)data.data() + total,
                             (int)hdr.data_len - total, 0);
                if (n <= 0) goto done;
                total += n;
            }
        }

        printf("[AGW] Frame kind='%c' (0x%02x) from=%.10s to=%.10s data_len=%u\n",
               hdr.data_kind, hdr.data_kind, hdr.call_from, hdr.call_to, hdr.data_len);
        handle_frame(sock, hdr, data);
    }

done:
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_sockets_.erase(
            std::remove(client_sockets_.begin(), client_sockets_.end(), sock),
            client_sockets_.end());
        if (connected_client_ == sock)
            connected_client_ = -1;
    }

    CLOSE_SOCKET((socket_t)sock);
    printf("[AGW] Client disconnected\n");
}

void AgwServer::handle_frame(int sock, const AgwHeader& hdr, const std::vector<uint8_t>& data) {
    switch (hdr.data_kind) {
        case AGW_VERSION: {
            // Reply with version info
            uint8_t ver[8] = {};
            // Major.Minor = 2000.100 (SoundModem-compatible)
            uint16_t major = 2000, minor = 100;
            memcpy(ver, &major, 2);
            memcpy(ver + 4, &minor, 2);
            send_frame(sock, 'R', "", "", ver, 8);
            break;
        }

        case AGW_PORT_INFO: {
            // Reply: "1 port(s);Port1 Iris FM Modem;"
            std::string info = "1;Port1 Iris FM Modem;";
            send_frame(sock, 'G', "", "", (const uint8_t*)info.c_str(),
                       (uint32_t)info.size() + 1);
            break;
        }

        case AGW_PORT_CAP: {
            // Reply with port capabilities (baud rate)
            uint8_t cap[12] = {};
            uint32_t baud = 9600;
            memcpy(cap, &baud, 4);
            send_frame(sock, 'g', "", "", cap, sizeof(cap));
            break;
        }

        case AGW_REGISTER: {
            // Register callsign — always succeed
            std::string call = get_call(hdr.call_from);
            printf("[AGW] Register callsign: %s\n", call.c_str());
            // Reply with success (data_kind = 'X', data_len = 1, data = 0x01)
            uint8_t ok = 1;
            send_frame(sock, 'X', call, "", &ok, 1);
            break;
        }

        case AGW_UNREGISTER: {
            // Unregister callsign
            std::string call = get_call(hdr.call_from);
            printf("[AGW] Unregister callsign: %s\n", call.c_str());
            uint8_t ok = 1;
            send_frame(sock, 'x', call, "", &ok, 1);
            break;
        }

        case AGW_CONNECT: {
            // Connect request from client — track which client owns this connection
            std::string from = get_call(hdr.call_from);
            std::string to = get_call(hdr.call_to);
            printf("[AGW] Connect request: %s -> %s\n", from.c_str(), to.c_str());
            IRIS_LOG("AGW connect: %s -> %s (callback=%d)", from.c_str(), to.c_str(), connect_callback_ ? 1 : 0);
            connected_client_ = sock;
            if (connect_callback_) connect_callback_(to);
            break;
        }

        case AGW_DISCONNECT: {
            // Disconnect request — initiate ARQ disconnect.
            // Notification sent when ARQ state transitions to IDLE (via on_state_changed).
            std::string from = get_call(hdr.call_from);
            std::string to = get_call(hdr.call_to);
            printf("[AGW] Disconnect request: %s -> %s\n", from.c_str(), to.c_str());
            if (disconnect_callback_) disconnect_callback_();
            break;
        }

        case AGW_DATA: {
            // Connected-mode data from client — route to AX.25 session or ARQ
            if (!data.empty()) {
                if (connected_data_callback_)
                    connected_data_callback_(data.data(), data.size());
                else if (tx_callback_)
                    tx_callback_(data.data(), data.size());
            }
            break;
        }

        case AGW_TX_DATA_UI: {
            // Unproto UI data
            if (!data.empty() && tx_callback_) {
                tx_callback_(data.data(), data.size());
            }
            break;
        }

        case AGW_OUTSTANDING: {
            // Reply with actual pending frame count for flow control
            uint32_t count = 0;
            if (outstanding_callback_) count = (uint32_t)outstanding_callback_();
            send_frame(sock, 'Y', get_call(hdr.call_from), get_call(hdr.call_to),
                       (const uint8_t*)&count, 4);
            break;
        }

        case AGW_MON_ON: {
            // Monitor mode — acknowledge silently
            printf("[AGW] Monitor mode enabled\n");
            break;
        }

        case AGW_HEARD: {
            // Heard stations — reply with empty list
            send_frame(sock, 'H', "", "", nullptr, 0);
            break;
        }

        case AGW_RAW: {
            // Raw AX.25 frame
            if (!data.empty() && tx_callback_) {
                tx_callback_(data.data(), data.size());
            }
            break;
        }

        case AGW_PORT_OUTSTANDING: {
            // Port-level outstanding frames (Direwolf >= 1.2 / Pat fallback)
            uint32_t count = 0;
            if (outstanding_callback_) count = (uint32_t)outstanding_callback_();
            send_frame(sock, 'y', get_call(hdr.call_from), get_call(hdr.call_to),
                       (const uint8_t*)&count, 4);
            break;
        }

        default:
            printf("[AGW] Unknown frame type: 0x%02x ('%c')\n",
                   hdr.data_kind, hdr.data_kind >= 0x20 ? (char)hdr.data_kind : '?');
            break;
    }
}

void AgwServer::send_frame(int sock, uint8_t kind, const std::string& from,
                            const std::string& to, const uint8_t* data, uint32_t len) {
    AgwHeader hdr = {};
    hdr.port = 0;
    hdr.data_kind = kind;
    hdr.pid = 0xF0;
    set_call(hdr.call_from, from);
    set_call(hdr.call_to, to);
    hdr.data_len = len;

    send((socket_t)sock, (const char*)&hdr, sizeof(hdr), 0);
    if (data && len > 0) {
        send((socket_t)sock, (const char*)data, (int)len, 0);
    }
}

void AgwServer::broadcast_frame(uint8_t kind, const std::string& from,
                                 const std::string& to, const uint8_t* data, uint32_t len) {
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int s : client_sockets_) {
        send_frame(s, kind, from, to, data, len);
    }
}

void AgwServer::send_to_clients(const uint8_t* frame, size_t len,
                                 const std::string& from_call,
                                 const std::string& to_call) {
    // Route data to the connection-owning client if known, otherwise broadcast
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (connected_client_ >= 0) {
        send_frame(connected_client_, 'D', from_call, to_call, frame, (uint32_t)len);
    } else {
        for (int s : client_sockets_)
            send_frame(s, 'D', from_call, to_call, frame, (uint32_t)len);
    }
}

void AgwServer::notify_connected(const std::string& from_call, const std::string& to_call) {
    // Send 'C' frame to the connection-owning client (or broadcast)
    std::string msg = "*** CONNECTED With Station " + to_call + "\r";
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (connected_client_ >= 0) {
        send_frame(connected_client_, 'C', from_call, to_call,
                   (const uint8_t*)msg.c_str(), (uint32_t)msg.size() + 1);
    } else {
        for (int s : client_sockets_)
            send_frame(s, 'C', from_call, to_call,
                       (const uint8_t*)msg.c_str(), (uint32_t)msg.size() + 1);
    }
}

void AgwServer::notify_disconnected(const std::string& from_call, const std::string& to_call) {
    // Send 'd' frame to the connection-owning client, then clear tracking
    std::string msg = "*** DISCONNECTED From Station " + to_call + "\r";
    std::lock_guard<std::mutex> lock(clients_mutex_);
    if (connected_client_ >= 0) {
        send_frame(connected_client_, 'd', from_call, to_call,
                   (const uint8_t*)msg.c_str(), (uint32_t)msg.size() + 1);
        connected_client_ = -1;
    } else {
        for (int s : client_sockets_)
            send_frame(s, 'd', from_call, to_call,
                       (const uint8_t*)msg.c_str(), (uint32_t)msg.size() + 1);
    }
}

int AgwServer::client_count() const {
    return (int)client_sockets_.size();
}

} // namespace iris
