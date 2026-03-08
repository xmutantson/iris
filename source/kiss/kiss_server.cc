#include "kiss/kiss_server.h"
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

KissServer::KissServer() = default;

KissServer::~KissServer() { stop(); }

bool KissServer::start(int port) {
    if (running_) return false;

    socket_t s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == SOCKET_INVALID) return false;

    // Allow reuse
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
    accept_thread_ = std::thread(&KissServer::accept_thread, this);

    printf("[KISS] Server listening on port %d\n", port);
    return true;
}

void KissServer::stop() {
    running_ = false;

    if (listen_sock_ >= 0) {
        CLOSE_SOCKET((socket_t)listen_sock_);
        listen_sock_ = -1;
    }

    if (accept_thread_.joinable())
        accept_thread_.join();

    // Close all client sockets
    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int s : client_sockets_)
        CLOSE_SOCKET((socket_t)s);
    client_sockets_.clear();
}

void KissServer::accept_thread() {
    while (running_) {
        struct sockaddr_in client_addr = {};
        int addr_len = sizeof(client_addr);

        socket_t cs = accept((socket_t)listen_sock_,
                              (struct sockaddr*)&client_addr, (socklen_t*)&addr_len);

        if (cs == SOCKET_INVALID) continue;

        printf("[KISS] Client connected\n");

        {
            std::lock_guard<std::mutex> lock(clients_mutex_);
            client_sockets_.push_back((int)cs);
        }

        // Spawn client handler thread (detached)
        std::thread(&KissServer::client_thread, this, (int)cs).detach();
    }
}

void KissServer::client_thread(int sock) {
    KissCodec codec;

    codec.set_callback([this](uint8_t port, uint8_t cmd, const uint8_t* data, size_t len) {
        (void)port;
        if (cmd == KISS_CMD_DATA && tx_callback_) {
            tx_callback_(data, len);
        } else if (cmd >= KISS_CMD_TXDELAY && cmd <= KISS_CMD_DUPLEX && len >= 1 && param_callback_) {
            param_callback_(cmd, data[0]);
        }
    });

    uint8_t buf[2048];
    while (running_) {
        int n = recv((socket_t)sock, (char*)buf, sizeof(buf), 0);
        if (n <= 0) break;
        codec.feed(buf, n);
    }

    // Remove from client list
    {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        client_sockets_.erase(
            std::remove(client_sockets_.begin(), client_sockets_.end(), sock),
            client_sockets_.end());
    }

    CLOSE_SOCKET((socket_t)sock);
    printf("[KISS] Client disconnected\n");
}

void KissServer::send_to_clients(const uint8_t* frame, size_t len) {
    auto kiss_frame = KissCodec::encode(frame, len);

    std::lock_guard<std::mutex> lock(clients_mutex_);
    for (int s : client_sockets_) {
        send((socket_t)s, (const char*)kiss_frame.data(), (int)kiss_frame.size(), 0);
    }
}

int KissServer::client_count() const {
    // Not strictly thread-safe for reading size, but good enough for display
    return (int)client_sockets_.size();
}

} // namespace iris
