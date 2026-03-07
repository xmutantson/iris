#ifndef IRIS_AGW_SERVER_H
#define IRIS_AGW_SERVER_H

#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>
#include <cstdint>

namespace iris {

// AGW/PE (SoundModem-compatible) TCP server.
// Winlink Express and other packet apps connect via this protocol.
// Implements the subset needed for connected-mode data transfer.

// AGW frame header (36 bytes, little-endian)
#pragma pack(push, 1)
struct AgwHeader {
    uint8_t  port;           // Radio port (0)
    uint8_t  reserved1[3];
    uint8_t  data_kind;      // Frame type ('R','G','g','X','x','C','D','d','Y','k',...)
    uint8_t  reserved2;
    uint8_t  pid;            // Protocol ID (0xF0 for no L3)
    uint8_t  reserved3;
    char     call_from[10];  // Source callsign (null-padded)
    char     call_to[10];    // Destination callsign (null-padded)
    uint32_t data_len;       // Length of data following header
    uint32_t user;           // User field (reserved)
};
#pragma pack(pop)

static_assert(sizeof(AgwHeader) == 36, "AGW header must be 36 bytes");

class AgwServer {
public:
    AgwServer();
    ~AgwServer();

    // Called when a connected-mode data frame arrives from an AGW client
    using TxCallback = std::function<void(const uint8_t* frame, size_t len)>;
    void set_tx_callback(TxCallback cb) { tx_callback_ = cb; }

    // Called when AGW client requests ARQ connect/disconnect
    using ConnectCallback = std::function<void(const std::string& remote_call)>;
    using DisconnectCallback = std::function<void()>;
    using OutstandingCallback = std::function<int()>;
    void set_connect_callback(ConnectCallback cb) { connect_callback_ = cb; }
    void set_disconnect_callback(DisconnectCallback cb) { disconnect_callback_ = cb; }
    void set_outstanding_callback(OutstandingCallback cb) { outstanding_callback_ = cb; }

    // Start listening on given port
    bool start(int port = 8000);
    void stop();

    // Send a received frame to all connected AGW clients
    void send_to_clients(const uint8_t* frame, size_t len,
                         const std::string& from_call = "",
                         const std::string& to_call = "");

    // Notify clients of connection state changes
    void notify_connected(const std::string& from_call, const std::string& to_call);
    void notify_disconnected(const std::string& from_call, const std::string& to_call);

    // Set local callsign
    void set_callsign(const std::string& call) { callsign_ = call; }

    int client_count() const;
    bool is_running() const { return running_; }

private:
    void accept_thread();
    void client_thread(int sock);
    void handle_frame(int sock, const AgwHeader& hdr, const std::vector<uint8_t>& data);
    void send_frame(int sock, uint8_t kind, const std::string& from, const std::string& to,
                    const uint8_t* data = nullptr, uint32_t len = 0);
    void broadcast_frame(uint8_t kind, const std::string& from, const std::string& to,
                         const uint8_t* data = nullptr, uint32_t len = 0);

    int listen_sock_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    TxCallback tx_callback_;
    ConnectCallback connect_callback_;
    DisconnectCallback disconnect_callback_;
    OutstandingCallback outstanding_callback_;

    std::string callsign_ = "N0CALL";

    std::mutex clients_mutex_;
    std::vector<int> client_sockets_;
    int connected_client_ = -1;  // Socket of client that owns the ARQ connection
};

} // namespace iris

#endif
