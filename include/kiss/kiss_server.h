#ifndef IRIS_KISS_SERVER_H
#define IRIS_KISS_SERVER_H

#include "kiss/kiss.h"
#include <string>
#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <mutex>

namespace iris {

// KISS TCP server — accepts connections on a TCP port and bridges
// KISS frames between applications and the modem.
class KissServer {
public:
    KissServer();
    ~KissServer();

    // Called when a KISS data frame arrives from a client
    using TxCallback = std::function<void(const uint8_t* frame, size_t len)>;
    void set_tx_callback(TxCallback cb) { tx_callback_ = cb; }

    // Start listening on the given port
    bool start(int port = 8001);
    void stop();

    // Send a received frame to all connected clients
    void send_to_clients(const uint8_t* frame, size_t len);

    int client_count() const;
    bool is_running() const { return running_; }

private:
    void accept_thread();
    void client_thread(int sock);

    int listen_sock_ = -1;
    std::atomic<bool> running_{false};
    std::thread accept_thread_;
    TxCallback tx_callback_;

    std::mutex clients_mutex_;
    std::vector<int> client_sockets_;
};

} // namespace iris

#endif
