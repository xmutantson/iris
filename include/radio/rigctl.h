#ifndef IRIS_RIGCTL_H
#define IRIS_RIGCTL_H

#include <string>
#include <cstdint>
#include <memory>

namespace iris {

// PTT control via hamlib rigctld TCP protocol
class RigCtl {
public:
    RigCtl() = default;
    ~RigCtl();

    // Connect to rigctld
    bool connect(const std::string& host = "localhost", int port = 4532);
    void disconnect();
    bool is_connected() const { return sock_ >= 0; }

    // PTT control
    bool set_ptt(bool tx);
    bool get_ptt() const;

    // Frequency (Hz)
    bool set_frequency(uint64_t freq_hz);
    uint64_t get_frequency();

    // Mode string (e.g., "FM", "USB", "LSB")
    bool set_mode(const std::string& mode, int bandwidth = 0);

private:
    // Send command, receive response
    std::string command(const std::string& cmd);

    int sock_ = -1;
    bool ptt_state_ = false;
};

// Abstract PTT interface
class PttController {
public:
    virtual ~PttController() = default;
    virtual bool set_ptt(bool tx) = 0;
    virtual bool get_ptt() const = 0;
};

// rigctl-based PTT
class RigctlPtt : public PttController {
public:
    RigctlPtt(const std::string& host, int port);
    ~RigctlPtt() override;
    bool set_ptt(bool tx) override;
    bool get_ptt() const override;
private:
    RigCtl rig_;
};

// VOX-based PTT (no radio control — TX audio triggers VOX)
class VoxPtt : public PttController {
public:
    bool set_ptt(bool) override { return true; }  // no-op, VOX handles it
    bool get_ptt() const override { return false; }
};

// No PTT (for testing/loopback)
class NullPtt : public PttController {
public:
    bool set_ptt(bool tx) override { state_ = tx; return true; }
    bool get_ptt() const override { return state_; }
private:
    bool state_ = false;
};

// Factory
std::unique_ptr<PttController> create_ptt(const std::string& method,
                                           const std::string& host = "localhost",
                                           int port = 4532);

} // namespace iris

#endif
