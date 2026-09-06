#ifndef IRIS_RIGCTL_H
#define IRIS_RIGCTL_H

#include <string>
#include <cstdint>
#include <memory>
#include <vector>
#include <functional>

namespace iris {

// Global log callback for rigctl messages (set by main to forward to GUI)
void rigctl_set_log_callback(std::function<void(const std::string&)> cb);


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
    RigctlPtt(const std::string& host, int port, int model = 0,
              const std::string& device = "");
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

// Serial PTT (RTS/DTR toggling)
class SerialPtt : public PttController {
public:
    SerialPtt(const std::string& port, int baud = 9600);
    ~SerialPtt() override;
    bool set_ptt(bool tx) override;
    bool get_ptt() const override { return state_; }
private:
#ifdef _WIN32
    void* handle_ = (void*)-1;  // INVALID_HANDLE_VALUE
#else
    int fd_ = -1;
#endif
    bool state_ = false;
};

// CM108 HID PTT (USB GPIO on Digirig / Masters Communications interfaces)
class Cm108Ptt : public PttController {
public:
    Cm108Ptt(const std::string& device = "");
    ~Cm108Ptt() override;
    bool set_ptt(bool tx) override;
    bool get_ptt() const override { return state_; }
private:
    bool set_gpio(bool on);
#ifdef _WIN32
    void* handle_ = (void*)-1;
#else
    int fd_ = -1;
#endif
    bool state_ = false;
};

// No PTT (for testing/loopback)
class NullPtt : public PttController {
public:
    bool set_ptt(bool tx) override { state_ = tx; return true; }
    bool get_ptt() const override { return state_; }
private:
    bool state_ = false;
};

// Shutdown auto-launched rigctld (call at exit)
void rigctld_shutdown();

// Radio model entry from rigctld --list
struct RadioModel {
    int id;
    std::string manufacturer;
    std::string model;
    std::string display;  // "Manufacturer Model"
};

// Query rigctld for supported radio models (cached after first call)
const std::vector<RadioModel>& get_radio_models();

// Enumerate available serial/COM ports on the system
std::vector<std::string> enumerate_serial_ports();

// Factory
std::unique_ptr<PttController> create_ptt(const std::string& method,
                                           const std::string& host = "localhost",
                                           int port = 4532,
                                           const std::string& serial_port = "",
                                           int serial_baud = 9600,
                                           int rigctld_model = 0,
                                           const std::string& rigctld_device = "");

} // namespace iris

#endif
