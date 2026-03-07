#include "config/config.h"
#include "engine/modem.h"
#include "kiss/kiss_server.h"
#include "audio/audio.h"
#include "radio/rigctl.h"
#include "gui/gui.h"
#include "engine/speed_level.h"
#include <cstdio>
#include <cstring>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>

using namespace iris;

// Defined in tests.cc
extern int run_tests();

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
    g_running = false;
}

static void print_usage() {
    printf("Iris FM Data Modem v0.1\n");
    printf("Usage: iris [options]\n");
    printf("  --test           Run loopback tests\n");
    printf("  --config <file>  Load config from INI file (default: iris.ini)\n");
    printf("  --nogui          Disable GUI\n");
    printf("  --callsign <cs>  Set callsign\n");
    printf("  --mode <A|B|C>   Set channel mode\n");
    printf("  --port <n>       KISS TCP port (default: 8001)\n");
    printf("  --help           Show this help\n");
}

int main(int argc, char** argv) {
    // Parse command line
    std::string config_path = "iris.ini";
    bool run_test = false;
    bool use_gui = true;
    std::string cli_callsign;
    std::string cli_mode;
    int cli_port = -1;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--test") == 0) {
            run_test = true;
        } else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--nogui") == 0) {
            use_gui = false;
        } else if (strcmp(argv[i], "--callsign") == 0 && i + 1 < argc) {
            cli_callsign = argv[++i];
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            cli_mode = argv[++i];
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            cli_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage();
            return 0;
        }
    }

    // Run tests if requested
    if (run_test) {
        return run_tests();
    }

    // Load config
    IrisConfig config = load_config(config_path);

    // Apply CLI overrides
    if (!cli_callsign.empty()) config.callsign = cli_callsign;
    if (!cli_mode.empty()) config.mode = cli_mode;
    if (cli_port >= 0) config.kiss_port = cli_port;

    printf("Iris FM Data Modem v0.1\n");
    printf("  Callsign: %s\n", config.callsign.c_str());
    printf("  Mode: %s (%d baud)\n", config.mode.c_str(),
           mode_baud_rate(config.mode[0]));
    printf("  AX.25: %d baud\n", config.ax25_baud);
    printf("  KISS port: %d\n", config.kiss_port);

    // Signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize modem engine
    Modem modem;
    if (!modem.init(config)) {
        fprintf(stderr, "Failed to initialize modem\n");
        return 1;
    }

    // Initialize KISS server
    KissServer kiss;
    kiss.set_tx_callback([&modem](const uint8_t* frame, size_t len) {
        modem.queue_tx_frame(frame, len);
    });
    modem.set_rx_callback([&kiss](const uint8_t* frame, size_t len) {
        kiss.send_to_clients(frame, len);
    });

    if (!kiss.start(config.kiss_port)) {
        fprintf(stderr, "Failed to start KISS server on port %d\n", config.kiss_port);
        return 1;
    }
    printf("  KISS server listening on port %d\n", config.kiss_port);

    // Initialize PTT controller
    auto ptt = create_ptt(config.ptt_method, config.rigctl_host, config.rigctl_port);
    if (ptt) {
        printf("  PTT: %s\n", config.ptt_method.c_str());
    }

    // Initialize GUI
    IrisGui gui;
    auto audio_devices = enumerate_audio_devices();

    if (use_gui) {
        gui.init("Iris FM Data Modem");
        gui.set_callbacks({
            [&]() { gui.log("Connect requested"); },
            [&]() { gui.log("Disconnect requested"); },
            [&]() { modem.start_calibration(); gui.log("Calibration started"); },
            [&](const IrisConfig& new_cfg) {
                config = new_cfg;
                save_config(config_path, config);
                gui.log("Config saved");
            },
            [&]() { g_running = false; }
        });
    }

    printf("  Modem running. Press Ctrl+C to quit.\n\n");

    // Main loop
    auto last_status = std::chrono::steady_clock::now();
    while (g_running) {
        if (use_gui && gui.is_open()) {
            auto diag = modem.get_diagnostics();
            diag.kiss_clients = kiss.client_count();
            gui.update(diag, config, audio_devices);
            if (!gui.render_frame())
                g_running = false;
        }

        // Periodic status in nogui mode
        auto now = std::chrono::steady_clock::now();
        if (!use_gui && std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count() >= 5) {
            auto diag = modem.get_diagnostics();
            printf("[Status] KISS clients: %d, RX: %d, TX: %d, CRC errors: %d\n",
                   kiss.client_count(), diag.frames_rx, diag.frames_tx, diag.crc_errors);
            last_status = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    printf("\nShutting down...\n");
    gui.shutdown();
    kiss.stop();
    modem.shutdown();
    save_config(config_path, config);
    printf("Done.\n");
    return 0;
}
