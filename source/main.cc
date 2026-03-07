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
    printf("Iris FM Data Modem v0.2\n");
    printf("Usage: iris [options]\n");
    printf("  --test           Run loopback tests\n");
    printf("  --config <file>  Load config from INI file (default: iris.ini)\n");
    printf("  --nogui          Disable GUI\n");
    printf("  --noaudio        Disable audio I/O (for testing)\n");
    printf("  --callsign <cs>  Set callsign\n");
    printf("  --mode <A|B|C>   Set channel mode\n");
    printf("  --port <n>       KISS TCP port (default: 8001)\n");
    printf("  --list-audio     List audio devices and exit\n");
    printf("  --help           Show this help\n");
}

int main(int argc, char** argv) {
    std::string config_path = "iris.ini";
    bool run_test = false;
    bool use_gui = true;
    bool use_audio = true;
    bool list_audio = false;
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
        } else if (strcmp(argv[i], "--noaudio") == 0) {
            use_audio = false;
        } else if (strcmp(argv[i], "--list-audio") == 0) {
            list_audio = true;
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

    if (run_test)
        return run_tests();

    if (list_audio) {
        auto devices = enumerate_audio_devices();
        printf("Audio devices:\n");
        for (const auto& d : devices) {
            printf("  [%d] %s (in=%d, out=%d, %d Hz)\n",
                   d.id, d.name.c_str(), d.max_input_channels,
                   d.max_output_channels, d.default_sample_rate);
        }
        if (devices.empty()) printf("  (none found)\n");
        return 0;
    }

    // Load config
    IrisConfig config = load_config(config_path);
    if (!cli_callsign.empty()) config.callsign = cli_callsign;
    if (!cli_mode.empty()) config.mode = cli_mode;
    if (cli_port >= 0) config.kiss_port = cli_port;

    printf("Iris FM Data Modem v0.2\n");
    printf("  Callsign: %s\n", config.callsign.c_str());
    printf("  Mode: %s (%d baud)\n", config.mode.c_str(),
           mode_baud_rate(config.mode[0]));
    printf("  AX.25: %d baud\n", config.ax25_baud);
    printf("  KISS port: %d\n", config.kiss_port);
    printf("  TX level: %.2f\n", config.tx_level);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Initialize modem engine
    Modem modem;
    if (!modem.init(config)) {
        fprintf(stderr, "Failed to initialize modem\n");
        return 1;
    }

    // Initialize PTT
    auto ptt = create_ptt(config.ptt_method, config.rigctl_host, config.rigctl_port);
    if (ptt) {
        modem.set_ptt_controller(std::move(ptt));
        printf("  PTT: %s\n", config.ptt_method.c_str());
    } else {
        printf("  PTT: none\n");
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

    // Initialize audio
    std::unique_ptr<AudioCapture> capture;
    std::unique_ptr<AudioPlayback> playback;

    if (use_audio) {
        capture = create_capture();
        playback = create_playback();

        if (capture && playback) {
            // Set up capture callback -> modem RX
            capture->set_callback([&modem](float* samples, int frame_count, int channels) {
                if (channels == 1) {
                    modem.process_rx(samples, frame_count);
                } else {
                    // Downmix to mono
                    std::vector<float> mono(frame_count);
                    for (int i = 0; i < frame_count; i++) {
                        float sum = 0;
                        for (int ch = 0; ch < channels; ch++)
                            sum += samples[i * channels + ch];
                        mono[i] = sum / channels;
                    }
                    modem.process_rx(mono.data(), frame_count);
                }
            });

            // Set up playback callback -> modem TX
            playback->set_callback([&modem](float* samples, int frame_count, int channels) {
                if (channels == 1) {
                    modem.process_tx(samples, frame_count);
                } else {
                    // Generate mono, then duplicate to all channels
                    std::vector<float> mono(frame_count);
                    modem.process_tx(mono.data(), frame_count);
                    for (int i = 0; i < frame_count; i++) {
                        for (int ch = 0; ch < channels; ch++)
                            samples[i * channels + ch] = mono[i];
                    }
                }
            });

            bool cap_ok = capture->open(config.capture_device, config.sample_rate, 1, 1024);
            bool play_ok = playback->open(config.playback_device, config.sample_rate, 1, 1024);

            if (cap_ok && play_ok) {
                capture->start();
                playback->start();
                printf("  Audio: running (capture + playback)\n");
            } else {
                printf("  Audio: failed to open devices\n");
                capture.reset();
                playback.reset();
            }
        } else {
            printf("  Audio: not available on this platform\n");
        }
    } else {
        printf("  Audio: disabled\n");
    }

    // Initialize GUI
    IrisGui gui;
    auto audio_devices = enumerate_audio_devices();

    if (use_gui) {
        gui.init("Iris FM Data Modem");
        gui.set_callbacks({
            [&]() {
                modem.arq_connect("REMOTE");
                gui.log("ARQ connect requested");
            },
            [&]() {
                modem.arq_disconnect();
                gui.log("ARQ disconnect requested");
            },
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

        // ARQ timeouts
        modem.tick();

        auto now = std::chrono::steady_clock::now();
        if (!use_gui && std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count() >= 5) {
            auto diag = modem.get_diagnostics();
            const char* arq_str = "IDLE";
            if (diag.arq_state == ArqState::CONNECTING) arq_str = "CONNECTING";
            else if (diag.arq_state == ArqState::CONNECTED) arq_str = "CONNECTED";
            else if (diag.arq_state == ArqState::DISCONNECTING) arq_str = "DISCONNECTING";
            printf("[Status] KISS: %d, RX: %d, TX: %d, CRC: %d, SNR: %.1f dB, Level: %s, ARQ: %s\n",
                   kiss.client_count(), diag.frames_rx, diag.frames_tx, diag.crc_errors,
                   diag.snr_db, SPEED_LEVELS[diag.speed_level].name, arq_str);
            last_status = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    printf("\nShutting down...\n");
    if (capture) capture->stop();
    if (playback) playback->stop();
    gui.shutdown();
    kiss.stop();
    modem.shutdown();
    save_config(config_path, config);
    printf("Done.\n");
    return 0;
}
