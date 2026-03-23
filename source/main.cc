#include "config/config.h"
#include "engine/modem.h"
#include "kiss/kiss_server.h"
#include "agw/agw_server.h"
#include "audio/audio.h"
#include "radio/rigctl.h"
#include "gui/gui.h"
#include "engine/speed_level.h"
#include "common/logging.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <csignal>
#include <ctime>
#include <atomic>
#include <thread>
#include <chrono>
#include <memory>
#ifdef _WIN32
#include <winsock2.h>
#include <shlobj.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <unistd.h>
#endif

extern "C" {
#include "monocypher.h"
}

// Compute blake2b hash of the running executable, return first 8 hex chars.
static std::string compute_exe_hash() {
    std::string path;
#ifdef _WIN32
    char buf[MAX_PATH];
    if (GetModuleFileNameA(NULL, buf, MAX_PATH) > 0)
        path = buf;
#elif defined(__linux__)
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n > 0) { buf[n] = 0; path = buf; }
#elif defined(__APPLE__)
    char buf[4096];
    uint32_t sz = sizeof(buf);
    if (_NSGetExecutablePath(buf, &sz) == 0) path = buf;
#endif
    if (path.empty()) return "????????";

    FILE* f = fopen(path.c_str(), "rb");
    if (!f) return "????????";

    crypto_blake2b_ctx ctx;
    crypto_blake2b_init(&ctx, 16);  // 128-bit hash (we only use first 4 bytes)
    uint8_t chunk[65536];
    size_t n;
    while ((n = fread(chunk, 1, sizeof(chunk), f)) > 0)
        crypto_blake2b_update(&ctx, chunk, n);
    fclose(f);

    uint8_t hash[16];
    crypto_blake2b_final(&ctx, hash);

    char hex[9];
    snprintf(hex, sizeof(hex), "%02x%02x%02x%02x", hash[0], hash[1], hash[2], hash[3]);
    return hex;
}

using namespace iris;

// Defined in tests.cc
extern int run_tests();
extern int run_benchmark();

static std::atomic<bool> g_running{true};
static std::atomic<bool> g_restart{false};

static void signal_handler(int) {
    g_running = false;
}

// Ensure rigctld is killed on any exit path (including red X on Windows)
static void exit_cleanup() {
    rigctld_shutdown();
}

#ifdef _WIN32
static BOOL WINAPI console_ctrl_handler(DWORD event) {
    // CTRL_CLOSE_EVENT is sent when user clicks the red X on the console window.
    // Windows gives us ~5 seconds to clean up before force-killing the process.
    rigctld_shutdown();
    g_running = false;
    return TRUE;
}
#endif

static Modulation parse_modulation(const char* s) {
    if (strcmp(s, "BPSK") == 0)   return Modulation::BPSK;
    if (strcmp(s, "QPSK") == 0)   return Modulation::QPSK;
    if (strcmp(s, "QAM16") == 0)  return Modulation::QAM16;
    if (strcmp(s, "QAM64") == 0)  return Modulation::QAM64;
    if (strcmp(s, "QAM256") == 0) return Modulation::QAM256;
    return Modulation::QAM256;
}

static void print_usage() {
    printf("Iris FM Data Modem v0.2\n");
    printf("Usage: iris [options]\n");
    printf("\nGeneral:\n");
    printf("  --test             Run loopback tests\n");
    printf("  --benchmark        Run OFDM FM channel SNR benchmark\n");
    printf("  --config <file>    Load config from INI file (default: iris.ini)\n");
    printf("  --help             Show this help\n");
    printf("  --log <file>       Log to file (tee stdout + file)\n");
    printf("\nStation:\n");
    printf("  --callsign <cs>    Set callsign\n");
    printf("  --ssid <n>         Set SSID (0-15)\n");
    printf("\nModem:\n");
    printf("  --mode <A|B|C>     Set channel mode\n");
    printf("  --ax25-only        Disable native PHY upgrade\n");
    printf("  --native-hail      Use native PHY for hailing (both sides must enable)\n");
    printf("  --fx25 [16|32|64]  Enable FX.25 FEC for AX.25 TX (default: 16 check bytes)\n");
    printf("  --ax25-baud <n>    AX.25 baud rate (1200 or 9600)\n");
    printf("  --max-mod <mod>    Max modulation (BPSK,QPSK,QAM16,QAM64,QAM256)\n");
    printf("  --speed-level <n>  Force speed level 0-7 (default: auto)\n");
    printf("  --tx-level <f>     TX level 0.0-1.0 (default: 0.5)\n");
    printf("  --rx-gain <f>      RX gain multiplier (default: 1.0)\n");
    printf("  --band-low <Hz>    Band low edge (default: 1200, must be >254 for CTCSS)\n");
    printf("  --band-high <Hz>   Band high edge (default: 2200)\n");
    printf("  --center-freq <Hz> Center frequency (default: auto = midpoint of band)\n");
    printf("\nAudio:\n");
    printf("  --noaudio          Disable audio I/O (for testing)\n");
    printf("  --loopback         Internal audio loopback (TX->RX, no hardware)\n");
    printf("  --noise <amp>      Add AWGN noise to loopback (amplitude, e.g. 0.01)\n");
    printf("  --fm-channel       Enable FM channel simulator in loopback\n");
    printf("  --fm-cfo <Hz>      FM channel frequency offset (default: 0)\n");
    printf("  --fm-multipath <ms> <gain>  FM multipath reflection (delay ms, gain 0-1)\n");
    printf("  --list-audio       List audio devices and exit\n");
    printf("  --capture <id>     Capture device ID (-1 = default)\n");
    printf("  --playback <id>    Playback device ID (-1 = default)\n");
    printf("  --sample-rate <n>  Audio sample rate (default: 48000)\n");
    printf("\nRadio/PTT:\n");
    printf("  --ptt <method>     PTT method: none, rigctl, vox, cm108, serial\n");
    printf("  --rigctl-host <h>  Rigctl host (default: localhost)\n");
    printf("  --rigctl-port <n>  Rigctl port (default: 4532)\n");
    printf("  --serial <port>    Serial port for PTT\n");
    printf("  --serial-baud <n>  Serial baud rate (default: 9600)\n");
    printf("  --ptt-pre <ms>     TXDelay: PTT pre-delay ms (default: 100)\n");
    printf("  --ptt-post <ms>    TXTail: PTT post-delay ms (default: 50)\n");
    printf("  --slottime <ms>    CSMA slot time ms (default: 100)\n");
    printf("  --persist <0-255>  p-persistence (default: 63 = 25%%)\n");
    printf("\nSecurity:\n");
    printf("  --encrypt <mode>   Encryption: off, strict, fast (default: off)\n");
    printf("  --psk <hex>        Pre-shared key in hex (up to 64 bytes)\n");
    printf("\nNetwork:\n");
    printf("  --port <n>         KISS TCP port (default: 8001)\n");
    printf("  --agw-port <n>     AGW/PE TCP port (default: 8000)\n");
    printf("\nGUI:\n");
    printf("  --nogui            Disable GUI\n");
    printf("  --no-constellation Disable constellation display\n");
    printf("  --no-waterfall     Disable waterfall display\n");
}

int main(int argc, char** argv) {
    // Force line-buffered stdout so redirected output is visible
    setvbuf(stdout, nullptr, _IOLBF, 0);

#ifdef _WIN32
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
#endif

    std::string config_path = "iris.ini";
    bool run_test = false;
    bool run_bench = false;
    bool use_gui = true;
    bool use_audio = true;
    bool use_loopback = false;
    bool list_audio = false;
    std::string cli_callsign;
    std::string cli_mode;
    std::string log_path;
    int cli_port = -1;
    int cli_agw_port = -1;

    // Extended CLI overrides (applied after config load)
    int cli_ssid = -1;
    bool cli_ax25_only = false;
    bool cli_native_hail = false;
    int cli_fx25_mode = -1;  // -1 = not set
    int cli_ax25_baud = -1;
    std::string cli_max_mod;
    float cli_tx_level = -1.0f;
    float cli_rx_gain = -1.0f;
    int cli_capture = -2;   // -2 = not set
    int cli_playback = -2;
    int cli_sample_rate = -1;
    std::string cli_ptt;
    std::string cli_rigctl_host;
    int cli_rigctl_port = -1;
    std::string cli_serial;
    int cli_serial_baud = -1;
    int cli_ptt_pre = -1;
    int cli_ptt_post = -1;
    int cli_slottime = -1;
    int cli_persist = -1;
    bool cli_no_constellation = false;
    bool cli_no_waterfall = false;
    std::string cli_encrypt;
    std::string cli_psk;
    float cli_band_low = -1.0f;
    float cli_band_high = -1.0f;
    float cli_center_freq = -1.0f;
    int cli_speed_level = -1;
    float cli_noise = 0.0f;
    float cli_bp_low = 0.0f, cli_bp_high = 0.0f;
    float cli_deemph_us = 0.0f;
    bool cli_fm_channel = false;
    float cli_fm_cfo = 0.0f;
    float cli_fm_mp_delay = 0.0f;
    float cli_fm_mp_gain = 0.0f;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--test") == 0) {
            run_test = true;
        } else if (strcmp(argv[i], "--benchmark") == 0) {
            run_bench = true;
        } else if (strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else if (strcmp(argv[i], "--nogui") == 0) {
            use_gui = false;
        } else if (strcmp(argv[i], "--noaudio") == 0) {
            use_audio = false;
        } else if (strcmp(argv[i], "--loopback") == 0) {
            use_loopback = true;
        } else if (strcmp(argv[i], "--list-audio") == 0) {
            list_audio = true;
        } else if (strcmp(argv[i], "--callsign") == 0 && i + 1 < argc) {
            cli_callsign = argv[++i];
        } else if (strcmp(argv[i], "--ssid") == 0 && i + 1 < argc) {
            cli_ssid = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            cli_mode = argv[++i];
        } else if (strcmp(argv[i], "--ax25-only") == 0) {
            cli_ax25_only = true;
        } else if (strcmp(argv[i], "--native-hail") == 0) {
            cli_native_hail = true;
        } else if (strcmp(argv[i], "--fx25") == 0) {
            // Optional argument: 16, 32, or 64. Default to 16 if none given.
            if (i + 1 < argc && argv[i + 1][0] != '-') {
                cli_fx25_mode = atoi(argv[++i]);
                if (cli_fx25_mode != 16 && cli_fx25_mode != 32 && cli_fx25_mode != 64) {
                    printf("Error: --fx25 must be 16, 32, or 64\n");
                    return 1;
                }
            } else {
                cli_fx25_mode = 16;
            }
        } else if (strcmp(argv[i], "--ax25-baud") == 0 && i + 1 < argc) {
            cli_ax25_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--max-mod") == 0 && i + 1 < argc) {
            cli_max_mod = argv[++i];
        } else if (strcmp(argv[i], "--tx-level") == 0 && i + 1 < argc) {
            cli_tx_level = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--rx-gain") == 0 && i + 1 < argc) {
            cli_rx_gain = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--band-low") == 0 && i + 1 < argc) {
            cli_band_low = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--band-high") == 0 && i + 1 < argc) {
            cli_band_high = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--center-freq") == 0 && i + 1 < argc) {
            cli_center_freq = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--capture") == 0 && i + 1 < argc) {
            cli_capture = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--playback") == 0 && i + 1 < argc) {
            cli_playback = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--sample-rate") == 0 && i + 1 < argc) {
            cli_sample_rate = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--ptt") == 0 && i + 1 < argc) {
            cli_ptt = argv[++i];
        } else if (strcmp(argv[i], "--rigctl-host") == 0 && i + 1 < argc) {
            cli_rigctl_host = argv[++i];
        } else if (strcmp(argv[i], "--rigctl-port") == 0 && i + 1 < argc) {
            cli_rigctl_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--serial") == 0 && i + 1 < argc) {
            cli_serial = argv[++i];
        } else if (strcmp(argv[i], "--serial-baud") == 0 && i + 1 < argc) {
            cli_serial_baud = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--ptt-pre") == 0 && i + 1 < argc) {
            cli_ptt_pre = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--ptt-post") == 0 && i + 1 < argc) {
            cli_ptt_post = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--slottime") == 0 && i + 1 < argc) {
            cli_slottime = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--persist") == 0 && i + 1 < argc) {
            cli_persist = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--port") == 0 && i + 1 < argc) {
            cli_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--agw-port") == 0 && i + 1 < argc) {
            cli_agw_port = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--log") == 0 && i + 1 < argc) {
            log_path = argv[++i];
        } else if (strcmp(argv[i], "--no-constellation") == 0) {
            cli_no_constellation = true;
        } else if (strcmp(argv[i], "--no-waterfall") == 0) {
            cli_no_waterfall = true;
        } else if (strcmp(argv[i], "--encrypt") == 0 && i + 1 < argc) {
            cli_encrypt = argv[++i];
        } else if (strcmp(argv[i], "--psk") == 0 && i + 1 < argc) {
            cli_psk = argv[++i];
        } else if (strcmp(argv[i], "--speed-level") == 0 && i + 1 < argc) {
            cli_speed_level = atoi(argv[++i]);
        } else if (strcmp(argv[i], "--noise") == 0 && i + 1 < argc) {
            cli_noise = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--bandpass") == 0 && i + 1 < argc) {
            // Parse "low-high" format, e.g. "300-3000"
            char* bp = argv[++i];
            char* dash = strchr(bp, '-');
            if (dash) {
                *dash = '\0';
                cli_bp_low = (float)atof(bp);
                cli_bp_high = (float)atof(dash + 1);
            }
        } else if (strcmp(argv[i], "--deemphasis") == 0 && i + 1 < argc) {
            cli_deemph_us = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--fm-channel") == 0) {
            cli_fm_channel = true;
        } else if (strcmp(argv[i], "--fm-cfo") == 0 && i + 1 < argc) {
            cli_fm_cfo = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--fm-multipath") == 0 && i + 2 < argc) {
            cli_fm_mp_delay = (float)atof(argv[++i]);
            cli_fm_mp_gain = (float)atof(argv[++i]);
        } else if (strcmp(argv[i], "--help") == 0 || strcmp(argv[i], "-h") == 0) {
            print_usage();
            return 0;
        }
    }

    if (run_test)
        return run_tests();
    if (run_bench)
        return run_benchmark();

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

    // Initialize logging (--log takes priority, then config log_enabled -> AppData)
    // Load config first so we can check log_enabled
    IrisConfig config = load_config(config_path);
    if (!cli_callsign.empty()) config.callsign = cli_callsign;
    if (cli_ssid >= 0) config.ssid = cli_ssid;
    if (!cli_mode.empty()) config.mode = cli_mode;
    if (cli_ax25_only) config.ax25_only = true;
    if (cli_native_hail) config.native_hail = true;
    if (cli_fx25_mode >= 0) config.fx25_mode = cli_fx25_mode;
    if (cli_ax25_baud > 0) config.ax25_baud = cli_ax25_baud;
    if (!cli_max_mod.empty()) config.max_modulation = parse_modulation(cli_max_mod.c_str());
    if (cli_tx_level >= 0.0f) config.tx_level = cli_tx_level;
    if (cli_rx_gain >= 0.0f) config.rx_gain = cli_rx_gain;
    if (cli_capture != -2) config.capture_device = cli_capture;
    if (cli_playback != -2) config.playback_device = cli_playback;
    if (cli_sample_rate > 0) config.sample_rate = cli_sample_rate;
    if (!cli_ptt.empty()) config.ptt_method = cli_ptt;
    if (!cli_rigctl_host.empty()) config.rigctl_host = cli_rigctl_host;
    if (cli_rigctl_port >= 0) config.rigctl_port = cli_rigctl_port;
    if (!cli_serial.empty()) config.serial_port = cli_serial;
    if (cli_serial_baud > 0) config.serial_baud = cli_serial_baud;
    if (cli_ptt_pre >= 0) config.ptt_pre_delay_ms = cli_ptt_pre;
    if (cli_ptt_post >= 0) config.ptt_post_delay_ms = cli_ptt_post;
    if (cli_slottime >= 0) config.slottime_ms = cli_slottime;
    if (cli_persist >= 0) config.persist = cli_persist;
    if (cli_port >= 0) config.kiss_port = cli_port;
    if (cli_agw_port >= 0) config.agw_port = cli_agw_port;
    if (cli_no_constellation) config.show_constellation = false;
    if (cli_no_waterfall) config.show_waterfall = false;
    if (cli_band_low >= 0.0f) config.band_low_hz = cli_band_low;
    if (cli_band_high >= 0.0f) config.band_high_hz = cli_band_high;
    if (cli_center_freq >= 0.0f) config.center_freq_hz = cli_center_freq;
    if (cli_bp_low > 0.0f && cli_bp_high > cli_bp_low) {
        config.sim_bandpass_low = cli_bp_low;
        config.sim_bandpass_high = cli_bp_high;
    }
    if (cli_deemph_us > 0.0f) config.sim_deemph_us = cli_deemph_us;
    if (!cli_encrypt.empty()) {
        if (cli_encrypt == "strict") config.encryption_mode = 1;
        else if (cli_encrypt == "fast") config.encryption_mode = 2;
        else config.encryption_mode = 0;
    }
    if (!cli_psk.empty()) config.psk_hex = cli_psk;

    // Set data directory for caches
    {
#ifdef _WIN32
        char appdata[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, appdata))) {
            char iris_dir[512];
            snprintf(iris_dir, sizeof(iris_dir), "%s\\Iris", appdata);
            _mkdir(iris_dir);
            config.data_dir = iris_dir;
        }
#else
        const char* home = getenv("HOME");
        if (home) {
            char iris_dir[512];
            snprintf(iris_dir, sizeof(iris_dir), "%s/.config/iris", home);
            mkdir(iris_dir, 0755);
            config.data_dir = iris_dir;
        }
#endif
    }

    // Set up logging
    if (!log_path.empty()) {
        // Explicit --log path
        if (!Logger::instance().open(log_path)) {
            fprintf(stderr, "Warning: failed to open log file: %s\n", log_path.c_str());
        }
    } else if (config.log_enabled) {
        // Auto-log to AppData/Iris/logs/YYYYMMDD_HHMMSS.log
        char logs_dir[600] = ".";
#ifdef _WIN32
        char appdata[MAX_PATH];
        if (SUCCEEDED(SHGetFolderPathA(NULL, CSIDL_APPDATA, NULL, 0, appdata))) {
            char iris_dir[512];
            snprintf(iris_dir, sizeof(iris_dir), "%s\\Iris", appdata);
            _mkdir(iris_dir);
            snprintf(logs_dir, sizeof(logs_dir), "%s\\logs", iris_dir);
            _mkdir(logs_dir);
        }
#else
        const char* home = getenv("HOME");
        if (home) {
            char iris_dir[512];
            snprintf(iris_dir, sizeof(iris_dir), "%s/.config/iris", home);
            mkdir(iris_dir, 0755);
            snprintf(logs_dir, sizeof(logs_dir), "%s/logs", iris_dir);
            mkdir(logs_dir, 0755);
        }
#endif
        time_t now = time(NULL);
        struct tm* t = localtime(&now);
        char auto_log[700];
        snprintf(auto_log, sizeof(auto_log), "%s/%04d%02d%02d_%02d%02d%02d.log",
                 logs_dir, t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                 t->tm_hour, t->tm_min, t->tm_sec);
        if (!Logger::instance().open(auto_log)) {
            fprintf(stderr, "Warning: failed to open log file: %s\n", auto_log);
        } else {
            printf("  Logging to: %s\n", auto_log);
        }
    }

    std::string exe_hash = compute_exe_hash();
    printf("Iris FM Data Modem v0.2  [%s]\n", exe_hash.c_str());
    IRIS_LOG("Iris FM Data Modem v0.2  build=%s", exe_hash.c_str());
    printf("  Callsign: %s\n", config.callsign.c_str());
    printf("  Mode: %s (%d baud)\n", config.mode.c_str(),
           mode_baud_rate(config.mode[0]));
    printf("  AX.25: %d baud\n", config.ax25_baud);
    printf("  KISS port: %d\n", config.kiss_port);
    printf("  TX level: %.2f\n", config.tx_level);

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    std::atexit(exit_cleanup);
#ifdef _WIN32
    SetConsoleCtrlHandler(console_ctrl_handler, TRUE);
#endif

    // Initialize modem engine
    Modem modem;
    if (!modem.init(config)) {
        fprintf(stderr, "Failed to initialize modem\n");
        return 1;
    }

    if (use_loopback)
        modem.set_loopback_mode(true);
    if (cli_noise > 0.0f)
        loopback_set_noise(cli_noise);
    if (cli_fm_channel) {
        // Standard NBFM channel: 530µs pre/de-emphasis, 300-3000 Hz bandpass
        loopback_set_fm_channel(530.0f, 300.0f, 3000.0f, cli_fm_cfo, 0.95f);
        printf("  FM channel sim: pre-emphasis=530µs, BP=300-3000 Hz, CFO=%.1f Hz\n", cli_fm_cfo);
        if (cli_fm_mp_delay > 0.0f) {
            loopback_set_fm_multipath(cli_fm_mp_delay, cli_fm_mp_gain);
            printf("  FM multipath: delay=%.2f ms, gain=%.2f\n", cli_fm_mp_delay, cli_fm_mp_gain);
        }
    }
    if (cli_speed_level >= 0)
        modem.force_speed_level(cli_speed_level);

    // Start listening for incoming ARQ connections
    modem.arq_listen();

    // Initialize KISS server
    KissServer kiss;
    kiss.set_tx_callback([&modem](const uint8_t* frame, size_t len) {
        modem.queue_tx_frame(frame, len);
    });
    kiss.set_client_callback([&modem](int count) {
        modem.set_kiss_passthrough(count > 0);
    });
    kiss.set_param_callback([&config, &modem](uint8_t cmd, uint8_t value) {
        switch (cmd) {
            case KISS_CMD_TXDELAY:
                config.ptt_pre_delay_ms = value * 10;
                printf("[KISS] TXDELAY = %d ms\n", config.ptt_pre_delay_ms);
                modem.set_txdelay_ms(config.ptt_pre_delay_ms);
                break;
            case KISS_CMD_P:
                config.persist = value;
                printf("[KISS] Persist = %d\n", config.persist);
                break;
            case KISS_CMD_SLOT:
                config.slottime_ms = value * 10;
                printf("[KISS] SlotTime = %d ms\n", config.slottime_ms);
                break;
            case KISS_CMD_TXTAIL:
                config.ptt_post_delay_ms = value * 10;
                printf("[KISS] TXTail = %d ms\n", config.ptt_post_delay_ms);
                break;
            default: break;
        }
    });

    if (!kiss.start(config.kiss_port)) {
        fprintf(stderr, "Failed to start KISS server on port %d\n", config.kiss_port);
        return 1;
    }
    printf("  KISS server listening on port %d\n", config.kiss_port);

    // Initialize AGW server (Winlink Express / SoundModem compatible)
    AgwServer agw;
    agw.set_callsign(config.callsign);
    agw.set_tx_callback([&modem](const uint8_t* frame, size_t len) {
        modem.queue_tx_frame(frame, len);
    });
    agw.set_connect_callback([&modem, &agw, &config](const std::string& remote) {
        // Guard: if session already active, re-send CONNECTED notification
        // instead of nuking the existing connection.  Winlink Express sometimes
        // retries AGW 'C' before processing the CONNECTED notification, which
        // would destroy the session and create an infinite SABM loop.
        if (modem.ax25_state() != Ax25SessionState::DISCONNECTED) {
            IRIS_LOG("AGW connect to %s — session already active, re-notifying",
                     remote.c_str());
            agw.notify_connected(config.callsign, remote);
            return;
        }
        // Always start with AX.25 SABM (works with any TNC).
        // If native_hail is enabled and AFSK fails after a few retries,
        // the modem escalates to native BPSK hailing automatically.
        modem.ax25_connect(remote);
    });
    agw.set_disconnect_callback([&modem]() {
        if (modem.ax25_state() != Ax25SessionState::DISCONNECTED)
            modem.ax25_disconnect();
        else
            modem.arq_disconnect();
    });
    agw.set_outstanding_callback([&modem]() -> int {
        if (modem.ax25_state() != Ax25SessionState::DISCONNECTED)
            return modem.ax25_pending_frames();
        return modem.arq_pending_frames();
    });
    agw.set_connected_data_callback([&modem](const uint8_t* data, size_t len) {
        modem.send_connected_data(data, len);
    });

    // Deliver RX frames to both KISS and AGW clients
    modem.set_rx_callback([&kiss, &agw, &modem, &config](const uint8_t* frame, size_t len) {
        kiss.send_to_clients(frame, len);
        agw.send_to_clients(frame, len, modem.arq_remote_callsign(), config.callsign);
    });

    // Notify AGW clients when ARQ state changes (CONNECTED / IDLE)
    modem.set_state_callback([&agw, &modem, &config](ArqState state, const std::string& remote) {
        if (state == ArqState::CONNECTED)
            agw.notify_connected(config.callsign, remote);
        else if (state == ArqState::IDLE) {
            agw.notify_disconnected(config.callsign, remote);
            // Re-enter LISTENING so we can accept new connections
            modem.arq_listen();
        }
    });

    // Notify AGW clients when AX.25 session state changes
    modem.set_ax25_state_callback([&agw, &modem, &config](Ax25SessionState state, const std::string& remote) {
        if (state == Ax25SessionState::CONNECTED)
            agw.notify_connected(config.callsign, remote);
        else if (state == Ax25SessionState::DISCONNECTED) {
            // Don't notify disconnect or relisten if ARQ is actively hailing
            // (native hail escalation in progress — AX.25 gave up but ARQ continues)
            if (modem.arq_state() != ArqState::HAILING &&
                modem.arq_state() != ArqState::CONNECTING) {
                agw.notify_disconnected(config.callsign, remote);
                modem.arq_listen();
            }
        }
    });

    if (!agw.start(config.agw_port)) {
        fprintf(stderr, "Warning: failed to start AGW server on port %d\n", config.agw_port);
    } else {
        printf("  AGW server listening on port %d\n", config.agw_port);
    }

    // Initialize audio
    std::unique_ptr<AudioCapture> capture;
    std::unique_ptr<AudioPlayback> playback;

    if (use_loopback) {
        loopback_reset();
        capture = create_loopback_capture();
        playback = create_loopback_playback();
    } else if (use_audio) {
        capture = create_capture();
        playback = create_playback();
    }

    if (capture && playback) {
        // Set up capture callback -> modem RX
        capture->set_callback([&modem](float* samples, int frame_count, int channels) {
            if (channels == 1) {
                modem.process_rx(samples, frame_count);
            } else {
                // Downmix to mono (pre-allocated to avoid heap alloc per callback)
                static std::vector<float> mono;
                if (mono.size() < (size_t)frame_count)
                    mono.resize(frame_count);
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
                // Generate mono, then duplicate to all channels (pre-allocated)
                static std::vector<float> mono;
                if (mono.size() < (size_t)frame_count)
                    mono.resize(frame_count);
                modem.process_tx(mono.data(), frame_count);
                for (int i = 0; i < frame_count; i++) {
                    for (int ch = 0; ch < channels; ch++)
                        samples[i * channels + ch] = mono[i];
                }
            }
        });

        // Let modem wait for audio pipeline to flush before releasing PTT
        auto* pb_ptr = playback.get();
        modem.set_tx_drain_hooks(
            [pb_ptr]() { pb_ptr->mark_drain(); },
            [pb_ptr]() { return pb_ptr->is_drained(); });

        // 4800 frames = 100ms at 48 kHz.  Larger buffer prevents WASAPI
        // DATA_DISCONTINUITY glitches under load (two-instance VB-Cable, etc.).
        bool cap_ok = capture->open(config.capture_device, config.sample_rate, 1, 4800);
        bool play_ok = playback->open(config.playback_device, config.sample_rate, 1, 4800);

        if (cap_ok && play_ok) {
            capture->start();
            playback->start();
            printf("  Audio: %s\n", use_loopback ? "loopback (TX->RX)" : "running (capture + playback)");
        } else {
            printf("  Audio: failed to open devices\n");
            capture.reset();
            playback.reset();
        }
    } else if (use_audio || use_loopback) {
        printf("  Audio: not available on this platform\n");
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
            [&]() { modem.start_probe(); gui.log("Standalone probe started"); },
            [&](const std::string& call) { modem.start_autotune(call); },
            [&](const IrisConfig& new_cfg) {
                config = new_cfg;
                save_config(config_path, config);
                gui.log("Config saved — restarting...");
                g_restart = true;
                g_running = false;
            },
            [&]() { g_running = false; }
        });
    }

    // Set up rigctl log callback to forward to GUI event log
    rigctl_set_log_callback([&gui](const std::string& msg) {
        gui.log(msg);
    });

    // Modem frame events → GUI event log
    modem.set_gui_log([&gui](const std::string& msg) {
        gui.log(msg);
    });

    // Modem packet log → GUI packet log
    modem.set_packet_log([&gui](bool is_tx, const std::string& proto, const std::string& desc) {
        gui.log_packet(is_tx, proto, desc);
    });

    // Initialize PTT (after GUI so log messages appear in event log)
    auto ptt = create_ptt(config.ptt_method, config.rigctl_host, config.rigctl_port,
                          config.serial_port, config.serial_baud,
                          config.rigctld_model, config.rigctld_device);
    if (ptt) {
        modem.set_ptt_controller(std::move(ptt));
        printf("  PTT: %s\n", config.ptt_method.c_str());
    } else {
        printf("  PTT: none\n");
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

        // Sync calibrated TX level from modem back to main config (for save on shutdown)
        if (modem.config().calibrated_tx_level > 0 &&
            modem.config().calibrated_tx_level != config.calibrated_tx_level) {
            config.tx_level = modem.config().tx_level;
            config.calibrated_tx_level = modem.config().calibrated_tx_level;
        }

        auto now = std::chrono::steady_clock::now();
        if (!use_gui && std::chrono::duration_cast<std::chrono::seconds>(now - last_status).count() >= 5) {
            auto diag = modem.get_diagnostics();
            const char* arq_str = "IDLE";
            if (diag.arq_state == ArqState::LISTENING) arq_str = "LISTENING";
            else if (diag.arq_state == ArqState::HAILING) arq_str = "HAILING";
            else if (diag.arq_state == ArqState::CONNECTING) arq_str = "CONNECTING";
            else if (diag.arq_state == ArqState::CONNECTED) arq_str = "CONNECTED";
            else if (diag.arq_state == ArqState::DISCONNECTING) arq_str = "DISCONNECTING";
            printf("[Status] KISS: %d, AGW: %d, RX: %d, TX: %d, CRC: %d, SNR: %.1f dB, Level: %s, ARQ: %s, DCD: %s (tone=%.0f, RMS=%.4f, thr=%.3f)\n",
                   kiss.client_count(), agw.client_count(), diag.frames_rx, diag.frames_tx,
                   diag.crc_errors, diag.snr_db, SPEED_LEVELS[diag.speed_level].name, arq_str,
                   diag.dcd_busy ? "BUSY" : "clear", diag.dcd_tone_energy, diag.rx_raw_rms, config.dcd_threshold);
            last_status = now;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    printf("\nShutting down...\n");
    if (capture) capture->stop();
    if (playback) playback->stop();
    gui.shutdown();
    agw.stop();
    kiss.stop();
    modem.shutdown();
    rigctld_shutdown();
    save_config(config_path, config);
    Logger::instance().close();

    if (g_restart) {
        printf("Restarting...\n");
#ifdef _WIN32
        // Re-launch ourselves with same arguments
        char exe_path[MAX_PATH];
        GetModuleFileNameA(NULL, exe_path, MAX_PATH);
        std::string cmd = std::string("\"") + exe_path + "\"";
        for (int i = 1; i < argc; i++) {
            cmd += " \"";
            cmd += argv[i];
            cmd += "\"";
        }
        STARTUPINFOA si = {};
        si.cb = sizeof(si);
        PROCESS_INFORMATION pi = {};
        CreateProcessA(exe_path, (LPSTR)cmd.c_str(), NULL, NULL, FALSE,
                       0, NULL, NULL, &si, &pi);
        CloseHandle(pi.hProcess);
        CloseHandle(pi.hThread);
#else
        execv(argv[0], argv);
        // If execv fails, just exit normally
#endif
    }

    printf("Done.\n");
    return 0;
}
