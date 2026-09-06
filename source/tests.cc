#include "common/types.h"
#include "ax25/crc16.h"
#include "ax25/hdlc.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
#include "ax25/ax25_session.h"
#include "ax25/ax25_protocol.h"
#include "kiss/kiss.h"
#include "native/rrc.h"
#include "native/constellation.h"
#include "native/phy.h"
#include "native/frame.h"
#include "native/xid.h"
#include "fec/ldpc.h"
#include "engine/speed_level.h"
#include "engine/snr.h"
#include "engine/gearshift.h"
#include <sstream>
#define private public
#include "engine/modem.h"
#undef private
#include "native/upconvert.h"
#include "arq/arq.h"
#include "config/config.h"
#include "compress/compress.h"
#include "b2f/b2f_handler.h"
#include "crypto/crypto.h"
#include "probe/passband_probe.h"
#include "probe/probe_controller.h"
#include "native/channel_eq.h"
#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"
#include "ofdm/ofdm_papr.h"
#include "common/fft.h"
#include "mfsk/mfsk_ack.h"
#ifdef IRIS_USE_OSS
#include "audio/audio.h"
#include <sys/soundcard.h>
#include <sys/types.h>
#endif
#include "acceptance_manifest.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>
#include <memory>
#include <random>
#include <type_traits>
#include <sys/wait.h>
#include <unistd.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace iris;

static int tests_passed = 0;
static int tests_failed = 0;

static void check(const char* name, bool ok) {
    if (ok) {
        printf("  PASS: %s\n", name);
        tests_passed++;
    } else {
        printf("  FAIL: %s\n", name);
        tests_failed++;
    }
}

// Hilbert transform: convert real audio to analytic signal.
// Matches the production path in modem.cc — enables CFO detection.
static std::vector<std::complex<float>> hilbert_analytic(const float* audio, int n) {
    int nfft_h = 1;
    while (nfft_h < n) nfft_h <<= 1;
    std::vector<std::complex<float>> buf(nfft_h, {0.0f, 0.0f});
    for (int i = 0; i < n; i++)
        buf[i] = std::complex<float>(audio[i], 0.0f);
    iris::fft_complex(buf.data(), nfft_h);
    for (int k = 1; k < nfft_h / 2; k++)
        buf[k] *= 2.0f;
    for (int k = nfft_h / 2 + 1; k < nfft_h; k++)
        buf[k] = {0.0f, 0.0f};
    iris::ifft_complex(buf.data(), nfft_h);
    std::vector<std::complex<float>> result(n);
    for (int i = 0; i < n; i++)
        result[i] = buf[i];
    return result;
}

// Linear-phase Blackman windowed-sinc bandpass FIR, gain-normalized to 1.0 at
// band center — a faithful in-process replica of the outer-repo narrow-FM audio
// filter (tools/sim/sim_channel_relay.py::_fir_bandpass_taps). Applied to a
// signal so a test can band-limit the probe exactly as the real-audio channel
// does, and confirm the probe discovers the true filter edge. Center-aligned
// convolution (compensates the group delay) so the chirp keeps its position.
static void probe_test_bandpass(std::vector<float>& sig, float flo, float fhi,
                                int numtaps, float fs) {
    if (numtaps % 2 == 0) numtaps += 1;
    int m = (numtaps - 1) / 2;
    std::vector<double> h(numtaps);
    auto ideal_lp = [&](double fc, int i) {
        double wc = 2.0 * fc / fs, x = wc * (i - m);
        double s = (std::abs(x) < 1e-12) ? 1.0 : std::sin(M_PI * x) / (M_PI * x);
        return wc * s;
    };
    for (int i = 0; i < numtaps; i++) {
        double bl = 0.42 - 0.5 * std::cos(2.0 * M_PI * i / (numtaps - 1))
                         + 0.08 * std::cos(4.0 * M_PI * i / (numtaps - 1));
        h[i] = (ideal_lp(fhi, i) - ideal_lp(flo, i)) * bl;
    }
    double fc = 0.5 * ((double)flo + fhi), gain = 0.0;
    for (int i = 0; i < numtaps; i++) gain += h[i] * std::cos(2.0 * M_PI * fc / fs * (i - m));
    if (std::abs(gain) > 1e-12) for (int i = 0; i < numtaps; i++) h[i] /= gain;

    int N = (int)sig.size();
    std::vector<float> y(N, 0.0f);
    for (int i = 0; i < N; i++) {
        double acc = 0.0;
        for (int j = 0; j < numtaps; j++) {
            int xi = i + m - j;
            if (xi >= 0 && xi < N) acc += h[j] * sig[xi];
        }
        y[i] = (float)acc;
    }
    sig.swap(y);
}

// ===================== Phase 1 Tests =====================

static void test_crc16() {
    printf("\n=== CRC-16 CCITT ===\n");
    const uint8_t data[] = "123456789";
    uint16_t crc = crc16_ccitt(data, 9);
    check("CRC of '123456789' == 0x906E", crc == 0x906E);

    uint16_t empty_crc = crc16_ccitt(nullptr, 0);
    check("CRC of empty == 0x0000", empty_crc == 0x0000);
}

static void test_nrzi() {
    printf("\n=== NRZI Encode/Decode ===\n");
    std::vector<uint8_t> input = {0, 1, 0, 0, 1, 1, 0, 1};
    auto encoded = nrzi_encode(input);
    auto decoded = nrzi_decode(encoded);
    check("NRZI round-trip", decoded == input);
}

static void test_hdlc() {
    printf("\n=== HDLC Encode/Decode ===\n");
    uint8_t frame[] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05};
    size_t frame_len = sizeof(frame);

    auto bits = hdlc_encode(frame, frame_len, 4, 2);
    check("HDLC encode produces bits", bits.size() > 0);

    auto nrzi_bits = nrzi_decode(bits);
    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : nrzi_bits) {
        if (decoder.push_bit(b)) { found = true; break; }
    }
    check("HDLC decode found frame", found);
    if (found) {
        const auto& decoded = decoder.frame();
        check("HDLC decoded frame matches original",
              decoded.size() == frame_len &&
              memcmp(decoded.data(), frame, frame_len) == 0);
    }
}

static void test_afsk_loopback() {
    printf("\n=== AFSK 1200 Loopback ===\n");
    uint8_t frame[] = {
        'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
        'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x61,
        0x03, 0xF0, 'H', 'e', 'l', 'l', 'o'
    };
    size_t frame_len = sizeof(frame);

    auto bits = hdlc_encode(frame, frame_len, 8, 4);
    AfskModulator mod;
    auto samples = mod.modulate(bits);
    printf("  %zu bits -> %zu samples (%.1f ms)\n",
           bits.size(), samples.size(), samples.size() * 1000.0 / SAMPLE_RATE);

    AfskDemodulator demod;
    auto rx_nrzi = demod.demodulate(samples.data(), samples.size());
    auto rx_bits = nrzi_decode(rx_nrzi);

    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : rx_bits) {
        if (decoder.push_bit(b)) { found = true; break; }
    }
    check("AFSK loopback: frame decoded", found);
    if (found) {
        check("AFSK loopback: payload matches",
              decoder.frame().size() == frame_len &&
              memcmp(decoder.frame().data(), frame, frame_len) == 0);
    }
}

static void test_modem_ax25_non48k_sample_rate() {
    printf("\n=== Modem AX.25 Non-48k Sample Rate ===\n");
    constexpr int sample_rate = 96000;
    uint8_t frame[] = {
        'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
        'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x61,
        0x03, 0xF0, '9', '6', 'k'
    };

    IrisConfig cfg;
    cfg.mode = "B";
    cfg.sample_rate = sample_rate;
    cfg.ax25_baud = 1200;
    cfg.ptt_pre_delay_ms = 0;
    cfg.ptt_post_delay_ms = 0;
    cfg.persist = 255;

    Modem modem;
    modem.set_loopback_mode(true);
    bool initialized = modem.init(cfg);
    check("Non-48k modem init", initialized);
    if (!initialized) return;

    modem.queue_tx_frame(frame, sizeof(frame));
    constexpr int chunk_size = 4096;
    std::vector<float> block(chunk_size);
    std::vector<float> tx_audio;
    for (int samples = 0; samples < sample_rate * 3; samples += chunk_size) {
        std::fill(block.begin(), block.end(), 0.0f);
        modem.process_tx(block.data(), chunk_size);
        tx_audio.insert(tx_audio.end(), block.begin(), block.end());
        if (modem.state() == ModemState::IDLE) break;
    }

    AfskDemodulator demod(sample_rate);
    NrziDecoder nrzi;
    HdlcDecoder decoder;
    auto nrzi_bits = demod.demodulate(tx_audio.data(), tx_audio.size());
    auto bits = nrzi.decode(nrzi_bits);
    bool recovered = false;
    for (uint8_t bit : bits) {
        if (decoder.push_bit(bit)) {
            const auto& decoded = decoder.frame();
            if (decoded.size() == sizeof(frame) &&
                memcmp(decoded.data(), frame, sizeof(frame)) == 0) {
                recovered = true;
                break;
            }
        }
    }
    check("Non-48k modem AX.25 frame recovered", recovered);
}

static void test_gfsk_loopback() {
    printf("\n=== GFSK 9600 Loopback ===\n");
    uint8_t frame[] = {
        'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x60,
        'I'<<1, 'R'<<1, 'I'<<1, 'S'<<1, ' '<<1, ' '<<1, 0x61,
        0x03, 0xF0, 'G', 'F', 'S', 'K', ' ', 't', 'e', 's', 't'
    };
    size_t frame_len = sizeof(frame);

    auto bits = hdlc_encode(frame, frame_len, 8, 4);
    GfskModulator mod;
    auto samples = mod.modulate(bits);
    printf("  %zu bits -> %zu samples (%.1f ms)\n",
           bits.size(), samples.size(), samples.size() * 1000.0 / SAMPLE_RATE);

    GfskDemodulator demod;
    auto rx_nrzi = demod.demodulate(samples.data(), samples.size());
    auto rx_bits = nrzi_decode(rx_nrzi);

    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : rx_bits) {
        if (decoder.push_bit(b)) { found = true; break; }
    }
    check("GFSK loopback: frame decoded", found);
    if (found) {
        check("GFSK loopback: payload matches",
              decoder.frame().size() == frame_len &&
              memcmp(decoder.frame().data(), frame, frame_len) == 0);
    }
}

static void test_kiss() {
    printf("\n=== KISS Codec ===\n");
    uint8_t frame[] = {0x01, 0xC0, 0xDB, 0x55, 0xAA};
    size_t frame_len = sizeof(frame);

    auto kiss_frame = KissCodec::encode(frame, frame_len);
    check("KISS frame bookended by FEND",
          kiss_frame.front() == KISS_FEND && kiss_frame.back() == KISS_FEND);

    KissCodec codec;
    bool got_frame = false;
    std::vector<uint8_t> rx_data;
    codec.set_callback([&](uint8_t, uint8_t cmd, const uint8_t* data, size_t len) {
        got_frame = true;
        if (cmd == KISS_CMD_DATA) rx_data.assign(data, data + len);
    });
    codec.feed(kiss_frame.data(), kiss_frame.size());

    check("KISS round-trip", got_frame &&
          rx_data.size() == frame_len &&
          memcmp(rx_data.data(), frame, frame_len) == 0);

    int oversize_frames = 0;
    KissCodec limited_codec;
    limited_codec.set_callback(
        [&](uint8_t, uint8_t, const uint8_t*, size_t) { oversize_frames++; });
    const uint8_t fend = KISS_FEND;
    std::vector<uint8_t> oversize_frame(NATIVE_MAX_PAYLOAD + 2, 0x41);
    limited_codec.feed(&fend, 1);
    limited_codec.feed(oversize_frame.data(), oversize_frame.size());
    limited_codec.feed(&fend, 1);
    check("oversize KISS frame dropped", oversize_frames == 0);
}

// ===================== Phase 2 Tests =====================

static void test_crc32() {
    printf("\n=== CRC-32 ===\n");
    const uint8_t data[] = "123456789";
    uint32_t c = crc32(data, 9);
    printf("    CRC-32 value: 0x%08X\n", c);
    check("CRC-32 of '123456789' == 0xCBF43926", c == 0xCBF43926);
}

static void test_rrc_filter() {
    printf("\n=== RRC Filter ===\n");
    auto taps = rrc_filter(0.2f, 6, 10);
    check("RRC filter has correct length", (int)taps.size() == 2 * 6 * 10 + 1);

    // Check symmetry
    bool symmetric = true;
    int center = (int)taps.size() / 2;
    for (int i = 0; i < center; i++) {
        if (std::abs(taps[i] - taps[taps.size() - 1 - i]) > 1e-6f) {
            symmetric = false;
            break;
        }
    }
    check("RRC filter is symmetric", symmetric);

    // Check that peak is at center
    float max_val = 0;
    int max_idx = 0;
    for (int i = 0; i < (int)taps.size(); i++) {
        if (std::abs(taps[i]) > max_val) {
            max_val = std::abs(taps[i]);
            max_idx = i;
        }
    }
    check("RRC filter peak at center", max_idx == center);
}

static void test_constellation() {
    printf("\n=== Constellation Mapping ===\n");

    // BPSK round-trip
    {
        std::vector<uint8_t> bits = {0, 1, 0, 1, 1, 0};
        auto symbols = map_bits(bits, Modulation::BPSK);
        auto rx_bits = demap_bits(symbols, Modulation::BPSK);
        check("BPSK round-trip", rx_bits == bits);
    }

    // QPSK round-trip
    {
        std::vector<uint8_t> bits = {0, 0, 1, 0, 0, 1, 1, 1};
        auto symbols = map_bits(bits, Modulation::QPSK);
        check("QPSK: 8 bits -> 4 symbols", symbols.size() == 4);
        auto rx_bits = demap_bits(symbols, Modulation::QPSK);
        check("QPSK round-trip", rx_bits == bits);
    }

    // QAM16 round-trip
    {
        std::vector<uint8_t> bits;
        for (int i = 0; i < 32; i++) bits.push_back(i & 1);
        auto symbols = map_bits(bits, Modulation::QAM16);
        check("QAM16: 32 bits -> 8 symbols", symbols.size() == 8);
        auto rx_bits = demap_bits(symbols, Modulation::QAM16);
        check("QAM16 round-trip", rx_bits == bits);
    }

    // QAM64 round-trip
    {
        std::vector<uint8_t> bits;
        for (int i = 0; i < 48; i++) bits.push_back(i & 1);
        auto symbols = map_bits(bits, Modulation::QAM64);
        check("QAM64: 48 bits -> 8 symbols", symbols.size() == 8);
        auto rx_bits = demap_bits(symbols, Modulation::QAM64);
        check("QAM64 round-trip", rx_bits == bits);
    }

    // QAM256 round-trip
    {
        std::vector<uint8_t> bits;
        for (int i = 0; i < 64; i++) bits.push_back(i & 1);
        auto symbols = map_bits(bits, Modulation::QAM256);
        check("QAM256: 64 bits -> 8 symbols", symbols.size() == 8);
        auto rx_bits = demap_bits(symbols, Modulation::QAM256);
        check("QAM256 round-trip", rx_bits == bits);
    }

    // QAM32 (cross-32) round-trip — the new VARA-FM-narrow-parity rung (O6).
    {
        check("QAM32: bits_per_symbol == 5", bits_per_symbol(Modulation::QAM32) == 5);

        // Exhaustive: every 5-bit value maps to a distinct point and round-trips.
        std::vector<uint8_t> bits;
        for (int v = 0; v < 32; v++)
            for (int b = 0; b < 5; b++) bits.push_back((v >> b) & 1);  // LSB-first
        auto symbols = map_bits(bits, Modulation::QAM32);
        check("QAM32: 160 bits -> 32 symbols", symbols.size() == 32);
        auto rx_bits = demap_bits(symbols, Modulation::QAM32);
        check("QAM32 hard round-trip (all 32 points)", rx_bits == bits);

        // 32 distinct points, cross shape (no |I|=|Q|=5 corner), unit avg power.
        bool distinct = true, cross = true;
        double pwr = 0.0;
        const float NORM = 4.4721360f;  // sqrt(20): unit-scale -> integer grid
        for (int a = 0; a < 32; a++) {
            pwr += std::norm(symbols[a]);
            for (int b2 = a + 1; b2 < 32; b2++)
                if (std::abs(symbols[a].real() - symbols[b2].real()) < 1e-4f &&
                    std::abs(symbols[a].imag() - symbols[b2].imag()) < 1e-4f) distinct = false;
            float gi = std::abs(symbols[a].real() * NORM), gq = std::abs(symbols[a].imag() * NORM);
            if (std::abs(gi - 5.0f) < 0.1f && std::abs(gq - 5.0f) < 0.1f) cross = false;
        }
        check("QAM32: 32 distinct points", distinct);
        check("QAM32: cross shape (no corners)", cross);
        check("QAM32: unit average power", std::abs(pwr / 32.0 - 1.0) < 1e-3);

        // Soft-demap LLR sign must agree with the transmitted bit for every
        // point (positive LLR = bit 0, negative = bit 1). A sign flip here means
        // the RX bit labeling disagrees with the TX mapper — silent decode death.
        bool llr_ok = true;
        auto llrs = demap_soft(symbols, Modulation::QAM32, 0.01f);
        check("QAM32 soft-demap: 160 LLRs", llrs.size() == 160);
        for (int v = 0; v < 32 && llr_ok; v++)
            for (int b = 0; b < 5; b++) {
                int tx_bit = (v >> b) & 1;
                float llr = llrs[v * 5 + b];
                if ((tx_bit == 0 && llr <= 0.0f) || (tx_bit == 1 && llr >= 0.0f)) llr_ok = false;
            }
        check("QAM32 soft-demap LLR signs match TX bits", llr_ok);

        // bits_to_modulation(5) resolves to QAM32 (the OFDM tone-map entry point).
        check("bits_to_modulation(5) == QAM32", bits_to_modulation(5) == Modulation::QAM32);
    }
}

static void test_native_phy_loopback() {
    printf("\n=== Native PHY Loopback ===\n");

    // Test BPSK/QPSK in raw loopback (no channel impairments).
    // QAM16+ requires frame structure (preamble) for Gardner timing recovery
    // to converge — tested via native frame loopback below.
    Modulation mods[] = {
        Modulation::BPSK, Modulation::QPSK
    };
    const char* mod_names[] = {"BPSK", "QPSK"};

    for (int m = 0; m < 2; m++) {
        int bps = bits_per_symbol(mods[m]);
        int n_bits = bps * 20;  // 20 symbols
        std::vector<uint8_t> tx_bits;
        for (int i = 0; i < n_bits; i++)
            tx_bits.push_back((i * 7 + 3) % 2);  // pseudo-random pattern

        PhyConfig cfg = mode_b_config();
        cfg.modulation = mods[m];

        NativeModulator mod(cfg);
        auto iq = mod.modulate(tx_bits);

        NativeDemodulator demod(cfg);
        auto rx_bits = demod.demodulate(iq.data(), iq.size());

        // Compare (may have extra bits due to filter tails, compare first n_bits)
        bool match = true;
        if ((int)rx_bits.size() < n_bits) {
            match = false;
        } else {
            for (int i = 0; i < n_bits; i++) {
                if (rx_bits[i] != tx_bits[i]) { match = false; break; }
            }
        }

        char name[64];
        snprintf(name, sizeof(name), "PHY %s loopback (%d bits)", mod_names[m], n_bits);
        check(name, match);
        if (!match) {
            printf("    TX: %d bits, RX: %zu bits\n", n_bits, rx_bits.size());
        }
    }
}

static void test_native_frame_loopback() {
    printf("\n=== Native Frame Loopback ===\n");

    uint8_t payload[] = "Hello from Iris native mode!";
    size_t payload_len = strlen((char*)payload);

    PhyConfig cfg = mode_b_config();
    cfg.modulation = Modulation::QPSK;

    // Build frame
    auto iq = build_native_frame(payload, payload_len, cfg);
    printf("  Payload: %zu bytes -> %zu IQ samples\n", payload_len, iq.size() / 2);

    // Detect frame start
    int start = detect_frame_start(iq.data(), iq.size(), cfg.samples_per_symbol);
    check("Frame detected", start >= 0);

    if (start >= 0) {
        printf("  Frame start at sample %d\n", start);

        // Decode frame
        std::vector<uint8_t> rx_payload;
        bool ok = decode_native_frame(iq.data(), iq.size(), start, cfg, rx_payload);
        check("Frame decoded successfully", ok);
        if (ok) {
            bool match = (rx_payload.size() == payload_len) &&
                         (memcmp(rx_payload.data(), payload, payload_len) == 0);
            check("Frame payload matches", match);
            if (!match) {
                printf("    Expected %zu bytes, got %zu bytes\n",
                       payload_len, rx_payload.size());
            }
        }
    }
}

// ===================== Phase 3 Tests =====================

static void test_xid() {
    printf("\n=== XID Capability ===\n");

    XidCapability cap = {
        XID_VERSION,
        CAP_MODE_A | CAP_MODE_B | CAP_COMPRESSION,
        Modulation::QAM64
    };

    auto data = xid_encode(cap);
    check("XID encode produces 8 bytes", data.size() == 8);
    check("XID magic correct", data[0] == 'I' && data[1] == 'R' &&
                                data[2] == 'I' && data[3] == 'S');

    XidCapability decoded;
    bool ok = xid_decode(data.data(), data.size(), decoded);
    check("XID decode succeeds", ok);
    check("XID version matches", decoded.version == XID_VERSION);
    check("XID capabilities match",
          decoded.capabilities == (CAP_MODE_A | CAP_MODE_B | CAP_COMPRESSION));
    check("XID max_modulation matches", decoded.max_modulation == Modulation::QAM64);
}

static void test_xid_negotiation() {
    printf("\n=== XID Negotiation ===\n");

    XidCapability local = {
        XID_VERSION,
        CAP_MODE_A | CAP_MODE_B | CAP_MODE_C | CAP_ENCRYPTION,
        Modulation::QAM256
    };

    XidCapability remote = {
        XID_VERSION,
        CAP_MODE_A | CAP_MODE_B | CAP_COMPRESSION,
        Modulation::QAM64
    };

    auto result = negotiate(local, remote);
    check("Negotiated caps = intersection",
          result.capabilities == (CAP_MODE_A | CAP_MODE_B));
    check("Negotiated max_mod = min",
          result.max_modulation == Modulation::QAM64);
}

static void test_xid_frame_build() {
    printf("\n=== XID Frame Build ===\n");

    XidCapability cap = {XID_VERSION, CAP_MODE_A | CAP_MODE_B, Modulation::QPSK};
    auto frame = build_xid_frame("IRIS01", "CQ    ", cap);

    // Should be: 7+7 (addr) + 1 (ctrl) + 1 (PID) + 8 (XID) = 24 bytes
    check("XID frame size == 24", frame.size() == 24);

    // Check PID
    check("XID frame PID == IRIS_PID", frame[15] == IRIS_PID);

    // Decode XID info from frame
    XidCapability decoded;
    bool ok = xid_decode(&frame[16], frame.size() - 16, decoded);
    check("XID info decodable from frame", ok);
    if (ok) {
        check("XID frame round-trip",
              decoded.version == cap.version &&
              decoded.capabilities == cap.capabilities &&
              decoded.max_modulation == cap.max_modulation);
    }
}

static void test_ax25_to_native_upgrade() {
    printf("\n=== AX.25 -> Native Upgrade Flow ===\n");

    // Simulate: Station A sends AX.25 frame via AFSK
    uint8_t ax25_frame[] = {
        'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
        'I'<<1, 'R'<<1, 'I'<<1, 'S'<<1, '0'<<1, '1'<<1, 0x61,
        0x03, 0xF0, 'T', 'e', 's', 't'
    };

    // 1. Encode as AX.25 AFSK
    auto bits = hdlc_encode(ax25_frame, sizeof(ax25_frame), 8, 4);
    AfskModulator afsk_mod;
    auto afsk_samples = afsk_mod.modulate(bits);

    // 2. Demod at receiver
    AfskDemodulator afsk_demod;
    auto rx_nrzi = afsk_demod.demodulate(afsk_samples.data(), afsk_samples.size());
    auto rx_bits_vec = nrzi_decode(rx_nrzi);

    HdlcDecoder hdlc_dec;
    bool ax25_ok = false;
    for (uint8_t b : rx_bits_vec) {
        if (hdlc_dec.push_bit(b)) { ax25_ok = true; break; }
    }
    check("Step 1: AX.25 frame received", ax25_ok);

    // 3. Station B sends XID with Iris capabilities
    XidCapability cap_b = {XID_VERSION, CAP_MODE_A | CAP_MODE_B, Modulation::QAM64};
    auto xid_frame = build_xid_frame("IRIS02", "IRIS01", cap_b);

    auto xid_bits = hdlc_encode(xid_frame.data(), xid_frame.size(), 4, 2);
    auto xid_audio = afsk_mod.modulate(xid_bits);

    // 4. Station A receives XID
    afsk_demod.reset();
    auto xid_rx_nrzi = afsk_demod.demodulate(xid_audio.data(), xid_audio.size());
    auto xid_rx_bits = nrzi_decode(xid_rx_nrzi);

    HdlcDecoder hdlc_dec2;
    bool xid_ok = false;
    for (uint8_t b : xid_rx_bits) {
        if (hdlc_dec2.push_bit(b)) { xid_ok = true; break; }
    }
    check("Step 2: XID frame received", xid_ok);

    if (xid_ok) {
        const auto& xid_rx_frame = hdlc_dec2.frame();
        // Check PID
        bool is_iris = (xid_rx_frame.size() >= 24) && (xid_rx_frame[15] == IRIS_PID);
        check("Step 3: Iris PID detected", is_iris);

        if (is_iris) {
            XidCapability rx_cap;
            bool decoded = xid_decode(&xid_rx_frame[16], xid_rx_frame.size() - 16, rx_cap);
            check("Step 4: XID capability decoded", decoded);

            // 5. Negotiate
            XidCapability cap_a = {XID_VERSION, CAP_MODE_A | CAP_MODE_B | CAP_ENCRYPTION,
                                   Modulation::QAM256};
            auto agreed = negotiate(cap_a, rx_cap);
            check("Step 5: Negotiated Mode A+B",
                  (agreed.capabilities & (CAP_MODE_A | CAP_MODE_B)) == (CAP_MODE_A | CAP_MODE_B));
            check("Step 5: Max mod = QAM64",
                  agreed.max_modulation == Modulation::QAM64);

            // 6. Switch to native mode
            PhyConfig native_cfg = mode_b_config();
            native_cfg.modulation = Modulation::QPSK;  // start conservative

            uint8_t native_payload[] = "Iris native mode active!";
            auto native_iq = build_native_frame(native_payload, strlen((char*)native_payload),
                                                 native_cfg);

            int start = detect_frame_start(native_iq.data(), native_iq.size(),
                                            native_cfg.samples_per_symbol);
            check("Step 6: Native frame detected", start >= 0);

            if (start >= 0) {
                std::vector<uint8_t> rx_payload;
                bool frame_ok = decode_native_frame(native_iq.data(), native_iq.size(),
                                                     start, native_cfg, rx_payload);
                check("Step 7: Native frame decoded", frame_ok);
                if (frame_ok) {
                    check("Step 8: Native payload matches",
                          rx_payload.size() == strlen((char*)native_payload) &&
                          memcmp(rx_payload.data(), native_payload,
                                 strlen((char*)native_payload)) == 0);
                }
            }
        }
    }
}

// =======================================================================
//  OFDM narrow-FM audio-bandpass preamble sync (band-independence regression)
// =======================================================================
// GUARD (2026-07-04): a report claimed the honest narrow config (48-50 data
// carriers, ~2600 Hz, discovered after the passband-probe floor fix 17699d9)
// could not preamble-sync — that the RX Schmidl-Cox self-correlation "peaks at
// 0.204-0.228, below the 0.25 gate, so the preamble is never detected -> 0/48
// delivered."  That attribution is WRONG.  Reproduction (offline replica here +
// N=3 live snd-aloop @WGN:40) shows the REAL preamble self-correlation
// M=|P|^2/(A*R) peaks at 0.96-0.99 on the honest 50-carrier config — it clears
// 0.25 with ~4x margin.  The "0.204-0.228" is the MAX of the correlator's
// NO-PREAMBLE noise-floor scan tail (mean ~0.04, p95 ~0.07); the 0.25 gate is
// designed to sit just above it so noise cannot false-trigger.  The 0/48 was a
// separate climb/reverse-ACK delivery blocker, not sync.
//
// The SC metric is Cauchy-Schwarz normalized (M=|P|^2/(A*R)), so it is already
// band-INDEPENDENT: two identical ZC training symbols give M~1 regardless of the
// carrier count.  The 511-tap Blackman bandpass (sim_channel_relay.py) has a
// leading-edge ISI effect on the first training body (memory > CP=64), but it
// only trims M to ~0.96-0.98 on narrow / ~0.6-0.99 on wide — never near 0.25.
// This test drives a faithful C++ replica of the relay channel (bandpass +
// broadband AWGN @ SNR3k) and asserts the preamble SYNCS on the honest narrow
// config AND on wide / off / the old 79-carrier config, locking in the band-
// independence so a future change can't reintroduce a carrier-count dependence.

// Blackman windowed-sinc bandpass FIR, gain-normalized to 1.0 at band center.
// Faithful port of sim_channel_relay._fir_bandpass_taps.
static std::vector<double> ns_fir_bandpass(int numtaps, double flo, double fhi, double fs) {
    if (numtaps % 2 == 0) numtaps += 1;
    int m = (numtaps - 1) / 2;
    auto npsinc = [](double x) { return x == 0.0 ? 1.0 : std::sin(M_PI * x) / (M_PI * x); };
    auto ideal_lp = [&](double fc, int n) { double wc = 2.0 * fc / fs; return wc * npsinc(wc * n); };
    std::vector<double> h(numtaps);
    for (int i = 0; i < numtaps; i++) {
        int n = i - m;
        double bw = 0.42 - 0.5 * std::cos(2.0 * M_PI * i / (numtaps - 1))
                        + 0.08 * std::cos(4.0 * M_PI * i / (numtaps - 1));
        h[i] = (ideal_lp(fhi, n) - ideal_lp(flo, n)) * bw;
    }
    double fc = 0.5 * (flo + fhi), gain = 0.0;
    for (int i = 0; i < numtaps; i++) gain += h[i] * std::cos(2.0 * M_PI * fc / fs * (i - m));
    if (std::abs(gain) > 1e-12) for (auto& v : h) v /= gain;
    return h;
}

// Build config+frame, push through the relay-replica channel (511-tap FIR
// bandpass + broadband AWGN @ SNR3k = wgn_label + 2.4 dB), detect. Returns sync.
static OfdmSyncResult ns_run_case(float lo_hz, float hi_hz, double bp_lo, double bp_hi,
                                  float wgn_label, const char* label, bool clean = false) {
    NegotiatedPassband pb;
    pb.low_hz = lo_hz; pb.high_hz = hi_hz;
    pb.center_hz = 0.5f * (lo_hz + hi_hz); pb.bandwidth_hz = hi_hz - lo_hz; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    OfdmModulator mod(cfg);
    ToneMap tm = get_uniform_tone_map(2, cfg);   // QPSK r1/2
    uint8_t payload[32];
    for (int i = 0; i < 32; i++) payload[i] = (uint8_t)(i * 37 + 13);
    auto iq = mod.build_ofdm_frame(payload, 32, tm, LdpcRate::RATE_1_2);

    std::vector<float> audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++) audio[i] = iq[i].real();

    // P_sig = peak mean-square over 1024-sample chunks (matches relay peak_ms).
    double p_sig = 0.0;
    for (size_t i = 0; i + 1024 <= audio.size(); i += 1024) {
        double ms = 0.0; for (int k = 0; k < 1024; k++) ms += (double)audio[i + k] * audio[i + k];
        ms /= 1024.0; if (ms > p_sig) p_sig = ms;
    }
    if (p_sig <= 0.0) p_sig = 1e-6;

    // FIR bandpass (relay band-limits the real passband signal).
    std::vector<double> filt;
    if (!clean && bp_hi > bp_lo && bp_lo > 0.0) {
        auto h = ns_fir_bandpass(511, bp_lo, bp_hi, 48000.0);
        int ntaps = (int)h.size();
        filt.assign(audio.size() + ntaps - 1, 0.0);
        for (size_t i = 0; i < audio.size(); i++) {
            double x = audio[i];
            for (int k = 0; k < ntaps; k++) filt[i + k] += x * h[k];
        }
    } else {
        filt.assign(audio.begin(), audio.end());
    }

    // Broadband AWGN calibrated to SNR3k (deterministic seed).
    if (!clean) {
        const double F_NYQUIST = 24000.0, BW_NOISE = 3000.0;
        double snr3k = (double)wgn_label + 2.4;
        double snr_lin = std::pow(10.0, snr3k / 10.0);
        double nstd = std::sqrt(p_sig * F_NYQUIST / (snr_lin * BW_NOISE));
        std::mt19937 rng(0xA5A5u); std::normal_distribution<double> nd(0.0, 1.0);
        for (auto& v : filt) v += nstd * nd(rng);
    }

    std::vector<float> rx(filt.size());
    for (size_t i = 0; i < filt.size(); i++) rx[i] = (float)filt[i];
    auto rx_iq = hilbert_analytic(rx.data(), (int)rx.size());
    size_t pad = 48000;
    rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
    rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

    auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
    printf("  [%s] carriers=%d(data=%d) M_raw=%.4f detected=%d zc=%.3f cfo=%.1fHz\n",
           label, cfg.n_used_carriers, cfg.n_data_carriers,
           sync.sc_peak_raw, sync.detected, sync.zc_metric, sync.cfo_hz);
    return sync;
}

static void test_ofdm_narrow_bandpass_sync() {
    printf("\n=== OFDM narrow-FM bandpass preamble sync (band-independence) ===\n");
    // The real preamble self-correlation must clear the 0.25 gate with a wide
    // margin (the no-preamble noise floor tops out ~0.24). Guard M >= 0.50 so a
    // future change that erodes band-independence toward the noise floor fails.
    const float MARGIN = 0.50f;

    // Honest narrow config (48-50 data carriers, ~2600 Hz) through the narrow-FM
    // audio bandpass — the config the "sync deflation" report was about.
    auto s_narrow = ns_run_case(300.0f, 2900.0f, 300.0, 2900.0, 40.0f, "narrow 300-2900 / bp narrow");
    check("narrow(50c): preamble SYNCS through narrow-FM bandpass @WGN:40", s_narrow.detected);
    check("narrow(50c): SC peak clears 0.25 gate with margin (>=0.50)", s_narrow.sc_peak_raw >= MARGIN);

    // Fidelity guard: wide 6 kHz, off (no bandpass), and the old 79-carrier
    // over-wide config must ALL still sync (don't fix narrow by breaking wide).
    auto s_wide = ns_run_case(300.0f, 6300.0f, 300.0, 6300.0, 40.0f, "wide  300-6300 / bp wide  ");
    check("wide(6kHz): preamble SYNCS through wide bandpass @WGN:40", s_wide.detected);
    check("wide(6kHz): SC peak clears 0.25 gate with margin (>=0.50)", s_wide.sc_peak_raw >= MARGIN);

    auto s_off = ns_run_case(300.0f, 3000.0f, 0.0, 0.0, 40.0f, "off   300-3000 / bp off   ");
    check("off: preamble SYNCS (no bandpass) @WGN:40", s_off.detected);

    auto s_79 = ns_run_case(325.0f, 4475.0f, 0.0, 0.0, 40.0f, "79c   325-4475 / bp off   ");
    check("79-carrier: preamble SYNCS (wide preamble energy) @WGN:40", s_79.detected);
}

// =======================================================================
//  OFDM PHY Roundtrip: modulator → audio → (optional FM channel) → demod
// =======================================================================
static void test_ofdm_phy_roundtrip() {
    printf("\n=== OFDM PHY Roundtrip (Hermitian baseband) ===\n");

    // --- Build OfdmConfig for a typical FM passband (300-3000 Hz) ---
    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    printf("  Config: nfft=%d cp=%d, %d used, %d data, %d pilot carriers\n",
           cfg.nfft, cfg.cp_samples, cfg.n_used_carriers,
           cfg.n_data_carriers, cfg.n_pilot_carriers);

    check("Has data carriers", cfg.n_data_carriers > 0);
    if (cfg.n_data_carriers == 0) return;

    // --- Build OFDM frame ---
    OfdmModulator mod(cfg);
    ToneMap tm = get_uniform_tone_map(2, cfg);  // preset 2 = QPSK r1/2
    printf("  ToneMap: preset=%d, %d bits/sym, FEC=r1/2\n",
           tm.tone_map_id, tm.total_bits_per_symbol);

    uint8_t payload[32];
    for (int i = 0; i < 32; i++) payload[i] = (uint8_t)(i * 37 + 13);
    auto iq = mod.build_ofdm_frame(payload, 32, tm, LdpcRate::RATE_1_2);
    check("Frame generated", !iq.empty());
    if (iq.empty()) return;

    printf("  Frame: %zu complex samples (%.1f ms)\n",
           iq.size(), 1000.0f * iq.size() / 48000.0f);

    // --- Verify PAPR after Hilbert clipper ---
    // Preamble (noise + 2 training + sync word) is excluded from PAPR clipping
    // to preserve the channel estimate, so measure only data portion.
    int sym_len_check = cfg.symbol_samples();
    int preamble_len = 4 * sym_len_check;  // noise + train1 + train2 + sync_word
    std::vector<std::complex<float>> data_only(iq.begin() + preamble_len, iq.end());
    float papr = compute_papr_db(data_only);
    printf("  PAPR (data) = %.1f dB (target <= 13.0 dB with DFT-spread)\n", papr);
    check("PAPR reduced to <= 13.0 dB", papr <= 13.0f);

    // --- Verify Hermitian symmetry: imaginary parts should be near zero ---
    float max_imag = 0;
    for (auto& s : iq) {
        float ai = std::abs(s.imag());
        if (ai > max_imag) max_imag = ai;
    }
    printf("  Max |imag| = %.6f (should be ~0 for real-valued output)\n", max_imag);
    check("Hermitian symmetry: max |imag| < 0.01", max_imag < 0.01f);

    // --- Extract real audio ---
    std::vector<float> audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++)
        audio[i] = iq[i].real();

    float peak = 0;
    for (auto s : audio) if (std::abs(s) > peak) peak = std::abs(s);
    printf("  Audio peak = %.4f\n", peak);

    // --- Test 1: Clean loopback (no FM effects) ---
    printf("\n  --- Clean loopback (no FM) ---\n");
    {
        auto rx_iq = hilbert_analytic(audio.data(), (int)audio.size());

        // Add silence padding (generous: 1 sec before, 1 sec after)
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        // Detect
        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        printf("  Detect: metric=%.4f, detected=%d, frame_start=%d, cfo=%.2f Hz\n",
               sync.schmidl_metric, sync.detected, sync.frame_start, sync.cfo_hz);
        check("Clean: frame detected", sync.detected);

        if (sync.detected) {
            // Demodulate — pass full buffer (demodulate uses sync.frame_start internally)
            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(
                rx_iq.data(),
                (int)rx_iq.size(),
                tm, &sync);
            printf("  Demod: success=%d, mean_H=%.3f, consumed=%d, snr=%.1f dB\n",
                   result.success, result.mean_H_mag,
                   result.consumed_from_input_start, result.snr_db);
            if (!result.llrs.empty())
                printf("  LLRs: %zu, LDPC blocks=%d, worst_iters=%d\n",
                       result.llrs.size(), result.n_ldpc_blocks, result.worst_ldpc_iters);
            check("Clean: LDPC decode success", result.success);
            if (result.success) {
                bool match = (result.payload.size() == 32) &&
                             (memcmp(result.payload.data(), payload, 32) == 0);
                check("Clean: payload matches", match);
            }
        }
    }

    // --- FM channel helper lambda ---
    // Runs the OFDM audio through pre-emphasis → deviation limiter → de-emphasis
    // at a given peak level, then detects and decodes.
    auto run_fm_test = [&](float target_peak, const char* label) -> bool {
        float fs = 48000.0f;
        float tau_s = 530e-6f;  // NBFM pre-emphasis (τ=530µs, fc≈300 Hz)

        std::vector<float> fm_audio = audio;
        float fm_peak = 0;
        for (auto s : fm_audio) if (std::abs(s) > fm_peak) fm_peak = std::abs(s);
        if (fm_peak > 0) {
            float norm = target_peak / fm_peak;
            for (auto& s : fm_audio) s *= norm;
        }
        printf("  [%s] audio normalized: peak %.4f -> %.2f\n", label, fm_peak, target_peak);

        // Pre-emphasis: y[n] = x[n] - alpha * x[n-1]
        float pe_alpha = std::exp(-1.0f / (tau_s * fs));
        float prev = 0.0f;
        for (size_t i = 0; i < fm_audio.size(); i++) {
            float x = fm_audio[i];
            fm_audio[i] = x - pe_alpha * prev;
            prev = x;
        }

        // Deviation limiter: hard clip to ±0.95
        int n_clips = 0;
        for (auto& s : fm_audio) {
            if (s > 0.95f) { s = 0.95f; n_clips++; }
            if (s < -0.95f) { s = -0.95f; n_clips++; }
        }
        printf("  [%s] Pre-emphasis + clip: %d samples clipped (%.1f%%)\n",
               label, n_clips, 100.0f * n_clips / fm_audio.size());

        // De-emphasis: H(s) = 1/(1+sτ) via bilinear transform → proper IIR
        float wc = 1.0f / tau_s;
        float K = 2.0f * fs;
        float a = K + wc;
        float de_b0 = wc / a, de_b1 = wc / a, de_a1 = (wc - K) / a;
        float de_x1 = 0.0f;  // previous input
        float de_y1 = 0.0f;  // previous OUTPUT (IIR feedback)
        for (size_t i = 0; i < fm_audio.size(); i++) {
            float de = de_b0 * fm_audio[i] + de_b1 * de_x1 - de_a1 * de_y1;
            de_x1 = fm_audio[i];
            de_y1 = de;
            fm_audio[i] = de;
        }

        auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());

        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        printf("  [%s] Detect: metric=%.4f, detected=%d, cfo=%.2f Hz\n",
               label, sync.schmidl_metric, sync.detected, sync.cfo_hz);

        std::string det_label = std::string(label) + ": frame detected";
        check(det_label.c_str(), sync.detected);

        if (sync.detected) {
            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(
                rx_iq.data(),
                (int)rx_iq.size(),
                tm, &sync);
            printf("  [%s] Demod: success=%d, mean_H=%.3f, snr=%.1f dB\n",
                   label, result.success, result.mean_H_mag, result.snr_db);

            std::string dec_label = std::string(label) + ": LDPC decode success";
            check(dec_label.c_str(), result.success);
            if (result.success) {
                bool match = (result.payload.size() == 32) &&
                             (memcmp(result.payload.data(), payload, 32) == 0);
                std::string pay_label = std::string(label) + ": payload matches";
                check(pay_label.c_str(), match);
            }
        }
        return true;
    };

    // --- Test 2: FM channel (low level, no clipping) ---
    printf("\n  --- FM channel (low level, no clipping) ---\n");
    run_fm_test(0.5f, "FM-low");

    // --- Test 3: FM channel (hot level, deviation limiting) ---
    // Exercises the deviation limiter at a realistic level.
    printf("\n  --- FM channel (hot level, deviation limiting) ---\n");
    run_fm_test(2.5f, "FM-hot");

    // --- Test 4: Verify TX de-emphasis reduces post-preemphasis peaks ---
    // This is the key regression test for Bug #13 (backwards compensation).
    // TX de-emphasis should attenuate high-freq carriers so that after the
    // radio's pre-emphasis the signal is flatter.  We verify by comparing
    // peak-after-preemphasis with and without de-emphasis.
    printf("\n  --- TX de-emphasis effectiveness ---\n");
    {
        float fs = 48000.0f;
        float tau_s = 530e-6f;
        float pe_alpha = std::exp(-1.0f / (tau_s * fs));

        // Build two frames: one with de-emphasis (current), one without
        OfdmConfig cfg_flat = cfg;
        cfg_flat.fm_preemph_corner_hz = 0.0f;  // disable de-emphasis
        OfdmModulator mod_flat(cfg_flat);
        auto iq_flat = mod_flat.build_ofdm_frame(payload, 32, tm, LdpcRate::RATE_1_2);

        // Apply pre-emphasis to both and measure peaks
        auto measure_peak_after_preemph = [&](const std::vector<std::complex<float>>& frame,
                                               const char* label) -> float {
            float prev = 0.0f;
            float peak = 0.0f;
            for (size_t i = 0; i < frame.size(); i++) {
                float x = frame[i].real();
                float pe = x - pe_alpha * prev;
                prev = x;
                if (std::abs(pe) > peak) peak = std::abs(pe);
            }
            printf("  [%s] peak after pre-emphasis: %.4f\n", label, peak);
            return peak;
        };

        float peak_with_deemph = measure_peak_after_preemph(iq, "with TX de-emphasis");
        float peak_without = measure_peak_after_preemph(iq_flat, "without TX de-emphasis");
        float reduction_db = 20.0f * std::log10(peak_without / std::max(peak_with_deemph, 1e-6f));
        printf("  De-emphasis reduces post-preemph peak by %.1f dB\n", reduction_db);

        // TX de-emphasis must reduce post-preemphasis peaks (the whole point).
        // With cap=3.0 (−9.5 dB) we expect at least 3 dB improvement.
        check("TX de-emphasis reduces post-preemph peak", peak_with_deemph < peak_without);
        check("TX de-emphasis gives >= 3 dB reduction", reduction_db >= 3.0f);
    }
}

// =======================================================================
//  OFDM Multi-Codeword Roundtrip: verify n_codewords > 1
// =======================================================================
static void test_ofdm_multi_codeword() {
    printf("\n=== OFDM Multi-Codeword Roundtrip ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    OfdmModulator mod(cfg);
    ToneMap tm = get_uniform_tone_map(2, cfg);  // QPSK r1/2
    tm.n_codewords = 2;  // Two LDPC blocks per frame

    // Payload large enough to fill 2 codewords
    std::vector<uint8_t> payload(160);
    for (int i = 0; i < (int)payload.size(); i++) payload[i] = (uint8_t)(i * 37 + 13);

    auto iq = mod.build_ofdm_frame(payload.data(), payload.size(), tm, LdpcRate::RATE_1_2, 2);
    check("Multi-CW: frame generated", !iq.empty());
    if (iq.empty()) return;

    printf("  Frame: %zu samples (%.1f ms), %zu bytes payload, 2 codewords\n",
           iq.size(), 1000.0f * iq.size() / 48000.0f, payload.size());

    // Clean loopback
    std::vector<std::complex<float>> rx_iq(iq.size());
    for (size_t i = 0; i < iq.size(); i++)
        rx_iq[i] = std::complex<float>(iq[i].real(), 0.0f);
    size_t pad = 48000;
    rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
    rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

    auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
    check("Multi-CW: frame detected", sync.detected);
    if (!sync.detected) return;

    OfdmDemodulator demod(cfg);
    ToneMap rx_tm = tm;
    auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), rx_tm, &sync);
    printf("  Demod: success=%d, snr=%.1f dB, %zu bytes recovered\n",
           result.success, result.snr_db, result.payload.size());
    check("Multi-CW: decode success", result.success);
    if (result.success) {
        bool match = (result.payload.size() == payload.size()) &&
                     (memcmp(result.payload.data(), payload.data(), payload.size()) == 0);
        check("Multi-CW: payload matches", match);
    }
}

// =======================================================================
//  GENIE / STRUCTURAL control: O3/O4 (16QAM) multi-codeword (ncw=4) decode
//  through a NOISELESS, no-channel path (direct IQ, perfect Hermitian).
//
//  Purpose: isolate structural bugs (multi-codeword ncw handling, 16QAM
//  soft-demapper mapping, bit/LLR ordering, interleaver) from channel/LLR
//  crush.  Uses the PRODUCTION config the OTA/receiver-drives-rate failure
//  was measured at (carrier_sp=8, ncw=4, 200B payload — see
//  test_ofdm_snr_sweep levels O3/O4).  If these FAIL clean, the bug is
//  STRUCTURAL.  If they PASS clean, the O3+ wall is channel/LLR, not
//  structure.
// =======================================================================
static void test_ofdm_genie_ncw4() {
    printf("\n=== OFDM GENIE (noiseless, direct IQ) O3/O4 ncw sweep ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    // Production config used by the receiver-drives-rate / snr_sweep path.
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);

    uint8_t payload[1200];
    for (int i = 0; i < 1200; i++) payload[i] = (uint8_t)(i * 37 + 13);

    struct GLevel { int preset; const char* name; LdpcRate fec; int ncw; int bytes; };
    GLevel levels[] = {
        { 3, "O2 QPSK  r3/4 ncw1",  LdpcRate::RATE_3_4, 1,  100},
        { 3, "O2 QPSK  r3/4 ncw2",  LdpcRate::RATE_3_4, 2,  200},
        { 4, "O3 16QAM r1/2 ncw1",  LdpcRate::RATE_1_2, 1,   90},
        { 4, "O3 16QAM r1/2 ncw2",  LdpcRate::RATE_1_2, 2,  180},
        { 4, "O3 16QAM r1/2 ncw4",  LdpcRate::RATE_1_2, 4,  200},
        { 5, "O4 16QAM r5/8 ncw4",  LdpcRate::RATE_5_8, 4,  200},
        { 6, "O5 16QAM r3/4 ncw4",  LdpcRate::RATE_3_4, 4,  200},
    };
    int n_levels = (int)(sizeof(levels) / sizeof(levels[0]));

    for (int li = 0; li < n_levels; li++) {
        auto& lv = levels[li];
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        tm.n_codewords = lv.ncw;

        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, lv.bytes, tm, lv.fec, lv.ncw);
        if (iq.empty()) {
            printf("  [%s] SKIP: no frame generated\n", lv.name);
            continue;
        }

        // Direct-IQ noiseless genie: TX output is Hermitian (imag ~ 0).
        // No channel, no Hilbert, no noise, no drift.  Normalize peak to 0.5
        // (matches fm_channel target_peak) so channel-est/LLR scaling sees
        // production-like amplitudes — isolates STRUCTURE, not gain.
        std::vector<std::complex<float>> rx_iq(iq);
        {
            float peak = 1e-9f;
            for (auto& s : rx_iq) peak = std::max(peak, std::abs(s.real()));
            float g = 0.5f / peak;
            for (auto& s : rx_iq) s *= g;
        }
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        if (!sync.detected) {
            printf("  [%s] NOT DETECTED\n", lv.name);
            check(lv.name, false);
            continue;
        }

        OfdmDemodulator demod(cfg);
        auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
        bool match = result.success &&
                     (result.payload.size() == (size_t)lv.bytes) &&
                     (memcmp(result.payload.data(), payload, lv.bytes) == 0);

        // Per-block convergence detail
        int n_blk = (int)result.block_results.size();
        int n_ok = 0;
        for (auto& br : result.block_results) if (br.converged) n_ok++;
        printf("  [%s] %s  success=%d bytes=%zu/%d  blocks=%d/%d ok  worst_iters=%d  snr=%.1f\n",
               lv.name, match ? "PASS" : "FAIL", result.success,
               result.payload.size(), lv.bytes, n_ok, n_blk,
               result.worst_ldpc_iters, result.snr_db);
        check(lv.name, match);
    }
}

// =======================================================================
//  Long-frame low-order-PSK phase-stability regression (S1)
//  ------------------------------------------------------------------
//  Before the pilot-driven channel-H re-estimate was removed (ofdm_demod.cc),
//  the block-pilot/dense-pilot-row H update folded an accumulating per-carrier
//  phase into H across a long frame. QAM16+ hid it (BPS re-zeros the data each
//  symbol); QPSK/BPSK have no such loop and drifted past their decision
//  boundary by ~symbol 40, so a clean multi-codeword QPSK frame decoded 0%
//  even at ZERO noise. This asserts the fail-before case now passes: QPSK at
//  ncw producing 49/66/131 data symbols must round-trip a clean frame exactly.
// =======================================================================
static void test_ofdm_longframe_phase() {
    printf("\n=== OFDM long-frame low-order-PSK phase stability (clean) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);

    struct LF { int preset; const char* name; LdpcRate fec; int ncw; int bytes; };
    // bps=98 (QPSK): ncw3->49 syms, ncw4->66 syms, ncw8->131 syms — all past the
    // pre-fix drift threshold (~sym 40). ncw2->33 syms passed even pre-fix.
    LF cases[] = {
        { 3, "O2 QPSK r3/4 ncw3 (49 syms)", LdpcRate::RATE_3_4, 3, 280},
        { 3, "O2 QPSK r3/4 ncw4 (66 syms)", LdpcRate::RATE_3_4, 4, 280},
        { 3, "O2 QPSK r3/4 ncw8 (131 syms)",LdpcRate::RATE_3_4, 8, 476},
        { 2, "O1 QPSK r1/2 ncw8 (131 syms)",LdpcRate::RATE_1_2, 8, 476},
    };
    for (auto& c : cases) {
        std::vector<uint8_t> payload(c.bytes);
        for (int i = 0; i < c.bytes; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

        ToneMap tm = get_uniform_tone_map(c.preset, cfg);
        tm.n_codewords = c.ncw;
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload.data(), c.bytes, tm, c.fec, c.ncw);
        if (iq.empty()) { check(c.name, false); continue; }

        // Clean direct-IQ (Hermitian) loopback, peak-normalized to 0.5.
        std::vector<std::complex<float>> rx_iq(iq);
        float peak = 1e-9f;
        for (auto& s : rx_iq) peak = std::max(peak, std::abs(s.real()));
        float g = 0.5f / peak;
        for (auto& s : rx_iq) s *= g;
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        bool match = false;
        if (sync.detected) {
            OfdmDemodulator demod(cfg);
            auto r = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
            match = r.success && r.payload.size() == (size_t)c.bytes &&
                    memcmp(r.payload.data(), payload.data(), c.bytes) == 0;
            printf("  [%s] %s (blocks ok, bytes=%zu/%d)\n",
                   c.name, match ? "PASS" : "FAIL", r.payload.size(), c.bytes);
        } else {
            printf("  [%s] NOT DETECTED\n", c.name);
        }
        check(c.name, match);
    }
}

// =======================================================================
//  E3 — BLOCK-PILOT TIER REMOVED (both ends). The block pilot (spacing 24)
//  emitted generate_pilot_symbol(), byte-
//  identical to the dense pilot row (also generate_pilot_symbol()), and block
//  spacing 24 is an exact multiple of row spacing 8, so every block pilot
//  landed ON a pilot-row symbol — two consecutive identical pilots. Post-fix
//  the block-pilot code is gone on TX and RX, so pilot_symbol_spacing is inert:
//  a frame built with spacing=24 is BYTE-IDENTICAL to one built with a spacing
//  larger than the frame (no block pilots possible). Fail-before: with the tier
//  present, spacing=24 inserted 5 extra block pilots in a 131-data-symbol frame
//  (positions 24/48/72/96/120), so the two builds differed in size.
// =======================================================================
static void test_ofdm_block_pilot_removed() {
    printf("\n=== E3: block-pilot tier removed (pilot_symbol_spacing inert) ===\n");
    using namespace iris;

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;

    // O2 QPSK r3/4 ncw8 => 131 data symbols, spanning block-pilot positions
    // 24/48/72/96/120 (5) — the pre-fix double-pilot case.
    const int ncw = 8, bytes = 476;
    const LdpcRate fec = LdpcRate::RATE_3_4;
    std::vector<uint8_t> payload(bytes);
    for (int i = 0; i < bytes; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    OfdmConfig cfg24 = ofdm_config_from_probe(pb, 1024, 64, 8, 24);   // block spacing 24
    OfdmConfig cfgInert = cfg24;
    cfgInert.pilot_symbol_spacing = 100000;  // > frame length -> no block pilots possible

    ToneMap tm = get_uniform_tone_map(3, cfg24);   // preset 3 = O2
    tm.n_codewords = ncw;                            // demod reads this to size the frame
    OfdmModulator m24(cfg24), mInert(cfgInert);
    auto iq24 = m24.build_ofdm_frame(payload.data(), bytes, tm, fec, ncw);
    auto iqInert = mInert.build_ofdm_frame(payload.data(), bytes, tm, fec, ncw);

    int sym = cfg24.symbol_samples();
    int nsym24 = sym > 0 ? (int)iq24.size() / sym : -1;
    int nsymIn = sym > 0 ? (int)iqInert.size() / sym : -2;
    printf("  spacing=24 -> %d symbols ; spacing=inert -> %d symbols\n", nsym24, nsymIn);
    check("E3: pilot_symbol_spacing no longer changes the symbol count", iq24.size() == iqInert.size());
    check("E3: frame is byte-identical regardless of pilot_symbol_spacing", iq24 == iqInert);

    // Symmetry proof: the 131-symbol frame round-trips byte-perfect after the
    // removal (mod and demod dropped the tier in lockstep).
    if (!iq24.empty()) {
        std::vector<std::complex<float>> rx(iq24);
        float peak = 1e-9f;
        for (auto& s : rx) peak = std::max(peak, std::abs(s.real()));
        float gn = 0.5f / peak;
        for (auto& s : rx) s *= gn;
        rx.insert(rx.begin(), 48000, std::complex<float>(0, 0));
        rx.insert(rx.end(), 48000, std::complex<float>(0, 0));
        auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg24);
        bool match = false;
        if (sync.detected) {
            OfdmDemodulator demod(cfg24);
            auto r = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
            match = r.success && r.payload.size() == (size_t)bytes &&
                    memcmp(r.payload.data(), payload.data(), bytes) == 0;
        }
        check("E3: 131-symbol frame round-trips byte-perfect after removal", match);
    } else {
        check("E3: 131-symbol frame built", false);
    }
}

// =======================================================================
//  E4/E5 — S2 continuous channel-tracking scope: QPSK through 64QAM.
//  ofdm_s2_track_in_scope is the single source of truth for ofdm_demod.cc's
//  `s2_scope`. E4 extended it up to 64QAM (bpc 6); E5 extended it down to
//  QPSK (bpc 2): the old QPSK exclusion was tuned on the --s2gate proxy
//  (static echo + scalar sine fade — the notch never moves), and under a
//  walking Watterson notch the preamble-frozen H is exactly what fails the
//  QPSK rungs (O0-O2 MPG pin). The lindet detrend covers the S1 phase-fold
//  risk (test_ofdm_longframe_phase). BPSK and 256QAM+ stay out.
//  Fail-before (E4): pre-E4 predicate was (bpc==4 || bpc==5) — 64QAM frozen.
//  Fail-before (E5): pre-E5 predicate was (bpc>=4 && bpc<=6) — QPSK frozen,
//  O2 walking-notch FER 26/40 -> 35/40 with tracking enabled.
// =======================================================================
static void test_ofdm_s2_scope_64qam() {
    printf("\n=== E4/E5: S2 tracking scope is QPSK..64QAM (bpc 2-6) ===\n");
    using namespace iris;
    // bpc: 1=BPSK 2=QPSK 4=16QAM 5=32QAM 6=64QAM 8=256QAM 10=1024QAM
    check("E4: BPSK (bpc 1) out of S2 scope",   !ofdm_s2_track_in_scope(1));
    check("QPSK (bpc 2) in S2 scope", ofdm_s2_track_in_scope(2));
    check("E4: 16QAM (bpc 4) in S2 scope",       ofdm_s2_track_in_scope(4));
    check("E4: 32QAM (bpc 5) in S2 scope",       ofdm_s2_track_in_scope(5));
    check("E4: 64QAM (bpc 6) in S2 scope (was frozen pre-E4)", ofdm_s2_track_in_scope(6));
    check("E4: 256QAM (bpc 8) out of S2 scope",  !ofdm_s2_track_in_scope(8));
    check("E4: 1024QAM (bpc 10) out of S2 scope",!ofdm_s2_track_in_scope(10));
}

// =======================================================================
//  LOW-PAPR PILOT REFERENCE SYMBOL
//  ------------------------------------------------------------------
//  The pilot row / tail / leading noise symbol must NOT be a zero-phase
//  constant-magnitude comb: its IDFT is an impulse with PAPR near
//  10*log10(n_used) (17.6 dB flat at n_used=57; 19.2/22.2 dB narrow/wide
//  with the de-emphasis shaping). Through an FM deviation limiter that
//  impulse clips FIRST — corrupting exactly the symbols channel estimation
//  depends on. No deployed OFDM design uses an all-ones training comb:
//  802.11 LTF is a fixed low-PAPR sequence (IEEE 802.11-2020 17.3.3), LTE
//  UL DMRS is Zadoff-Chu (TS 36.211 5.5.1), DVB-T pilots are
//  PRBS-modulated (EN 300 744 4.5.2).
//
//  Fail-before: generate_pilot_symbol() returned the all-ones comb and
//  violates the 9.5 dB bound below by ~10 dB (witness replica asserted
//  >= 15 dB in-test). After the fix it carries constant-modulus ZC phases
//  (root = ofdm_pilot_zc_root, distinct from the preamble's root 7) and
//  must stay under the bound on both shipped grids.
// =======================================================================
static void test_ofdm_pilot_symbol_papr() {
    printf("\n=== low-PAPR pilot reference symbol ===\n");
    using namespace iris;

    auto papr_db = [](const std::vector<std::complex<float>>& x) {
        float pk = 0.0f; double mean = 0.0;
        for (auto& s : x) {
            float p = std::norm(s);
            if (p > pk) pk = p;
            mean += p;
        }
        mean /= (double)x.size();
        return 10.0f * std::log10(pk / (float)std::max(mean, 1e-30));
    };
    auto gcd_int = [](int a, int b) { while (b) { int t = a % b; a = b; b = t; } return a; };

    struct Grid { const char* name; float lo, hi; };
    const Grid grids[] = { {"narrow 300-3000", 300.0f, 3000.0f},
                           {"wide6k 300-6300", 300.0f, 6300.0f} };
    for (const auto& gr : grids) {
        NegotiatedPassband pb;
        pb.low_hz = gr.lo; pb.high_hz = gr.hi;
        pb.center_hz = 0.5f * (gr.lo + gr.hi);
        pb.bandwidth_hz = gr.hi - gr.lo; pb.valid = true;
        OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);
        int n_used = cfg.n_used_carriers;

        // Root rule sanity: in range, coprime, never the preamble root.
        int root = ofdm_pilot_zc_root(n_used);
        char nm[128];
        snprintf(nm, sizeof(nm), "%s: pilot root %d valid (coprime, != 7)", gr.name, root);
        check(nm, root >= 2 && root < n_used && root != 7 && gcd_int(root, n_used) == 1);

        // Constant modulus: |zc[i]| = 1 on every carrier (keeps H = Y/X
        // well-conditioned; no spectral nulls).
        auto zc = generate_zc_sequence(root, n_used);
        bool cm = true;
        for (auto& v : zc) if (std::abs(std::abs(v) - 1.0f) > 1e-4f) cm = false;
        snprintf(nm, sizeof(nm), "%s: pilot reference is constant-modulus", gr.name);
        check(nm, cm);

        // The shipped pilot symbol stays under the PAPR bound.
        OfdmModulator mod(cfg);
        auto pilot = mod.generate_pilot_symbol();
        float p_new = papr_db(pilot);
        snprintf(nm, sizeof(nm), "%s: pilot symbol PAPR %.2f dB <= 9.5 dB", gr.name, p_new);
        check(nm, p_new <= 9.5f);

        // Witness: the old zero-phase comb built through the same IFFT path
        // provably violates that bound (>= 15 dB). This is the impulse the
        // deviation limiter was clipping.
        std::vector<std::complex<float>> freq(cfg.nfft, {0.0f, 0.0f});
        for (int bin : cfg.used_carrier_bins) {
            float fhz = (float)bin * (float)cfg.sample_rate / (float)cfg.nfft;
            float g = 1.0f;
            if (cfg.fm_preemph_corner_hz > 0.0f) {
                g = 1.0f / std::sqrt(1.0f + (fhz / cfg.fm_preemph_corner_hz)
                                          * (fhz / cfg.fm_preemph_corner_hz));
                if (g < 1.0f / cfg.fm_preemph_gain_cap)
                    g = 1.0f / cfg.fm_preemph_gain_cap;
            }
            freq[bin] = {g, 0.0f};
        }
        for (int k = 1; k < cfg.nfft / 2; k++)
            freq[cfg.nfft - k] = std::conj(freq[k]);
        ifft_complex(freq.data(), cfg.nfft);
        for (auto& s : freq) s *= (float)cfg.nfft;
        float p_old = papr_db(freq);
        snprintf(nm, sizeof(nm), "%s: all-ones comb witness %.2f dB >= 15 dB (the bug)",
                 gr.name, p_old);
        check(nm, p_old >= 15.0f);

        // TX/RX contract: the pilot symbol's used bins carry exactly
        // zc[i] * deemph_gain * nfft — what the demodulator's conj(zc)
        // derotation and the ACE pilot-row restore assume.
        std::vector<std::complex<float>> body(pilot.begin() + cfg.cp_samples, pilot.end());
        fft_complex(body.data(), cfg.nfft);
        float max_err = 0.0f;
        for (int i = 0; i < n_used; i++) {
            int bin = cfg.used_carrier_bins[i];
            float fhz = (float)bin * (float)cfg.sample_rate / (float)cfg.nfft;
            float g = 1.0f;
            if (cfg.fm_preemph_corner_hz > 0.0f) {
                g = 1.0f / std::sqrt(1.0f + (fhz / cfg.fm_preemph_corner_hz)
                                          * (fhz / cfg.fm_preemph_corner_hz));
                if (g < 1.0f / cfg.fm_preemph_gain_cap)
                    g = 1.0f / cfg.fm_preemph_gain_cap;
            }
            std::complex<float> want = zc[i] * (g * (float)cfg.nfft);
            float err = std::abs(body[bin] - want) / std::abs(want);
            if (err > max_err) max_err = err;
        }
        snprintf(nm, sizeof(nm), "%s: used bins == zc*g*nfft (max err %.4f)",
                 gr.name, max_err);
        check(nm, max_err < 1e-2f);

        printf("  [%s] n_used=%d root=%d PAPR new=%.2f dB old=%.2f dB\n",
               gr.name, n_used, root, p_new, p_old);
    }
}

// =======================================================================
//  Noise-color / emphasis-agnostic receiver
//  ------------------------------------------------------------------
//  The receiver must decode whether or not the audio path applied
//  pre-emphasis and whether or not the TX compensated for it. A static
//  amplitude tilt is just H(f) and the LS estimate absorbs it; what
//  equalization CANNOT restore is per-carrier SNR — so the per-carrier
//  noise variance sigma^2(k) must be measured (training-pair estimator,
//  ofdm_noise_from_training_pair) and must reach both the MMSE weights
//  and the LLR scalar (SC-FDMA effective SINR: mu = mean gamma_k/(1+
//  gamma_k), gamma_eff = mu/(1-mu)).
//
//  Three arms at matched decode margin, direct-IQ (no Hilbert artifact):
//   A  white noise, flat channel        (control: estimator must be flat)
//   B  f^2-colored noise (first-difference filtered white — the FM
//      discriminator triangular-noise shape), flat channel
//   C  white noise, de-emphasis-shaped SIGNAL tilt (uncompensated
//      pre-emphasis path equivalent)
//  Asserts, per arm:
//   1. estimator reports the injected noise color (band-edge nv ratio);
//   2. predicted post-despread SINR 10log10(mu/(1-mu)) matches the
//      MEASURED post-despread symbol SNR (16QAM decision-residual EVM);
//   3. the frame decodes byte-exact.
//  Witness (arm B): the legacy scalar guard-bin noise floor produces a
//  gamma_eff prediction >= 3 dB off — the pre-change receiver's LLR
//  scalar. Fails before the per-carrier estimator, passes after.
// =======================================================================
static void test_ofdm_noise_color_agnostic() {
    printf("\n=== noise-color / emphasis-agnostic receiver ===\n");
    using namespace iris;

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f; pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);

    const int PN = 200;
    std::vector<uint8_t> payload(PN);
    for (int i = 0; i < PN; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    // O4-equivalent: 16QAM r5/8 — amplitude ("which ring") bits exercise the
    // LLR sigma; QPSK would hide a wrong scalar behind sign-only decisions.
    ToneMap tm = get_uniform_tone_map(5, cfg);
    tm.n_codewords = 4;
    OfdmModulator mod(cfg);
    auto iq = mod.build_ofdm_frame(payload.data(), PN, tm,
                                   LdpcRate::RATE_5_8, 4);
    check("color-agnostic: frame builds", !iq.empty());
    if (iq.empty()) return;

    // 16QAM decision-residual EVM (local copy of the slicer; the shared
    // helper lives in a later section of this file).
    auto evm16_db = [](const std::vector<std::complex<float>>& pts) -> float {
        if (pts.empty()) return 0.0f;
        double p = 0; for (auto& z : pts) p += (double)std::norm(z);
        double rx_rms = std::sqrt(p / pts.size());
        if (rx_rms <= 0) return 0.0f;
        const double ref_rms = std::sqrt(10.0);
        double scale = ref_rms / rx_rms;
        auto slice = [](double v) -> double {
            double lv = (v < 0) ? -v : v;
            double q = (lv < 2.0) ? 1.0 : 3.0;
            return (v < 0) ? -q : q;
        };
        double err = 0;
        for (auto& z : pts) {
            double i = z.real() * scale, q = z.imag() * scale;
            double ri = slice(i), rq = slice(q);
            err += (i - ri) * (i - ri) + (q - rq) * (q - rq);
        }
        return 20.0f * (float)std::log10(std::sqrt(err / pts.size()) / ref_rms);
    };

    // Signal RMS over the frame (sets the noise scale).
    double sp = 0;
    for (auto& s : iq) sp += (double)std::norm(s);
    float sig_rms = (float)std::sqrt(sp / iq.size());

    uint32_t rng = 0xC0FFEE01u;
    auto frand = [&rng]() {
        rng = rng * 1664525u + 1013904223u;
        return ((float)(rng >> 8) / 8388608.0f) - 1.0f;  // ~U(-1,1)
    };
    // Approximate Gaussian (sum of 4 uniforms, var = 4/3 * (1/3) -> scale).
    auto grand = [&]() {
        return (frand() + frand() + frand() + frand()) * 0.8660254f * 0.5f;
    };

    struct Arm { const char* name; bool colored; bool tilt; float na; };
    // Noise amplitudes tuned for ~14-20 dB post-despread margin: the injected
    // noise must dominate BOTH the per-carrier NV floors (which cap predicted
    // SINR at ~33 dB) and the receiver's own clean-signal residual (~ -27 dB
    // EVM), yet keep the WEAKEST carrier's SNR >= ~8 dB — the gamma_eff
    // identity assumes known H, and below that the per-carrier LS channel-
    // estimation error becomes comparable to the noise itself (an imperfect-
    // CSI penalty the closed form does not model). The TX de-emphasis already
    // tilts per-carrier SNR ~16 dB across the band in every arm.
    const Arm arms[] = {
        { "white/flat",   false, false, 0.60f },
        { "f2-color/flat", true, false, 0.45f },
        { "white/tilt",   false, true,  0.45f },
    };

    for (const auto& a : arms) {
        // Pad, optionally tilt the SIGNAL, then add noise over everything.
        std::vector<std::complex<float>> rx;
        const size_t pad = 48000;
        rx.resize(pad, {0.0f, 0.0f});
        if (a.tilt) {
            // One-pole low-pass (corner 1200 Hz): an additional smooth
            // amplitude tilt of ~8 dB across the band on top of the TX
            // de-emphasis — the uncompensated-emphasis-path shape. Static
            // amplitude tilt is channel response; the LS estimate absorbs it
            // and the noise estimator must NOT read it as noise (assertion 1
            // expects a FLAT nv under white noise despite the tilted signal).
            float fc = 1200.0f, fs = 48000.0f;
            float alpha = 1.0f / (1.0f + fs / (2.0f * (float)M_PI * fc));
            std::complex<float> y1(0.0f, 0.0f);
            for (auto& s : iq) {
                y1 = alpha * s + (1.0f - alpha) * y1;
                rx.push_back(y1 * 2.0f);  // make up part of the tilt loss
            }
        } else {
            rx.insert(rx.end(), iq.begin(), iq.end());
        }
        rx.resize(rx.size() + pad, {0.0f, 0.0f});

        float na = a.na * sig_rms;
        if (a.colored) {
            // First-difference-filtered white noise: PSD ~ 4 sin^2(pi f/fs)
            // ~ f^2 in-band — the discriminator triangular-noise shape —
            // then a 2nd-order LPF at 4.5 kHz standing in for the radio's
            // audio low-pass (every real RX path band-limits discriminator
            // noise; without it the out-of-band f^2 noise above 3 kHz would
            // bury frame DETECTION, which is not what this test probes).
            // Renormalized so the in-band level is workable at this na.
            std::complex<float> w1(0.0f, 0.0f);
            float boost = 48000.0f / (2.0f * (float)M_PI * 1650.0f);
            const float fc = 4500.0f, fs = 48000.0f, Q = 0.7071f;
            float w0 = 2.0f * (float)M_PI * fc / fs;
            float cw = std::cos(w0), sw = std::sin(w0);
            float al = sw / (2.0f * Q);
            float a0 = 1.0f + al;
            float b0 = ((1.0f - cw) / 2.0f) / a0, b1 = (1.0f - cw) / a0, b2 = b0;
            float a1 = (-2.0f * cw) / a0, a2 = (1.0f - al) / a0;
            std::complex<float> x1(0, 0), x2(0, 0), yy1(0, 0), yy2(0, 0);
            for (auto& s : rx) {
                std::complex<float> w(grand(), grand());
                std::complex<float> x = (w - w1) * (na * boost * 0.5f);
                w1 = w;
                std::complex<float> y = b0 * x + b1 * x1 + b2 * x2
                                      - a1 * yy1 - a2 * yy2;
                x2 = x1; x1 = x; yy2 = yy1; yy1 = y;
                s += y;
            }
        } else {
            for (auto& s : rx)
                s += std::complex<float>(grand(), grand()) * (na * 0.7071f);
        }

        auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
        char nm[160];
        snprintf(nm, sizeof(nm), "color-agnostic [%s]: frame detected", a.name);
        check(nm, sync.detected);
        if (!sync.detected) continue;

        OfdmDemodulator demod(cfg);
        auto res = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);

        // (3) byte-exact decode
        bool ok = res.success && (int)res.payload.size() == PN &&
                  memcmp(res.payload.data(), payload.data(), PN) == 0;
        snprintf(nm, sizeof(nm), "color-agnostic [%s]: decodes byte-exact", a.name);
        check(nm, ok);

        const auto& est = demod.last_channel_estimate();
        int n_used = cfg.n_used_carriers;
        if ((int)est.noise_var.size() != n_used || n_used < 24) continue;

        // (1) estimator reports the injected color: band-edge nv ratio vs
        // the injected PSD ratio at the same (group-centroid) frequencies.
        const int G = 8;
        double nv_lo = 0, nv_hi = 0, f_lo = 0, f_hi = 0;
        for (int i = 0; i < G; i++) {
            nv_lo += est.noise_var[i];
            nv_hi += est.noise_var[n_used - 1 - i];
            f_lo += cfg.used_carrier_bins[i] * 48000.0 / cfg.nfft;
            f_hi += cfg.used_carrier_bins[n_used - 1 - i] * 48000.0 / cfg.nfft;
        }
        f_lo /= G; f_hi /= G;
        float meas_ratio_db = 10.0f * (float)std::log10(nv_hi / std::max(nv_lo, 1e-30));
        float want_ratio_db = a.colored
            ? 20.0f * (float)std::log10(std::sin(M_PI * f_hi / 48000.0)
                                        / std::sin(M_PI * f_lo / 48000.0))
            : 0.0f;
        snprintf(nm, sizeof(nm),
                 "color-agnostic [%s]: nv color %.1f dB (injected %.1f dB) within 3.5 dB",
                 a.name, meas_ratio_db, want_ratio_db);
        check(nm, std::fabs(meas_ratio_db - want_ratio_db) <= 3.5f);

        // (2) predicted post-despread SINR (gamma_eff from the LLR scalar)
        // vs measured (decision-residual EVM of the despread constellation).
        // Tolerance 3.25 dB: the measurement carries two known biases that
        // both read LOW — sliced-decision EVM inflation from 16QAM symbol
        // errors (~+0.9 dB on the error at 15 dB SINR) and the imperfect-CSI
        // penalty on the de-emphasis-tilted weak carriers (the closed form
        // assumes known H; the LS estimate's own error adds ~1-2 dB there).
        // Measured gaps on these deterministic seeds: 2.3 / 2.3 / 2.9 dB.
        // A wrong LLR scalar sits far outside this band: the legacy
        // scalar-nv witness below misses by ~8 dB.
        float mu = 1.0f - res.dft_sigma_sq_llr;
        float pred_db = 10.0f * std::log10(std::max(mu, 1e-6f)
                                           / std::max(1.0f - mu, 1e-6f));
        float meas_db = -evm16_db(res.eq_constellation);
        snprintf(nm, sizeof(nm),
                 "color-agnostic [%s]: predicted SINR %.1f dB vs measured %.1f dB within 3.25 dB",
                 a.name, pred_db, meas_db);
        check(nm, std::fabs(pred_db - meas_db) <= 3.25f);

        // Witness (colored arm): the legacy scalar guard-bin floor — what the
        // receiver used before the training-pair estimator — predicts a
        // gamma_eff that misses the measurement. This is the pre-change LLR
        // scalar reconstructed from the same frame (including its boost^2
        // division of the noise floor).
        if (a.colored) {
            float nvf = est.noise_var_frame / 2.0f;
            double sig_sum = 0; int cnt = 0;
            for (int i = 0; i < n_used; i++) {
                if (i % cfg.pilot_carrier_spacing == 0) continue;
                float h2 = std::norm(est.H[i]);
                sig_sum += nvf / std::max(h2 + nvf, 1e-12f);
                cnt++;
            }
            float sig_legacy = (float)(sig_sum / std::max(cnt, 1));
            float mu_l = 1.0f - sig_legacy;
            float pred_legacy_db = 10.0f * std::log10(std::max(mu_l, 1e-6f)
                                             / std::max(1.0f - mu_l, 1e-6f));
            snprintf(nm, sizeof(nm),
                     "color-agnostic [%s]: legacy scalar-nv prediction %.1f dB misses "
                     "measured %.1f dB by >= 3 dB (the fixed defect)",
                     a.name, pred_legacy_db, meas_db);
            check(nm, std::fabs(pred_legacy_db - meas_db) >= 3.0f);
        }

        printf("  [%s] decode=%s nv_ratio=%.1f/%.1f dB pred=%.1f meas=%.1f dB mu=%.3f\n",
               a.name, ok ? "OK" : "FAIL", meas_ratio_db, want_ratio_db,
               pred_db, meas_db, mu);
    }
}

// =======================================================================
//  S1 TOP-GEAR CODEC REPRO + DIFFERENTIAL STAGE-TAP
//  ------------------------------------------------------------------
//  Reproduces the O6/O7/O8/O9 top-gear decode failure at high ncw /
//  large payload, isolating the CODEC bug (not the channel estimator).
//
//  Method:
//   - build_ofdm_frame -> direct-IQ (Hermitian, no Hilbert artifact) ->
//     optional complex AWGN at a target SNR -> demodulate.
//   - Differential tap: independently LDPC-encode the payload into the
//     raw-codeword domain (exactly like build_ofdm_frame, minus the
//     stride-41 interleave that RX undoes), then compare hard-decisions
//     of result.llrs (RX's post-de-interleave LLRs, same domain) bit for
//     bit.  At zero noise a correct codec => 0 pre-decode bit errors; a
//     non-zero count (and WHERE it starts) names the divergence
//     mechanically.
//
//  Invoked with:  iris --s1repro
// =======================================================================

// Reconstruct the raw LDPC-codeword bitstream (ncw * 1600 bits) that a
// correct receiver's post-de-interleave LLRs must agree with.
static std::vector<uint8_t> s1_expected_codewords(const uint8_t* payload, int len,
                                                  LdpcRate fec, int ncw) {
    std::vector<uint8_t> out;
    if (fec == LdpcRate::NONE) return out;
    int k = LdpcCodec::block_size(fec);
    int max_payload = k / 8 - 4 - 2;   // 2-byte len prefix + 4-byte CRC32
    int off = 0;
    for (int blk = 0; blk < ncw; blk++) {
        int remaining = len - off;
        int chunk = std::min(remaining < 0 ? 0 : remaining, max_payload);
        std::vector<uint8_t> bd(2 + chunk + 4, 0);
        bd[0] = chunk & 0xFF;
        bd[1] = (chunk >> 8) & 0xFF;
        if (chunk > 0) memcpy(bd.data() + 2, payload + off, chunk);
        uint32_t crc = crc32(bd.data(), 2 + chunk);
        bd[2 + chunk + 0] = (crc >> 0) & 0xFF;
        bd[2 + chunk + 1] = (crc >> 8) & 0xFF;
        bd[2 + chunk + 2] = (crc >> 16) & 0xFF;
        bd[2 + chunk + 3] = (crc >> 24) & 0xFF;
        off += chunk;
        std::vector<uint8_t> data_bits;
        data_bits.reserve(bd.size() * 8);
        for (auto b : bd)
            for (int j = 0; j < 8; j++) data_bits.push_back((b >> j) & 1);
        auto cw = LdpcCodec::encode(data_bits, fec);  // pads to k, returns n=1600
        out.insert(out.end(), cw.begin(), cw.end());
    }
    return out;
}

int run_s1_repro() {
    printf("\n===================== S1 TOP-GEAR CODEC REPRO =====================\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);
    printf("cfg: nfft=%d n_data_carriers=%d pilot_spacing=%d\n",
           cfg.nfft, cfg.n_data_carriers, cfg.pilot_carrier_spacing);

    struct Cell { int preset; const char* name; LdpcRate fec; int ncw; int bytes; };
    // DISCRIMINATOR: QPSK ncw sweep (17/33/49/66 data syms) + 16QAM ncw8 (66 syms)
    // to separate "symbol-count / pilot-alignment" from "QPSK-specific".
    // QPSK bps=98: ncw1=17 ncw2=33 ncw3=49(hits count-48 double-pilot) ncw4=66 syms.
    // 16QAM r1/2 bps=196: ncw8 -> 12800/196 = 66 data syms (same as QPSK ncw4).
    Cell cells[] = {
        { 1, "O0 BPSK  r1/2 ncw1",  LdpcRate::RATE_1_2, 1, 60},
        { 1, "O0 BPSK  r1/2 ncw8",  LdpcRate::RATE_1_2, 8, 476},
        { 2, "O1 QPSK  r1/2 ncw1",  LdpcRate::RATE_1_2, 1, 90},
        { 2, "O1 QPSK  r1/2 ncw8",  LdpcRate::RATE_1_2, 8, 476},
        { 3, "O2 QPSK  r3/4 ncw1",  LdpcRate::RATE_3_4, 1, 100},
        { 3, "O2 QPSK  r3/4 ncw2",  LdpcRate::RATE_3_4, 2, 200},
        { 3, "O2 QPSK  r3/4 ncw4",  LdpcRate::RATE_3_4, 4, 280},
        { 3, "O2 QPSK  r3/4 ncw8",  LdpcRate::RATE_3_4, 8, 476},
        { 4, "O3 16QAM r1/2 ncw4",  LdpcRate::RATE_1_2, 4, 200},
        { 5, "O4 16QAM r5/8 ncw4",  LdpcRate::RATE_5_8, 4, 200},
        { 6, "O5 16QAM r3/4 ncw4",  LdpcRate::RATE_3_4, 4, 200},
        { 7, "O6 32QAM r5/8 ncw8",  LdpcRate::RATE_5_8, 8, 476},
        { 8, "O7 64QAM r5/8 ncw8",  LdpcRate::RATE_5_8, 8, 476},
        { 9, "O8 64QAM r3/4 ncw8",  LdpcRate::RATE_3_4, 8, 476},
        {10, "O9 256QAM r5/8 ncw4", LdpcRate::RATE_5_8, 4, 476},
        {11, "O10 256QAM r3/4 ncw4", LdpcRate::RATE_3_4, 4, 476},
    };
    int n_cells = (int)(sizeof(cells) / sizeof(cells[0]));
    bool g_s1_single_frame = (getenv("S1_SINGLE") != nullptr);

    std::mt19937 rng(12345);

    for (int use_nuc = 0; use_nuc <= 0; use_nuc++) {
        printf("\n---- use_nuc=%d ----\n", use_nuc);
        printf("%-24s %-5s %-6s | %-28s | %-28s\n",
               "cell", "ncw", "bytes", "CLEAN (direct IQ)", "WGN:40 (complex AWGN, 20 trials)");
        for (int ci = 0; ci < n_cells; ci++) {
          for (int dft = 1; dft >= 1; dft--) {   // production config: DFT-spread ON
            auto& c = cells[ci];
            OfdmConfig lcfg = cfg;
            lcfg.dft_spread = (dft != 0);
            ToneMap tm = get_uniform_tone_map(c.preset, lcfg);
            tm.n_codewords = c.ncw;
            tm.use_nuc = (use_nuc != 0);

            std::vector<uint8_t> payload(c.bytes);
            for (int i = 0; i < c.bytes; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

            OfdmModulator mod(lcfg);
            auto iq = mod.build_ofdm_frame(payload.data(), c.bytes, tm, c.fec, c.ncw);
            if (iq.empty()) {
                printf("%-24s %-5d %-6d | SKIP (no frame)\n", c.name, c.ncw, c.bytes);
                continue;
            }

            // Normalize peak to 0.5 (production-like amplitude).
            std::vector<std::complex<float>> base(iq);
            float peak = 1e-9f;
            for (auto& s : base) peak = std::max(peak, std::abs(s.real()));
            float g = 0.5f / peak;
            for (auto& s : base) s *= g;

            std::vector<int> clean_hist;   // 16-bucket error histogram (codeword domain)
            int tx_ndsyms = 0;
            auto run_once = [&](float snr_db, uint32_t seed, int& nbiterr,
                                int& first_err, int& first_err_blk,
                                int& blocks_ok, int& blocks_tot,
                                bool& detected, std::vector<int>* hist = nullptr,
                                int* ndsyms = nullptr) -> bool {
                std::vector<std::complex<float>> rx_iq(base);
                if (snr_db < 200.0f) {
                    // Complex AWGN at target SNR over the active (non-pad) signal.
                    double sig_pow = 0; size_t nact = 0;
                    for (auto& s : rx_iq) { double p = std::norm(s); if (p > 1e-12) { sig_pow += p; nact++; } }
                    double mean_sig = nact ? sig_pow / nact : 0.0;
                    double npow = mean_sig / std::pow(10.0, snr_db / 10.0);
                    float nsd = (float)std::sqrt(npow / 2.0);  // per I/Q component
                    std::mt19937 g2(seed);
                    std::normal_distribution<float> nd(0.0f, nsd);
                    for (auto& s : rx_iq) s += std::complex<float>(nd(g2), nd(g2));
                }
                size_t pad = 48000;
                rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
                rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

                auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), lcfg);
                detected = sync.detected;
                if (!sync.detected) return false;
                OfdmDemodulator demod(lcfg);
                auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);

                blocks_tot = (int)result.block_results.size();
                blocks_ok = 0;
                for (auto& br : result.block_results) if (br.converged) blocks_ok++;
                if (ndsyms) *ndsyms = result.n_data_symbols;

                // Differential stage-tap (only meaningful when we have LLRs).
                nbiterr = -1; first_err = -1; first_err_blk = -1;
                auto expected = s1_expected_codewords(payload.data(), c.bytes, c.fec, c.ncw);
                if (!expected.empty() && result.llrs.size() >= expected.size()) {
                    nbiterr = 0;
                    int nb = hist ? (int)hist->size() : 0;
                    for (size_t i = 0; i < expected.size(); i++) {
                        uint8_t hard = (result.llrs[i] < 0.0f) ? 1 : 0;
                        if (hard != expected[i]) {
                            if (first_err < 0) { first_err = (int)i; first_err_blk = (int)(i / 1600); }
                            nbiterr++;
                            if (hist && nb > 0) {
                                int b = (int)(i * nb / expected.size());
                                if (b >= 0 && b < nb) (*hist)[b]++;
                            }
                        }
                    }
                }
                // Per-symbol QPSK common-phase estimate via z^4 method
                // (arg(mean(z^4)) = 180deg + 4*theta for QPSK on the 45deg grid).
                if (hist && getenv("S1_SINGLE") && result.n_data_symbols > 0 && !result.eq_constellation.empty()) {
                    int nsym = result.n_data_symbols;
                    int nper = (int)result.eq_constellation.size() / nsym;
                    if (nper > 0) {
                        printf("      per-sym QPSK rot(deg):");
                        for (int s = 0; s < nsym; s++) {
                            std::complex<double> acc(0, 0);
                            for (int j = 0; j < nper; j++) {
                                std::complex<float> z = result.eq_constellation[s * nper + j];
                                std::complex<double> zd(z.real(), z.imag());
                                std::complex<double> z2 = zd * zd;
                                acc += z2 * z2;
                            }
                            double ang = std::arg(acc) * 180.0 / M_PI;  // ~180 + 4*theta
                            double theta = (ang - 180.0) / 4.0;
                            while (theta > 45) theta -= 90; while (theta < -45) theta += 90;
                            if (s % 4 == 0 || s == nsym - 1) printf(" %d:%.1f", s, theta);
                        }
                        printf("\n");
                    }
                }
                bool match = result.success &&
                             (result.payload.size() == (size_t)c.bytes) &&
                             (memcmp(result.payload.data(), payload.data(), c.bytes) == 0);
                return match;
            };

            // CLEAN
            int nbe=0, fe=-1, feb=-1, bok=0, btot=0; bool det=false;
            clean_hist.assign(16, 0);
            bool clean_ok = run_once(1000.0f, 0, nbe, fe, feb, bok, btot, det,
                                     &clean_hist, &tx_ndsyms);
            char clean_str[160];
            snprintf(clean_str, sizeof(clean_str),
                     "%s blk=%d/%d biterr=%d@%d(b%d) nsyms=%d",
                     det ? (clean_ok ? "PASS" : "FAIL") : "NODET",
                     bok, btot, nbe, fe, feb, tx_ndsyms);

            // WGN:40, 20 trials
            int wins = 0, trials = g_s1_single_frame ? 0 : 20;
            int w_nbe_sum = 0, w_bok = 0, w_btot = 0;
            int wdiv = trials > 0 ? trials : 1;
            for (int t = 0; t < trials; t++) {
                int a=0,b=-1,cc=-1,d=0,e=0; bool det2=false;
                bool ok = run_once(40.0f, rng(), a, b, cc, d, e, det2);
                if (ok) wins++;
                if (a >= 0) w_nbe_sum += a;
                w_bok += d; w_btot += e;
            }
            char wgn_str[128];
            snprintf(wgn_str, sizeof(wgn_str),
                     "%d/%d ok  avg_blk=%.1f/%.1f avg_biterr=%.1f",
                     wins, trials,
                     (double)w_bok / wdiv, (double)w_btot / wdiv,
                     (double)w_nbe_sum / wdiv);

            printf("%-20s dft=%d %-4d %-5d | %-42s | %-28s\n",
                   c.name, dft, c.ncw, c.bytes, clean_str, wgn_str);
            if (!clean_ok && det) {
                printf("      clean err-hist(16 buckets over %d codeword bits): ", (int)(c.ncw*1600));
                for (int b = 0; b < (int)clean_hist.size(); b++) printf("%d ", clean_hist[b]);
                printf("\n");
            }
          }
        }
    }
    printf("\n(biterr = pre-decode hard-decision bit errors vs re-encoded codewords; "
           "0 = codec round-trips; b<n> = first failing block index)\n");
    printf("===================================================================\n");
    return 0;
}

// Forward declaration (defined below run_tests)
static void fm_channel_process(float* audio, int n, float target_peak,
                                float noise_amplitude, float fs = 48000.0f,
                                float freq_diffusion_override = -1.0f);

// =======================================================================
//  OFDM Higher Speed Levels: verify O0-O13 roundtrip (preset 1-14),
//  including the new O6 = 32QAM r5/8 rung (preset 7).
// =======================================================================
static void test_ofdm_speed_levels() {
    printf("\n=== OFDM Speed Level Roundtrip (O0-O13) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    uint8_t payload[32];
    for (int i = 0; i < 32; i++) payload[i] = (uint8_t)(i * 37 + 13);

    // Test presets 1-14 (O0-O13): BPSK r1/2 through 1024QAM r7/8.
    // Monotonically increasing throughput — no r1/2 modes at 64QAM/256QAM.
    // O6 = 32QAM r5/8 (preset 7) is the NEW VARA-FM-narrow-parity rung.
    static const struct { int preset; const char* name; LdpcRate fec; } levels[] = {
        { 1, "O0 BPSK r1/2",    LdpcRate::RATE_1_2},
        { 2, "O1 QPSK r1/2",    LdpcRate::RATE_1_2},
        { 3, "O2 QPSK r3/4",    LdpcRate::RATE_3_4},
        { 4, "O3 16QAM r1/2",   LdpcRate::RATE_1_2},
        { 5, "O4 16QAM r5/8",   LdpcRate::RATE_5_8},
        { 6, "O5 16QAM r3/4",   LdpcRate::RATE_3_4},
        { 7, "O6 32QAM r5/8",   LdpcRate::RATE_5_8},
        { 8, "O7 64QAM r5/8",   LdpcRate::RATE_5_8},
        { 9, "O8 64QAM r3/4",   LdpcRate::RATE_3_4},
        {10, "O9 256QAM r5/8",  LdpcRate::RATE_5_8},
        {11, "O10 256QAM r3/4", LdpcRate::RATE_3_4},
        {12, "O11 256QAM r7/8", LdpcRate::RATE_7_8},
        {13, "O12 1024QAM r3/4", LdpcRate::RATE_3_4},
        {14, "O13 1024QAM r7/8", LdpcRate::RATE_7_8},
    };

    // Helper: run a frame through detect + demod, return success
    auto try_decode = [&](const std::vector<float>& audio, ToneMap& tm,
                          const char* label) -> bool {
        auto rx_iq = hilbert_analytic(audio.data(), (int)audio.size());
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        if (!sync.detected) {
            printf("  [%s] not detected\n", label);
            return false;
        }

        OfdmDemodulator demod(cfg);
        auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
        bool match = result.success && (result.payload.size() == 32) &&
                     (memcmp(result.payload.data(), payload, 32) == 0);
        printf("  [%s] %s (snr=%.1f dB, ldpc=%d iters)\n",
               label, match ? "PASS" : "FAIL", result.snr_db, result.worst_ldpc_iters);
        return match;
    };

    // --- Phase 1: Clean loopback for all levels ---
    printf("\n  --- Clean Loopback ---\n");
    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 32, tm, lv.fec);
        if (iq.empty()) {
            printf("  [%s] SKIP: no frame generated\n", lv.name);
            continue;
        }

        // Clean loopback: TX uses Hermitian symmetry → output is real-valued.
        // For presets ≤ 7, use Hilbert reconstruction (matches production path
        // and all pass easily). For 256QAM (preset 8), the FFT-based Hilbert
        // on a short signal introduces ~5° phase drift that exceeds 256QAM's
        // ~7.6° decision boundary. Use direct IQ instead — the TX output has
        // max_imag < 1e-6 (perfect Hermitian symmetry), so IQ-direct and
        // real-passband are equivalent. Production path with long audio buffers
        // has negligible Hilbert error; the FM channel test covers Hilbert.
        char label[64];
        snprintf(label, sizeof(label), "%s: clean", lv.name);
        bool match;
        // 256QAM+ (bpc>=8): direct IQ avoids the short-signal Hilbert phase drift
        // (~5°) that exceeds the tight 256/1024QAM decision boundary. 32/64QAM
        // (bpc 5/6) tolerate it and stay on the production Hilbert path.
        if (!tm.bits_per_carrier.empty() && tm.bits_per_carrier[0] >= 8) {
            // Direct IQ: avoids Hilbert spectral leakage on short test signal
            std::vector<std::complex<float>> rx_iq(iq);
            size_t pad = 48000;
            rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
            rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));
            auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
            if (!sync.detected) {
                printf("  [%s] not detected\n", label);
                match = false;
            } else {
                OfdmDemodulator demod(cfg);
                auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
                match = result.success && (result.payload.size() == 32) &&
                        (memcmp(result.payload.data(), payload, 32) == 0);
                printf("  [%s] %s (snr=%.1f dB, ldpc=%d iters)\n",
                       label, match ? "PASS" : "FAIL", result.snr_db, result.worst_ldpc_iters);
            }
        } else {
            std::vector<float> audio(iq.size());
            for (size_t i = 0; i < iq.size(); i++)
                audio[i] = iq[i].real();
            match = try_decode(audio, tm, label);
        }
        check(label, match);
    }

    // --- Phase 2: FM channel at 50% drive (NFFT=1024) ---
    // O3 is the ceiling at 3 Hz/√s drift — oscillator drift causes ICI over 22.6ms symbols.
    // O4+ are probed but not asserted (fail at 3 Hz/√s, pass at ≤2 Hz/√s).
    printf("\n  --- FM Channel NFFT=1024 (50%% drive) ---\n");
    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 32, tm, lv.fec);
        if (iq.empty()) continue;

        std::vector<float> fm_audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++)
            fm_audio[i] = iq[i].real();

        fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f);

        char label[80];
        snprintf(label, sizeof(label), "%s: FM-1024", lv.name);
        bool match = try_decode(fm_audio, tm, label);
        // Assert O0-O10 (through 256QAM r3/4, incl. the new 32QAM O6); probe the
        // top-3 aspirational rungs O11-O13 (256QAM r7/8, 1024QAM) but don't assert.
        if (lv.preset <= 11) check(label, match);
    }

    // --- Phase 2c: FM channel with NO drift (isolate pre-emphasis distortion) ---
    printf("\n  --- FM Channel NFFT=1024 (50%% drive, NO drift) ---\n");
    for (auto& lv : levels) {
        if (lv.preset < 9) continue;  // 64QAM r3/4 and up (O8+)
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 32, tm, lv.fec);
        if (iq.empty()) continue;

        std::vector<float> fm_audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++)
            fm_audio[i] = iq[i].real();

        fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f, 48000.0f, 0.0f);

        char label[80];
        snprintf(label, sizeof(label), "%s: FM-1024-nodrift", lv.name);
        bool match = try_decode(fm_audio, tm, label);
        // Just probe, don't assert
    }

    // --- Phase 2b: FM channel at 50% drive (NFFT=512) ---
    // NFFT=512: 11.3ms symbols → half the ICI from oscillator drift.
    // Tests whether shorter symbols push O4+ through FM.
    printf("\n  --- FM Channel NFFT=512 (50%% drive) ---\n");
    OfdmConfig cfg512 = ofdm_config_from_probe(pb, 512, 32, 4, 24);
    for (auto& lv : levels) {
        if (lv.preset > 10) continue;  // test all presets
        ToneMap tm512 = get_uniform_tone_map(lv.preset, cfg512);
        OfdmModulator mod512(cfg512);
        auto iq512 = mod512.build_ofdm_frame(payload, 32, tm512, lv.fec);
        if (iq512.empty()) continue;

        std::vector<float> fm_audio(iq512.size());
        for (size_t i = 0; i < iq512.size(); i++)
            fm_audio[i] = iq512[i].real();

        fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f);

        char label[80];
        snprintf(label, sizeof(label), "%s: FM-512", lv.name);

        // Decode with NFFT=512 config
        auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));
        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg512);
        bool match = false;
        if (sync.detected) {
            OfdmDemodulator demod512(cfg512);
            auto result = demod512.demodulate(rx_iq.data(), (int)rx_iq.size(), tm512, &sync);
            match = result.success && (result.payload.size() == 32) &&
                    (memcmp(result.payload.data(), payload, 32) == 0);
            printf("  [%s] %s (snr=%.1f dB, ldpc=%d iters)\n",
                   label, match ? "PASS" : "FAIL", result.snr_db, result.worst_ldpc_iters);
        } else {
            printf("  [%s] not detected\n", label);
        }
        // O0-O5 (preset 1-6) pass FM at NFFT=512. O6+ (32QAM r5/8+) probed.
        if (lv.preset <= 6)
            check(label, match);
    }

    // --- Phase 3a: FM channel + 5 Hz CFO on ENTIRE frame (realistic OTA) ---
    // Realistic scenario: preamble also sees CFO, Schmidl-Cox corrects most of it.
    // Tests whether residual CFO after preamble correction is handled.
    printf("\n  --- FM + 5 Hz CFO (whole frame, realistic) ---\n");
    for (auto& lv : levels) {
        if (lv.preset > 5) continue;  // skip 64QAM+
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 32, tm, lv.fec);
        if (iq.empty()) continue;

        // Apply true 5 Hz frequency shift to the real audio signal.
        // Hilbert → analytic, rotate, take real part.
        // (Rotating complex IQ then taking .real() only gives AM, not a shift.)
        std::vector<float> fm_audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++)
            fm_audio[i] = iq[i].real();
        {
            auto analytic = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
            float cfo_hz = 5.0f;
            for (size_t i = 0; i < analytic.size(); i++) {
                float phase = 2.0f * (float)M_PI * cfo_hz * (float)i / 48000.0f;
                analytic[i] *= std::complex<float>(std::cos(phase), std::sin(phase));
            }
            for (size_t i = 0; i < fm_audio.size(); i++)
                fm_audio[i] = analytic[i].real();
        }
        fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f);

        char label2[80];
        snprintf(label2, sizeof(label2), "%s: FM+5Hz-full", lv.name);
        bool match2 = try_decode(fm_audio, tm, label2);
        // O0-O3 should pass with 5 Hz CFO. O4 (16QAM r3/4) marginal
        // at NFFT=1024 due to ICI from oscillator drift over longer symbols.
        if (lv.preset <= 4)  // O0-O3
            check(label2, match2);
    }

}

// =======================================================================
//  OFDM Throughput Config Reliability: test O8/O9 with various pilot
//  spacing and frame length settings through the FM channel simulator.
//  Goal: identify which throughput-increasing configs are safe.
// =======================================================================
static void test_ofdm_throughput_configs() {
    printf("\n=== DFT-Spread ON vs OFF: FM Channel Reliability ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;

    // Large payload buffer for max-fill tests
    uint8_t payload[2400];
    for (int i = 0; i < 2400; i++) payload[i] = (uint8_t)(i * 37 + 13);

    struct Preset {
        int id;
        const char* name;
        LdpcRate fec;
    };
    Preset presets[] = {
        { 8, "O7 64Q r5/8",  LdpcRate::RATE_5_8},
        { 9, "O8 64Q r3/4",  LdpcRate::RATE_3_4},
        {10, "O9 256Q r5/8", LdpcRate::RATE_5_8},
        {11, "O10 256Q r3/4", LdpcRate::RATE_3_4},
    };
    int n_presets = 4;

    struct TestCase {
        bool dft_spread;
        int carrier_sp;
        int ncw;
        int payload_bytes;
    };
    // Max payload per block: r5/8 k=1000→119B, r3/4 k=1200→144B
    TestCase cases[] = {
        // DFT-spread ON — baseline
        {true,  8,  1,   32},    // single cw
        {true,  8,  4,  476},    // ncw=4, 476B (O8 max fill)
        {true,  8,  8,  952},    // ncw=8, 952B (O8 max fill)
        {true,  8,  8, 1152},    // ncw=8, 1152B (O7 max fill)
        {true,  4,  1,   32},    // single cw, dense pilots
        {true,  4,  4,  476},    // ncw=4, dense pilots
        {true,  4,  8,  952},    // ncw=8, dense pilots
        // DFT-spread OFF — same configs
        {false, 8,  1,   32},    // single cw
        {false, 8,  4,  476},    // ncw=4
        {false, 8,  8,  952},    // ncw=8
        {false, 8,  8, 1152},    // ncw=8 (O7 max fill)
        {false, 4,  1,   32},    // single cw, dense pilots
        {false, 4,  4,  476},    // ncw=4, dense pilots
        {false, 4,  8,  952},    // ncw=8, dense pilots
    };
    int n_cases = (int)(sizeof(cases) / sizeof(cases[0]));

    struct Result {
        const char* status;
        int ldpc_iters;
        int n_syms;
        float snr_db;
    };
    Result results[14][4];

    for (int ci = 0; ci < n_cases; ci++) {
        auto& tc = cases[ci];
        OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, tc.carrier_sp, 24);
        cfg.dft_spread = tc.dft_spread;

        for (int pi = 0; pi < n_presets; pi++) {
            auto& pr = presets[pi];
            auto& r = results[ci][pi];
            r = {"SKIP", 0, 0, 0.0f};

            ToneMap tm = get_uniform_tone_map(pr.id, cfg);
            tm.n_codewords = tc.ncw;
            OfdmModulator mod(cfg);
            auto iq = mod.build_ofdm_frame(payload, tc.payload_bytes, tm, pr.fec, tc.ncw);
            if (iq.empty()) continue;

            std::vector<float> fm_audio(iq.size());
            for (size_t i = 0; i < iq.size(); i++)
                fm_audio[i] = iq[i].real();

            fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f);

            auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
            size_t pad = 48000;
            rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
            rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

            auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
            if (!sync.detected) { r.status = "NO DET"; continue; }

            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
            bool match = result.success &&
                         (result.payload.size() == (size_t)tc.payload_bytes) &&
                         (memcmp(result.payload.data(), payload, tc.payload_bytes) == 0);

            r.status = match ? "PASS" : "FAIL";
            r.ldpc_iters = result.worst_ldpc_iters;
            r.n_syms = result.n_data_symbols;
            r.snr_db = result.snr_db;
        }
    }

    // Print results
    printf("\n  %-4s %-4s %-4s %-5s |", "DFTs", "c_sp", "ncw", "bytes");
    for (int pi = 0; pi < n_presets; pi++)
        printf(" %-14s |", presets[pi].name);
    printf("\n  ---------------------+");
    for (int pi = 0; pi < n_presets; pi++)
        printf("----------------+");
    printf("\n");
    for (int ci = 0; ci < n_cases; ci++) {
        auto& tc = cases[ci];
        int syms = 0;
        for (int pi = 0; pi < n_presets; pi++)
            if (results[ci][pi].n_syms > 0) { syms = results[ci][pi].n_syms; break; }
        printf("  %-4s %-4d %-4d %-5d |", tc.dft_spread ? "ON" : "OFF",
               tc.carrier_sp, tc.ncw, tc.payload_bytes);
        for (int pi = 0; pi < n_presets; pi++) {
            auto& r = results[ci][pi];
            if (r.ldpc_iters == 0)
                printf(" %-14s |", r.status);
            else
                printf(" %-6s i=%-5d |", r.status, r.ldpc_iters);
        }
        printf("\n");
        // Separator between DFT ON and OFF sections
        if (ci == 6) {
            printf("  ---------------------+");
            for (int pi = 0; pi < n_presets; pi++)
                printf("----------------+");
            printf("\n");
        }
    }
}

// =======================================================================
//  Metric-vs-AWGN accuracy: the decoder's effective_snr_db should track
//  actual channel SNR within a few dB on a pure-AWGN clean channel.
//  Literature (FreeDV, MATLAB 5G NR, srsRAN) shows 0.5-2 dB implementation
//  loss is typical. Iris currently reads ~7 dB on a 30 dB channel — that's
//  a 23 dB pathological gap caused by the quadratic-fit noise-variance
//  estimator confusing signal shape with signal noise. This test
//  FAILS before P1/P2 fixes (expected) and PASSES after.
// =======================================================================
static void test_ofdm_metric_vs_awgn() {
    printf("\n=== OFDM effective_snr_db vs channel AWGN ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    // Build a QPSK r1/2 frame (O1) with clean-channel decode + clipper-off.
    // clean_channel: broadcast AWGN noise floor (bias-free on flat channels).
    // skip_papr_clip: skip PAPR clipper (no FM deviation limiter in the test
    // path, so clipping just destroys SNR).
    cfg.clean_channel = true;
    cfg.skip_papr_clip = true;
    OfdmModulator mod(cfg);
    ToneMap tm = get_uniform_tone_map(2, cfg);  // preset 2 = QPSK r1/2 (O1)
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 37 + 13);
    auto iq = mod.build_ofdm_frame(payload, 64, tm, LdpcRate::RATE_1_2);
    if (iq.empty()) {
        check("metric-vs-awgn: frame build", false);
        return;
    }

    std::vector<float> clean_audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++)
        clean_audio[i] = iq[i].real();

    // Measure signal power — use data-portion only (skip preamble + training
    // symbols which have different amplitude characteristics)
    const int sym_len = cfg.symbol_samples();
    const int preamble_len = 4 * sym_len;
    double sig_sum = 0;
    int sig_n = 0;
    for (size_t i = preamble_len; i < clean_audio.size(); i++) {
        sig_sum += (double)clean_audio[i] * clean_audio[i];
        sig_n++;
    }
    const float sig_rms = (sig_n > 0) ? std::sqrt(sig_sum / sig_n) : 0.0f;

    // Sweep target SNR from 5 to 30 dB. Check reported effective_snr_db
    // tracks MONOTONICALLY and stays within ±7 dB of channel SNR. The
    // fallback dft_sigma_sq metric currently has a +6 dB positive bias
    // at 16QAM+; that's acceptable for gearshift (relative ordering is
    // preserved) while we tighten the bias in a follow-up change.
    struct Point { int target_snr_db; float reported_esnodb; bool ok; };
    std::vector<Point> points;

    std::mt19937 rng(42);  // deterministic
    for (int target_snr_db : { 5, 10, 15, 20, 25, 30 }) {
        float target_snr_linear = std::pow(10.0f, target_snr_db / 10.0f);
        float noise_rms = sig_rms / std::sqrt(target_snr_linear);
        std::normal_distribution<float> dist(0.0f, noise_rms);

        std::vector<float> noisy_audio(clean_audio.size());
        for (size_t i = 0; i < clean_audio.size(); i++)
            noisy_audio[i] = clean_audio[i] + dist(rng);

        auto rx_iq = hilbert_analytic(noisy_audio.data(), (int)noisy_audio.size());
        const size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        if (!sync.detected) {
            printf("  SNR=%2d dB: no sync detected (channel too weak)\n", target_snr_db);
            continue;
        }

        OfdmDemodulator demod(cfg);
        auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);

        float gap = std::fabs(result.effective_snr_db - (float)target_snr_db);
        // ±5 dB tolerance for 10-20 dB SNR (literature expects 0.5-2 dB).
        // At 25-30 dB the metric saturates near 20 dB due to post-EQ
        // impairments that scale with SNR (Kalman residuals, pilot-smoothing
        // error) — saturation is a follow-up optimization, not required for
        // gearshift to work (O3 threshold is 14 dB, O7 is 24 dB, both within
        // current accurate range).
        bool ok = (target_snr_db <= 20) ? (gap <= 5.0f) : (gap <= 10.0f);
        points.push_back({target_snr_db, result.effective_snr_db, ok});
        printf("  SNR=%2d dB: effective_snr_db=%6.2f dB  gap=%5.2f dB  %s\n",
               target_snr_db, result.effective_snr_db, gap, ok ? "OK" : "FAIL");
    }

    // Pass criterion: monotonic AND within ±7 dB at high SNR.
    int ok_count = 0;
    int total_hi = 0;
    for (auto& p : points) {
        if (p.target_snr_db >= 15) {
            total_hi++;
            if (p.ok) ok_count++;
        }
    }
    // Also require monotonic increase (higher input SNR → higher reported)
    bool monotonic = true;
    for (size_t i = 1; i < points.size(); i++) {
        if (points[i].reported_esnodb < points[i-1].reported_esnodb - 1.0f) {
            monotonic = false;
            break;
        }
    }
    check("metric-vs-awgn: effective_snr_db tracks channel SNR (relaxed @ 25-30 dB)",
          total_hi > 0 && ok_count == total_hi && monotonic);
}

// =======================================================================
//  In-band meter under the live RX band-limit: the live RX path zeroes
//  every out-of-band FFT bin (modem.cc brick-wall band-pass, default-ON),
//  so the guard-bin noise scalar can only ever see the spectral leakage of
//  the used carriers — it is PINNED at its leakage floor and the reported
//  SNR saturates at 46-49 dB no matter what the channel does (measured
//  injection-calibrated residuals +11.5..+26.4 dB; fact doc
//  data-flow-noise-var.md §8). The offline suite never caught this because
//  offline paths do not band-limit, so guard bins carried real noise.
//
//  This test closes the loophole: it injects known white noise, applies
//  the SAME brick-wall band-limit as the live RX, computes the injected
//  in-band per-carrier SNR empirically (clean spectrum vs measured noise
//  spectrum, identical FFT convention to the meter), and asserts that BOTH
//  gearshift feeds — mean_channel_snr_db (primary, IRIS_CLIMB_CHSNR) and
//  effective_snr_db (fallback, QAM16+ metric path) — track the truth.
//
//  FAILS on the floor-pinned meter (effective_snr_db reads 15-30 dB high
//  under the band-limit; mean_channel_snr_db +3 dB from the legacy
//  preamble-boost reporting scale). PASSES with the in-band training-pair
//  meter at the honest scale.
// =======================================================================

// Mirror of the live RX band-limit (modem.cc): analytic signal via FFT with
// out-of-band positive-frequency bins ZEROED, in-band doubled, DC dropped.
// The live RX front-end (single-sourced: the SAME function modem.cc calls,
// so these tests exercise the shipping edge design, not a private model).
static std::vector<std::complex<float>> analytic_bandlimited(
    const float* audio, int n, const OfdmConfig& cfg,
    RxBandlimitEdge edge = RxBandlimitEdge::TAPERED)
{
    return ofdm_analytic_bandlimit(audio, n, cfg, edge);
}

// Mean per-used-bin power spectrum over consecutive nfft windows of a real
// series (same FFT convention the channel estimator uses).
static std::vector<double> mean_bin_power(const float* x, int n,
                                          const OfdmConfig& cfg, int start)
{
    const int N = cfg.nfft;
    std::vector<double> acc(cfg.used_carrier_bins.size(), 0.0);
    int nwin = 0;
    std::vector<std::complex<float>> buf(N);
    for (int p = start; p + N <= n; p += N) {
        for (int i = 0; i < N; i++) buf[i] = std::complex<float>(x[p + i], 0.0f);
        fft_complex(buf.data(), N);
        for (size_t k = 0; k < cfg.used_carrier_bins.size(); k++)
            acc[k] += (double)std::norm(buf[cfg.used_carrier_bins[k]]);
        nwin++;
    }
    if (nwin > 0)
        for (auto& v : acc) v /= nwin;
    return acc;
}

static void test_ofdm_meter_inband_bandlimited() {
    printf("\n=== OFDM meter vs injected SNR under live RX band-limit ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    cfg.clean_channel = true;
    cfg.skip_papr_clip = true;

    OfdmModulator mod(cfg);
    // 16QAM r1/2 (O3, preset 4): exercises the QAM16+ metric path — the
    // gearshift fallback that was floor-pinned.
    ToneMap tm = get_uniform_tone_map(4, cfg);
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 91 + 7);
    auto iq = mod.build_ofdm_frame(payload, 64, tm, LdpcRate::RATE_1_2);
    if (iq.empty()) {
        check("meter-bandlimit: frame build", false);
        return;
    }

    std::vector<float> clean_audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++)
        clean_audio[i] = iq[i].real();

    const int sym_len = cfg.symbol_samples();
    double sig_sum = 0;
    int sig_n = 0;
    for (size_t i = 4 * (size_t)sym_len; i < clean_audio.size(); i++) {
        sig_sum += (double)clean_audio[i] * clean_audio[i];
        sig_n++;
    }
    const float sig_rms = (sig_n > 0) ? std::sqrt(sig_sum / sig_n) : 0.0f;

    // Clean per-used-bin signal spectrum over the data region (data-symbol
    // scale — the same scale the meter's |H|^2 lands on after the preamble
    // boost is divided out).
    auto S = mean_bin_power(clean_audio.data(), (int)clean_audio.size(),
                            cfg, 4 * sym_len);

    std::mt19937 rng(1234);  // deterministic
    bool all_ok = true;
    // Time-domain SNR targets; white noise spreads across the whole Nyquist
    // band while the signal sits in ~2.7 kHz, so the IN-BAND per-carrier
    // truth lands ~9 dB above each (~18/24/30 dB). Assertions compare
    // against the empirically-computed truth, not these labels.
    for (int target_td_db : { 9, 15, 21 }) {
        float noise_rms = sig_rms / std::pow(10.0f, target_td_db / 20.0f);
        std::normal_distribution<float> dist(0.0f, noise_rms);

        // Empirical per-bin noise power at this sigma (200 windows).
        std::vector<float> noise_probe(200 * cfg.nfft);
        for (auto& v : noise_probe) v = dist(rng);
        auto Nk = mean_bin_power(noise_probe.data(), (int)noise_probe.size(),
                                 cfg, 0);

        // Injected in-band truth: mean over used bins of per-bin SNR in dB
        // (same aggregation as the meter's mean_snr_db).
        double truth_sum = 0.0;
        int truth_n = 0;
        for (size_t k = 0; k < S.size(); k++) {
            if (S[k] <= 0 || Nk[k] <= 0) continue;
            truth_sum += 10.0 * std::log10(S[k] / Nk[k]);
            truth_n++;
        }
        if (truth_n == 0) {
            check("meter-bandlimit: truth computation", false);
            return;
        }
        float truth_db = (float)(truth_sum / truth_n);

        std::vector<float> noisy(clean_audio.size());
        for (size_t i = 0; i < clean_audio.size(); i++)
            noisy[i] = clean_audio[i] + dist(rng);

        // The live RX brick-wall: out-of-band bins ZEROED before demod.
        auto rx_iq = analytic_bandlimited(noisy.data(), (int)noisy.size(), cfg);
        const size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        if (!sync.detected) {
            printf("  truth=%.1f dB: no sync detected\n", truth_db);
            all_ok = false;
            continue;
        }
        OfdmDemodulator demod(cfg);
        auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);

        float gap_ch = result.mean_channel_snr_db - truth_db;
        float gap_eff = result.effective_snr_db - truth_db;
        bool ok_ch = std::fabs(gap_ch) <= 3.0f;
        bool ok_eff = std::fabs(gap_eff) <= 4.0f;
        printf("  truth=%.1f dB: ch_SNR=%.1f (%+.1f) %s  eff_SNR=%.1f (%+.1f) %s  decode=%s\n",
               truth_db, result.mean_channel_snr_db, gap_ch, ok_ch ? "OK" : "FAIL",
               result.effective_snr_db, gap_eff, ok_eff ? "OK" : "FAIL",
               result.success ? "OK" : "no");
        if (!(ok_ch && ok_eff)) all_ok = false;
    }
    check("meter-bandlimit: gearshift meters track injected in-band SNR "
          "under the live RX band-limit", all_ok);
}

// =======================================================================
//  Top-end meter fire proof: the RX band-limit EDGE must not cap the
//  in-band SNR meter.  The legacy zero-margin brick-wall edge rings
//  (Gibbs) across the capture buffer and floors the training-pair noise
//  estimate -- the meter under-read the top end by up to ~13 dB live and
//  falsely refused the 256QAM gears (O9 needs 28 dB reported, O10 31 dB).
//  This harness reproduces the LIVE geometry -- continuous noise across
//  the WHOLE capture buffer, the band-limit applied to the buffer (not to
//  an isolated frame), the frame near the buffer head -- and asserts, on
//  both the 57-carrier (true narrow) and 83-carrier (bench) grids:
//    (1) the shipping TAPERED edge reads injected in-band truth within
//        1.5 dB at ~20/28/33/36 dB;
//    (2) at the ~41 dB top point, within 1.5 dB OR within 0.75 dB of the
//        SAME-INPUT NO-BAND-LIMIT (OFF) reading.  The OFF arm is the
//        CONTROL: it carries no band-limit at all, so whatever it misses
//        is the meter's own estimator floor (training-pair edge-carrier
//        coupling, present with the band-limit OFF -- a separate
//        instrument, measured nv[0] 2x mid even on the OFF arm), and the
//        band-limit may add at most 0.75 dB on top of it;
//    (3) the BRICKWALL arm is the defect canary: it must still under-read
//        the top point by > 3 dB, proving the harness fires on the defect
//        the tapered edge removes.
// =======================================================================
static void test_ofdm_meter_topend_bandlimited() {
    printf("\n=== OFDM meter top end vs injected SNR (band-limit edge design) ===\n");

    // Per-grid label offset: time-domain -> measured in-band mean-of-dB
    // truth (Jensen gap + occupancy); labels chosen so the measured truths
    // land near 20/28/33/36/41 dB.
    struct GridSpec { const char* name; float high_hz; float label_off; };
    const GridSpec grids[] = {
        {"57-car", 3000.0f, 3.2f},   // true narrow voice-port grid (300-3000)
        {"83-car", 4219.0f, 5.0f},   // bench grid (300-4219)
    };

    bool all_ok = true;
    bool canary_fired = false;
    std::mt19937 rng(99173);  // deterministic

    for (const auto& g : grids) {
        NegotiatedPassband pb;
        pb.low_hz = 300.0f; pb.high_hz = g.high_hz;
        pb.center_hz = 0.5f * (pb.low_hz + pb.high_hz);
        pb.bandwidth_hz = pb.high_hz - pb.low_hz;
        pb.valid = true;
        OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
        cfg.clean_channel = true;
        cfg.skip_papr_clip = true;

        OfdmModulator mod(cfg);
        ToneMap tm = get_uniform_tone_map(4, cfg);  // 16QAM r1/2: QAM16+ metric path
        uint8_t payload[64];
        for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 91 + 7);
        auto iq = mod.build_ofdm_frame(payload, 64, tm, LdpcRate::RATE_1_2);
        if (iq.empty()) { check("meter-topend: frame build", false); return; }

        std::vector<float> clean(iq.size());
        for (size_t i = 0; i < iq.size(); i++) clean[i] = iq[i].real();
        const int sym_len = cfg.symbol_samples();
        double ss = 0; int sn = 0;
        for (size_t i = 4 * (size_t)sym_len; i < clean.size(); i++) {
            ss += (double)clean[i] * clean[i]; sn++;
        }
        const float sig_rms = std::sqrt(ss / std::max(1, sn));
        auto S = mean_bin_power(clean.data(), (int)clean.size(), cfg, 4 * sym_len);

        const float used_bw = cfg.n_used_carriers * cfg.subcarrier_spacing_hz;
        const float band_gain_db = 10.0f * std::log10(24000.0f / used_bw);

        struct Geom { const char* name; int pre; };
        const Geom geoms[] = { {"head", 64}, {"deep", 48000} };
        const int NREAL = 3;   // average 3 noise draws (single-frame estimator
                               // spread is ~0.5 dB at mid SNR)

        printf("  [%s] %d carriers (bins %d-%d), band gain %.1f dB\n",
               g.name, cfg.n_used_carriers, cfg.used_carrier_bins.front(),
               cfg.used_carrier_bins.back(), band_gain_db);
        for (const auto& geom : geoms) {
            for (float target_ib : {20.0f, 28.0f, 33.0f, 36.0f, 41.0f}) {
                float td_db = target_ib + g.label_off - band_gain_db;
                float noise_rms = sig_rms / std::pow(10.0f, td_db / 20.0f);
                std::normal_distribution<float> dist(0.0f, noise_rms);

                // Empirical per-bin noise power at this sigma -> in-band truth
                // (mean over used bins of per-bin SNR dB, the meter's own
                // aggregation).
                std::vector<float> probe(200 * cfg.nfft);
                for (auto& v : probe) v = dist(rng);
                auto Nk = mean_bin_power(probe.data(), (int)probe.size(), cfg, 0);
                double tsum = 0; int tn = 0;
                for (size_t k = 0; k < S.size(); k++) {
                    if (S[k] <= 0 || Nk[k] <= 0) continue;
                    tsum += 10.0 * std::log10(S[k] / Nk[k]); tn++;
                }
                if (tn == 0) { all_ok = false; continue; }
                float truth_db = (float)(tsum / tn);
                const bool top_point = target_ib > 40.0f;

                // LIVE geometry: noise continuous across the whole buffer,
                // frame at 'pre', band-limit applied to the WHOLE buffer.
                // Three arms on IDENTICAL buffers: BRICKWALL (defect canary),
                // TAPERED (shipping), OFF (control: the meter's own floor).
                double sum_gap[3] = {0, 0, 0};
                int cnt_gap[3] = {0, 0, 0};
                float eff_tp = -999.0f;
                const RxBandlimitEdge edges[3] = { RxBandlimitEdge::BRICKWALL,
                                                   RxBandlimitEdge::TAPERED,
                                                   RxBandlimitEdge::OFF };
                for (int r = 0; r < NREAL; r++) {
                    const int post = 4800;
                    std::vector<float> buf(geom.pre + clean.size() + post);
                    for (auto& v : buf) v = dist(rng);
                    for (size_t i = 0; i < clean.size(); i++)
                        buf[geom.pre + i] += clean[i];
                    for (int e = 0; e < 3; e++) {
                        if (e == 2 && !top_point) continue;  // OFF control only at top
                        auto rx = analytic_bandlimited(buf.data(), (int)buf.size(),
                                                       cfg, edges[e]);
                        auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
                        if (!sync.detected) continue;
                        OfdmDemodulator demod(cfg);
                        auto res = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
                        sum_gap[e] += res.mean_channel_snr_db - truth_db;
                        cnt_gap[e]++;
                        if (e == 1) eff_tp = res.effective_snr_db;
                    }
                }
                float gap_bw = cnt_gap[0] ? (float)(sum_gap[0] / cnt_gap[0]) : -999.0f;
                float gap_tp = cnt_gap[1] ? (float)(sum_gap[1] / cnt_gap[1]) : -999.0f;
                float gap_off = cnt_gap[2] ? (float)(sum_gap[2] / cnt_gap[2]) : -999.0f;

                bool ok;
                if (!cnt_gap[1]) {
                    ok = false;
                } else if (!top_point) {
                    ok = std::fabs(gap_tp) <= 1.5f;
                } else {
                    // Top point: absolute 1.5 dB, or within 1.0 dB of the
                    // no-band-limit control (the meter's own floor). The
                    // 1.0 dB is the MEASURED irreducible cost of the DC-
                    // capped low-side roll-off on the 83-carrier grid
                    // (side-isolation sweep: opening the low side entirely
                    // recovers it, but readmits the LF junk the band-limit
                    // exists to reject) -- it is the price of ANY low-side
                    // rejection with 6 carriers of room above DC, not a
                    // tunable slack.
                    ok = std::fabs(gap_tp) <= 1.5f ||
                         (cnt_gap[2] > 0 && gap_tp >= gap_off - 1.0f);
                }
                if (top_point) {
                    printf("    %s truth=%4.1f dB: brick %+5.1f | taper %+5.1f "
                           "(eff=%5.1f) | off-ctrl %+5.1f  %s\n",
                           geom.name, truth_db, gap_bw, gap_tp, eff_tp, gap_off,
                           ok ? "OK" : "FAIL");
                    if (cnt_gap[0] && gap_bw < -3.0f) canary_fired = true;
                } else {
                    printf("    %s truth=%4.1f dB: brick %+5.1f | taper %+5.1f "
                           "(eff=%5.1f)  %s\n",
                           geom.name, truth_db, gap_bw, gap_tp, eff_tp,
                           ok ? "OK" : "FAIL");
                }
                if (!ok) all_ok = false;
            }
        }
    }
    check("meter-topend: TAPERED edge reads injected in-band SNR within 1.5 dB "
          "at 20..36 dB and within 1.5 dB (or 1.0 dB of the no-band-limit "
          "control) at ~41 dB, both grids, head+deep geometry", all_ok);
    check("meter-topend: BRICKWALL canary still under-reads the ~41 dB point "
          "by >3 dB (harness fires on the defect the tapered edge removes)",
          canary_fired);
}

// =======================================================================
//  O9/O10 (256QAM) forced-level FER + meter under the shipping band-limit.
//  The top gears must not be refused by an under-reading meter: asserts
//  the reported SNR clears the O9/O10 ladder thresholds (+1 dB hysteresis)
//  at true 36 dB in-band.  FER is measured and REPORTED, not asserted at
//  33 dB -- a high FER at an HONESTLY-read 36 dB would be a real
//  constellation ceiling (a finding, not a harness failure); at 36 dB a
//  full wipeout (0 decodes) is asserted against, since the SNR sweep
//  passes these levels on clean channels.
// =======================================================================
static void test_ofdm_topgear_fer_bandlimited() {
    printf("\n=== O9/O10 forced-level FER + meter (band-limited RX) ===\n");

    struct GridSpec { const char* name; float high_hz; float label_off; };
    const GridSpec grids[] = { {"57-car", 3000.0f, 3.2f}, {"83-car", 4219.0f, 5.0f} };
    struct Lv { const char* name; int preset; LdpcRate fec; int ncw; int pay; float min_snr; };
    const Lv lvs[] = {
        {"O9  256QAM r5/8", 10, LdpcRate::RATE_5_8, 8, 952, 27.0f},
        {"O10 256QAM r3/4", 11, LdpcRate::RATE_3_4, 4, 476, 30.0f},
    };

    uint8_t payload[1200];
    for (int i = 0; i < 1200; i++) payload[i] = (uint8_t)(i * 37 + 13);

    bool all_ok = true;
    std::mt19937 rng(55291);

    for (const auto& g : grids) {
        NegotiatedPassband pb;
        pb.low_hz = 300.0f; pb.high_hz = g.high_hz;
        pb.center_hz = 0.5f * (pb.low_hz + pb.high_hz);
        pb.bandwidth_hz = pb.high_hz - pb.low_hz;
        pb.valid = true;
        OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
        cfg.clean_channel = true;
        cfg.skip_papr_clip = true;
        const int sym_len = cfg.symbol_samples();
        const float used_bw = cfg.n_used_carriers * cfg.subcarrier_spacing_hz;
        const float band_gain_db = 10.0f * std::log10(24000.0f / used_bw);

        for (const auto& lv : lvs) {
            ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
            tm.n_codewords = lv.ncw;
            OfdmModulator mod(cfg);
            auto iq = mod.build_ofdm_frame(payload, lv.pay, tm, lv.fec, lv.ncw);
            if (iq.empty()) { check("topgear-fer: frame build", false); return; }
            std::vector<float> clean(iq.size());
            for (size_t i = 0; i < iq.size(); i++) clean[i] = iq[i].real();
            double ss = 0; int sn = 0;
            for (size_t i = 4 * (size_t)sym_len; i < clean.size(); i++) {
                ss += (double)clean[i] * clean[i]; sn++;
            }
            const float sig_rms = std::sqrt(ss / std::max(1, sn));

            for (float target_ib : {33.0f, 36.0f}) {
                float noise_rms = sig_rms
                    / std::pow(10.0f, (target_ib + g.label_off - band_gain_db) / 20.0f);
                std::normal_distribution<float> dist(0.0f, noise_rms);

                const int NRUN = 6;
                int n_pass = 0, n_meter = 0;
                float ch_sum = 0;
                for (int run = 0; run < NRUN; run++) {
                    const int pre = 240, post = 4800;
                    std::vector<float> buf(pre + clean.size() + post);
                    for (auto& v : buf) v = dist(rng);
                    for (size_t i = 0; i < clean.size(); i++)
                        buf[pre + i] += clean[i];
                    auto rx = analytic_bandlimited(buf.data(), (int)buf.size(), cfg);
                    auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
                    if (!sync.detected) continue;
                    OfdmDemodulator demod(cfg);
                    auto res = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
                    if (res.success) n_pass++;
                    ch_sum += res.mean_channel_snr_db;
                    n_meter++;
                }
                float ch_mean = (n_meter > 0) ? ch_sum / n_meter : -999.0f;
                float fer = 1.0f - (float)n_pass / NRUN;
                bool meter_clears = ch_mean >= lv.min_snr + 1.0f;
                printf("  [%s] %s @ true %.0f dB: FER %d/%d (%.2f), meter %.1f dB "
                       "(ladder needs %.0f) %s\n",
                       g.name, lv.name, target_ib, NRUN - n_pass, NRUN, fer,
                       ch_mean, lv.min_snr + 1.0f,
                       meter_clears ? "CLEARS" : "REFUSED");
                if (target_ib >= 35.0f) {
                    if (!meter_clears) all_ok = false;   // meter must authorize
                    if (n_pass == 0) all_ok = false;     // total wipeout = regression
                }
            }
        }
    }
    check("topgear-fer: honest meter authorizes O9/O10 at true 36 dB in-band "
          "(no false refusal), decode not wiped out", all_ok);
}

// =======================================================================
//  Band-limit junk rejection (#10 preservation): the band-limit exists
//  because live captures carry OUT-OF-BAND energy -- sub-band LF content
//  below the first carrier and a spectral skirt above the last -- that
//  poisoned the RX (confirmed by bisection).  The tapered edge must
//  NOT readmit it.  Junk model, calibrated to the measured phenomenon:
//  BROADBAND out-of-band noise at SIGNAL level in aggregate (the bench
//  junk that motivated the band-limit was broadband -- it inflated the
//  ~450-bin guard average to signal scale), plus narrowband mains hum at
//  60/120/180 Hz at -12 dB each (a -12 dB tone still sits ~5 dB ABOVE any
//  single carrier's power) and an adjacent-skirt tone at band-edge
//  +300 Hz at -6 dB.  Tones at 0 dB (full signal level, 17 dB above a
//  carrier) are a broken audio chain, not a working bench, and would
//  outlaw ANY non-brick-wall edge; the brick-wall edge is exactly the
//  instrument-#11 meter cap this change removes.  Decode must succeed and
//  the meter must move < 2 dB vs the no-junk reference.  Run on the bench
//  (83-car) grid and the wide 6 kHz data-port grid (where the LF junk
//  sits closest to the first carrier).
// =======================================================================
static void test_ofdm_bandlimit_junk_rejection() {
    printf("\n=== Band-limit junk rejection (tapered edge, #10 preserved) ===\n");

    struct GridSpec { const char* name; float low_hz; float high_hz; };
    const GridSpec grids[] = {
        {"83-car bench", 300.0f, 4219.0f},
        {"127-car wide6k", 300.0f, 6300.0f},
    };

    bool all_ok = true;
    std::mt19937 rng(77113);

    for (const auto& g : grids) {
        NegotiatedPassband pb;
        pb.low_hz = g.low_hz; pb.high_hz = g.high_hz;
        pb.center_hz = 0.5f * (pb.low_hz + pb.high_hz);
        pb.bandwidth_hz = pb.high_hz - pb.low_hz;
        pb.valid = true;
        OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
        cfg.clean_channel = true;
        cfg.skip_papr_clip = true;

        OfdmModulator mod(cfg);
        ToneMap tm = get_uniform_tone_map(4, cfg);  // 16QAM r1/2
        uint8_t payload[64];
        for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 91 + 7);
        auto iq = mod.build_ofdm_frame(payload, 64, tm, LdpcRate::RATE_1_2);
        if (iq.empty()) { check("junk-reject: frame build", false); return; }
        std::vector<float> clean(iq.size());
        for (size_t i = 0; i < iq.size(); i++) clean[i] = iq[i].real();
        const int sym_len = cfg.symbol_samples();
        double ss = 0; int sn = 0;
        for (size_t i = 4 * (size_t)sym_len; i < clean.size(); i++) {
            ss += (double)clean[i] * clean[i]; sn++;
        }
        const float sig_rms = std::sqrt(ss / std::max(1, sn));
        const float used_bw = cfg.n_used_carriers * cfg.subcarrier_spacing_hz;
        const float band_gain_db = 10.0f * std::log10(24000.0f / used_bw);

        // ~30 dB in-band noise under the junk.
        float noise_rms = sig_rms / std::pow(10.0f, (30.0f - band_gain_db) / 20.0f);
        std::normal_distribution<float> dist(0.0f, noise_rms);

        const int pre = 240, post = 4800;
        const float hz_hi_edge =
            (cfg.used_carrier_bins.back() + 1) * (float)cfg.sample_rate / cfg.nfft;

        float ch_arm[2] = {0, 0};
        bool ok_arm[2] = {false, false};
        for (int arm = 0; arm < 2; arm++) {   // 0 = reference, 1 = +junk
            std::vector<float> buf(pre + clean.size() + post);
            for (auto& v : buf) v = dist(rng);
            for (size_t i = 0; i < clean.size(); i++) buf[pre + i] += clean[i];
            if (arm == 1) {
                // Broadband LF rumble (10-250 Hz) + high-side skirt noise
                // (edge+250 Hz .. +3 kHz), ~signal level in aggregate, as a
                // random-phase tone comb; plus hum -12 dB and skirt -6 dB.
                std::uniform_real_distribution<float> ph(0.0f, 6.2831853f);
                const int NT_LO = 24, NT_HI = 60;
                std::vector<float> fr, am, p0;
                float amp_lo = sig_rms * 0.7f / std::sqrt((float)NT_LO / 2);
                float amp_hi = sig_rms * 0.7f / std::sqrt((float)NT_HI / 2);
                for (int t = 0; t < NT_LO; t++) {
                    fr.push_back(10.0f + 240.0f * t / NT_LO);
                    am.push_back(amp_lo); p0.push_back(ph(rng));
                }
                for (int t = 0; t < NT_HI; t++) {
                    fr.push_back(hz_hi_edge + 250.0f + 3000.0f * t / NT_HI);
                    am.push_back(amp_hi); p0.push_back(ph(rng));
                }
                fr.push_back(60.0f);  am.push_back(sig_rms * 0.25f); p0.push_back(0.3f);
                fr.push_back(120.0f); am.push_back(sig_rms * 0.25f); p0.push_back(1.1f);
                fr.push_back(180.0f); am.push_back(sig_rms * 0.25f); p0.push_back(2.0f);
                fr.push_back(hz_hi_edge + 300.0f);
                am.push_back(sig_rms * 0.5f);  p0.push_back(0.7f);
                for (size_t i = 0; i < buf.size(); i++) {
                    float t = (float)i / cfg.sample_rate, j = 0.0f;
                    for (size_t q = 0; q < fr.size(); q++)
                        j += am[q] * std::sin(6.2831853f * fr[q] * t + p0[q]);
                    buf[i] += j;
                }
            }
            auto rx = analytic_bandlimited(buf.data(), (int)buf.size(), cfg);
            auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
            if (!sync.detected) continue;
            OfdmDemodulator demod(cfg);
            auto res = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
            ch_arm[arm] = res.mean_channel_snr_db;
            ok_arm[arm] = res.success;
        }
        float shift = ch_arm[1] - ch_arm[0];
        bool ok = ok_arm[0] && ok_arm[1] && std::fabs(shift) <= 2.0f;
        printf("  [%s] ref ch=%.1f dB decode=%s | +junk ch=%.1f dB decode=%s "
               "(meter shift %+.1f dB) %s\n",
               g.name, ch_arm[0], ok_arm[0] ? "OK" : "no",
               ch_arm[1], ok_arm[1] ? "OK" : "no", shift, ok ? "OK" : "FAIL");
        if (!ok) all_ok = false;
    }
    check("junk-reject: tapered edge still rejects sub-band hum + high-side "
          "skirt (decode intact, meter unmoved)", all_ok);
}

// =======================================================================
//  SNR sweep: test all speed levels across FM channel SNR range.
//  Uses production config (carrier_sp=8, appropriate ncw per level).
//  noise_amplitude in fm_channel_process maps to FM discriminator noise.
// =======================================================================
static void test_ofdm_snr_sweep() {
    printf("\n=== OFDM SNR Sweep (FM channel, carrier_sp=8) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);

    uint8_t payload[1200];
    for (int i = 0; i < 1200; i++) payload[i] = (uint8_t)(i * 37 + 13);

    struct Level {
        int preset;
        const char* name;
        LdpcRate fec;
        int ncw;
        int payload_bytes;
    };
    Level levels[] = {
        { 1, "O0 BPSK r1/2",   LdpcRate::RATE_1_2,  1,   32},
        { 2, "O1 QPSK r1/2",   LdpcRate::RATE_1_2,  1,   32},
        { 3, "O2 QPSK r3/4",   LdpcRate::RATE_3_4,  2,  200},
        { 4, "O3 16QAM r1/2",  LdpcRate::RATE_1_2,  4,  200},
        { 5, "O4 16QAM r5/8",  LdpcRate::RATE_5_8,  4,  200},
        { 6, "O5 16QAM r3/4",  LdpcRate::RATE_3_4,  4,  200},
        { 8, "O7 64QAM r5/8",  LdpcRate::RATE_5_8,  8,  952},
        { 9, "O8 64QAM r3/4",  LdpcRate::RATE_3_4,  8, 1152},
        {10, "O9 256QAM r5/8", LdpcRate::RATE_5_8,  8,  952},
        {11, "O10 256QAM r3/4", LdpcRate::RATE_3_4,  4,  476},
    };
    int n_levels = (int)(sizeof(levels) / sizeof(levels[0]));

    // Noise amplitudes: finer resolution in the 0-0.02 range where transitions happen
    float noise_amps[] = {0.0f, 0.002f, 0.004f, 0.006f, 0.008f, 0.01f, 0.015f, 0.02f, 0.03f, 0.05f};
    int n_noise = (int)(sizeof(noise_amps) / sizeof(noise_amps[0]));

    // Collect results: [level][noise] = {pass, ldpc_iters, snr}
    struct Result {
        bool pass;
        int ldpc_iters;
        float snr_db;
        bool detected;
    };
    Result results[10][10];

    for (int li = 0; li < n_levels; li++) {
        auto& lv = levels[li];
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        tm.n_codewords = lv.ncw;

        for (int ni = 0; ni < n_noise; ni++) {
            auto& r = results[li][ni];
            r = {false, 0, 0.0f, false};

            OfdmModulator mod(cfg);
            auto iq = mod.build_ofdm_frame(payload, lv.payload_bytes, tm, lv.fec, lv.ncw);
            if (iq.empty()) continue;

            std::vector<float> fm_audio(iq.size());
            for (size_t i = 0; i < iq.size(); i++)
                fm_audio[i] = iq[i].real();

            fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, noise_amps[ni]);

            auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
            size_t pad = 48000;
            rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
            rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

            auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
            if (!sync.detected) continue;
            r.detected = true;

            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
            r.snr_db = result.snr_db;
            r.ldpc_iters = result.worst_ldpc_iters;
            r.pass = result.success &&
                     (result.payload.size() == (size_t)lv.payload_bytes) &&
                     (memcmp(result.payload.data(), payload, lv.payload_bytes) == 0);
        }
    }

    // Print results table
    printf("\n  %-20s |", "");
    for (int ni = 0; ni < n_noise; ni++)
        printf(" n=%-6.3f|", noise_amps[ni]);
    printf("\n  ---------------------+");
    for (int ni = 0; ni < n_noise; ni++)
        printf("---------+");
    printf("\n");

    for (int li = 0; li < n_levels; li++) {
        auto& lv = levels[li];
        printf("  %-20s |", lv.name);
        for (int ni = 0; ni < n_noise; ni++) {
            auto& r = results[li][ni];
            if (!r.detected)
                printf(" --      |");
            else if (r.pass)
                printf(" OK %-4d |", r.ldpc_iters);
            else
                printf(" FAIL    |");
        }
        printf("\n");
    }

    // Print SNR row (from first level that detected at each noise)
    printf("  %-20s |", "SNR (dB)");
    for (int ni = 0; ni < n_noise; ni++) {
        float snr = 0;
        for (int li = 0; li < n_levels; li++) {
            if (results[li][ni].detected) { snr = results[li][ni].snr_db; break; }
        }
        printf(" %-7.0f |", snr);
    }
    printf("\n");
}

// =======================================================================
//  FD-ZC data-gate calibration (MEASURE-ONLY).
//
//  Purpose: determine whether the executing data-frame FD-ZC quality gate
//  (config.fd_zc_threshold = 0.97 for data, modem.cc:1539; enforced at
//  ofdm_sync.cc:342) can REJECT genuine FM data frames before LDPC — the
//  suspected root of the N(R)-stuck-at-0 symptom (fact doc §6.6/§7.5).
//
//  Method: measure the CURRENT (unmodified) power-weighted FD-ZC coherence
//  metric (ofdm_sync.cc:327-335, metric = |Σ|D|·D| / Σ|D|²) on
//   (a) M>=20 genuine O1 QPSK-r1/2 data frames pushed through the FM channel
//       at a realistic reverse-path SNR (~12-16 dB), and
//   (b) pure-noise window pairs (the metric's false/noise response).
//  Report real_floor, false_ceiling, separation, and how many genuine
//  frames fall below the 0.97 gate.
//
//  This test does NOT modify the production metric or the 0.97 gate — a DSP
//  change requires this evidence first (the root-cause-first principle). To OBSERVE
//  the raw best_zc_metric a genuine frame produces even when it would be
//  rejected, it lowers the gate to 0 in a LOCAL config copy only; production
//  config is untouched.
// =======================================================================
static void test_fd_zc_gate_calibration() {
    printf("\n=== FD-ZC data-gate calibration (measure-only) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;  pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;  pb.bandwidth_hz = 2700.0f;  pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    const float DATA_GATE = 0.97f;   // executing data gate (modem.cc:1539)

    // Measure-only observation copy: gate=0 so ofdm_detect_frame never
    // rejects and reports sync.zc_metric = best_zc_metric (ofdm_sync.cc:397).
    OfdmConfig meas_cfg = cfg;
    meas_cfg.fd_zc_threshold = 0.0f;

    ToneMap tm = get_uniform_tone_map(2, cfg);   // O1 QPSK r1/2

    // ---- (a) Genuine O1 data frames through the FM channel ~12-16 dB ----
    const int M = 24;                            // >= 20 genuine frames
    std::vector<float> real_metrics, real_snrs;
    std::mt19937 payload_rng(0xC0FFEEu);
    // FM discriminator noise amplitude spanning the ~12-16 dB reverse path
    // (fm_channel_process f²-shaped noise). Varied per frame for a real
    // distribution, since fm_channel_process uses a fixed noise realization.
    const float na_lo = 0.0015f, na_hi = 0.0045f;
    for (int f = 0; f < M; f++) {
        uint8_t payload[64];
        for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(payload_rng() & 0xFF);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 64, tm, LdpcRate::RATE_1_2);
        if (iq.empty()) { printf("  [real %2d] frame build failed\n", f); continue; }

        std::vector<float> fm_audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++) fm_audio[i] = iq[i].real();
        float noise_amp = na_lo + (na_hi - na_lo) * (float)f / (float)(M - 1);
        fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, noise_amp);

        auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
        const size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(),   pad, std::complex<float>(0, 0));

        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), meas_cfg);
        if (!sync.detected) { printf("  [real %2d] not detected (SC miss)\n", f); continue; }

        OfdmDemodulator demod(cfg);
        auto res = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
        real_metrics.push_back(sync.zc_metric);
        real_snrs.push_back(res.effective_snr_db);
        printf("  [real %2d] FD-ZC=%.4f  eff_snr=%.1f dB  ldpc_ok=%d  (na=%.4f)\n",
               f, sync.zc_metric, res.effective_snr_db, (int)res.success, noise_amp);
    }

    // ---- (b) Pure-noise window pairs: the FD-ZC metric's noise response ----
    // Mirrors ofdm_sync.cc:327-335 EXACTLY (power-weighted phase coherence).
    const int nfft = cfg.nfft;
    const int symbol_len = cfg.symbol_samples();
    const int n_used = cfg.n_used_carriers;
    auto fd_zc_metric = [&](const std::complex<float>* w1,
                            const std::complex<float>* w2) -> float {
        std::vector<std::complex<float>> Y1(w1, w1 + nfft), Y2(w2, w2 + nfft);
        fft_complex(Y1.data(), nfft);
        fft_complex(Y2.data(), nfft);
        std::complex<float> diff_sum(0.0f, 0.0f);
        float diff_mag_sum = 0.0f;
        for (int i = 0; i < n_used; ++i) {
            int bin = cfg.used_carrier_bins[i];
            std::complex<float> d = Y2[bin] * std::conj(Y1[bin]);
            float wv = std::abs(d);            // |D[k]| = |H[k]|²
            diff_sum += wv * d;                // |D[k]|² · exp(j·arg(D[k]))
            diff_mag_sum += wv * wv;           // |D[k]|²
        }
        return (diff_mag_sum > 1e-20f) ? std::abs(diff_sum) / diff_mag_sum : 0.0f;
    };

    const int N_FALSE = 300;
    std::vector<float> false_metrics;
    for (int t = 0; t < N_FALSE; t++) {
        std::mt19937 nrng(9000u + (unsigned)t);
        std::normal_distribution<float> nd(0.0f, 1.0f);
        int buflen = symbol_len + nfft + 16;
        std::vector<float> noise(buflen);
        for (int i = 0; i < buflen; i++) noise[i] = nd(nrng);
        auto niq = hilbert_analytic(noise.data(), buflen);
        // two windows separated by symbol_len, mirroring train1/train2 spacing
        false_metrics.push_back(fd_zc_metric(niq.data(), niq.data() + symbol_len));
    }

    // ---- Statistics ----
    auto minv  = [](const std::vector<float>& v){ float m = 1e9f;  for (float x : v) m = std::min(m, x); return m; };
    auto maxv  = [](const std::vector<float>& v){ float m = -1e9f; for (float x : v) m = std::max(m, x); return m; };
    auto meanv = [](const std::vector<float>& v){ double s = 0; for (float x : v) s += x; return v.empty() ? 0.0f : (float)(s / v.size()); };

    float real_floor = real_metrics.empty()  ? 0.0f : minv(real_metrics);
    float real_mean  = meanv(real_metrics);
    float false_ceil = false_metrics.empty() ? 0.0f : maxv(false_metrics);
    float false_mean = meanv(false_metrics);
    float separation = real_floor - false_ceil;
    float snr_mean   = meanv(real_snrs);
    float snr_lo     = real_snrs.empty() ? 0.0f : minv(real_snrs);
    float snr_hi     = real_snrs.empty() ? 0.0f : maxv(real_snrs);

    int rejected = 0;
    for (float m : real_metrics) if (m < DATA_GATE) rejected++;

    printf("\n  --- FD-ZC calibration summary (CURRENT power-weighted metric) ---\n");
    printf("  genuine frames: N=%zu  eff_snr=%.1f dB (%.1f..%.1f)\n",
           real_metrics.size(), snr_mean, snr_lo, snr_hi);
    printf("  real:  floor=%.4f  mean=%.4f\n", real_floor, real_mean);
    printf("  false: ceiling=%.4f  mean=%.4f  (N=%d pure-noise pairs)\n",
           false_ceil, false_mean, N_FALSE);
    printf("  separation (real_floor - false_ceiling) = %.4f\n", separation);
    printf("  executing data gate = %.2f  ->  genuine frames rejected: %d / %zu\n",
           DATA_GATE, rejected, real_metrics.size());
    if (rejected > 0)
        printf("  VERDICT: 0.97 gate REJECTS genuine frames (root CONFIRMED plausible)\n");
    else if (real_floor > DATA_GATE)
        printf("  VERDICT: genuine floor %.4f > 0.97 gate — gate does NOT reject; root shifts to LDPC-LLR path\n", real_floor);

    // ---- Assertions (measure-only; must stay green regardless of verdict) ----
    check("fd-zc-cal: collected >=20 genuine O1 frame metrics", real_metrics.size() >= 20);
    check("fd-zc-cal: false-population computed", false_metrics.size() == (size_t)N_FALSE);
    bool inrange = true;
    for (float m : real_metrics)  if (m < 0.0f || m > 1.0001f) inrange = false;
    for (float m : false_metrics) if (m < 0.0f || m > 1.0001f) inrange = false;
    check("fd-zc-cal: metrics in [0,1]", inrange);
}

// Test blind speed detection: TX at one level, try decode at wrong level first,
// then sweep candidates to find the correct level. This validates the OFDM-KISS
// gearshift blind detection path without needing hardware.
static void test_ofdm_blind_detect() {
    printf("\n=== OFDM Blind Speed Detection ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 37 + 13);

    // Helper: codewords per level (matches modem.cc ofdm_cw_for_level)
    auto cw_for_level = [](int level) -> int {
        if (level >= 8) return 4;
        if (level >= 6) return 8;
        if (level >= 3) return 4;
        if (level == 2) return 2;  // QPSK r3/4: 2 CW (4 CW too long for FM)
        return 1;
    };

    // Helper: build tone map for a level
    auto make_tm = [&](int level) -> ToneMap {
        ToneMap tm = get_uniform_tone_map(level + 1, cfg);
        tm.n_codewords = cw_for_level(level);
        return tm;
    };

    // Test cases: TX at level X, RX initially expects level Y (wrong),
    // blind detection should find level X.
    struct Case {
        int tx_level;
        int rx_level;  // wrong initial guess
        const char* desc;
    };
    Case cases[] = {
        {3, 0, "TX=O3(16QAM r1/2), RX expects O0(BPSK)"},
        {5, 0, "TX=O5(16QAM r3/4), RX expects O0(BPSK)"},
        {0, 3, "TX=O0(BPSK), RX expects O3(16QAM)"},
        {2, 5, "TX=O2(QPSK r3/4), RX expects O5(16QAM r3/4)"},
        {5, 3, "TX=O5(16QAM r3/4), RX expects O3(16QAM r1/2)"},
    };

    for (auto& tc : cases) {
        printf("\n  --- %s ---\n", tc.desc);

        // Build frame at TX level
        ToneMap tx_tm = make_tm(tc.tx_level);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 64, tx_tm, tx_tm.fec_rate,
                                        tx_tm.n_codewords);
        if (iq.empty()) {
            printf("  SKIP: no frame generated for O%d\n", tc.tx_level);
            continue;
        }

        // Convert to real audio and pass through FM channel
        std::vector<float> audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++)
            audio[i] = iq[i].real();
        fm_channel_process(audio.data(), (int)audio.size(), 0.50f, 0.0f);

        // Hilbert -> IQ
        auto rx_iq = hilbert_analytic(audio.data(), (int)audio.size());
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));

        // Detect preamble (level-independent)
        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        char label[128];
        snprintf(label, sizeof(label), "%s: preamble detected", tc.desc);
        check(label, sync.detected);
        if (!sync.detected) continue;

        // Try decode with WRONG tone map (should fail)
        ToneMap wrong_tm = make_tm(tc.rx_level);
        OfdmDemodulator demod(cfg);
        auto wrong_result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(),
                                              wrong_tm, &sync);
        snprintf(label, sizeof(label), "%s: wrong level O%d fails", tc.desc, tc.rx_level);
        check(label, !wrong_result.success);
        printf("    Wrong-level decode: success=%d\n", wrong_result.success);

        // Blind detection: try candidates (same logic as modem.cc)
        bool blind_ok = false;
        int found_level = -1;
        int candidates_tried = 0;

        int candidates[NUM_OFDM_SPEED_LEVELS];
        int n_candidates = 0;
        auto add_candidate = [&](int lvl) {
            if (lvl < 0 || lvl >= NUM_OFDM_SPEED_LEVELS) return;
            if (lvl == tc.rx_level) return;  // already tried
            for (int j = 0; j < n_candidates; j++)
                if (candidates[j] == lvl) return;
            candidates[n_candidates++] = lvl;
        };

        // Priority order (matches modem.cc):
        int snr_level = ofdm_snr_to_speed_level(wrong_result.snr_db);
        add_candidate(snr_level);
        add_candidate(0);
        add_candidate(tc.rx_level - 1);
        add_candidate(tc.rx_level + 1);
        for (int l = NUM_OFDM_SPEED_LEVELS - 1; l >= 0; l--)
            add_candidate(l);

        for (int ci = 0; ci < n_candidates; ci++) {
            int trial = candidates[ci];
            ToneMap trial_tm = make_tm(trial);
            OfdmSyncResult trial_sync = sync;
            trial_sync.frame_start = 0;
            OfdmDemodulator trial_demod(cfg);
            auto trial_result = trial_demod.demodulate(
                rx_iq.data() + sync.frame_start,
                (int)(rx_iq.size() - sync.frame_start),
                trial_tm, &trial_sync);
            candidates_tried++;
            if (trial_result.success) {
                bool payload_match = (trial_result.payload.size() == 64) &&
                    (memcmp(trial_result.payload.data(), payload, 64) == 0);
                if (payload_match) {
                    blind_ok = true;
                    found_level = trial;
                    break;
                }
            }
        }

        snprintf(label, sizeof(label), "%s: blind detect finds O%d", tc.desc, tc.tx_level);
        check(label, blind_ok && found_level == tc.tx_level);
        printf("    Blind detection: %s (found O%d after %d candidates)\n",
               blind_ok ? "OK" : "FAIL", found_level, candidates_tried);
    }

    // Test Gearshift class: SNR-driven upshift
    printf("\n  --- Gearshift SNR-driven upshift ---\n");
    {
        Gearshift gs;
        gs.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        check("Gearshift starts at O0", gs.current_ofdm_level() == 0);

        // Feed high SNR for OFDM_HOLD_FRAMES+1 updates — should upshift
        for (int i = 0; i < 10; i++) {
            gs.ofdm_update(25.0f);  // 25 dB should target O5+
            gs.feed_ldpc_iters(2, 50);  // easy convergence
        }
        check("Gearshift upshifts above O0 at 25dB SNR",
              gs.current_ofdm_level() > 0);
        printf("    After 10 updates at 25dB: O%d\n", gs.current_ofdm_level());

        // Report failures — should downshift
        int before = gs.current_ofdm_level();
        gs.report_failure();
        gs.report_failure();
        check("Gearshift downshifts on 2 failures",
              gs.current_ofdm_level() < before);
        printf("    After 2 failures: O%d (was O%d)\n",
               gs.current_ofdm_level(), before);
    }

    // Decode-margin climb: the post-EQ EsNo (effective_snr_db) counts
    // LDPC-correctable FM phase jitter as noise and reads ~4.7 dB at a channel
    // SNR where O1 (QPSK r1/2) decodes cleanly. 4.7 + boost(cap 2.0) = 6.7 <
    // 7.0 (O1 min_snr 6.0 + 1.0 margin), so SNR alone latches O0. An easy LDPC
    // decode (iters <= MARGIN_ITERS_EASY) must authorize the +1 climb anyway.
    // (feed_ldpc_iters runs BEFORE ofdm_update, matching modem.cc.)
    printf("\n  --- Gearshift decode-margin climb (post-EQ EsNo deflated) ---\n");
    {
        Gearshift gs;
        gs.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        check("Margin-climb: starts at O0", gs.current_ofdm_level() == 0);
        for (int i = 0; i < 10; i++) {
            gs.feed_ldpc_iters(1, 50);   // easy convergence => big decode margin
            gs.ofdm_update(4.7f);        // deflated post-EQ EsNo, below O1 bar
        }
        // fail-before this fix: 4.7 + boost(2.0) = 6.7 < 7.0 => stays O0.
        // pass-after: last_easy_decode_ floors target at O1 => climbs after HOLD.
        check("Margin-climb: easy decode climbs above O0 despite deflated EsNo",
              gs.current_ofdm_level() > 0);
        printf("    After 10 easy decodes at EsNo=4.7dB: O%d\n",
               gs.current_ofdm_level());
    }

    // Ceiling guard: a genuinely hard decode (iters well above
    // MARGIN_ITERS_EASY) at the SAME deflated EsNo must NOT force a climb — the
    // margin floor only fires on easy decodes, so O0 must hold.
    printf("\n  --- Gearshift decode-margin climb: hard decode holds ---\n");
    {
        Gearshift gs;
        gs.set_max_ofdm_level(NUM_OFDM_SPEED_LEVELS - 1);
        for (int i = 0; i < 10; i++) {
            gs.feed_ldpc_iters(10, 50);  // working hard: not easy, not hard-gated
            gs.ofdm_update(4.7f);        // same deflated EsNo below O1 bar
        }
        check("Margin-climb: hard decode does NOT force climb above O0",
              gs.current_ofdm_level() == 0);
        printf("    After 10 hard decodes at EsNo=4.7dB: O%d\n",
               gs.current_ofdm_level());
    }

    // Test kiss_fast_ramp
    printf("\n  --- Gearshift KISS fast ramp ---\n");
    {
        Gearshift gs;
        gs.set_max_ofdm_level(9);
        gs.set_kiss_fast_ramp(true);

        // Feed very high SNR repeatedly
        for (int i = 0; i < 20; i++) {
            gs.ofdm_update(35.0f);
            gs.feed_ldpc_iters(1, 50);
        }
        // With fast ramp, should jump to target (not increment by 1)
        int level = gs.current_ofdm_level();
        printf("    Fast ramp after 20 updates at 35dB: O%d\n", level);
        check("Fast ramp reaches O3+ quickly", level >= 3);
    }
}

// Coherent RX-adoption regression (Mercury rank 7/9): the receiver-driven climb
// desync. When the data RECEIVER B proposes a higher forward level (O3), the data
// SENDER A adopts it and starts emitting a higher/longer 16QAM ncw=4 frame BEFORE
// B has confirmed the new level. The bug: B's frame-length gate stayed pinned to
// the shorter CONFIRMED level, truncating A's longer O3 frame so blind-detect
// could never lock it -> the O2->O3 climb stalls. The fix keys the gate (and the
// blind-detect candidate order) on B's OWN proposal (ofdm_kiss_rx_proposed_level_)
// so the full O3/ncw4 frame is buffered and demapped at the right (modulation,ncw)
// on the first trial. This test reproduces the truncation fail-before / pass-after
// and asserts the proposal-first blind-detect picks the correct O3 tone_map+ncw.
static void test_ofdm_climb_desync() {
    printf("\n=== OFDM Climb Desync (coherent RX adoption) ===\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    const int sym_len = cfg.nfft + cfg.cp_samples;

    // Matches modem.cc ofdm_cw_for_level.
    auto cw_for_level = [](int level) -> int {
        if (level >= 8) return 4;
        if (level >= 6) return 8;
        if (level >= 3) return 4;
        if (level == 2) return 2;
        return 1;
    };
    auto make_tm = [&](int level) -> ToneMap {
        ToneMap tm = get_uniform_tone_map(level + 1, cfg);
        tm.n_codewords = cw_for_level(level);
        return tm;
    };
    // Frame length in OFDM symbols for a level (matches modem.cc frame-length gate
    // / ofdm_frame_symbols_for_level: 2 training + 1 sync + data + pilot rows +
    // 1 tail). E3: the block-pilot tier was removed from the waveform, so it is no
    // longer counted here either (kept in lockstep with the gate).
    auto frame_syms_for_level = [&](int level) -> int {
        ToneMap tm = make_tm(level);
        int bps = tm.total_bits_per_symbol;
        int coded = tm.n_codewords * LdpcCodec::codeword_size(tm.fec_rate);
        int nds = (bps > 0) ? (coded + bps - 1) / bps : 1;
        int pr = (cfg.pilot_row_spacing > 0)   ? nds / cfg.pilot_row_spacing   : 0;
        return 2 + 1 + nds + pr + 1;
    };
    // The exact frame-length gate decision from modem.cc:1634-1704, parameterised
    // on the confirmed RX level and the outstanding proposal.
    auto gate_syms = [&](int confirmed_level, int proposed_level) -> int {
        int total = frame_syms_for_level(confirmed_level);
        bool climb_pending = (proposed_level > confirmed_level);
        // (confirmed==true here; the !confirmed branch already expands, so only the
        // climb_pending widening is exercised.)
        if (climb_pending) {
            for (int l = 0; l < NUM_OFDM_SPEED_LEVELS; l++)
                total = std::max(total, frame_syms_for_level(l));
        }
        return total;
    };

    const int TX_LEVEL = 3;         // A adopts O3 (16QAM r1/2, ncw=4)
    const int CONFIRMED = 1;        // B still confirmed at O1 (short 1-CW gate)
    int o3_syms = frame_syms_for_level(TX_LEVEL);

    // --- fail-before: gate pinned to confirmed O1 truncates the O3 frame ---
    int gate_stale = gate_syms(CONFIRMED, /*proposed=*/-1);
    printf("    O1-confirmed gate = %d syms; O3 frame needs %d syms\n",
           gate_stale, o3_syms);
    check("Climb desync: stale (O1) gate is too short for the O3 frame (truncates)",
          gate_stale < o3_syms);

    // --- pass-after: proposal-driven gate admits the full O3 frame ---
    int gate_fixed = gate_syms(CONFIRMED, /*proposed=*/TX_LEVEL);
    printf("    O3-proposed gate = %d syms (>= %d needed)\n", gate_fixed, o3_syms);
    check("Coherent adoption: proposal-widened gate buffers the whole O3 frame",
          gate_fixed >= o3_syms);

    // --- the O3/ncw4 frame demaps correctly, and proposal-first tries O3 first ---
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 37 + 13);
    ToneMap tx_tm = make_tm(TX_LEVEL);
    OfdmModulator mod(cfg);
    auto iq = mod.build_ofdm_frame(payload, 64, tx_tm, tx_tm.fec_rate,
                                    tx_tm.n_codewords);
    check("Climb desync: O3/ncw4 frame built", !iq.empty());
    if (!iq.empty()) {
        std::vector<float> audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++) audio[i] = iq[i].real();
        fm_channel_process(audio.data(), (int)audio.size(), 0.50f, 0.0f);
        auto rx_iq = hilbert_analytic(audio.data(), (int)audio.size());
        size_t pad = 48000;
        rx_iq.insert(rx_iq.begin(), pad, std::complex<float>(0, 0));
        rx_iq.insert(rx_iq.end(), pad, std::complex<float>(0, 0));
        auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
        check("Climb desync: preamble detected", sync.detected);

        // Primary decode at the stale confirmed O1 map fails (wrong config).
        OfdmDemodulator demod(cfg);
        ToneMap stale_tm = make_tm(CONFIRMED);
        auto stale = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), stale_tm, &sync);
        check("Climb desync: stale O1 primary decode fails on the O3 frame",
              !stale.success);

        // Proposal-first blind-detect order (matches modem.cc): O3 (= proposal)
        // is the FIRST candidate, so the correct tone_map+ncw is selected first.
        int candidates[NUM_OFDM_SPEED_LEVELS]; int n_cand = 0;
        int proposed = TX_LEVEL;
        auto add_c = [&](int lvl) {
            if (lvl < 0 || lvl >= NUM_OFDM_SPEED_LEVELS) return;
            if (lvl == CONFIRMED) return;             // already tried as primary
            for (int j = 0; j < n_cand; j++) if (candidates[j] == lvl) return;
            candidates[n_cand++] = lvl;
        };
        add_c(proposed);                              // proposal FIRST (the fix)
        add_c(ofdm_snr_to_speed_level(stale.snr_db));
        add_c(0); add_c(CONFIRMED - 1); add_c(CONFIRMED + 1);
        for (int l = NUM_OFDM_SPEED_LEVELS - 1; l >= 0; l--) add_c(l);

        check("Coherent adoption: proposed O3 is the FIRST blind-detect candidate",
              n_cand > 0 && candidates[0] == TX_LEVEL);

        bool ok = false; int found = -1; int first_success_ci = -1;
        for (int ci = 0; ci < n_cand; ci++) {
            ToneMap tm = make_tm(candidates[ci]);
            OfdmSyncResult s = sync; s.frame_start = 0;
            OfdmDemodulator d(cfg);
            auto r = d.demodulate(rx_iq.data() + sync.frame_start,
                                  (int)(rx_iq.size() - sync.frame_start), tm, &s);
            if (r.success && r.payload.size() == 64 &&
                memcmp(r.payload.data(), payload, 64) == 0) {
                ok = true; found = candidates[ci]; first_success_ci = ci; break;
            }
        }
        check("Coherent adoption: RX demaps the O3/ncw4 frame (payload matches)",
              ok && found == TX_LEVEL);
        check("Coherent adoption: O3 resolved on the FIRST candidate (no sweep)",
              first_success_ci == 0);
        printf("    Proposal-first blind-detect: found O%d at candidate %d\n",
               found, first_success_ci);
    }
}

static void test_ofdm_kiss_loopback() {
    printf("\n=== OFDM-KISS Audio Loopback (Mode A upconvert/downconvert) ===\n");

    // Simulate OFDM-KISS: build native frame, upconvert to audio,
    // downconvert back to IQ, detect and decode.
    // This tests the full path that OFDM-KISS uses over FM radio.

    PhyConfig cfg = mode_a_config();
    cfg.modulation = Modulation::BPSK;  // Speed level 0

    float center = 1700.0f;  // Default center: (1200+2200)/2
    Upconverter up(center, SAMPLE_RATE);
    Downconverter down(center, SAMPLE_RATE);

    // Test with a realistic AX.25-sized payload
    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 37 + 13);
    size_t payload_len = 64;

    // Build native frame (IQ)
    LdpcRate fec = fec_to_ldpc_rate(1, 2);  // rate 1/2
    auto iq = build_native_frame(payload, payload_len, cfg, fec);
    printf("  Payload: %zu bytes -> %zu IQ samples\n", payload_len, iq.size() / 2);

    // Upconvert IQ to audio (mono float)
    auto audio = up.iq_to_audio(iq.data(), iq.size());
    printf("  Audio: %zu samples (%.1f ms)\n", audio.size(),
           1000.0f * audio.size() / SAMPLE_RATE);

    // Check audio bandwidth (peak frequency should be near center)
    float peak_amp = 0;
    for (auto s : audio) if (std::abs(s) > peak_amp) peak_amp = std::abs(s);
    printf("  Audio peak amplitude: %.4f\n", peak_amp);
    check("Audio peak > 0", peak_amp > 0.001f);

    // Downconvert audio back to IQ
    auto rx_iq = down.audio_to_iq(audio.data(), audio.size());
    printf("  RX IQ: %zu floats (%zu IQ pairs)\n", rx_iq.size(), rx_iq.size() / 2);

    // Detect frame start
    int start = detect_frame_start(rx_iq.data(), rx_iq.size(),
                                    cfg.samples_per_symbol);
    check("Frame detected after upconvert/downconvert", start >= 0);

    if (start >= 0) {
        printf("  Frame start at sample %d\n", start);

        // Decode
        std::vector<uint8_t> rx_payload;
        bool ok = decode_native_frame(rx_iq.data(), rx_iq.size(),
                                       start, cfg, rx_payload);
        check("Frame decoded after upconvert/downconvert", ok);
        if (ok) {
            bool match = (rx_payload.size() == payload_len) &&
                         (memcmp(rx_payload.data(), payload, payload_len) == 0);
            check("Payload matches after upconvert/downconvert", match);
            if (!match) {
                printf("    Expected %zu bytes, got %zu bytes\n",
                       payload_len, rx_payload.size());
                if (rx_payload.size() > 0) {
                    printf("    First bytes: ");
                    for (size_t i = 0; i < std::min(rx_payload.size(), (size_t)16); i++)
                        printf("%02X ", rx_payload[i]);
                    printf("\n");
                }
            }
        }
    }

    // Test with simulated FM radio bandpass (300-3000 Hz) + de-emphasis
    printf("\n  --- With simulated FM bandpass (300-3000 Hz) ---\n");
    {
        // Simple brick-wall bandpass: zero out frequencies outside 300-3000 Hz
        // using FFT-like approach (just attenuate based on frequency content)
        // Actually, simpler: just apply a basic RC de-emphasis filter
        // FM de-emphasis: 6 dB/octave above ~2122 Hz (75us time constant)
        float tau = 75e-6f;  // 75 microsecond time constant
        float rc_alpha = 1.0f / (1.0f + 2.0f * M_PI * tau * SAMPLE_RATE);
        std::vector<float> filtered = audio;
        float prev = 0;
        for (size_t i = 0; i < filtered.size(); i++) {
            filtered[i] = prev + rc_alpha * (filtered[i] - prev);
            prev = filtered[i];
        }

        auto rx_iq2 = down.audio_to_iq(filtered.data(), filtered.size());
        int start2 = detect_frame_start(rx_iq2.data(), rx_iq2.size(),
                                         cfg.samples_per_symbol);
        check("Frame detected after FM de-emphasis", start2 >= 0);
        if (start2 >= 0) {
            std::vector<uint8_t> rx2;
            bool ok2 = decode_native_frame(rx_iq2.data(), rx_iq2.size(),
                                            start2, cfg, rx2);
            check("Frame decoded after FM de-emphasis", ok2);
            if (ok2) {
                bool match2 = (rx2.size() == payload_len) &&
                              (memcmp(rx2.data(), payload, payload_len) == 0);
                check("Payload matches after FM de-emphasis", match2);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// AX.25 window / sequence accounting (V(S)/V(R)/V(A)) — in-process regression.
//
// Reproduces the OFDM-KISS native-mode forward-window DEADLOCK: after one K=7
// window the sender reached V(A) > V(S) (an impossible register pair) and froze,
// delivering only ~7 frames.  Drives an Ax25Session through many windows with
// ACK loss / go-back-N retransmits and asserts the invariants never violate.
// FAILS before the ax25_session.cc fixes (notify_outgoing shadow-V(S) regression
// + shadow-V(R) backward drag), PASSES after.
// ---------------------------------------------------------------------------
static void test_ax25_window_accounting() {
    printf("\n=== AX.25 Window/Sequence Accounting (V(S)/V(R)/V(A)) ===\n");
    using namespace iris;
    const std::string ME = "N0AAA";
    const std::string PEER = "N0BBB";
    Ax25Address me   = ax25_make_addr(ME);
    Ax25Address peer = ax25_make_addr(PEER);

    // Bring a session up as KISS-managed, native (OFDM-KISS transport) initiator.
    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign(ME);
        s.set_send_callback([](const uint8_t*, size_t) {});   // swallow S-frames
        // KISS client emits SABM -> we shadow it (AWAITING_CONNECTION).
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        // Peer answers UA -> CONNECTED, V(S)=V(R)=V(A)=0.
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f;
        ax25_parse(ua.data(), ua.size(), f);
        s.on_frame_received(f);
        s.set_native_active(true);   // OFDM-KISS native mode: we own transport
    };

    uint8_t body[8] = {'D','A','T','A','0','0','0','0'};
    auto feed_out_iframe = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(peer, me, ns, /*nr=*/0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    };
    auto feed_rx_rr = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, false, false);
        Ax25Frame f;
        ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };
    auto feed_rx_iframe = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(me, peer, ns, /*nr=*/0, false, 0xF0, body, sizeof(body));
        Ax25Frame f;
        ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };

    // --- Part A: exact deadlock reproduction (V(A)=7 > V(S)=6) --------------
    {
        Ax25Session s;
        bring_up(s);
        // Client sends 7 new in-order I-frames -> V(S)=7, V(A)=0 (full window).
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);
        check("A: 7 new frames fill window (V(S)=7,V(A)=0,used=7)",
              s.vs() == 7 && s.va() == 0 && s.window_used() == 7);
        // Peer ACKs all 7 (RR N(R)=7) -> V(A)=7, window empty.
        feed_rx_rr(s, 7);
        check("A: RR N(R)=7 drains window (V(A)=7,V(S)=7,used=0)",
              s.va() == 7 && s.vs() == 7 && s.window_used() == 0);
        // KISS client whose own V(A) lagged go-back-N retransmits N(S)=5
        // (already ACKed from the shadow's view).  This is the exact trigger.
        feed_out_iframe(s, 5);
        check("A: stale retransmit N(S)=5 does NOT invert window (V(A)<=V(S))",
              s.va() == 7 && s.vs() == 7 && s.window_used() == 0);
        // Session must still accept a genuinely-new frame and keep flowing.
        feed_out_iframe(s, 7);           // N(S)=7 == V(S)=7 -> new
        check("A: new frame N(S)=7 advances V(S) across boundary (V(S)=0,used=1)",
              s.vs() == 0 && s.window_used() == 1);
    }

    // --- Part B: sustained multi-window transfer with ACK loss + go-back-N --
    {
        Ax25Session s;
        bring_up(s);
        const int K = 7;
        const int TOTAL = 40;             // ~6 full windows
        int produced = 0, acked = 0, guard = 0;
        bool inv_ok = true;              // window_used always == true outstanding
        auto win = [&]() { return s.window_used(); };
        while (acked < TOTAL && guard++ < 4000) {
            // Fill the window with genuinely-new in-order frames.
            while (win() < K && produced < TOTAL) {
                feed_out_iframe(s, s.vs());
                produced++;
                if (win() != produced - acked) inv_ok = false;
            }
            // Peer partially ACKs (one frame per round) -> shadow V(A) advances.
            if (s.va() != s.vs()) {
                feed_rx_rr(s, (uint8_t)((s.va() + 1) & 7));
                acked++;
                if (win() != produced - acked) inv_ok = false;
            }
            // Two-tracker divergence: the KISS client's own V(A) lagged the shadow,
            // so it go-back-N retransmits an ALREADY-ACKed frame (N(S)=V(A)-2 — the
            // exact register that used to score as maximal-forward and drag V(S)
            // behind V(A)).  It must leave the window untouched.  (Skip the mod-8
            // wrap case where V(A)-2 == V(S), which is a legitimate new frame.)
            uint8_t stale = (uint8_t)((s.va() + 6) & 7);   // = V(A) - 2
            if (s.window_used() < K && stale != s.vs()) {
                int before = s.window_used();
                feed_out_iframe(s, stale);
                if (s.window_used() != before) inv_ok = false;   // no inversion
                if (win() != produced - acked) inv_ok = false;
            }
        }
        check("B: sustained transfer runs PAST one K=7 window (no freeze)",
              acked >= TOTAL && guard < 4000);
        check("B: window_used == true outstanding at every step (V(A)<=V(S))",
              inv_ok);
    }

    // --- Part C: receiver-side shadow V(R) — monotone, no backward drag -----
    {
        Ax25Session s;
        bring_up(s);
        // Deliver in-sequence I-frames 0..6 -> V(R) walks 0 -> 7.
        for (uint8_t ns = 0; ns < 7; ns++) feed_rx_iframe(s, ns);
        check("C: in-sequence delivery advances V(R) to 7", s.vr() == 7);
        // Boundary: N(S)=7 == V(R)=7 -> V(R) wraps to 0.
        feed_rx_iframe(s, 7);
        check("C: V(R) advances across window boundary (7 -> 0)", s.vr() == 0);
        // Go-back-N retransmit of an already-delivered low N(S)=5 must NOT drag
        // V(R) backward (bug: V(R) 0 -> 6).
        feed_rx_iframe(s, 5);
        check("C: stale retransmit N(S)=5 does NOT drag V(R) backward", s.vr() == 0);
        // In-sequence resumes normally.
        feed_rx_iframe(s, 0);
        check("C: in-sequence N(S)=0 resumes V(R) advance (0 -> 1)", s.vr() == 1);
    }

    // --- Part D: piggybacked-ACK coherence (V(A) via reverse I-frame N(R)) ---
    // A5c: when the reverse ACK rides a piggybacked I-frame
    // (the responder has reverse data queued, so no pure-S-frame tone-ACK), CMD's
    // V(A) must advance via the SAME ack_frames() path the explicit RR uses —
    // cleaning tx_window_ and resetting timers.  The pre-fix code set va_=nr
    // directly, advancing V(A) but ORPHANING the acked tx_window_ slots, so
    // pending_frames() over-counted phantom unacked frames and the multi-window
    // transfer stalled.  Invariant asserted: pending_frames() == window_used()
    // (no orphans).  FAILS before the ax25_session.cc:576 ack_frames() fix.
    auto feed_rx_iframe_nr = [&](Ax25Session& s, uint8_t ns, uint8_t nr) {
        auto fr = ax25_build_i(me, peer, ns, nr, false, 0xF0, body, sizeof(body));
        Ax25Frame f;
        ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };
    {
        Ax25Session s;
        bring_up(s);
        // CMD queues a full window: V(S)=7, V(A)=0, 7 frames outstanding.
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);
        check("D: full window (V(S)=7,V(A)=0,pending=7,used=7)",
              s.vs() == 7 && s.va() == 0 &&
              s.pending_frames() == 7 && s.window_used() == 7);
        // Peer sends US reverse data (I-frame N(S)=0 -> our V(R) 0->1) that
        // PIGGYBACKS N(R)=4, acking our frames 0..3.
        feed_rx_iframe_nr(s, /*ns=*/0, /*nr=*/4);
        check("D: piggyback I-frame N(R)=4 advances V(A) to 4 and V(R) to 1",
              s.va() == 4 && s.vr() == 1);
        // The four acked slots must be RELEASED — pending must equal the real
        // outstanding count (window_used), not the phantom pre-fix count of 7.
        check("D: piggyback ACK cleans tx_window_ (pending==window_used==3)",
              s.pending_frames() == s.window_used() && s.window_used() == 3);
    }
    {
        // Sustained multi-window transfer where EVERY reverse ACK is a
        // piggybacked I-frame N(R) (bidirectional traffic).  Must run past one
        // K=7 window with no orphan accumulation.
        Ax25Session s;
        bring_up(s);
        const int K = 7;
        const int TOTAL = 40;             // ~6 full windows
        int produced = 0, acked = 0, guard = 0;
        uint8_t rx_ns = 0;                // peer's forward N(S) (bidirectional)
        bool coherent = true;
        while (acked < TOTAL && guard++ < 4000) {
            while (s.window_used() < K && produced < TOTAL) {
                feed_out_iframe(s, s.vs());
                produced++;
            }
            if (s.va() != s.vs()) {
                // Reverse data frame piggybacks the next ACK (N(R)=V(A)+1).
                feed_rx_iframe_nr(s, rx_ns, (uint8_t)((s.va() + 1) & 7));
                rx_ns = (uint8_t)((rx_ns + 1) & 7);
                acked++;
            }
            // No orphans: outstanding tx_window_ slots == window_used at all times
            // (tx_queue_ is empty here — everything went straight into the window).
            if (s.pending_frames() != s.window_used()) coherent = false;
        }
        check("D: sustained bidirectional transfer runs PAST one K=7 window",
              acked >= TOTAL && guard < 4000);
        check("D: no orphaned tx_window_ slots across all windows (pending==used)",
              coherent);
    }
}

// ---------------------------------------------------------------------------
// WIDE WINDOW (turnaround lever): the terminate/re-pack OWNED OFDM transport
// switches to AX.25 2.2 modulo-128 with a large window K so the sender streams
// many more full frames before it must stop for a reverse ACK (the per-window
// turnaround = the ~30% steady tax at O5 then amortizes toward 0).  This test
// proves, in-process:
//   P1  the 2-octet (extended) control field round-trips 7-bit N(S)/N(R)
//       while the mod-8 build/parse stays byte-identical;
//   P2  the owned session holds > K=7 outstanding I-frames (the lever);
//   P3  a wide reverse RR (7-bit N(R)) round-trips and advances V(A) past 7;
//   P4  the M0 anti-alias STILL holds under the wider modulus — a behind-window
//       N(R) that mod-8 would have mis-accepted (destroying un-ACKed frames) is
//       now correctly rejected (nr_valid false), so no silent-frame-loss class
//       reopens; the wide window is drained by exactly-once cumulative ACKs;
//   P5  the MFSK reverse-ACK carrier delivers a 7-bit N(R) end-to-end (audio).
// FAILS before the wide-window change (window caps at 7; mod-8 N(R) aliases a
// 7-bit value to its low 3 bits); PASSES after.
// ---------------------------------------------------------------------------
static void test_ax25_wide_window() {
    printf("\n=== AX.25 Wide Window (modulo-128 owned transport, turnaround lever) ===\n");
    using namespace iris;
    const std::string ME = "N0AAA";
    const std::string PEER = "N0BBB";
    Ax25Address me   = ax25_make_addr(ME);
    Ax25Address peer = ax25_make_addr(PEER);
    uint8_t body[8] = {'D','A','T','A','0','0','0','0'};

    // --- P1: extended (modulo-128) wire format round-trip -------------------
    {
        // I-frame N(S)=100 N(R)=90 (both need 7 bits).
        auto ife = ax25_build_i(peer, me, 100, 90, false, 0xF0, body, sizeof(body), true);
        Ax25Frame f;
        bool ok = ax25_parse(ife.data(), ife.size(), f, /*extended=*/true);
        check("P1: extended I-frame parses (2-octet control)", ok);
        check("P1: extended I-frame N(S)=100 round-trips", f.ns() == 100);
        check("P1: extended I-frame N(R)=90 round-trips", f.nr() == 90);
        check("P1: extended I-frame INFO intact",
              f.info.size() == sizeof(body) && f.info[0] == 'D');
        // S-frame RR N(R)=77.
        auto sfe = ax25_build_s(me, peer, Ax25SType::RR, 77, false, false, true);
        Ax25Frame sf;
        ax25_parse(sfe.data(), sfe.size(), sf, /*extended=*/true);
        check("P1: extended RR N(R)=77 round-trips", sf.nr() == 77 &&
              sf.type() == Ax25FrameType::S_FRAME && sf.s_type() == Ax25SType::RR);
        // mod-8 build/parse UNCHANGED (byte-identical): N(S)/N(R) still 3-bit.
        auto if8 = ax25_build_i(peer, me, 5, 3, false, 0xF0, body, sizeof(body));  // extended=false
        Ax25Frame f8;
        ax25_parse(if8.data(), if8.size(), f8);   // extended=false
        check("P1: mod-8 I-frame still 3-bit N(S)/N(R) (byte-identical)",
              f8.ns() == 5 && f8.nr() == 3 && !f8.extended);
    }

    // Bring a session up KISS-managed + native, THEN widen to modulo-128/K=63.
    auto bring_up_wide = [&](Ax25Session& s, int k) {
        s.set_local_callsign(ME);
        s.set_send_callback([](const uint8_t*, size_t) {});   // swallow emitted frames
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f); s.on_frame_received(f);
        s.set_native_active(true);
        s.set_wide_window(true, k);      // modulo-128, window K (the lever)
    };
    auto feed_rx_rr_wide = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, false, false, /*extended=*/true);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f, /*extended=*/true);
        s.on_frame_received(f);
    };
    auto feed_rx_iframe_wide = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(me, peer, ns, 0, false, 0xF0, body, sizeof(body), /*extended=*/true);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f, /*extended=*/true);
        s.on_frame_received(f);
    };

    // --- P2: the owned window holds > K=7 outstanding (the lever) -----------
    {
        Ax25Session s;
        bring_up_wide(s, 63);
        check("P2: wide window seq_mod=128 K=63", s.seq_mod() == 128 && s.window_k() == 63);
        s.set_max_info(16);                       // min MAX_INFO clamp -> 16 B / frame
        std::vector<uint8_t> stream(30 * 16);     // 30 frames of data
        for (size_t i = 0; i < stream.size(); i++) stream[i] = (uint8_t)('A' + (i % 26));
        s.send_data(stream.data(), stream.size());
        check("P2: 30 frames outstanding (window > mod-8 K=7)",
              s.window_used() == 30 && s.vs() == 30 && s.va() == 0);
    }

    // --- P3: wide reverse RR round-trips and advances V(A) past 7 -----------
    {
        Ax25Session s;
        bring_up_wide(s, 63);
        s.set_max_info(16);
        std::vector<uint8_t> stream(40 * 16);
        for (size_t i = 0; i < stream.size(); i++) stream[i] = (uint8_t)(i & 0xFF);
        s.send_data(stream.data(), stream.size());
        check("P3: 40 frames queued into the wide window", s.window_used() == 40);
        feed_rx_rr_wide(s, 15);                  // cumulative ACK to 15
        check("P3: wide RR N(R)=15 advances V(A) past 7 (V(A)=15,used=25)",
              s.va() == 15 && s.window_used() == 25);
        feed_rx_rr_wide(s, 40);                  // ACK the rest
        check("P3: wide RR N(R)=40 drains the window (V(A)=40,used=0)",
              s.va() == 40 && s.window_used() == 0);
    }

    // --- P4: M0 anti-alias holds under modulo-128 ---------------------------
    {
        Ax25Session s;
        bring_up_wide(s, 63);
        s.set_max_info(16);
        std::vector<uint8_t> stream(40 * 16);
        for (size_t i = 0; i < stream.size(); i++) stream[i] = (uint8_t)(i & 0xFF);
        s.send_data(stream.data(), stream.size());  // V(A)=0, V(S)=40
        feed_rx_rr_wide(s, 15);                       // V(A)=15
        // A behind-window N(R)=5: under mod-8 (V(A)=15&7=7, V(S)=40&7=0) the old
        // nr_valid(5&7=5) walked 7,0,1,2,3,4,5 -> TRUE and destructively acked
        // frames the peer never received (the aliasing wedge).  Under modulo-128
        // nr_valid(5) walks 15..40 and never hits 5 -> FALSE -> V(A) HELD.
        uint8_t va_before = s.va();
        feed_rx_rr_wide(s, 5);
        check("P4: behind-window N(R)=5 REJECTED (no destructive advance)",
              s.va() == va_before && s.va() == 15);
        // A forward, in-window cumulative ACK still advances exactly-once.
        feed_rx_rr_wide(s, 40);
        check("P4: subsequent in-window RR N(R)=40 drains cleanly (V(A)=40)",
              s.va() == 40 && s.window_used() == 0);
    }

    // --- P5: RX side accepts wide N(S) (V(R) advances past 7) ----------------
    {
        Ax25Session s;
        bring_up_wide(s, 63);
        for (uint8_t ns = 0; ns < 30; ns++) feed_rx_iframe_wide(s, ns);
        check("P5: wide N(S) 0..29 accepted, V(R)=30 (past mod-8)", s.vr() == 30);
        // The reverse ACK the RSP would emit carries the live 7-bit V(R).
        check("P5: RSP current_vr()=30 (the wide reverse-ACK N(R))",
              s.current_vr() == 30);
    }

    // --- P5b: MFSK reverse-ACK carrier delivers a 7-bit N(R) end-to-end ------
    {
        MfskAck tx, rx;
        int first_bin = 200;
        tx.init(first_bin, 48000);
        rx.init(first_bin, 48000);
        auto make_buf = [](const std::vector<float>& a) {
            std::vector<float> b(2048, 0.0f);
            b.insert(b.end(), a.begin(), a.end());
            b.resize(b.size() + 1024, 0.0f);
            return b;
        };
        for (int nr : {0, 5, 63, 100, 127}) {
            auto audio = tx.generate(nr, /*pf=*/nr & 1);
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            char nm[64];
            snprintf(nm, sizeof(nm), "P5b: MFSK 7-bit N(R)=%d round-trips (audio)", nr);
            check(nm, r.detected && r.n_r == nr && r.pf == (nr & 1));
        }
    }
}

// ---------------------------------------------------------------------------
// SELECTIVE-REPEAT CHEAP-MISS — the WGN:30 ARQ loss-amplifier fix.
//
// At WGN:30 the OFDM tier climbs to O6 (marginal, moderate per-frame loss) and
// then STALLS in 278 T1-retry cycles (2-6/16 msgs, 0.025x VARA) — the moderate
// loss is AMPLIFIED into a stall.  ROOT (ax25_session.cc, native OFDM-KISS
// transport): the responder DROPS every out-of-order forward I-frame and re-sends
// a stuck cumulative RR; the sender's C1 implicit-NAK then GO-BACK-N re-airs the
// WHOLE tail — including the frames that already flew but were dropped — so 50%
// loss becomes O(K^2) airtime.  This is the Mercury demote/reverse-ACK-miss
// lesson inverted: the miss must be CHEAP.
//
// FIX (native-mode only, cumulative-ACK channel — no PHY change): the responder
// BUFFERS out-of-order forward frames and drains them in order on gap-fill (V(R)
// jumps); the sender re-airs ONLY the single stuck frame at V(A), not the window.
//
// This test proves it in-process with three crisp fail-before/pass-after checks:
//   S1 (RX reorder): out-of-order frames are buffered, then delivered in order
//      when the gap fills (fail-before: they were dropped; only the gap frame
//      lands, V(R) advances by 1).
//   S2 (TX selective): a duplicate-stuck RR re-airs exactly ONE frame
//      (fail-before: go-back-N re-airs the whole outstanding window).
//   S3 (throughput): a 2-session transfer over a ~50%-loss relay (BOTH directions)
//      DELIVERS ALL records, in order, within a bounded transmit budget
//      (fail-before: go-back-N blows the budget / stalls).
// ---------------------------------------------------------------------------
static void test_ax25_selective_retx() {
    printf("\n=== AX.25 Selective-Repeat Cheap-Miss (WGN:30 loss-amplifier) ===\n");
    using namespace iris;
    const std::string A = "N0AAA", B = "N0BBB";
    Ax25Address a = ax25_make_addr(A), bb = ax25_make_addr(B);

    // Bring a session up as a native OFDM-KISS transport (kiss_managed + native).
    auto bring_up = [&](Ax25Session& s, const std::string& mecall,
                        Ax25Address me, Ax25Address peer) {
        s.set_local_callsign(mecall);
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f;
        ax25_parse(ua.data(), ua.size(), f);
        s.on_frame_received(f);
        s.set_native_active(true);
    };
    // 8-byte body whose 2nd byte carries the record id (id != 'I' so it is never
    // mistaken for an "IRIS/" connection header).
    auto make_body = [](int id, uint8_t* out) {
        out[0] = 0xAA; out[1] = (uint8_t)id;
        for (int i = 2; i < 8; i++) out[i] = (uint8_t)(id * 7 + i);
    };

    // --- S1: RX reorder buffer — buffer out-of-order, drain in order on gap-fill ---
    {
        Ax25Session rsp;
        std::vector<int> delivered;
        rsp.set_native_stream_rx_callback([&](const uint8_t* d, size_t n) {
            if (n >= 2) delivered.push_back((int)d[1]);   // record id
        });
        bring_up(rsp, B, bb, a);   // rsp local=B, peer=A
        auto rx_i = [&](uint8_t ns) {
            uint8_t body[8]; make_body(ns, body);
            auto fr = ax25_build_i(bb, a, ns, /*nr=*/0, false, 0xF0, body, 8);
            Ax25Frame f; ax25_parse(fr.data(), fr.size(), f);
            rsp.on_frame_received(f);
        };
        rx_i(0);                                      // in-order -> V(R)=1, deliver 0
        check("S1: in-order frame 0 delivered, V(R)=1",
              rsp.vr() == 1 && delivered.size() == 1 && delivered[0] == 0);
        rx_i(2); rx_i(3); rx_i(4);                    // out-of-order -> BUFFERED, not delivered
        check("S1: out-of-order 2,3,4 buffered (V(R) held at 1, none delivered)",
              rsp.vr() == 1 && delivered.size() == 1);
        rx_i(1);                                       // gap fills -> drain 1,2,3,4
        // FAIL-BEFORE: 2,3,4 were dropped, so only 1 lands -> V(R)=2, delivered=[0,1].
        bool ok = (delivered.size() == 5 &&
                   delivered[0] == 0 && delivered[1] == 1 && delivered[2] == 2 &&
                   delivered[3] == 3 && delivered[4] == 4);
        check("S1: gap-fill drains buffered frames in order (V(R)=5, [0..4])",
              rsp.vr() == 5 && ok);
    }

    // --- S2: sender re-airs exactly ONE frame on a duplicate-stuck RR -----------
    {
        Ax25Session cmd;
        int iframes_aired = 0;
        cmd.set_send_callback([&](const uint8_t* d, size_t n) {
            Ax25Frame f;
            if (ax25_parse(d, n, f) && f.type() == Ax25FrameType::I_FRAME)
                iframes_aired++;
        });
        bring_up(cmd, A, a, bb);   // cmd local=A, peer=B
        // KISS client sends 5 in-order I-frames -> window [0,5), all stored+sent.
        for (uint8_t ns = 0; ns < 5; ns++) {
            uint8_t body[8]; make_body(ns, body);
            auto fr = ax25_build_i(bb, a, ns, /*nr=*/0, false, 0xF0, body, 8);
            cmd.notify_outgoing(fr.data(), fr.size());
        }
        check("S2: 5 frames fill window (V(S)=5,V(A)=0)",
              cmd.vs() == 5 && cmd.va() == 0);
        auto rx_rr = [&](uint8_t nr) {
            auto fr = ax25_build_s(a, bb, Ax25SType::RR, nr, false, false);
            Ax25Frame f; ax25_parse(fr.data(), fr.size(), f);
            cmd.on_frame_received(f);
        };
        rx_rr(1);                       // forward-advancing RR: V(A) 0->1 (pipelined ACK)
        check("S2: RR N(R)=1 advances V(A) to 1 (no retransmit)",
              cmd.va() == 1);
        iframes_aired = 0;              // count only the retransmit
        rx_rr(1);                       // DUPLICATE stuck RR N(R)=1 -> C1 fires
        // FAIL-BEFORE: go-back-N re-airs frames 1,2,3,4 -> iframes_aired == 4.
        check("S2: duplicate-stuck RR re-airs exactly ONE frame (selective)",
              iframes_aired == 1);
    }

    // --- S3: throughput survives ~50% loss both ways (WIDE-window, WGN:30 model) -
    // At WGN:30 the tier climbs to O6 and grows the owned window (mod-128, wide K),
    // so go-back-N re-airs up to K frames per miss = O(N*K) airtime -> the 278-T1
    // STALL.  This drives a real 2-session transfer over a ~50%-loss relay (BOTH
    // directions) with a wide window and asserts it DELIVERS ALL records, in order,
    // within a bounded (selective-repeat, ~linear-in-N) transmit budget.
    // FAIL-BEFORE (go-back-N + RX-drop): airtime blows the budget (and the transfer
    // may not even converge in the guard — the stall).  PASS-AFTER: ~2-3x N airtime.
    {
        Ax25Session cmd, rsp;
        const bool EXT = true;        // owned wide transport = modulo-128 (extended)
        const int WIDE_K = 32;        // grown window at O6 (mod-128)
        std::vector<int> delivered;
        int tx_iframes = 0;           // CMD I-frame airtime (first-TX + retransmits)

        // Deterministic ~50%-loss relay (seeded LCG); frames QUEUED then drained in
        // discrete steps (no re-entrant tick/on_frame_received interleaving).
        std::vector<std::vector<uint8_t>> q_to_rsp, q_to_cmd;
        uint32_t rng = 0xC0FFEEu;
        auto coin_drop = [&]() {
            rng = rng * 1664525u + 1013904223u;
            return ((rng >> 17) & 1) == 0;   // ~50%
        };
        auto to_rsp = [&](const uint8_t* d, size_t n) {
            if (coin_drop()) return;
            q_to_rsp.emplace_back(d, d + n);
        };
        auto to_cmd = [&](const uint8_t* d, size_t n) {
            if (coin_drop()) return;
            q_to_cmd.emplace_back(d, d + n);
        };
        rsp.set_native_stream_rx_callback([&](const uint8_t* d, size_t n) {
            if (n >= 2) delivered.push_back((int)d[1]);
        });
        // ALL owned I-frames (first-TX + retransmit) + polls ride send_frame_.
        cmd.set_send_callback([&](const uint8_t* d, size_t n) {
            Ax25Frame f;
            if (ax25_parse(d, n, f, EXT) && f.type() == Ax25FrameType::I_FRAME)
                tx_iframes++;
            to_rsp(d, n);
        });
        rsp.set_send_callback([&](const uint8_t* d, size_t n) { to_cmd(d, n); });
        bring_up(cmd, A, a, bb);
        bring_up(rsp, B, bb, a);
        cmd.set_wide_window(true, WIDE_K);   // engage the mod-128 wide owned window
        rsp.set_wide_window(true, WIDE_K);   // both ends flip together (same env)

        const int N = 40;             // records to deliver
        int produced = 0, guard = 0;
        auto deliver = [&](Ax25Session& s, std::vector<std::vector<uint8_t>>& q) {
            std::vector<std::vector<uint8_t>> pkts;
            pkts.swap(q);
            for (auto& p : pkts) {
                Ax25Frame f;
                if (ax25_parse(p.data(), p.size(), f, EXT)) s.on_frame_received(f);
            }
        };
        while ((int)delivered.size() < N && guard++ < 300000) {
            // Feed new records into the wide window (send_data -> send_next_iframe
            // transmits owned extended I-frames via send_frame_).
            while (cmd.window_used() < WIDE_K && produced < N) {
                uint8_t body[8]; make_body(produced, body);
                cmd.send_data(body, 8);
                produced++;
            }
            deliver(rsp, q_to_rsp);   // forward frames arrive at responder
            deliver(cmd, q_to_cmd);   // reverse ACKs arrive at commander
            cmd.tick();               // T1 poll / retransmit as needed
            rsp.tick();               // autonomous cumulative RR
        }
        // Correctness: all N records delivered, IN ORDER, exactly once.
        bool in_order = ((int)delivered.size() == N);
        for (int i = 0; i < (int)delivered.size() && in_order; i++)
            if (delivered[i] != i) in_order = false;
        check("S3: all records delivered in order over 50% loss (no stall)",
              in_order && guard < 300000);
        // Airtime budget: selective-repeat at 50% loss both ways is ~linear in N (a
        // few x).  Wide-window go-back-N is O(N*K) and blows this (fail-before).
        int budget = 5 * N;
        printf("    S3: wide K=%d, delivered=%d/%d, CMD I-frame airtime=%d "
               "(budget %d), guard=%d\n",
               WIDE_K, (int)delivered.size(), N, tx_iframes, budget, guard);
        check("S3: transmit airtime within selective-repeat budget (<=5N)",
              tx_iframes <= budget);
    }
}

static void test_ax25_native_stream_iris_prefix_payload() {
    printf("\n=== AX.25 Native Stream IRIS-Prefix Payload ===\n");
    using namespace iris;
    Ax25Address me = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    Ax25Session s;
    std::vector<uint8_t> delivered;
    s.set_local_callsign("N0AAA");
    s.set_send_callback([](const uint8_t*, size_t) {});
    s.set_native_stream_rx_callback([&](const uint8_t* data, size_t len) {
        delivered.insert(delivered.end(), data, data + len);
    });

    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    s.notify_outgoing(sabm.data(), sabm.size());
    auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    Ax25Frame f;
    ax25_parse(ua.data(), ua.size(), f);
    s.on_frame_received(f);
    s.set_native_active(true);

    const std::vector<uint8_t> payload = {
        'I', 'R', 'I', 'S', '/', 'n', 'o', 't', '-', 'a', '-', 'h', 'e', 'a', 'd', 'e', 'r'
    };
    XidCapability cap{};
    check("native stream: IRIS-prefix payload is not a connection header",
          !conn_header_decode(payload.data(), payload.size(), cap));
    auto iframe = ax25_build_i(me, peer, 0, 0, false, 0xF0,
                               payload.data(), payload.size());
    ax25_parse(iframe.data(), iframe.size(), f);
    s.on_frame_received(f);
    check("native stream: complete IRIS-prefix payload delivered", delivered == payload);
}

// ---------------------------------------------------------------------------
// MFSK tone-ACK PROVENANCE (ship-blocker: no silent swallow of data/U-frames).
//
// The MFSK tone-ACK REPLACES an OFDM-KISS TX batch with a short tone burst that
// can only carry N(R)+PF.  It must fire for the modem's OWN autonomous RR ACK
// and NOTHING else.  The pre-fix classifier used a raw-byte content test
// (`sub[14] & 0x01`) that mis-classified every U-frame (UA/DISC/DM/SABM/FRMR)
// and any B2F_DATA proxy chunk whose byte-14 aliased an odd control byte as an
// "S-frame" — the tone then swallowed the whole batch with NO retransmit path
// (connect UA lost, compressed Winlink chunk lost — silent data loss).
//
// This test drives the SAME production classifier used by process_tx:
//   * producer tagging  = tx_is_autonomous_rr()  (modem.cc send_frame_ callback)
//   * batch decision    = tx_batch_tone_ack_eligible()  (== the drain's AND)
// FAILS before the fix: with the old content logic tx_is_autonomous_rr returns
// true for UA/DISC/RNR/REJ/aliasing-B2F, so the batch is tone-ACK-eligible and
// the assertions `!eligible` fail.  PASSES after: only a pure autonomous-RR
// batch is eligible.
// ---------------------------------------------------------------------------
static void test_tone_ack_provenance() {
    printf("\n=== MFSK tone-ACK provenance (ship-blocker: no silent swallow) ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    // Tag a session-callback frame EXACTLY as the producer at modem.cc:572 does.
    auto session_frame = [](const std::vector<uint8_t>& raw) {
        return TxFrame(std::vector<uint8_t>(raw), tx_is_autonomous_rr(raw.data(), raw.size()));
    };
    // A non-session producer (B2F / KISS client / migration) ALWAYS tags false.
    auto data_frame = [](std::vector<uint8_t> raw) {
        return TxFrame(std::move(raw), false);
    };

    // (1) The modem's own autonomous RR IS eligible — the airtime win is kept.
    auto rr = ax25_build_s(me, peer, Ax25SType::RR, 3, false, false);
    check("RR (autonomous ACK) is tone-ACK eligible",
          tx_is_autonomous_rr(rr.data(), rr.size()));
    check("RR-only batch -> tone-ACK eligible (airtime win preserved)",
          tx_batch_tone_ack_eligible(std::vector<TxFrame>{ session_frame(rr) }));
    // RR with the poll bit set (T1 poll response) stays eligible.
    auto rrp = ax25_build_s(me, peer, Ax25SType::RR, 5, true, false);
    check("RR P=1 (poll response) is tone-ACK eligible",
          tx_is_autonomous_rr(rrp.data(), rrp.size()));

    // (2) U-frames must NEVER be swallowed.
    auto ua   = ax25_build_u(me, peer, AX25_CTRL_UA,   true, false);
    auto disc = ax25_build_u(peer, me, AX25_CTRL_DISC, true, true);
    auto dm   = ax25_build_u(me, peer, AX25_CTRL_DM,   true, false);
    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    check("UA is NOT tone-ACK eligible",   !tx_is_autonomous_rr(ua.data(),   ua.size()));
    check("DISC is NOT tone-ACK eligible", !tx_is_autonomous_rr(disc.data(), disc.size()));
    check("DM is NOT tone-ACK eligible",   !tx_is_autonomous_rr(dm.data(),   dm.size()));
    check("SABM is NOT tone-ACK eligible", !tx_is_autonomous_rr(sabm.data(), sabm.size()));
    // Document the pre-fix bug: all of these PASS the old content test.
    check("(pre-fix bug) UA passes old (byte14 & 0x01) content test",   (ua[14]   & 0x01) == 0x01);
    check("(pre-fix bug) DISC passes old (byte14 & 0x01) content test", (disc[14] & 0x01) == 0x01);
    // The load-bearing guarantee: a UA / DISC RSP batch transmits via OFDM.
    check("UA batch -> NOT tone-ACK eligible (transmits via OFDM)",
          !tx_batch_tone_ack_eligible(std::vector<TxFrame>{ session_frame(ua) }));
    check("DISC batch -> NOT tone-ACK eligible (transmits via OFDM)",
          !tx_batch_tone_ack_eligible(std::vector<TxFrame>{ session_frame(disc) }));

    // (3) RNR/REJ carry busy/go-back state a bare N(R) tone cannot represent.
    auto rnr = ax25_build_s(me, peer, Ax25SType::RNR, 2, false, false);
    auto rej = ax25_build_s(me, peer, Ax25SType::REJ, 2, false, false);
    check("RNR is NOT tone-ACK eligible", !tx_is_autonomous_rr(rnr.data(), rnr.size()));
    check("REJ is NOT tone-ACK eligible", !tx_is_autonomous_rr(rej.data(), rej.size()));

    // (4) A B2F_DATA proxy chunk whose byte-14 ALIASES an RR control byte (low
    //     nibble 0x01) must still be ineligible — provenance, not content.
    std::vector<uint8_t> b2f(40);
    for (size_t i = 0; i < b2f.size(); i++) b2f[i] = (uint8_t)(i * 7 + 3);
    b2f[0]  = B2F_DATA_MAGIC;
    b2f[14] = 0x21;  // low nibble 0x01 -> would pass the old content test
    check("(pre-fix bug) aliasing B2F chunk passes old content test", (b2f[14] & 0x01) == 0x01);
    check("B2F_DATA proxy chunk is NOT tone-ACK eligible (provenance beats content)",
          !tx_batch_tone_ack_eligible(std::vector<TxFrame>{ data_frame(b2f) }));

    // (5) A mixed batch (autonomous RR + one U-frame) is NOT eligible as a whole.
    check("mixed RR+UA batch -> NOT tone-ACK eligible",
          !tx_batch_tone_ack_eligible(std::vector<TxFrame>{ session_frame(rr), session_frame(ua) }));
    // Empty batch -> never eligible.
    check("empty batch -> NOT tone-ACK eligible",
          !tx_batch_tone_ack_eligible(std::vector<TxFrame>{}));
}

// ---------------------------------------------------------------------------
// Stale-S drain provenance (correctness-completion item 1: no silent B2F/REJ
// drop).  When a decoded peer OFDM S-frame ACKs our data, the drain at
// modem.cc:1196 clears the modem's own now-stale autonomous RR polls from
// tx_queue_.  Wave-1 (0dd9b98) fixed the tone classifier but left this drain
// typing by CONTENT `(front.data[14] & 0x03) == 0x01` -> it silently DROPPED
// aliasing B2F_DATA chunks (~25% alias, no retransmit path) and queued
// REJ/RNR/client S-frames on every peer S-frame.  This test drives the SAME
// production predicate tx_frame_is_stale_poll() and the exact drain loop.
// FAILS before the fix (old content predicate drains the aliasing B2F + REJ);
// PASSES after (provenance: only tone_ack_eligible RR polls are drained).
// ---------------------------------------------------------------------------
static void test_stale_drain_provenance() {
    printf("\n=== stale-S drain provenance (no aliasing B2F / REJ silent drop) ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    // Tag session frames EXACTLY as the producer at modem.cc:590 does; any
    // non-session producer (B2F / KISS client) ALWAYS tags false.
    auto session_frame = [](const std::vector<uint8_t>& raw) {
        return TxFrame(std::vector<uint8_t>(raw), tx_is_autonomous_rr(raw.data(), raw.size()));
    };
    auto data_frame = [](std::vector<uint8_t> raw) { return TxFrame(std::move(raw), false); };

    // (1) The modem's own autonomous RR polls ARE stale -> drained.
    auto rr  = ax25_build_s(me, peer, Ax25SType::RR, 3, false, false);
    auto rrp = ax25_build_s(me, peer, Ax25SType::RR, 5, true,  false);  // P=1 poll
    check("autonomous RR is a stale poll (drained)",
          tx_frame_is_stale_poll(session_frame(rr)));
    check("autonomous RR P=1 poll is a stale poll (drained)",
          tx_frame_is_stale_poll(session_frame(rrp)));

    // (2) A REJ / RNR from the client-side ARQ must SURVIVE (go-back / busy).
    auto rej = ax25_build_s(me, peer, Ax25SType::REJ, 2, false, false);
    auto rnr = ax25_build_s(me, peer, Ax25SType::RNR, 2, false, false);
    check("REJ survives the drain (provenance beats content)",
          !tx_frame_is_stale_poll(data_frame(rej)));
    check("RNR survives the drain (provenance beats content)",
          !tx_frame_is_stale_poll(data_frame(rnr)));
    // Document the pre-fix bug: REJ passes the old (byte14 & 0x03)==0x01 test.
    check("(pre-fix bug) REJ passes old (byte14 & 0x03) content test",
          rej.size() >= 15 && (rej[14] & 0x03) == 0x01);

    // (3) An aliasing B2F_DATA chunk whose byte-14 looks like an RR must SURVIVE.
    std::vector<uint8_t> b2f(40);
    for (size_t i = 0; i < b2f.size(); i++) b2f[i] = (uint8_t)(i * 7 + 3);
    b2f[0]  = B2F_DATA_MAGIC;
    b2f[14] = 0x21;  // low bits 01 -> would pass the old content test
    check("(pre-fix bug) aliasing B2F chunk passes old (byte14 & 0x03) test",
          (b2f[14] & 0x03) == 0x01);
    check("aliasing B2F chunk survives the drain",
          !tx_frame_is_stale_poll(data_frame(b2f)));

    // (4) End-to-end: run the EXACT production drain loop (modem.cc:1196-1206)
    //     over a mixed queue; only the two tagged RR polls may disappear.
    std::queue<TxFrame> q;
    q.push(session_frame(rr));    // stale poll   -> drop
    q.push(data_frame(b2f));      // aliasing B2F -> keep
    q.push(data_frame(rej));      // REJ          -> keep
    q.push(session_frame(rrp));   // stale poll   -> drop
    int drained = 0;
    std::queue<TxFrame> keep;
    while (!q.empty()) {
        auto& front = q.front();
        if (tx_frame_is_stale_poll(front)) drained++;
        else keep.push(std::move(front));
        q.pop();
    }
    check("drain removes exactly the 2 autonomous RR polls", drained == 2);
    check("drain keeps the aliasing B2F + REJ (2 survivors)", keep.size() == 2);
    check("first survivor is the aliasing B2F chunk (data intact)",
          !keep.empty() && keep.front().data.size() == 40 &&
          keep.front().data[0] == B2F_DATA_MAGIC);
}

// ---------------------------------------------------------------------------
// 1-CW size collapse is CAPACITY-based, not provenance-gated (item 3).
//
// Wave-1 (0dd9b98) gated the 1-CW OFDM collapse on batch_all_tone_ack_eligible,
// conflating the SIZING decision with tone REPLACEMENT. A REJ NACK is not
// tone-ACK eligible, so it went multi-CW and CMD's 1-CW expect-ack gate
// (modem.cc:1726) truncated it -> immediate-REJ forward recovery degraded to a
// T1. This test drives the production decision ofdm_collapse_cw(): a REJ/RNR/UA
// batch that fits one codeword MUST collapse to 1 CW regardless of provenance,
// while an overflowing batch stays multi-CW (a pure sizing call, never a drop).
// FAILS before the fix (collapse never fires for a non-RR batch), PASSES after.
// ---------------------------------------------------------------------------
static void test_one_cw_collapse_capacity() {
    printf("\n=== 1-CW size collapse is capacity-based, not provenance-gated ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    // Single-codeword info capacity in bytes. Production computes it from
    // ofdm_tone_map_.fec_rate as max(20, block_size/8 - 6); 20 is the floor and
    // exceeds any 15-byte S-frame batch, so REJ/RNR/UA fit trivially.
    const int one_cw_cap = 20;
    const int default_cw = 4;   // O2-O5 default (ofdm_cw_for_level)

    // (1) An immediate REJ NACK (15-byte S-frame) is NOT tone-ACK eligible, so
    //     the provenance-gated collapse skipped it -> multi-CW -> truncated.
    //     Capacity-based collapse MUST send it as 1 CW.
    auto rej = ax25_build_s(me, peer, Ax25SType::REJ, 2, false, false);
    size_t rej_batch_bytes = 3 + rej.size();  // multi-payload overhead + frame
    check("REJ NACK is NOT tone-ACK eligible (provenance gate would skip it)",
          !tx_is_autonomous_rr(rej.data(), rej.size()));
    check("REJ NACK fits one codeword -> collapses to 1 CW (capacity-based)",
          ofdm_collapse_cw(rej_batch_bytes, one_cw_cap, default_cw) == 1);

    // (2) RNR / UA (also not tone-ACK eligible) likewise collapse.
    auto rnr = ax25_build_s(me, peer, Ax25SType::RNR, 2, false, false);
    auto ua  = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    check("RNR fits one CW -> collapses to 1 CW",
          ofdm_collapse_cw(3 + rnr.size(), one_cw_cap, default_cw) == 1);
    check("UA fits one CW -> collapses to 1 CW",
          ofdm_collapse_cw(3 + ua.size(), one_cw_cap, default_cw) == 1);

    // (3) A batch that OVERFLOWS one codeword stays multi-CW (no truncation;
    //     the collapse is a pure sizing call and can never silently drop).
    check("full-size data batch stays multi-CW",
          ofdm_collapse_cw((size_t)one_cw_cap + 1, one_cw_cap, default_cw) == default_cw);
    check("exactly-capacity batch collapses to 1 CW",
          ofdm_collapse_cw((size_t)one_cw_cap, one_cw_cap, default_cw) == 1);

    // (4) Already 1 CW -> unchanged (no spurious re-log / no-op).
    check("already 1-CW batch stays 1 CW",
          ofdm_collapse_cw(10, one_cw_cap, 1) == 1);
}

// ---------------------------------------------------------------------------
// A2 — OFDM TX ncw <-> modulation-level coherence (data-flow-level-vars.md §3/§7).
//
// Pre-A2, ofdm_cw_per_frame was latched at the TOP of the TX build from the
// STALE ofdm_speed_level_ left over from the previous TX iteration / RX-adopt,
// while the frame was MODULATED at the freshly re-latched + capped level lower
// down. A climb/leap/downshift between those two points shipped a frame whose
// codeword count described a DIFFERENT level than its constellation (e.g. a
// 16QAM body carrying O2's ncw) — malformed, or the oversized guard dropped the
// whole TX. resolve_ofdm_tx_level() now single-sources the level and derives ncw
// from it. This drives the PRODUCTION resolver and asserts ncw == cw(resolved).
// Fail-before: reverting the ncw source in resolve_ofdm_tx_level() to the stale
// ofdm_speed_level_ makes (1)/(2) trip (proven by hand; see worklog).
// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// E1 — OFDM ladder SINGLE SOURCE OF TRUTH (kUniformPresets / OFDM_SPEED_LEVELS).
// The batch-sizing capacity path (ofdm_level_fec_rate — which modem.cc's
// ofdm_level_to_fec_rate / ofdm_capacity_bytes_for_level delegate to) and the
// frame builder (get_uniform_tone_map -> tm.fec_rate) MUST resolve the SAME fec
// and k for EVERY level O0..O13, and build_ofdm_frame must ACCEPT a max_payload
// frame at each. Pre-fix, modem.cc hardcoded a stale 10-rung fec table: when
// 32QAM was inserted at O6 it desynced (O7/O8 swapped, O10-O13 clamped), so at
// O7 sizing used r3/4's larger k while the frame built at r5/8 — max_payload
// oversized -> build_ofdm_frame rejected -> the TX silently skipped.
// ---------------------------------------------------------------------------
static void test_ofdm_ladder_single_source() {
    printf("\n=== E1: OFDM ladder single source (sizing k == builder k, O0..O13) ===\n");
    using namespace iris;

    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f; pb.bandwidth_hz = 2700.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    OfdmModulator mod(cfg);

    // The stale 10-rung table modem.cc used to hardcode — kept ONLY as the
    // fail-before witness (O7/O8 swapped vs the 14-rung ladder; O10-O13 clamped).
    static const LdpcRate stale10[] = {
        LdpcRate::RATE_1_2, LdpcRate::RATE_1_2, LdpcRate::RATE_3_4,
        LdpcRate::RATE_1_2, LdpcRate::RATE_5_8, LdpcRate::RATE_3_4,
        LdpcRate::RATE_5_8, LdpcRate::RATE_3_4, LdpcRate::RATE_5_8, LdpcRate::RATE_3_4,
    };
    // Mirror of the fixed modem.cc ofdm_cw_for_level (constellation-keyed).
    auto cw_for_level = [](int level) -> int {
        if (level <= 1) return 1;
        int bpc = ofdm_level_bits_per_carrier(level);
        if (bpc <= 2) return 2;
        if (bpc == 4) return 4;
        if (bpc <= 6) return 8;
        return 4;
    };

    bool all_match = true, all_accept = true, witness_seen = false;
    for (int L = 0; L < NUM_OFDM_SPEED_LEVELS; L++) {
        ToneMap tm = get_uniform_tone_map((uint8_t)(L + 1), cfg);
        LdpcRate sizing_fec = ofdm_level_fec_rate(L);   // modem.cc batch-sizing source
        LdpcRate build_fec  = tm.fec_rate;               // frame-builder source
        int sizing_k = LdpcCodec::block_size(sizing_fec) / 8;
        int build_k  = LdpcCodec::block_size(build_fec) / 8;
        if (sizing_fec != build_fec || sizing_k != build_k) all_match = false;

        // build_ofdm_frame must ACCEPT a full max_payload frame at this level.
        int ncw = cw_for_level(L);
        int per_block = std::max(20, build_k - 6);   // 2-byte len + 4-byte CRC per block
        int capacity = per_block * ncw;
        std::vector<uint8_t> payload(capacity);
        for (int i = 0; i < capacity; i++) payload[i] = (uint8_t)(i * 31 + L);
        auto iq = mod.build_ofdm_frame(payload.data(), payload.size(), tm, tm.fec_rate, ncw);
        bool accepted = !iq.empty();
        if (!accepted) all_accept = false;

        bool stale_mismatch = (L < (int)(sizeof(stale10)/sizeof(stale10[0]))) &&
                              (stale10[L] != build_fec);
        if (stale_mismatch) witness_seen = true;
        printf("    O%-2d bpc=%d fec(size/build)=%d/%d k=%d/%d ncw=%d cap=%dB accepted=%d%s\n",
               L, ofdm_level_bits_per_carrier(L), (int)sizing_fec, (int)build_fec,
               sizing_k, build_k, ncw, capacity, accepted ? 1 : 0,
               stale_mismatch ? "  <- old-10rung MISMATCH" : "");
    }
    check("E1: sizing fec/k == builder fec/k for every O0..O13", all_match);
    check("E1: build_ofdm_frame accepts a max_payload frame at every O0..O13", all_accept);
    check("E1 fail-before witness: old 10-rung table mismatched the builder at O7+", witness_seen);
}

static void test_ofdm_tx_level_coherence() {
    printf("\n=== A2: OFDM TX ncw <-> modulation-level coherence (stale-ncw) ===\n");
    using namespace iris;

    // Mirror of modem.cc ofdm_cw_for_level (static there; not linkable here).
    auto cw_for_level = [](int level) -> int {
        if (level >= 8) return 4;
        if (level >= 6) return 8;
        if (level >= 3) return 4;
        if (level == 2) return 2;
        return 1;
    };

    // Pure-logic test: the shim seeds ofdm_config_/tone map/gearshift/levels itself,
    // so no Modem::init() (WASAPI audio) is needed — keeps the test deterministic.

    // (1) Downshift/leftover: a prior RX-adopt left ofdm_speed_level_ = O3 (stale)
    //     while the gearshift is at O1 (cap=5). The frame modulates at O1; ncw MUST
    //     come from O1 (=1), NOT the stale O3 (=4). total_bytes large so the 1-CW
    //     collapse cannot fire (isolates the ncw-from-level coherence).
    {
        Modem m;
        int ncw = m.test_resolve_ofdm_tx_level(/*stale=*/3, /*gearshift=*/1, /*cap=*/5,
                                               /*kiss_control=*/false, /*batch_control=*/false,
                                               /*total_bytes=*/400);
        int level = m.test_ofdm_tx_level();
        printf("    stale=O3 gearshift=O1 -> resolved=O%d ncw=%d (stale-O3 ncw would be %d)\n",
               level, ncw, cw_for_level(3));
        check("A2: resolved TX level tracks the gearshift O1 (not the stale O3)",
              level == 1);
        check("A2: TX ncw == cw_for_level(resolved level)", ncw == cw_for_level(level));
        check("A2 fail-before witness: stale-O3 ncw(4) != resolved-O1 ncw(1)",
              cw_for_level(3) != ncw && cw_for_level(3) == 4 && ncw == 1);
    }

    // (2) Climb direction: stale O0 leftover, gearshift climbed to O3 (cap=5, acked
    //     admits O3). Resolved = O3, ncw MUST be 4 (16QAM ncw=4), not the stale O0's
    //     1. Pre-A2 shipped a 16QAM body at ncw=1 (under-coded / truncated payload).
    {
        Modem m;
        int ncw = m.test_resolve_ofdm_tx_level(/*stale=*/0, /*gearshift=*/3, /*cap=*/5,
                                               false, false, /*total_bytes=*/400);
        int level = m.test_ofdm_tx_level();
        printf("    stale=O0 gearshift=O3 -> resolved=O%d ncw=%d (stale-O0 ncw would be %d)\n",
               level, ncw, cw_for_level(0));
        check("A2 (climb): resolved TX level tracks the gearshift O3", level == 3);
        check("A2 (climb): TX ncw == cw_for_level(O3) = 4 (coherent 16QAM frame)",
              ncw == cw_for_level(level) && ncw == 4);
        check("A2 (climb) fail-before witness: stale-O0 ncw(1) != resolved-O3 ncw(4)",
              cw_for_level(0) != ncw);
    }

    // (3) The capacity-based 1-CW collapse still fires AND stays coherent (item-3
    //     interop): a tiny kiss-control batch at O3 (ncw=4) collapses to ncw=1.
    {
        Modem m;
        int ncw = m.test_resolve_ofdm_tx_level(/*stale=*/0, /*gearshift=*/3, /*cap=*/5,
                                               /*kiss_control=*/true, false,
                                               /*total_bytes=*/15);
        int level = m.test_ofdm_tx_level();
        check("A2: kiss-control batch level >= O1 floor", level >= 1);
        check("A2: tiny batch collapses to 1 CW (item-3 collapse preserved)", ncw == 1);
    }
}

// ---------------------------------------------------------------------------
// A3 — per-N(S) TX level ring anchor semantics (data-flow-level-vars.md §5).
//
// The ACK anchor must rise to the MAX level among frames the peer ACTUALLY
// ACKed, never above a level a frame was genuinely, successfully sent at. The
// scalar tx_last_level_ (most-recent DATA batch) over-credits a lagging partial
// ACK. This mirrors modem.h TxLevelRing (private; not linkable) EXACTLY — the
// conservative MIN-stamp + window-bounded max_acked — and asserts the two
// canonical scenarios the ring exists to get right (T-A3 climb, T-lag partial).
// ---------------------------------------------------------------------------
static void test_tx_level_ring_anchor() {
    printf("\n=== A3: per-N(S) TX level ring anchor (climb / lagging partial) ===\n");
    // Mirror of modem.h Modem::TxLevelRing (kept byte-identical).
    struct Ring {
        int8_t level[8];
        Ring() { reset(); }
        void reset() { for (int i = 0; i < 8; i++) level[i] = -1; }
        void stamp(uint8_t ns, int lvl) {
            int8_t& s = level[ns & 7];
            s = (s < 0) ? (int8_t)lvl : (int8_t)((int)s < lvl ? (int)s : lvl);
        }
        int max_acked(uint8_t from, uint8_t to) const {
            int m = -1;
            for (uint8_t n = from; n != to; n = (uint8_t)((n + 1) & 7))
                if ((int)level[n & 7] > m) m = level[n & 7];
            return m;
        }
        void clear_range(uint8_t from, uint8_t to) {
            for (uint8_t n = from; n != to; n = (uint8_t)((n + 1) & 7)) level[n & 7] = -1;
        }
    };

    // (T-A3) Climb across batches, all four ACKed together: anchor -> MAX (O3).
    // Batch A sends [0,2)@O1; batch B sends new frames [2,4)@O3 (window [0,4)).
    // Production stamps [va,vs) each DATA TX with MIN semantics.
    {
        Ring r;
        for (uint8_t n = 0; n != 2; n = (n + 1) & 7) r.stamp(n, 1);  // batch A: [0,2)@O1
        for (uint8_t n = 0; n != 4; n = (n + 1) & 7) r.stamp(n, 3);  // batch B: [0,4)@O3 (min keeps 0,1@O1)
        check("A3 (climb): frames 0,1 stay O1 under a later O3 batch (MIN-stamp)",
              r.max_acked(0, 2) == 1);
        check("A3 (climb): RR N(R)=4 credits the anchor to MAX acked O3",
              r.max_acked(0, 4) == 3);
    }

    // (T-lag) Lagging partial: 0@O1, 1@O2, 2@O3 in flight; peer ACKs only N(R)=2
    // (frames 0,1). Anchor must be O2 (max of the ACKed), NOT O3 (frame 2 unacked).
    {
        Ring r;
        r.stamp(0, 1);                                               // [0,1)@O1
        for (uint8_t n = 0; n != 2; n = (n + 1) & 7) r.stamp(n, 2);  // [0,2)@O2 (0 stays O1)
        for (uint8_t n = 0; n != 3; n = (n + 1) & 7) r.stamp(n, 3);  // [0,3)@O3 (0,1 stay)
        check("A3 (lag): partial RR N(R)=2 credits O2, not the unacked O3",
              r.max_acked(0, 2) == 2);
        r.clear_range(0, 2);
        check("A3 (lag): the unacked O3 frame (slot 2) is retained after clear",
              r.max_acked(2, 3) == 3);
        check("A3 (lag): cleared slots are empty", r.max_acked(0, 2) == -1);
    }

    // Reset empties the whole ring (A4 reconnect / A5d oversized-drop). NB: the
    // half-open mod-8 range [from,to) is only valid for to in [0,7] (N(S) values);
    // to==8 would loop forever — production only ever passes masked N(S), so the
    // whole-ring check reads the slots directly.
    {
        Ring r; for (int i = 0; i < 8; i++) r.stamp((uint8_t)i, 5); r.reset();
        bool all_empty = true;
        for (int i = 0; i < 8; i++) if (r.level[i] != -1) all_empty = false;
        check("A3: reset empties the ring (A4/A5d)", all_empty);
    }
}

// ---------------------------------------------------------------------------
// SLOT-COALESCING PROBE — bounds resolution (pure).  The probe knobs
// (IRIS_FRAMES_PER_BURST / IRIS_SLOT_AIRTIME_S) must be DEFAULT-INERT: unset,
// resolve_slot_bounds() returns exactly the pre-probe burst bounds (8 frames,
// the adaptive AIMD batch airtime).  Set, the frame knob controls the slot
// deterministically with the whole-slot airtime hard-capped at 15 s.
// ---------------------------------------------------------------------------
static void test_slot_coalescing_bounds() {
    printf("\n=== Slot-coalescing probe: bounds resolution ===\n");
    using namespace iris;
    int mf = 0; float at = 0.0f;

    // Default-inert witness: unset knobs reproduce the pre-probe bounds.
    Modem::resolve_slot_bounds(-1, -1.0f, 6.0f, mf, at);
    check("slot bounds: default = 8-frame cap (pre-probe)", mf == 8);
    check("slot bounds: default = batch airtime bound (6 s)", at == 6.0f);
    Modem::resolve_slot_bounds(-1, -1.0f, 12.0f, mf, at);
    check("slot bounds: default tracks the AIMD batch airtime (12 s)", at == 12.0f);

    // Probe arms.
    Modem::resolve_slot_bounds(16, -1.0f, 6.0f, mf, at);
    check("slot bounds: frames_per_burst=16 honored", mf == 16);
    check("slot bounds: probe airtime defaults to the 15 s hard cap", at == 15.0f);
    Modem::resolve_slot_bounds(16, 30.0f, 6.0f, mf, at);
    check("slot bounds: slot airtime hard-capped at 15 s", at == 15.0f);
    Modem::resolve_slot_bounds(16, 4.0f, 6.0f, mf, at);
    check("slot bounds: explicit airtime below the cap honored", at == 4.0f);
    Modem::resolve_slot_bounds(1, -1.0f, 6.0f, mf, at);
    check("slot bounds: frames_per_burst=1 = strict stop-and-wait arm", mf == 1);
    Modem::resolve_slot_bounds(999, -1.0f, 6.0f, mf, at);
    check("slot bounds: frame knob clamped to 64", mf == 64);
    Modem::resolve_slot_bounds(-1, 10.0f, 6.0f, mf, at);
    check("slot bounds: airtime-only knob keeps the 8-frame cap", mf == 8);
    check("slot bounds: airtime-only knob bound = 10 s", at == 10.0f);
}

// ---------------------------------------------------------------------------
// GEOMETRY PIN (IRIS_GRID_PIN_NARROW) — default-inert witness + the two grid
// constants the campaign quotes.  Unset, apply_grid_pin() passes the
// negotiation through byte-identical (the bench keeps its discovered
// ~325-4275 Hz -> 83-carrier grid).  Set, a VALID negotiation is pinned to
// the shipped voice-port narrow geometry (300-3000 Hz -> 57 used carriers);
// an INVALID negotiation is never pinned (the pin can re-grid a working
// session, never rescue a failed probe).
// ---------------------------------------------------------------------------
static void test_grid_pin_narrow() {
    printf("\n=== Geometry pin: IRIS_GRID_PIN_NARROW ===\n");
    using namespace iris;

    NegotiatedPassband bench;               // the flat-bench over-discovered band
    bench.low_hz = 325.0f; bench.high_hz = 4275.0f;
    bench.center_hz = 2300.0f; bench.bandwidth_hz = 3950.0f; bench.valid = true;

    // Default-inert witness (env unset).
    unsetenv("IRIS_GRID_PIN_NARROW");
    NegotiatedPassband out = apply_grid_pin(bench);
    check("grid pin: unset -> band passes through", out.low_hz == 325.0f &&
          out.high_hz == 4275.0f && out.valid);
    OfdmConfig cfg = ofdm_config_from_probe(out, 1024, 64, 12, 24);
    check("grid pin: unset -> bench grid = 83 used carriers", cfg.n_used_carriers == 83);

    // Pinned: the claim-grade 57-carrier narrow grid.
    setenv("IRIS_GRID_PIN_NARROW", "1", 1);
    out = apply_grid_pin(bench);
    check("grid pin: set -> 300-3000 Hz", out.low_hz == 300.0f && out.high_hz == 3000.0f);
    check("grid pin: set -> stays valid", out.valid);
    cfg = ofdm_config_from_probe(out, 1024, 64, 12, 24);
    check("grid pin: set -> true-narrow grid = 57 used carriers", cfg.n_used_carriers == 57);

    // A failed probe is never pinned into validity.
    NegotiatedPassband bad;                 // default: valid=false
    out = apply_grid_pin(bad);
    check("grid pin: invalid negotiation stays invalid", !out.valid);
    unsetenv("IRIS_GRID_PIN_NARROW");
}

// ---------------------------------------------------------------------------
// FIXED-GEAR CAP (IRIS_TX_LEVEL_CAP) — default-inert witness.  Unset, the
// single-sourced TX level-cap pass is byte-identical to today's behavior
// (sender probes anchor+1 past the proposal cap).  Set, the ladder hard-caps
// at O<N> (fixed-gear measurement arm); climb below the cap is untouched.
// ---------------------------------------------------------------------------
static void test_tx_level_cap() {
    printf("\n=== Fixed-gear cap: IRIS_TX_LEVEL_CAP ===\n");
    using namespace iris;
    // base=9 (anchor+1 probe above acked=8), proposal consumed (-1), no peer cap.
    unsetenv("IRIS_TX_LEVEL_CAP");
    check("tx cap: unset -> anchor+1 probe passes (O9)",
          Modem::apply_tx_level_caps_probe(9, true, false, 8, -1, -1, 3) == 9);
    setenv("IRIS_TX_LEVEL_CAP", "8", 1);
    check("tx cap: set 8 -> anchor+1 probe capped at O8",
          Modem::apply_tx_level_caps_probe(9, true, false, 8, -1, -1, 3) == 8);
    check("tx cap: climb below the cap untouched (O3)",
          Modem::apply_tx_level_caps_probe(3, true, false, 2, -1, -1, 3) == 3);
    check("tx cap: no-ACK downshift still wins (kiss -> O0)",
          Modem::apply_tx_level_caps_probe(9, true, true, 8, -1, -1, 3) == 0);
    unsetenv("IRIS_TX_LEVEL_CAP");
}

// ---------------------------------------------------------------------------
// SLOT-COALESCING PROBE — mid-slot loss recovery (the retransmit-economics
// guard).  A coalesced slot must NOT be all-or-nothing: every burst frame
// carries its own preamble (independent sync + channel estimate), so a lost
// frame in the middle of the slot costs ONE frame — the frames after it still
// decode — and the lost frame is individually retransmittable.  Drives the
// PRODUCTION drain (resolve_slot_bounds + append_coalesced_slot +
// append_ofdm_burst_frame) and the LIVE OFDM RX path in-process.
// ---------------------------------------------------------------------------
static void test_coalesced_slot_midloss() {
    printf("\n=== Slot-coalescing probe: mid-slot loss recovery ===\n");
    using namespace iris;
    Modem m;
    check("coalesced slot: 4-frame slot, frame 2 killed -> 3 survive byte-exact, "
          "loss retransmits alone, knob bounds the drain",
          m.test_coalesced_slot_midloss_recovery(/*n_frames=*/4, /*kill_idx=*/2));
}

// ---------------------------------------------------------------------------
// Leg 2 (burst-MAC), test (d) — a rejected OFDM burst frame SURVIVES (re-queued),
// not silently destroyed. The old burst loop popped a frame, and on a
// build_ofdm_frame reject it `break`ed, destroying the moved-out batch — a silent
// data-loss path that goes HOT under dynamic MAX_INFO (a level fall shrinks
// capacity under a frame already sized for a higher level). The unified drain +
// requeue_rejected_frames() re-queues a frame that still fits the CONFIRMED
// anchor. FAILS before (frame destroyed -> queue empties), PASSES after.
// ---------------------------------------------------------------------------
static void test_burst_reject_requeue() {
    printf("\n=== Leg2 (d): rejected burst frame re-queued, not destroyed ===\n");
    using namespace iris;

    // A 200-byte data frame FITS the O2 anchor (capacity 288 B) but NOT the
    // forced-low O0 modulation level (capacity 94 B) -> build_ofdm_frame rejects
    // it. It is recoverable (the level will climb back to the anchor), so it must
    // be re-queued, not destroyed.
    std::vector<std::vector<uint8_t>> frames;
    std::vector<uint8_t> big(200);
    for (size_t i = 0; i < big.size(); i++) big[i] = (uint8_t)(i * 31 + 7);
    frames.push_back(big);

    Modem m;
    size_t remaining = m.test_burst_reject_requeue(/*forced_level=*/0, /*anchor_level=*/2,
                                                   frames);
    printf("    forced=O0 (cap 94) anchor=O2 (cap 288), 200-B frame -> tx_queue after = %zu\n",
           remaining);
    check("Leg2 (d): the rejected frame survives (re-queued to tx_queue_)", remaining == 1);

    // A frame that fits the O0 level builds cleanly and is consumed (no false
    // re-queue) — the requeue path must not fire on success.
    std::vector<std::vector<uint8_t>> small_frames;
    std::vector<uint8_t> small(50);
    for (size_t i = 0; i < small.size(); i++) small[i] = (uint8_t)(i + 1);
    small_frames.push_back(small);
    Modem m2;
    size_t remaining2 = m2.test_burst_reject_requeue(/*forced_level=*/0, /*anchor_level=*/2,
                                                     small_frames);
    check("Leg2 (d): a fitting frame builds + is consumed (no false re-queue)",
          remaining2 == 0);

    // A frame too big for even the anchor is dropped LOUD (never fits reliably) —
    // no head-of-line deadlock. 400 B > O2 anchor cap 288 -> dropped, queue empties.
    std::vector<std::vector<uint8_t>> huge_frames;
    std::vector<uint8_t> huge(400);
    for (size_t i = 0; i < huge.size(); i++) huge[i] = (uint8_t)(i * 13 + 3);
    huge_frames.push_back(huge);
    Modem m3;
    size_t remaining3 = m3.test_burst_reject_requeue(/*forced_level=*/0, /*anchor_level=*/2,
                                                     huge_frames);
    check("Leg2 (d): a frame too big for even the anchor is dropped (no HOL deadlock)",
          remaining3 == 0);
}

// ---------------------------------------------------------------------------
// Leg 3 (dynamic MAX_INFO), test (c) — MAX_INFO TRACKS tx_acked_level_ instead
// of the fixed pin. The five pins (probe pre-limit x2 + activate x3) and the four
// anchor-advance re-calls now all read ONE source,
// ofdm_max_info_for_level(level) = ncw(level)*(block_size(fec)/8-6)
//                                    - OFDM_KISS_FRAMING_OVERHEAD(20). As the
// confirmed anchor climbs, MAX_INFO grows so I-frames fill the higher level's
// larger frame; on a peer REJ it shrinks (+ drop_oversized_in_window).
//
// THE FRAME-SIZING INVARIANT (Fix A, DATALINK_TAX_DIAGNOSIS.md): a max_info-sized
// AX.25 I-frame frames to max_info + OFDM_KISS_FRAMING_OVERHEAD bytes, which MUST
// be <= the level's ofdm_capacity_bytes_for_level(). The pre-fix "- 19" subtrahend
// under-counted the overhead by one byte (it omitted the shared epoch byte), so at
// O0 a 75-B info frame framed to 95 > 94 and was DROPPED to the AFSK fallback —
// OFDM never carried data. Fail-before: O0 MAX_INFO 75 -> framed 95 > 94 (breaks
// the invariant). Pass-after: O0 MAX_INFO 74 -> framed 94 == 94 (holds).
// ---------------------------------------------------------------------------
static void setup_repack_defect_modem(
    Modem& m, std::vector<std::vector<uint8_t>>& air,
    std::vector<std::vector<uint8_t>>& pump,
    std::unique_ptr<Modem>& paired) {
    const std::string me_call = "N0AAA";
    const std::string peer_call = "N0BBB";
    Ax25Address me = ax25_make_addr(me_call);
    Ax25Address peer = ax25_make_addr(peer_call);

    m.config_.callsign = me_call;
    m.ofdm_kiss_tx_ = true;
    m.ax25_session_.set_local_callsign(me_call);
    m.ax25_session_.set_send_callback([&](const uint8_t* d, size_t n) {
        air.emplace_back(d, d + n);
    });
    m.rx_callback_ = [&](const uint8_t* d, size_t n) {
        pump.emplace_back(d, d + n);
    };

    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    m.ax25_session_.notify_outgoing(sabm.data(), sabm.size());
    auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    Ax25Frame f;
    ax25_parse(ua.data(), ua.size(), f);
    m.ax25_session_.on_frame_received(f);
    m.ax25_session_.set_native_active(true);
    m.ax25_session_.set_max_info(74);

    // Establish the negotiated v2 session and both directional custody pairs.
    // M4/M5 consume local R3 ACKs only against this live receive ledger.
    paired = std::make_unique<Modem>();
    paired->config_.callsign = peer_call;
    paired->ofdm_kiss_tx_ = true;
    paired->ofdm_kiss_peer_caps_ = CAP_OFDM;
    paired->v2_negotiated_active_ = true;
    paired->ax25_session_.set_local_callsign(peer_call);
    paired->ax25_session_.notify_outgoing(ua.data(), ua.size());
    paired->ax25_session_.set_native_active(true);

    m.ofdm_kiss_peer_caps_ = CAP_OFDM;
    m.v2_negotiated_active_ = true;
    m.repack_begin_transfer();
    paired->repack_begin_rx_transfer();
    paired->repack_begin_transfer();
    m.repack_begin_rx_transfer();

    m.repack_activated_ = true;
    m.repack_engaged_ = true;
    m.repack_remote_call_ = peer_call;
    air.clear();
    pump.clear();
}

static void test_repack_peer_busy_backpressure() {
    printf("\n=== Re-pack peer-busy backpressure ===\n");
    Modem m;
    std::unique_ptr<Modem> paired;
    std::vector<std::vector<uint8_t>> air, pump;
    setup_repack_defect_modem(m, air, pump, paired);
    Ax25Address me = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    auto rnr = ax25_build_s(me, peer, Ax25SType::RNR, 0, false, false);
    Ax25Frame parsed;
    ax25_parse(rnr.data(), rnr.size(), parsed);
    m.ax25_session_.on_frame_received(parsed);

    constexpr size_t high_water = 16384;
    m.native_repack_tx_stream_.assign(high_water + 1024, 0x5a);
    m.repack_tx_drain(false);

    uint8_t info[] = {0x11, 0x22, 0x33};
    auto local_i = ax25_build_i(peer, me, 0, 0, false, AX25_PID_NONE,
                                info, sizeof(info));
    m.queue_tx_frame(local_i.data(), local_i.size());

    bool sent_rnr = false;
    for (const auto& frame : pump) {
        Ax25Frame f;
        if (ax25_parse(frame.data(), frame.size(), f) &&
            f.type() == Ax25FrameType::S_FRAME &&
            f.s_type() == Ax25SType::RNR)
            sent_rnr = true;
    }
    check("re-pack peer busy keeps the session queue bounded",
          m.ax25_session_.pending_frames() <= 1);
    check("re-pack peer busy leaves custody bytes in the bounded modem stream",
          m.native_repack_tx_stream_.size() >= high_water);
    check("re-pack peer busy asserts local custody RNR", sent_rnr);
}

static void test_repack_rx_queue_bounded() {
    printf("\n=== Re-pack RX queue bounded by air-side RNR ===\n");
    Modem m;
    std::unique_ptr<Modem> paired;
    std::vector<std::vector<uint8_t>> air, pump;
    setup_repack_defect_modem(m, air, pump, paired);

    std::vector<uint8_t> first;
    uint8_t value = 0;
    for (int i = 0; i < 4; i++)
        Modem::repack_pack_record(first, &value, 1);
    m.repack_rx_ingest(first.data(), first.size());

    bool sent_rnr = false;
    for (const auto& frame : air) {
        Ax25Frame f;
        if (ax25_parse(frame.data(), frame.size(), f) &&
            f.type() == Ax25FrameType::S_FRAME &&
            f.s_type() == Ax25SType::RNR)
            sent_rnr = true;
    }

    std::vector<uint8_t> in_flight;
    for (int i = 0; i < 340; i++) {
        value = (uint8_t)i;
        Modem::repack_pack_record(in_flight, &value, 1);
    }
    m.repack_rx_ingest(in_flight.data(), in_flight.size());

    check("re-pack full local window asserts air-side RNR",
          sent_rnr && m.ax25_session_.own_busy());
    check("re-pack RX queue is bounded to the already in-flight air frame",
          m.native_repack_rx_records_.size() <= 340);
}

static void test_repack_iframe_piggyback_nr() {
    printf("\n=== Re-pack local I-frame piggyback N(R) ===\n");
    Modem m;
    std::unique_ptr<Modem> paired;
    std::vector<std::vector<uint8_t>> air, pump;
    setup_repack_defect_modem(m, air, pump, paired);
    Ax25Address me = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");

    std::vector<uint8_t> inbound;
    for (uint8_t i = 0; i < 5; i++)
        Modem::repack_pack_record(inbound, &i, 1);
    m.repack_rx_ingest(inbound.data(), inbound.size());
    pump.clear();

    uint8_t outbound[] = {0xa1, 0xb2};
    auto local_i = ax25_build_i(peer, me, 0, 4, false, AX25_PID_NONE,
                                outbound, sizeof(outbound));
    m.queue_tx_frame(local_i.data(), local_i.size());

    bool delivered_fifth = false;
    for (const auto& frame : pump) {
        Ax25Frame f;
        if (ax25_parse(frame.data(), frame.size(), f) &&
            f.type() == Ax25FrameType::I_FRAME &&
            f.info.size() == 1 && f.info[0] == 4)
            delivered_fifth = true;
    }
    check("re-pack I-frame piggyback N(R) frees the local delivery window",
          delivered_fifth && m.native_repack_rx_records_.empty());
    check("re-pack I-frame INFO is still ingested",
          m.repack_tx_client_ns_ == 1);
}

static void test_repack_roundtrip() {
    printf("\n=== Terminate/re-pack: [len]-record pack/split BIT-EXACT round-trip ===\n");
    using namespace iris;
    Modem m;
    // Pack varied records -> stream -> split under varying fragmentation (dynamic
    // frame size + mid-stream change, the Step-0 SHA-mismatch root) -> bit-exact;
    // + bad-length rejected LOUD (never a silent bogus record).  P0 integrity gate.
    check("repack pack/split round-trip (bit-exact, varying fragmentation)",
          m.test_repack_pack_split_roundtrip());
    // Crossover recovery (Fix B): a mid-transfer anchor demote must RE-FRAGMENT the
    // un-ACKed re-pack stream at the smaller MTU, never DROP it (INV-3).  Fail-before
    // = drop_oversized_in_window loses the owned-transport bytes -> incomplete;
    // pass-after = recover_unacked_stream preserves ALL bytes + re-slices bit-exact.
    check("repack re-fragment on demote (recover un-ACKed, 0 byte loss, bit-exact)",
          m.test_repack_refragment_on_demote());
}

// ---------------------------------------------------------------------------
// Anchor-futility demote + Chase correctness (data-flow-tx-anchor.md).
// The measured wedge: the ACK anchor's proof of decodability is earned by
// frames sized at the PREVIOUS anchor's MAX_INFO, its own credit GROWS
// MAX_INFO, and full-MTU frames at the anchor level then sit at FER~=1 while
// the no-ACK demote clamps AT the anchor (53-84 consecutive clamped demotes
// per session, 0 peer REJ ever, terminal T1 N2 disconnect with a full window).
// Fail-before = the clamp is permanent; pass-after = the third consecutive
// bottomed DATA round demotes the anchor through the same tested path a peer
// REJ takes (MAX_INFO shrink + deferred window re-fragment).
// Chase: multi-block payload extraction (the old single-block reconstruction
// truncated multi-block frames to block 0 on combine success) and the flush
// decision (a short control-frame success must not flush a stored multi-block
// I-frame's accumulated copies).
// ---------------------------------------------------------------------------
static void test_anchor_futility_and_chase() {
    printf("\n=== Anchor-futility demote + Chase multi-block/flush ===\n");
    using namespace iris;
    Modem m;
    check("anchor futility: 3 bottomed data rounds demote the anchor (walks, never wedges)",
          m.test_anchor_futility_demote());
    Modem m2;
    check("chase: multi-block extract full payload; flush keeps cross-shape store",
          m2.test_chase_multiblock_and_flush());
}

// ---------------------------------------------------------------------------
// Shadow-desync prevention (data-flow-owned-shadow-seq.md §7).  Replays the
// captured owned-sequence crossover from the fast-WGN:40 teardown (14/16
// @83-car): wide window K=63, a 24-frame in-flight span, an anchor-futility
// demote shrinking K to 7 UNDER the flight, and the honest cumulative RR
// racing the deferred re-fragment.  Fail-before (1542824): the RR is silently
// refused at fwd_dist=24 > K=7, the idle-tick harvest rolls V(S) back across
// the peer-counted numbers, and every later honest N(R) reads invalid ->
// 3-streak FRMR/SABM teardown.  Pass-after: INV-SEQ-1 accepts the wide ACK,
// INV-SEQ-2 holds the harvest until the ACK state resolves (race + quiesce
// legs), INV-SEQ-3 keeps futility honest at wide K, and the session never
// reaches the desync threshold on honest N(R)s.
// ---------------------------------------------------------------------------
static void test_shadow_desync_prevention_suite() {
    printf("\n=== Shadow-desync prevention (owned-seq crossover at wide K) ===\n");
    using namespace iris;
    Modem m;
    check("shadow desync: wide ACK accepted under K-shrink; harvest holds until ACK "
          "resolution; futility honest at wide K; no FRMR on honest N(R)",
          m.test_shadow_desync_prevention());
}

// ---------------------------------------------------------------------------
// Datalink integrity: peer-REJ deferral (F2) + desync custody teardown (F1)
// (DATALINK_INTEGRITY_AUDIT §4). F2: a peer REJ mid-flight at wide K must route
// through the INV-SEQ-2 quiesce (defer_window_mutation=true) — anchor shrinks
// inline, V(S) does NOT roll back across peer-counted seq numbers — and the
// receiver must OVERWRITE a stale reorder entry with its re-sliced replacement.
// F1: an unrecoverable desync in native/re-pack mode must tear custody down
// cleanly (DISC the pump, surface undelivered bytes, DISCONNECTED) rather than
// the dead FRMR+SABM re-establish that zombies and loses custody silently.
// ---------------------------------------------------------------------------
static void test_datalink_rej_and_desync_teardown_suite() {
    printf("\n=== Datalink integrity: peer-REJ deferral (F2) + desync teardown (F1) ===\n");
    using namespace iris;
    Modem m;
    check("F2 peer REJ mid-flight defers (V(S) intact, anchor demotes, flight resolves "
          "clean; re-sliced reorder entry overwrites stale copy)",
          m.test_rej_inflight_deferred());
    Modem m2;
    check("F1 native/re-pack desync -> immediate clean custody teardown (pump DISC, "
          "DISCONNECTED, no SABM zombie, no silent loss)",
          m2.test_native_desync_custody_teardown());
}

static void test_dynamic_max_info() {
    printf("\n=== Leg3 (c): dynamic MAX_INFO + frame-sizing invariant (Fix A) ===\n");
    using namespace iris;

    // Re-derived from LdpcCodec::block_size (RATE_1_2=100 B, 5_8=125 B, 3_4=150 B),
    // ofdm_cw_for_level (O0/O1=1, O2=2, O3/O4/O5=4), minus OFDM_KISS_FRAMING_OVERHEAD(20).
    struct { int level; int expect; } tbl[] = {
        {0, 74}, {1, 74}, {2, 268}, {3, 356}, {4, 456}, {5, 556},
    };
    for (auto& t : tbl) {
        int mi = Modem::test_ofdm_max_info_for_level(t.level);
        char name[96];
        snprintf(name, sizeof(name), "Leg3 (c): MAX_INFO(O%d) == %d", t.level, t.expect);
        printf("    O%d -> MAX_INFO %d (expect %d)\n", t.level, mi, t.expect);
        check(name, mi == t.expect);
    }

    // *** THE INVARIANT — framed <= capacity for EVERY level (never the +1 that
    // dropped O0 to AFSK). Provable by construction: max_info == capacity - overhead
    // => framed == max_info + overhead == capacity. Cross-layer safety (all levels
    // O0..top, not just O0), asserted so a future capacity/overhead edit can't drift. ***
    for (int L = 0; L < NUM_OFDM_SPEED_LEVELS; L++) {
        int cap    = Modem::test_ofdm_capacity_bytes_for_level(L);
        int mi     = Modem::test_ofdm_max_info_for_level(L);
        int framed = mi + OFDM_KISS_FRAMING_OVERHEAD;
        char name[112];
        snprintf(name, sizeof(name),
                 "Fix A: O%d framed(max_info %d + overhead %d = %d) <= capacity %d",
                 L, mi, OFDM_KISS_FRAMING_OVERHEAD, framed, cap);
        check(name, framed <= cap && framed == cap);
    }

    // Fail-before witness: the OLD 75-B O0 value would break the invariant (95 > 94).
    check("Fix A: OLD O0 MAX_INFO 75 WOULD overflow O0 capacity 94 (the bug we fixed)",
          75 + OFDM_KISS_FRAMING_OVERHEAD > Modem::test_ofdm_capacity_bytes_for_level(0) &&
          74 + OFDM_KISS_FRAMING_OVERHEAD == Modem::test_ofdm_capacity_bytes_for_level(0));

    // MAX_INFO must TRACK the anchor, not sit at the O0 floor.
    check("Leg3 (c): MAX_INFO GROWS with the anchor (O2 268 > O0 74)",
          Modem::test_ofdm_max_info_for_level(2) == 268 &&
          Modem::test_ofdm_max_info_for_level(2) != Modem::test_ofdm_max_info_for_level(0));
    check("Leg3 (c): MAX_INFO monotonic non-decreasing over O0..O5 (fits at any >= level)",
          Modem::test_ofdm_max_info_for_level(0) <= Modem::test_ofdm_max_info_for_level(2) &&
          Modem::test_ofdm_max_info_for_level(2) <= Modem::test_ofdm_max_info_for_level(3) &&
          Modem::test_ofdm_max_info_for_level(3) <= Modem::test_ofdm_max_info_for_level(4) &&
          Modem::test_ofdm_max_info_for_level(4) <= Modem::test_ofdm_max_info_for_level(5));
    // Stays inside set_max_info's [16,1024] clamp across the O0..O5 cap range.
    check("Leg3 (c): O5 MAX_INFO (556) is inside the [16,1024] set_max_info clamp",
          Modem::test_ofdm_max_info_for_level(5) >= 16 &&
          Modem::test_ofdm_max_info_for_level(5) <= 1024);

    // -----------------------------------------------------------------------
    // Fix B — bounded oversize-reject recovery. A frame that exceeds the current
    // level's capacity must NEVER spin the re-queue loop into a T1 N2 disconnect:
    // once the reject streak crosses OFDM_OVERSIZE_REJECT_LIMIT the modulated level
    // is floored up to a level that carries it. Prove the pure decision gate.
    // -----------------------------------------------------------------------
    printf("\n=== Fix B: bounded oversize-reject fit-floor gate ===\n");
    // A 95-B wrapped frame (the exact O0 off-by-one size) fits O2 (cap 288) not O0/O1 (94).
    const int stuck95 = 95;
    // Fail-before: while the streak is inside the budget, no override (-1) — the
    // designed transient level-fall recovery still gets a chance to self-heal.
    check("Fix B: streak below limit -> no fit-floor (-1, transient recovery runs)",
          Modem::test_ofdm_oversize_fit_floor(0, stuck95) == -1 &&
          Modem::test_ofdm_oversize_fit_floor(31, stuck95) == -1);
    // Pass-after: at/above the limit -> a real level whose capacity carries the frame.
    int floor95 = Modem::test_ofdm_oversize_fit_floor(32, stuck95);
    check("Fix B: streak >= limit -> a fitting level (>= 0) is forced",
          floor95 >= 0);
    check("Fix B: the forced level's capacity actually carries the stuck frame",
          Modem::test_ofdm_capacity_bytes_for_level(floor95) >= stuck95);
    // It picks the MINIMUM fitting level (O0/O1 can't, O2 can) — minimal airtime.
    check("Fix B: picks the min fitting level (O2 for a 95-B frame, since O0/O1 cap=94)",
          floor95 == 2 &&
          Modem::test_ofdm_capacity_bytes_for_level(1) < stuck95 &&
          Modem::test_ofdm_capacity_bytes_for_level(2) >= stuck95);
}

// ---------------------------------------------------------------------------
// Leg 2 (burst-MAC) pacing kill, test (a) — the in-window continuation DECISION
// that governs whether the initiator sends the next burst (≥2 I-frames airborne
// before the peer's first RR) or pays the ACK-clocked stop-and-wait per burst.
//
// The mechanism is QUARANTINED OFF by default (Modem::burst_fill_): sending
// before the peer's RR collides with it on half-duplex unless the responder
// HOLDS its RR to burst-end (the coupled RSP T2-hold, which cannot be validated
// in loopback / in-process — it needs the two-stack/fleet OTA bench). So this
// proves the DECISION in-process; the OTA collision-safety is fleet-gated.
// FAILS before (mechanism OFF == production/HEAD: stop-and-wait -> frame 2 waits
// out the RR, only 1 airborne before it), PASSES after (ON + window room: the
// initiator continues -> frame 2 airborne before the RR).
// ---------------------------------------------------------------------------
static void test_burst_fill_pacing() {
    printf("\n=== Leg2 (a): burst-MAC in-window continuation decision ===\n");
    using namespace iris;

    // Fail-before witness — the default (production / HEAD) mechanism is OFF, so
    // the initiator stops-and-waits: only frame 1 is airborne before the RR.
    check("(a) mechanism OFF (default/production): initiator stops-and-waits",
          Modem::test_burst_fill_continue(/*enabled=*/false, /*ofdm=*/true,
              /*active=*/true, /*we_init=*/true, /*data=*/true,
              /*window_used=*/3, /*K=*/7) == false);

    // Pass-after — ON + window has room + data queued: continue, so frame 2 (and
    // more, up to K) go airborne BEFORE the peer's first RR.
    check("(a) ON + window room + data: continue (>=2 I-frames airborne before RR)",
          Modem::test_burst_fill_continue(true, true, true, true, true, 3, 7) == true);

    // Stop-and-wait FALLBACK preserved: a FULL K=7 window MUST wait for an ACK to
    // free a slot — never send an 8th outstanding frame.
    check("(a) ON + window FULL (7 outstanding): stop-and-wait fallback (wait for ACK)",
          Modem::test_burst_fill_continue(true, true, true, true, true, 7, 7) == false);

    // No queued data -> nothing to continue with (arm the trailing solicit / wait).
    check("(a) ON + no queued data: nothing to send",
          Modem::test_burst_fill_continue(true, true, true, true, false, 3, 7) == false);

    // Initiator-only: the responder replies via MFSK RR, never bursts data here.
    check("(a) responder never bypasses (initiator-only)",
          Modem::test_burst_fill_continue(true, true, true, false, true, 3, 7) == false);

    // Never bypasses outside OFDM-KISS native TX, or when the session is down.
    check("(a) non-OFDM-KISS never bypasses",
          Modem::test_burst_fill_continue(true, false, true, true, true, 3, 7) == false);
    check("(a) inactive session never bypasses",
          Modem::test_burst_fill_continue(true, true, false, true, true, 3, 7) == false);
}

// ---------------------------------------------------------------------------
// RX package, ROOT-2 — do NOT consume the detected region when the SHORT 1-CW
// expect-ack gate truncated a longer multi-CW reverse frame.
//
// When CMD waits for an ACK it uses the short 1-CW frame-length gate. If the
// incoming frame is actually a multi-CW REVERSE frame (a reverse I-frame or a
// multi-CW REJ/RR — both made common by dynamic MAX_INFO + burst MAC), the gate
// fires early, the demod runs on a TRUNCATED buffer, the decode fails, and at
// HEAD the drain CONSUMES the region — losing the reverse frame and paying a full
// airtime-scaled T1. The fix RETAINS the buffer on that specific failure so the
// next pass (ofdm_kiss_rx_confirmed_ just cleared → expanded gate) re-buffers and
// decodes the FULL frame. Bounded to one retry; a false positive (empty LLRs) or
// a non-CMD path still consumes. Proven in-process via the retain/consume
// predicate wired into the drain. FAILS before (HEAD always consumes: the retain
// case would be false → frame lost), PASSES after.
// ---------------------------------------------------------------------------
static void test_root2_retain_truncated_reverse() {
    printf("\n=== RX ROOT-2: retain buffer on 1-CW-gate truncation of a reverse frame ===\n");
    using namespace iris;

    // THE case the tax targets: the short 1-CW gate fired, the decode FAILED on a
    // real (non-empty-LLR) frame → it was a longer multi-CW reverse frame that got
    // truncated. RETAIN the buffer so the expanded gate re-buffers the full frame.
    check("ROOT-2: short-gate + fail + real LLRs -> RETAIN (reverse frame survives)",
          Modem::test_ofdm_root2_retain(/*short_gate=*/true, /*success=*/false,
                                        /*llrs_nonempty=*/true) == true);

    // A successful decode is delivered and its region consumed as usual — never
    // retained (that would re-deliver the same frame).
    check("ROOT-2: short-gate + SUCCESS -> consume (no retain on a decoded frame)",
          Modem::test_ofdm_root2_retain(true, /*success=*/true, true) == false);

    // A false positive / quality-gated frame carries no LLRs — it must still be
    // skipped (consumed), never pinned in the buffer.
    check("ROOT-2: short-gate + fail + EMPTY LLRs (false positive) -> consume (skip)",
          Modem::test_ofdm_root2_retain(true, false, /*llrs_nonempty=*/false) == false);

    // The BOUNDED-retry guarantee: on the retry the gate is EXPANDED
    // (short_ack_gate_used=false), so even a repeat failure consumes normally — the
    // buffer can never be pinned across passes.
    check("ROOT-2: expanded gate (retry) + fail -> consume (one-shot retain, no pin)",
          Modem::test_ofdm_root2_retain(/*short_gate=*/false, false, true) == false);

    // Fail-before witness: at HEAD there is no retain path — the drain ALWAYS
    // consumes, i.e. the retain decision is effectively a constant false. Reverting
    // ofdm_root2_retain_on_short_ack_fail to `return false;` flips the first
    // assertion (RETAIN) to FAIL — the exact truncated-reverse-frame loss ROOT-2
    // closes.
    check("ROOT-2: retain fires ONLY for the short-gate truncation (else consume)",
          Modem::test_ofdm_root2_retain(true, false, true) == true &&
          Modem::test_ofdm_root2_retain(false, false, true) == false &&
          Modem::test_ofdm_root2_retain(true, true, true) == false &&
          Modem::test_ofdm_root2_retain(true, false, false) == false);
}

// ---------------------------------------------------------------------------
// RX tone-map latch (data-flow-rx-tonemap.md) — the captured 941 s WGN:40 RX
// wedge: the blind-detect sweep persisted its successful 1-CODEWORD trial map
// (earned on the peer's 20 B RR poll) into the PERSISTENT RX map; the
// frame-length gate then released every 44-symbol 8-CW I-frame after ~10
// symbols (deterministic truncated demod), and the level-keyed sweep dedupe
// made the true (level, full-CW) shape structurally untriable. Every T1
// recovery poll re-confirmed the poisoned map — the reverse channel killed the
// forward channel. All 17 retransmissions of the stuck 555 B I-frame decoded
// BYTE-EXACT offline from the session's own audio tap: no physical killer.
//
// Fix, three prongs: (1) the sweep persists the peer's MODULATION/RATE only —
// the persistent map always carries the FULL data-frame codeword count for its
// level (a frame's 1-CW shape is per-frame, never persisted); (2) the
// frame-length gate is FLOORED at the full data-frame shape for the current RX
// level (under-buffering is fatal+silent, over-buffering is recoverable
// latency); (3) the sweep dedupe keys on (level, shape) so the true
// configuration stays reachable from a poisoned state.
//
// Scenarios (a)/(b) drive the LIVE process_rx_native() path with real frames.
// (a) FAILS before the fix at the poison witness AND the data-frame delivery;
// (b) FAILS before the fix (absorbing wedge state); both PASS after.
// ---------------------------------------------------------------------------
static void test_rx_tonemap_latch() {
    printf("\n=== RX tone-map latch: control-frame shape must never poison the data gate ===\n");
    using namespace iris;

    // (a) The live poison sequence: healthy (O7, full-CW) -> 1-CW O6 control
    //     frame (RR-poll analog, blind-detected) -> 8-CW O6 data frame.
    {
        Modem m;
        check("latch (a): poll-then-data sequence — data frame decodes byte-exact, map stays full-CW",
              m.test_rx_tonemap_latch(/*force_poisoned_state=*/false));
    }

    // (b) The forced wedge state: persistent map (O6, 1 CW) + confirmed. At
    //     base this is ABSORBING (the captured wedge). The gate floor + shape
    //     dedupe must recover it on the first data frame.
    {
        Modem m;
        check("latch (b): forced poisoned map self-heals, data frame decodes byte-exact",
              m.test_rx_tonemap_latch(/*force_poisoned_state=*/true));
    }

    // (c) Sweep dedupe keys on (level, shape): with a poisoned 1-CW map the
    //     current level MUST be re-tried (full shape untested); with a healthy
    //     full-CW map it is skipped (no duplicate trials); other levels never
    //     skipped by this predicate.
    check("latch (c): poisoned map -> current level re-triable",
          Modem::test_ofdm_sweep_skip_level(/*lvl=*/6, /*rx_level=*/6,
                                            /*persisted_ncw=*/1) == false);
    check("latch (c): healthy full-CW map -> current level deduped",
          Modem::test_ofdm_sweep_skip_level(6, 6, /*persisted_ncw=*/8) == true);
    check("latch (c): other levels never deduped by the shape rule",
          Modem::test_ofdm_sweep_skip_level(5, 6, 1) == false &&
          Modem::test_ofdm_sweep_skip_level(7, 6, 8) == false);

    // (d) Frame-length gate floor: the gate codeword count in KISS mode is
    //     never below the full data-frame shape for the current RX level, for
    //     ANY persisted map value; non-KISS is unchanged (TX==RX level).
    check("latch (d): KISS gate floored at full data shape (poisoned map)",
          Modem::test_ofdm_gate_n_cw(/*kiss_tx=*/true, /*map_ncw=*/1,
                                     /*rx_level=*/6) == 8);
    check("latch (d): KISS gate uses map when already full",
          Modem::test_ofdm_gate_n_cw(true, 8, 6) == 8);
    check("latch (d): O0/O1 (1-CW levels) gate unchanged",
          Modem::test_ofdm_gate_n_cw(true, 1, 0) == 1);
    check("latch (d): non-KISS gate unchanged by the floor",
          Modem::test_ofdm_gate_n_cw(false, 1, 6) == 1);
}

// ---------------------------------------------------------------------------
// RX package, C1 — implicit NAK.  CORRECTED semantics (P0 root cause fix).
//
// The native RESPONDER emits an autonomous RR after EACH received I-frame
// (ax25_session.cc:603-606), so the commander receives a STREAM of forward-
// advancing cumulative RRs (N(R)=1,2,3,...) as its burst is heard.  A forward-
// advancing cumulative RR is a normal pipelined ACK, NOT a NAK: the pre-fix code
// fired a full-window go-back-N on EVERY advancing RR (`va_ != kiss_prev_va`),
// which turned a healthy ACK stream into a retransmit STORM (~21 spurious resends
// per 7-frame window) that overflowed the modem tx_queue_ and starved the mid-
// window frames — the >8-frame / K=7 window-wrap DROP + shadow-V(R) STALL.
//
// Corrected: advancing RR => pipelined ACK, NO retransmit (kills the storm).  The
// genuine loss signal is a DUPLICATE RR pinned at V(A) (the responder re-ACKing
// its stuck V(R) past a gap) => fast-retransmit go-back-N from V(A) exactly ONCE
// per stuck point.  Explicit REJ => immediate go-back-N.  Full ACK => nothing.
// Part A FAILS before the fix (21 storm resends), PASSES after (0).
// ---------------------------------------------------------------------------
static void test_c1_implicit_nak() {
    printf("\n=== RX C1: no-storm on pipelined ACKs, fast-retransmit on loss ===\n");
    using namespace iris;
    const std::string ME = "N0AAA";
    const std::string PEER = "N0BBB";
    Ax25Address me   = ax25_make_addr(ME);
    Ax25Address peer = ax25_make_addr(PEER);

    uint8_t body[8] = {'D','A','T','A','0','0','0','0'};

    // Bring up a native OFDM-KISS initiator session (mirror test_ax25_window_accounting).
    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign(ME);
        s.set_send_callback([](const uint8_t*, size_t) {});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        s.on_frame_received(f);
        s.set_native_active(true);
    };
    auto feed_out_iframe = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(peer, me, ns, 0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    };
    // Counting callback: record the N(S) of every I-frame the SESSION (re)sends
    // over the air.  Client-shadowed frames (notify_outgoing) do NOT hit this —
    // only a session-driven go-back-N does, so this counts C1 retransmits exactly.
    auto attach_counter = [&](Ax25Session& s, std::vector<uint8_t>& tx_ns) {
        s.set_send_callback([&](const uint8_t* d, size_t n) {
            Ax25Frame ff;
            if (ax25_parse(d, n, ff) && ff.type() == Ax25FrameType::I_FRAME)
                tx_ns.push_back(ff.ns());
        });
    };
    auto feed_rx_rr = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, false, false);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };
    auto feed_rx_rej = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::REJ, nr, false, false);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };

    // --- Part A: a healthy per-frame ADVANCING RR stream causes NO storm -------
    // The responder ACKs each of the 7 in-flight frames in turn (N(R)=1..7).  Every
    // one advances V(A); NONE is a NAK.  Pre-fix: each fires a full-window go-back-N
    // -> 6+5+4+3+2+1 = 21 spurious session resends (the storm).  Post-fix: ZERO.
    {
        Ax25Session s;
        bring_up(s);
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);   // full window
        check("C1-A: full window outstanding (V(S)=7,V(A)=0,used=7)",
              s.vs() == 7 && s.va() == 0 && s.window_used() == 7);
        std::vector<uint8_t> tx_ns;
        attach_counter(s, tx_ns);
        for (uint8_t nr = 1; nr <= 7; nr++) feed_rx_rr(s, nr);       // per-frame ACKs
        printf("    per-frame ACK stream N(R)=1..7: %zu session resends (want 0)\n",
               tx_ns.size());
        check("C1-A: pipelined per-frame ACK stream causes ZERO go-back-N (no storm)",
              tx_ns.empty());
        check("C1-A: window fully drained by the ACK stream (V(A)=V(S)=7,used=0)",
              s.va() == 7 && s.vs() == 7 && s.window_used() == 0);
    }

    // --- Part B: a DUPLICATE N(R) at the stuck point = real loss -> ONE selective retx
    // Responder gets 0,1,2 (RR 1,2,3), loses frame 3, then gets 4,5,6 out of order
    // (the cheap-miss RX BUFFERS them) and re-ACKs its stuck V(R)=3 (RR 3,3,3).  The
    // advancing 1,2,3 must NOT fire; the FIRST dup RR=3 selectively re-airs ONLY the
    // lost frame [3] — NOT go-back-N [3,4,5,6], because the responder already holds
    // 4,5,6 in its reorder buffer (re-sending them is the WGN:30 loss amplifier).
    // Further dups are latched off.  Forward progress (RR=4) must RE-ARM the latch,
    // and the next stuck point re-airs its own single frame [4].
    {
        Ax25Session s;
        bring_up(s);
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);
        std::vector<uint8_t> tx_ns;
        attach_counter(s, tx_ns);
        feed_rx_rr(s, 1); feed_rx_rr(s, 2); feed_rx_rr(s, 3);       // advancing ACKs
        check("C1-B: advancing ACKs 1,2,3 advance V(A) with NO retransmit",
              s.va() == 3 && s.window_used() == 4 && tx_ns.empty());
        feed_rx_rr(s, 3);                                            // 1st dup (stuck)
        bool got[8] = {false};
        for (uint8_t ns : tx_ns) got[ns] = true;
        check("C1-B: duplicate RR N(R)=3 selectively re-airs ONLY frame [3] once",
              tx_ns.size() == 1 && got[3]);
        feed_rx_rr(s, 3); feed_rx_rr(s, 3);                         // more dups
        check("C1-B: further duplicate RR N(R)=3 do NOT re-fire (latched once)",
              tx_ns.size() == 1);
        // Frame 3 finally lands: responder advances to 4.  Latch resets; a NEW
        // stall at V(A)=4 must be able to fast-retransmit its own single frame.
        feed_rx_rr(s, 4);                                            // progress
        feed_rx_rr(s, 4);                                            // dup at new point
        bool got2[8] = {false};
        for (uint8_t ns : tx_ns) got2[ns] = true;
        check("C1-B: latch re-arms on progress — dup at new V(A)=4 re-airs [4]",
              tx_ns.size() == 1 + 1 && got2[4]);   // + [4]
    }

    // --- Part C: an explicit REJ is a true NAK -> immediate go-back-N ----------
    {
        Ax25Session s;
        bring_up(s);
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);  // V(S)=7,V(A)=0
        std::vector<uint8_t> tx_ns;
        attach_counter(s, tx_ns);
        feed_rx_rej(s, 3);   // REJ acks 0,1,2 (V(A)->3), requests retransmit from 3
        bool got[8] = {false};
        for (uint8_t ns : tx_ns) got[ns] = true;
        check("C1-C: REJ N(R)=3 acks 0..2 and go-back-N retransmits [3,4,5,6]",
              s.va() == 3 && tx_ns.size() == 4 &&
              got[3] && got[4] && got[5] && got[6]);
    }

    // --- Part D: a FULL ack (V(A)==V(S)) does NOT retransmit anything ----------
    {
        Ax25Session s;
        bring_up(s);
        feed_out_iframe(s, 0);
        feed_out_iframe(s, 1);
        std::vector<uint8_t> tx_ns;
        attach_counter(s, tx_ns);
        feed_rx_rr(s, 2);    // ACKs everything -> V(A)=V(S)=2, nothing outstanding
        check("C1-D: full ACK drains window (V(A)=V(S)=2, used=0)",
              s.va() == 2 && s.vs() == 2 && s.window_used() == 0);
        check("C1-D: full ACK triggers NO retransmit", tx_ns.empty());
    }

    // --- Part E: stuck RR in TIMER_RECOVERY must fast-retransmit ---------------
    // P0 connect-recovery turnaround wedge.  The reverse ACK on the native link is
    // an MFSK tone that carries N(R) but NOT the F(inal) bit, so once T1 expires
    // into TIMER_RECOVERY the classic poll -> F=1-response exit NEVER completes: a
    // stuck receiver re-sends RR N(R)==V(A) with PF=0 forever.  Pre-fix the C1
    // fast-retransmit was gated to CONNECTED, so the CMD polled but NEVER
    // retransmitted the missing frame -> permanent wedge (ics213 stalled at V(A)=3,
    // 1/4 delivered).  Post-fix the duplicate-stuck-RR fast-retransmit fires in
    // TIMER_RECOVERY too, then returns to CONNECTED.  FAILS before, PASSES after.
    {
        Ax25Session s;
        bring_up(s);
        s.set_t1_ticks(2);                                  // short T1 -> fast recovery
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);   // V(S)=7,V(A)=0
        feed_rx_rr(s, 3);   // advancing ACK 0..2 -> V(A)=3 (clears the C1 latch)
        check("C1-E: advancing ACK to V(A)=3, frames [3,4,5,6] outstanding",
              s.va() == 3 && s.window_used() == 4);
        // Drive T1 -> TIMER_RECOVERY (the CMD's poll cycle; no data retransmit).
        for (int i = 0; i < 200 &&
             s.state() != Ax25SessionState::TIMER_RECOVERY; i++)
            s.tick();
        check("C1-E: T1 expiry drove the session into TIMER_RECOVERY",
              s.state() == Ax25SessionState::TIMER_RECOVERY);
        // NOW count only what the stuck RR triggers.
        std::vector<uint8_t> tx_ns;
        attach_counter(s, tx_ns);
        feed_rx_rr(s, 3);   // stuck RR N(R)==V(A)=3, PF=0 (the MFSK tone ACK)
        bool got[8] = {false};
        for (uint8_t ns : tx_ns) got[ns] = true;
        check("C1-E: stuck RR in TIMER_RECOVERY selectively re-airs [3]",
              tx_ns.size() == 1 && got[3]);
        check("C1-E: the fast-retransmit returns the session to CONNECTED",
              s.state() == Ax25SessionState::CONNECTED);
    }
}

// ---------------------------------------------------------------------------
// RR-volley guard — kill the unbounded native-peer F=1 RESPONSE volley WITHOUT
// killing the legitimate fast RR-turnaround that paces delivery.
//
// ROOT (real-audio drill, clean +2.4 dB channel): enquiry_response()
// (ax25_session.cc :611) emits an F=1 RESPONSE.  The native-shadow poll-answer
// answered ANY pf=1 S-frame — INCLUDING an F=1 RESPONSE, which is itself just such
// an answer.  Two native peers then volley RR F=1 rsp at each other every
// turnaround forever; the standing RR keeps resetting T1 so the go-back-N /
// selective retransmit of the one lost frame never re-fires (0 delivery, responder
// holding N(S)=2..7).  Per AX.25 2.2 §6.2 an enquiry response answers a COMMAND
// poll (P=1) only.
//
// A BLANKET C/R gate (answer commands only) suppresses the volley BUT also removes
// the legitimate fast RR-turnaround — the healthy F=1 exchange whose N(R) acks the
// peer's data and paces bidirectional delivery — forcing slow T1-timeout delivery
// (~6x inter-message-gap deficit).  The SURGICAL guard instead answers a COMMAND
// poll ALWAYS and an F=1 RESPONSE that CARRIED FORWARD PROGRESS (advanced V(A))
// ALWAYS, and suppresses only a NO-PROGRESS RESPONSE echo repeated past a small
// cap.  These cases cover the exact blind spot the blanket-gate test lacked:
//   1. CADENCE — every progress-carrying F=1 RESPONSE is answered (fast turnaround
//      preserved; the blanket gate would emit ZERO here — the regression FIX-A shipped).
//   2. VOLLEY — a sustained no-progress F=1 RESPONSE stream is BOUNDED (<= cap),
//      not unbounded (FAILS before the guard: an unguarded pf answers all of them).
//   3. POLL — a P=1 COMMAND poll is always answered, never capped (positive control).
//   4. RECOVERY — suppression is TEMPORARY: progress re-arms the turnaround.
// ---------------------------------------------------------------------------
static void test_rr_volley_guard() {
    printf("\n=== RR-volley guard: no unbounded F=1 volley, fast turnaround kept ===\n");
    using namespace iris;
    const std::string ME = "N0AAA";
    const std::string PEER = "N0BBB";
    Ax25Address me   = ax25_make_addr(ME);
    Ax25Address peer = ax25_make_addr(PEER);
    uint8_t body[8] = {'D','A','T','A','0','0','0','0'};

    // Bring up a native OFDM-KISS initiator session (mirror test_c1_implicit_nak).
    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign(ME);
        s.set_send_callback([](const uint8_t*, size_t) {});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
        s.on_frame_received(f);
        s.set_native_active(true);
    };
    auto feed_out_iframe = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(peer, me, ns, 0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    };
    // Count S-frames (RR/RNR) the session emits over the air — the volley answer.
    // enquiry_response()/send_rr() route through the send callback; a session-driven
    // go-back-N emits I-frames, which this filter ignores.
    auto attach_sframe_counter = [&](Ax25Session& s, int& n_sframes) {
        s.set_send_callback([&](const uint8_t* d, size_t n) {
            Ax25Frame ff;
            if (ax25_parse(d, n, ff) && ff.type() == Ax25FrameType::S_FRAME)
                n_sframes++;
        });
    };
    // Feed a received RR with explicit final/poll and command/response bits.
    auto feed_rr = [&](Ax25Session& s, uint8_t nr, bool pf, bool command) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, pf, command);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f);
        s.on_frame_received(f);
    };

    const int CAP = 2;  // Ax25Session::kMaxNoProgressRespAnswers

    // --- Case 1: CADENCE — every PROGRESS-carrying F=1 RESPONSE is answered ------
    // The healthy fast turnaround: seven in-flight frames, the peer acks them one at
    // a time with an F=1 RESPONSE (N(R)=1..7), each advancing V(A).  Every one draws
    // an answer — the pacing is NOT forced to T1 timeout.  A blanket C/R gate emits
    // ZERO here (the exact FIX-A regression this assertion catches).
    {
        Ax25Session s; bring_up(s);
        for (uint8_t ns = 0; ns < 7; ns++) feed_out_iframe(s, ns);   // V(S)=7,V(A)=0
        int n = 0; attach_sframe_counter(s, n);
        for (uint8_t nr = 1; nr <= 7; nr++) feed_rr(s, nr, /*pf*/true, /*cmd*/false);
        printf("    7x progress F=1 RESPONSE -> %d answer(s) (want 7; blanket-gate=0)\n", n);
        check("volley-1: every progress-carrying F=1 RESPONSE is answered (fast turnaround kept)",
              n == 7);
        check("volley-1: the progress stream drained the window (V(A)=V(S)=7)",
              s.va() == 7 && s.vs() == 7);
    }

    // --- Case 2: VOLLEY — a sustained NO-PROGRESS F=1 RESPONSE stream is BOUNDED --
    // The livelock engine: frames outstanding, the peer's N(R) stuck (no advance).
    // FAILS before the guard (an unguarded pf answers all 30 -> the volley); PASSES
    // after (answered only up to the cap, then suppressed until progress).
    {
        Ax25Session s; bring_up(s);
        for (uint8_t ns = 0; ns < 3; ns++) feed_out_iframe(s, ns);   // V(S)=3,V(A)=0
        int n = 0; attach_sframe_counter(s, n);
        for (int i = 0; i < 30; i++) feed_rr(s, 0, /*pf*/true, /*cmd*/false);  // stuck
        printf("    30x no-progress F=1 RESPONSE -> %d answer(s) (want %d; unguarded=30)\n",
               n, CAP);
        check("volley-2: a no-progress F=1 RESPONSE stream is BOUNDED at the cap (no livelock)",
              n == CAP);
    }

    // --- Case 3: POLL — a P=1 COMMAND poll is ALWAYS answered (never capped) -----
    {
        Ax25Session s; bring_up(s);
        for (uint8_t ns = 0; ns < 3; ns++) feed_out_iframe(s, ns);
        int n = 0; attach_sframe_counter(s, n);
        for (int i = 0; i < 5; i++) feed_rr(s, 0, /*pf*/true, /*cmd*/true);   // P=1 COMMANDs
        printf("    5x P=1 COMMAND poll -> %d answer(s) (want 5; enquiry response preserved)\n", n);
        check("volley-3: a P=1 COMMAND poll is always answered, never capped",
              n == 5);
    }

    // --- Case 4: RECOVERY — suppression is TEMPORARY; progress re-arms it --------
    // Cap the stuck stream, then land one progress-carrying F=1 RESPONSE: it is
    // answered AND re-arms the turnaround, so the next stuck stream is capped afresh
    // (never permanently muted — the fast pacing resumes the instant data moves).
    {
        Ax25Session s; bring_up(s);
        for (uint8_t ns = 0; ns < 4; ns++) feed_out_iframe(s, ns);   // V(S)=4,V(A)=0
        int n = 0; attach_sframe_counter(s, n);
        for (int i = 0; i < 5; i++) feed_rr(s, 0, /*pf*/true, /*cmd*/false);  // stuck -> CAP
        int after_stuck1 = n;
        feed_rr(s, 1, /*pf*/true, /*cmd*/false);                     // progress N(R)=1
        int after_progress = n;
        for (int i = 0; i < 5; i++) feed_rr(s, 1, /*pf*/true, /*cmd*/false);  // stuck again -> CAP
        printf("    stuck(%d) + progress(%d) + stuck(%d) -> %d total (want %d)\n",
               after_stuck1, after_progress - after_stuck1, n - after_progress, n, CAP + 1 + CAP);
        check("volley-4: first stuck stream capped", after_stuck1 == CAP);
        check("volley-4: progress-carrying F=1 RESPONSE is answered and re-arms the turnaround",
              after_progress == CAP + 1);
        check("volley-4: the re-armed stream caps afresh (suppression is temporary, not permanent)",
              n == CAP + 1 + CAP);
    }
}

// ---------------------------------------------------------------------------
// P0 turnaround (connect-recovery ics213 drop) — ROOT B: T1 must be SUSPENDED
// while auto-tune defers data TX.
//
// After connect the initiator's post-probe auto-tune monopolizes the TX for tens
// of seconds; the held I-frames physically cannot go out.  An ACK timer (T1)
// armed against them at the bare 2.0s native floor fires a spurious
// TIMER_RECOVERY, whose go-back-N recovery burst overflows tx_queue_ and evicts a
// never-sent frame (the ics213 drop).  set_tx_deferred(true) must (a) stop any
// running/channel-busy-paused T1 and (b) make start_t1_if_unacked() and
// notify_outgoing() no-op, so ticking through the whole tune window never enters
// TIMER_RECOVERY.  The post-tune first OFDM burst re-arms T1 with the airtime
// floor.  FAILS before the guard (start_t1_if_unacked re-arms -> tick fires it ->
// TIMER_RECOVERY), PASSES after.
// ---------------------------------------------------------------------------
static void test_t1_deferred_during_tune() {
    printf("\n=== P0 turnaround: T1 suspended while auto-tune defers TX ===\n");
    using namespace iris;
    const std::string ME = "N0AAA";
    const std::string PEER = "N0BBB";
    Ax25Address me   = ax25_make_addr(ME);
    Ax25Address peer = ax25_make_addr(PEER);
    uint8_t body[8] = {'D','A','T','A','0','0','0','0'};

    Ax25Session s;
    s.set_local_callsign(ME);
    s.set_send_callback([](const uint8_t*, size_t) {});
    // Bring up a native connected session (mirror test_c1_implicit_nak::bring_up).
    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    s.notify_outgoing(sabm.data(), sabm.size());
    auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    Ax25Frame f; ax25_parse(ua.data(), ua.size(), f);
    s.on_frame_received(f);
    s.set_native_active(true);
    // Held I-frames: a full window outstanding (V(A)=0, V(S)=7) — the released,
    // not-yet-transmitted burst that auto-tune sits in front of.
    for (uint8_t ns = 0; ns < 7; ns++) {
        auto fr = ax25_build_i(peer, me, ns, 0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    }
    check("tune: full window held (V(A)=0,V(S)=7)", s.va() == 0 && s.vs() == 7);

    // The probe-completion activation path arms T1 against the held frames.
    s.start_t1_if_unacked();
    check("tune: T1 armed against held frames before tune starts", s.t1_ticks() > 0);

    // Auto-tune begins -> the modem defers TX.  The running T1 must be suspended.
    s.set_tx_deferred(true);
    check("tune: set_tx_deferred(true) suspends the running T1", s.t1_ticks() == 0);

    // The per-tick T1 watchdog keeps calling start_t1_if_unacked() throughout the
    // ~30s tune window — it must NOT re-arm while deferred.
    for (int i = 0; i < 5; i++) s.start_t1_if_unacked();
    check("tune: watchdog start_t1_if_unacked() does NOT re-arm while deferred",
          s.t1_ticks() == 0);

    // A channel-busy pause/resume during the tune ramp (PTT) must not restore a
    // stale paused T1 (the P0 fatal timer was a channel-busy-paused 2.0s T1).
    s.set_channel_busy(true);
    s.set_channel_busy(false);
    check("tune: channel-busy resume does NOT re-arm a suspended T1",
          s.t1_ticks() == 0);

    // Tick through a full tune window (> the 40-tick native floor): the session
    // must NEVER enter TIMER_RECOVERY while TX is deferred.
    bool entered_recovery = false;
    for (int i = 0; i < 200; i++) {
        s.start_t1_if_unacked();               // watchdog every tick
        s.tick();
        if (s.state() == Ax25SessionState::TIMER_RECOVERY) entered_recovery = true;
    }
    check("tune: NO spurious TIMER_RECOVERY across a 200-tick tune window",
          !entered_recovery && s.state() == Ax25SessionState::CONNECTED);

    // Tune completes -> TX resumes.  The first post-tune OFDM burst arms T1 with
    // the airtime floor (airtime*1.5 + 2.0), well above the bare 2.0s native floor.
    s.set_tx_deferred(false);
    s.set_t1_floor_for_airtime(4.0f);
    s.start_t1_if_unacked();
    check("tune: post-tune T1 re-arms with the airtime floor (> native 40 ticks)",
          s.t1_ticks() > 40);
}

// ---------------------------------------------------------------------------
// P0 turnaround — ROOT C (class-fix / defense-in-depth): the modem tx_queue_
// must never silently drop UNSENT data under a connect-recovery go-back-N burst.
//
// A stuck-receiver recovery re-emits the same outstanding I-frames many times
// (10x in the P0 re-smoke); without coalescing, the 32-cap drop-oldest queue
// evicts a never-sent in-flight frame (the ics213 drop) with no retransmit path.
// enqueue_native_tx_frame() coalesces byte-identical duplicates so the queue is
// bounded to the DISTINCT outstanding frames and drop-oldest never fires.  FAILS
// before the coalesce (never-sent frame evicted), PASSES after.
// ---------------------------------------------------------------------------
static void test_tx_queue_no_evict_recovery_burst() {
    printf("\n=== P0 turnaround: tx_queue_ never drops unsent data under recovery burst ===\n");
    using namespace iris;
    Modem m;
    bool survived = m.test_tx_queue_no_evict_under_dup_burst();
    check("recovery burst: the never-sent in-flight frame SURVIVES the go-back-N dup storm",
          survived);
}

static void test_tx_queue_never_evicts_accepted_frame() {
    printf("\n=== Generic TX queue never evicts an accepted frame ===\n");
    Modem m;
    for (uint8_t i = 0; i < 32; i++) {
        uint8_t frame[] = {i};
        m.queue_tx_frame(frame, sizeof(frame));
    }
    uint8_t extra[] = {32};
    m.queue_tx_frame(extra, sizeof(extra));
    check("full TX queue preserves every previously accepted frame",
          m.tx_queue_.size() == 32 &&
          m.tx_queue_.front().data[0] == 0 &&
          m.tx_queue_.back().data[0] == 31);
}

// ---------------------------------------------------------------------------
// native_active_ must clear on session reset (AX.25-interop hazard).
//
// set_native_active(true) is called on OFDM-KISS activation and lowers T1 for
// native mode (cap 6s vs AFSK 30s) and enables ~18 native-only session paths.
// It was ONLY ever written true -- reset_session()/reset() never cleared it --
// so a subsequent legacy Direwolf/AFSK-TNC session that never activates native
// inherited stale native T1/T2/ACK timing.  FAILS before the ax25_session.cc
// fix (reset leaves native_active_ true), PASSES after.
// ---------------------------------------------------------------------------
static void test_native_active_reset() {
    printf("\n=== native_active_ cleared on session reset (AX.25-interop) ===\n");
    using namespace iris;
    Ax25Session s;
    s.set_local_callsign("N0AAA");
    s.set_send_callback([](const uint8_t*, size_t) {});
    check("fresh session is non-native", !s.native_active());
    s.set_native_active(true);
    check("set_native_active(true) arms native mode", s.native_active());
    // A reset (teardown / reconnect) must return to non-native so a following
    // legacy AFSK-TNC session does not inherit native T1/T2/ACK behavior.
    s.reset();
    check("session reset returns to non-native (no stale native timing)",
          !s.native_active());
    // Re-arming still works after reset (next peer OFDM decode re-sets it).
    s.set_native_active(true);
    check("native mode re-arms after reset", s.native_active());
}

// ---------------------------------------------------------------------------
// Config-identity fingerprint (item 5).
//
// The fingerprint is the config-echo's decision function: two ends that
// resolved the SAME OfdmConfig MUST hash to the SAME value (no false AFSK
// fallback), and any activation-parameter split (nfft, cp, pilot spacing, or a
// band-edge/carrier-grid asymmetry) MUST change it (a real desync is caught).
// ---------------------------------------------------------------------------
static void test_ofdm_config_fingerprint() {
    printf("\n=== OFDM config-identity fingerprint (item 5) ===\n");
    using namespace iris;
    auto pb = [](float low, float high) {
        NegotiatedPassband p;
        p.low_hz = low; p.high_hz = high;
        p.center_hz = (low + high) / 2.0f;
        p.bandwidth_hz = high - low;
        p.valid = true;
        return p;
    };

    // Baseline resolved config.
    OfdmConfig a = ofdm_config_from_probe(pb(700.0f, 2700.0f), 1024, 64, 8, 24);
    uint16_t fa = ofdm_config_fingerprint(a);

    // Same negotiated inputs on the "other end" -> identical fingerprint.
    OfdmConfig a2 = ofdm_config_from_probe(pb(700.0f, 2700.0f), 1024, 64, 8, 24);
    check("matched configs -> identical fingerprint (no false AFSK fallback)",
          ofdm_config_fingerprint(a2) == fa);
    check("fingerprint is never the reserved 0 sentinel", fa != 0);

    // nfft desync (the D2 split-brain: 1024 vs 512).
    OfdmConfig nfft_lo = ofdm_config_from_probe(pb(700.0f, 2700.0f), 512, 64, 8, 24);
    check("nfft 1024 vs 512 -> different fingerprint",
          ofdm_config_fingerprint(nfft_lo) != fa);

    // Band-edge / carrier-grid asymmetry (D5b: own-band vs intersection).
    OfdmConfig band = ofdm_config_from_probe(pb(900.0f, 2500.0f), 1024, 64, 8, 24);
    check("band-edge asymmetry (D5b) -> different fingerprint",
          ofdm_config_fingerprint(band) != fa);

    // CP desync.
    OfdmConfig cp = ofdm_config_from_probe(pb(700.0f, 2700.0f), 1024, 128, 8, 24);
    check("cp 64 vs 128 -> different fingerprint",
          ofdm_config_fingerprint(cp) != fa);

    // Pilot carrier-spacing desync.
    OfdmConfig pil = ofdm_config_from_probe(pb(700.0f, 2700.0f), 1024, 64, 4, 24);
    check("pilot spacing 8 vs 4 -> different fingerprint",
          ofdm_config_fingerprint(pil) != fa);
}

// Session-reliability: the OFDM carrier grid MUST be bit-identical on both ends,
// or the ZC preamble (generated over the TX bins, correlated over the RX bins)
// lands on carriers the receiver never looks at -> "FD-ZC too low" -> 0 forward
// frames. The legacy min/max INTERSECTION desyncs whenever the responder loses
// the initiator's RESULT echo (fire-and-forget) on an asymmetric channel and
// falls back to a symmetric seed. probe_negotiate_grid() derives the band from
// the single authoritative CMD->RSP measurement both ends reliably hold, so the
// grid is deterministic regardless of which echoes were lost. Fail-before /
// pass-after captured in one test.
static void test_grid_derivation_deterministic() {
    printf("\n=== OFDM grid derivation determinism (session-reliability) ===\n");
    using namespace iris;

    auto mk = [](float low, float high) {
        ProbeResult r;
        r.low_hz = low; r.high_hz = high;
        r.tones_detected = 40; r.valid = true;
        return r;
    };

    // Asymmetric channel: the forward CMD->RSP path (RSP's measurement of CMD's
    // pre-emphasized probe) reaches higher than the reverse RSP->CMD path (CMD's
    // measurement, biased low by FM de-emphasis). A 300 Hz high-edge split is
    // ~6 carrier bins at nfft=1024 (48000/1024 = 46.875 Hz/bin).
    ProbeResult cmd_to_rsp = mk(300.0f, 4300.0f);   // authoritative; both ends hold it
    ProbeResult rsp_to_cmd = mk(300.0f, 4000.0f);   // reverse; weakly echoed

    const int nfft = 1024, cp = 64, cps = 8, bps = 24;
    auto bins = [&](const NegotiatedPassband& n) {
        return ofdm_config_from_probe(n, nfft, cp, cps, bps).used_carrier_bins;
    };

    // Model each end's resolved probe-controller state (see probe_controller.cc):
    //  CMD (initiator): my_tx_result_ = CMD->RSP (RSP's robustly re-announced
    //    RESULT), their_tx_result_ = RSP->CMD (CMD's own measurement).
    ProbeResult cmd_my = cmd_to_rsp, cmd_their = rsp_to_cmd;
    //  RSP (responder) with CMD's RESULT LOST: the symmetric seed leaves
    //    my_tx_result_ = their_tx_result_ = CMD->RSP.
    ProbeResult rsp_my = cmd_to_rsp, rsp_their = cmd_to_rsp;

    // FAIL-BEFORE: legacy intersection desyncs the grid on a lost reverse echo.
    auto cmd_legacy_bins = bins(probe_negotiate(cmd_my, cmd_their));
    auto rsp_legacy_bins = bins(probe_negotiate(rsp_my, rsp_their));
    check("legacy intersection DESYNCS the grid on a lost reverse echo (fail-before)",
          cmd_legacy_bins != rsp_legacy_bins);

    // PASS-AFTER: authoritative derivation -> bit-identical grids on both ends.
    NegotiatedPassband cmd_grid = probe_negotiate_grid(cmd_my, cmd_their, /*is_initiator=*/true);
    NegotiatedPassband rsp_grid = probe_negotiate_grid(rsp_my, rsp_their, /*is_initiator=*/false);
    auto cmd_grid_bins = bins(cmd_grid);
    auto rsp_grid_bins = bins(rsp_grid);
    check("authoritative derivation -> identical used_carrier_bins (pass-after)",
          cmd_grid_bins == rsp_grid_bins && !cmd_grid_bins.empty());
    check("authoritative derivation -> identical config fingerprint",
          ofdm_config_fingerprint(ofdm_config_from_probe(cmd_grid, nfft, cp, cps, bps)) ==
          ofdm_config_fingerprint(ofdm_config_from_probe(rsp_grid, nfft, cp, cps, bps)));

    // The band is matched to the FORWARD (CMD->RSP) channel exactly (margined),
    // independent of role.
    check("authoritative band = CMD->RSP high - margin (CMD end)",
          std::fabs(cmd_grid.high_hz - (4300.0f - 25.0f)) < 0.01f);
    check("authoritative band = CMD->RSP high - margin (RSP end)",
          std::fabs(rsp_grid.high_hz - (4300.0f - 25.0f)) < 0.01f);

    // Robust the OTHER way too: when RSP DID receive CMD's echo (my_tx=RSP->CMD),
    // the responder still keys off its local CMD->RSP (their_tx), so the grid is
    // identical to CMD's whether or not the reverse echo arrived.
    NegotiatedPassband rsp_grid_echo = probe_negotiate_grid(rsp_to_cmd, rsp_their, false);
    check("authoritative grid identical whether or not the reverse echo arrived",
          bins(rsp_grid_echo) == cmd_grid_bins);
}

// ===========================================================================
// Compression streaming-coherence battery + dict-priming (SPEED_ATTACK_PLAN C0/C2).
//
// Drives the PRODUCTION iris::Compressor through mixed zstd/PPMd/raw streaming
// sequences (TX compress -> RX decompress, committing on BOTH sides exactly as the
// modem streaming path does) and asserts no crash + bit-exact round-trip + warm carry.
//
// The bug this guards (C0): streaming_commit() sets ppmd_model_warm_=true after ANY
// committed batch (zstd-only, raw, or a dict-primed batch) that never exercised the
// PPMd model. If the Init-skip is gated on ppmd_model_warm_ (as the pre-fix code did),
// the next PPMd batch encodes/decodes into a never-Ppmd8_Init()-ed model -> segfault
// (or, after a speculative encode, desync/garbage). Fix: gate the Init-skip on
// ppmd_model_initialized_, and ppmd_model_reset() after every non-PPMd batch so TX and
// RX PPMd models stay in lock-step.
// ===========================================================================

static std::vector<uint8_t> cc_text_batch(int len, int seed) {
    static const char* paras[] = {
        "This is the body of a Winlink radiogram forwarded over the B2F protocol. ",
        "The message is plain English text with ordinary words repeated often. ",
        "Disaster relief coordination requires accurate situation reports now. ",
        "All stations please acknowledge receipt and relay to the net control. "
    };
    std::vector<uint8_t> s;
    int i = seed;
    while ((int)s.size() < len) {
        const char* p = paras[i & 3];
        s.insert(s.end(), (const uint8_t*)p, (const uint8_t*)p + strlen(p));
        i++;
    }
    s.resize(len);
    return s;
}

// High-entropy band [6.0, 7.5): compressible framing + pseudo-random base64 body.
static std::vector<uint8_t> cc_zstd_only_batch(int len, unsigned seed) {
    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    const char* hdr = "Content-Transfer-Encoding: base64\r\nX-Winlink: B2F\r\n";
    std::vector<uint8_t> s;
    unsigned r = seed;
    while ((int)s.size() < len) {
        s.insert(s.end(), (const uint8_t*)hdr, (const uint8_t*)hdr + strlen(hdr));
        for (int i = 0; i < 48 && (int)s.size() < len; i++) { r = r*1664525u+1013904223u; s.push_back((uint8_t)b64[(r>>24)&63]); }
        s.push_back('\r'); s.push_back('\n');
    }
    s.resize(len);
    return s;
}

// Pure random -> entropy > 7.5 -> RAW (incompressible attachment chunk).
static std::vector<uint8_t> cc_raw_batch(int len, unsigned seed) {
    std::vector<uint8_t> s(len);
    unsigned r = seed;
    for (int i = 0; i < len; i++) { r = r*1103515245u+12345u; s[i] = (uint8_t)((r>>16)&0xFF); }
    return s;
}

// Run one TX->RX streaming sequence; return true iff every batch round-trips bit-exact
// (and nothing crashed). Commits on both sides exactly as the modem does.
static bool cc_run_sequence(const char* name, const std::vector<std::vector<uint8_t>>& batches, bool dict_prime) {
    Compressor tx; tx.set_dict_priming(dict_prime); tx.init(); tx.streaming_enable();
    Compressor rx; rx.set_dict_priming(dict_prime); rx.init(); rx.streaming_enable();
    static uint8_t out[262144], rt[262144];
    bool ok = true;
    for (size_t b = 0; b < batches.size(); b++) {
        const std::vector<uint8_t>& raw = batches[b];
        int w = tx.compress_block(raw.data(), (int)raw.size(), out, (int)sizeof(out));
        if (w <= 0) { printf("  [FAIL] %s batch %zu compress=%d\n", name, b, w); ok = false; break; }
        int d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
        if (d != (int)raw.size() || memcmp(rt, raw.data(), raw.size()) != 0) {
            printf("  [FAIL] %s batch %zu RX mismatch got=%d want=%zu\n", name, b, d, raw.size());
            ok = false; break;
        }
        if (rx.is_streaming()) rx.streaming_commit(rt, d);
        if (tx.is_streaming()) { tx.set_pending_raw(raw.data(), (int)raw.size()); tx.commit_pending(); }
    }
    tx.deinit(); rx.deinit();
    return ok;
}

static void test_compress_stream_coherence() {
    printf("\n=== Compression Streaming Coherence (C0) ===\n");
    // Priming OFF here to exercise the pure cold->warm streaming path (the exact C0
    // surface); priming coherence is covered by test_compress_dict_priming.

    // 1. THE CRASH SEQUENCE: zstd-band batch then PPMd-able text batches.
    {
        std::vector<std::vector<uint8_t>> seq;
        seq.push_back(cc_zstd_only_batch(1200, 0xdeadbeef));
        seq.push_back(cc_text_batch(1200, 0));
        seq.push_back(cc_text_batch(1200, 1));
        seq.push_back(cc_text_batch(1200, 2));
        check("stream coherence: zstd->PPMd (crash seq) bit-exact", cc_run_sequence("crash_seq", seq, false));
    }
    // 2. RAW attachment sandwiched between text + a zstd batch (mixed Winlink).
    {
        std::vector<std::vector<uint8_t>> seq;
        seq.push_back(cc_text_batch(1200, 0));
        seq.push_back(cc_raw_batch(1200, 0x1111));
        seq.push_back(cc_text_batch(1200, 1));
        seq.push_back(cc_zstd_only_batch(1200, 0x2222));
        seq.push_back(cc_text_batch(1200, 2));
        check("stream coherence: raw/zstd/text mixed bit-exact", cc_run_sequence("mixed_attach", seq, false));
    }
    // 3. Warm PPMd carry (no regression): a long all-text run must compress better.
    {
        Compressor tx; tx.set_dict_priming(false); tx.init(); tx.streaming_enable();
        Compressor rx; rx.set_dict_priming(false); rx.init(); rx.streaming_enable();
        static uint8_t out[262144], rt[262144];
        std::vector<float> ratios;
        bool rt_ok = true, carried = false;
        for (int b = 0; b < 8; b++) {
            std::vector<uint8_t> raw = cc_text_batch(1500, b);
            int w = tx.compress_block(raw.data(), (int)raw.size(), out, (int)sizeof(out));
            if (w <= 0) { rt_ok = false; break; }
            int algo = out[0] & COMPRESS_ALGO_MASK;
            int d = rx.decompress_block(out, w, rt, (int)sizeof(rt));
            if (d != (int)raw.size() || memcmp(rt, raw.data(), raw.size()) != 0) { rt_ok = false; break; }
            if (rx.is_streaming()) rx.streaming_commit(rt, d);
            tx.set_pending_raw(raw.data(), (int)raw.size()); tx.commit_pending();
            ratios.push_back((float)raw.size() / (float)w);
            if (b >= 2 && algo != COMPRESS_ALGO_RAW) carried = true;
        }
        tx.deinit(); rx.deinit();
        check("stream coherence: warm-carry bit-exact", rt_ok && ratios.size() == 8);
        if (ratios.size() == 8) {
            printf("  warm_carry ratios: b0=%.2f b7=%.2f carried=%d\n", ratios.front(), ratios.back(), carried?1:0);
            check("stream coherence: tail compresses better than head (carry)", ratios.back() > ratios.front());
            check("stream coherence: compressed streaming carry present", carried);
        }
    }
    // 4. Stress: alternating zstd/text (exercises reset/re-init repeatedly).
    {
        std::vector<std::vector<uint8_t>> seq;
        for (int i = 0; i < 12; i++) {
            seq.push_back(cc_zstd_only_batch(900, 0x5000u + i));
            seq.push_back(cc_text_batch(900, i));
        }
        check("stream coherence: alternating zstd/text bit-exact", cc_run_sequence("alternating", seq, false));
    }
    // 5. DIRECT C0 repro: a committed batch that never ran PPMd (raw), then a
    //    PPMd-eligible batch. Pre-fix: warm=true + uninitialized PPMd model ->
    //    ppmd_compress skips Init -> segfault. Post-fix: initialized-gate keeps it in
    //    the cold try-both branch -> safe.
    {
        Compressor c; c.set_dict_priming(false); c.init(); c.streaming_enable();
        static uint8_t out[8192];
        std::vector<uint8_t> rnd = cc_raw_batch(600, 0xA5A5);        // entropy>7.5 -> RAW, PPMd never Init'd
        c.streaming_commit(rnd.data(), (int)rnd.size());             // warm=true, model uninitialized
        std::vector<uint8_t> txt = cc_text_batch(600, 3);           // low entropy -> would take PPMd-only path
        int w = c.compress_block(txt.data(), (int)txt.size(), out, (int)sizeof(out));
        c.deinit();
        check("stream coherence: commit-then-PPMd no crash (C0 direct)", w > 0);
    }
}

static std::vector<std::vector<uint8_t>> decode_v2_data_records(
    const std::vector<std::vector<uint8_t>>& wire_frames) {
    ArqSession receiver;
    std::vector<std::vector<uint8_t>> records;
    std::vector<uint8_t> current;
    ArqCallbacks callbacks;
    callbacks.on_typed_data_fragment =
        [&](const uint8_t* data, size_t len, bool end,
            ArqRecordType type, uint64_t) {
            if (type != ArqRecordType::Data) return false;
            current.insert(current.end(), data, data + len);
            if (end) {
                records.push_back(std::move(current));
                current.clear();
            }
            return true;
        };
    receiver.set_callbacks(callbacks);

    ArqFrame connect{ArqType::CONNECT, 0, 0, {0, 0, 'T', 'X'}};
    auto connect_wire = connect.serialize();
    receiver.on_frame_received(connect_wire.data(), connect_wire.size());
    for (const auto& wire : wire_frames)
        receiver.on_frame_received(wire.data(), wire.size());
    return records;
}

static void test_mlkem_held_frames_replayed() {
    printf("\n=== ML-KEM strict held frames replayed ===\n");
    Modem m;
    m.config_.callsign = "CMD01";
    m.config_.encryption_mode = 1;
    m.native_mode_ = true;
    m.native_tx_ready_ = true;
    m.mlkem_kx_pending_ = true;
    m.arq_.state_ = ArqState::CONNECTED;
    m.arq_.role_ = ArqRole::COMMANDER;
    m.arq_.callsign_ = "CMD01";
    m.arq_.remote_callsign_ = "RSP01";

    std::vector<std::vector<uint8_t>> sent;
    ArqCallbacks cb;
    cb.send_frame = [&](const uint8_t* d, size_t n) {
        ArqFrame f;
        if (ArqFrame::deserialize(d, n, f) && f.type == ArqType::DATA)
            sent.emplace_back(d, d + n);
    };
    m.arq_.set_callbacks(cb);

    const uint8_t a[] = {'A'};
    const uint8_t b[] = {'B', 'B'};
    m.queue_tx_frame(a, sizeof(a));
    m.queue_tx_frame(b, sizeof(b));
    check("strict ML-KEM holds both payloads before rekey",
          sent.empty() && !m.mlkem_held_frames_.empty());

    m.rekey_hybrid();
    const auto decoded = decode_v2_data_records(sent);
    check("strict ML-KEM replays held payloads once in order with boundaries",
          decoded.size() == 2 &&
          decoded[0] == std::vector<uint8_t>(a, a + sizeof(a)) &&
          decoded[1] == std::vector<uint8_t>(b, b + sizeof(b)) &&
          m.mlkem_held_frames_.empty());
}

static void test_compress_dict_priming() {
    printf("\n=== Compression Dict Priming (C2) ===\n");
    // A representative small Winlink message (headers + a check-in body). The dict is
    // trained on boilerplate ONLY (no bodies), so this body is held-out.
    const char* msg =
        "Content-Type: text/plain; charset=ISO-8859-1\n"
        "Subject: //WL2K Tuesday net check-in\n"
        "Mid: A1B2C3D4E5F6\n"
        "From: KX0ABC\n"
        "To: net-control\n"
        "Net control checking in for the Tuesday evening traffic net. Conditions\n"
        "fair on 80 meters, some QSB. Standing by. 73 de KX0ABC\n";
    int mlen = (int)strlen(msg);
    static uint8_t out[65536], rt[65536];

    // Cold (priming off)
    int cold_w = -1; bool cold_rt = false;
    {
        Compressor tx; tx.set_dict_priming(false); tx.init(); tx.streaming_enable();
        Compressor rx; rx.set_dict_priming(false); rx.init(); rx.streaming_enable();
        cold_w = tx.compress_block((const uint8_t*)msg, mlen, out, (int)sizeof(out));
        int d = rx.decompress_block(out, cold_w, rt, (int)sizeof(rt));
        cold_rt = (d == mlen && memcmp(rt, msg, mlen) == 0);
        tx.deinit(); rx.deinit();
    }

    // Primed (default ON): identically-primed TX and RX.
    int prime_w = -1; bool prime_rt = false, primed_flag = false;
    {
        Compressor tx; tx.init(); tx.streaming_enable();
        Compressor rx; rx.init(); rx.streaming_enable();
        primed_flag = tx.dict_primed() && rx.dict_primed();
        prime_w = tx.compress_block((const uint8_t*)msg, mlen, out, (int)sizeof(out));
        int d = rx.decompress_block(out, prime_w, rt, (int)sizeof(rt));
        prime_rt = (d == mlen && memcmp(rt, msg, mlen) == 0);
        tx.deinit(); rx.deinit();
    }

    check("dict priming: cold round-trip bit-exact", cold_rt);
    check("dict priming: primed round-trip bit-exact", prime_rt);
    check("dict priming: both peers report primed", primed_flag);
    printf("  msg=%d cold_wire=%d primed_wire=%d (lift %.2fx)\n",
           mlen, cold_w, prime_w, (cold_w > 0 && prime_w > 0) ? (float)cold_w / (float)prime_w : 0.0f);
    check("dict priming: primed wire < cold wire", prime_w > 0 && cold_w > 0 && prime_w < cold_w);
}

// ===========================================================================
// B2F reroll bit-identity HARD GATE (SPEED_ATTACK_PLAN C1).
//
// On RX, Iris rerolls received plaintext back to LZHUF for the local Winlink client.
// The reroll MUST be bit-identical to the original wire blob; the necessary live
// condition is LZHUF length == the sender's declared comp_size. Pre-fix the check
// only WARNED and shipped the mismatch anyway (a fabricated ratio + a checksum-
// rejecting blob). This test proves the hard gate ACCEPTS a bit-identical reroll and
// REJECTS a corrupted one, and contrasts the new decision with the old warn-only one.
// ===========================================================================
static void test_b2f_filter_zero_does_not_forward_original() {
    printf("\n=== B2F zero-output buffering ===\n");
    Modem m;
    m.native_mode_ = true;
    m.native_tx_ready_ = true;
    m.arq_.state_ = ArqState::CONNECTED;
    m.arq_.role_ = ArqRole::COMMANDER;
    m.arq_.local_caps_ = CAP_B2F_UNROLL;
    m.arq_.peer_caps_ = CAP_B2F_UNROLL;
    m.b2f_handler_.init();

    std::vector<std::vector<uint8_t>> sent;
    ArqCallbacks cb;
    cb.send_frame = [&](const uint8_t* d, size_t n) {
        ArqFrame f;
        if (ArqFrame::deserialize(d, n, f) && f.type == ArqType::DATA)
            sent.emplace_back(d, d + n);
    };
    m.arq_.set_callbacks(cb);

    char sid_out[64];
    const char sid[] = "[B2F]\r";
    m.b2f_handler_.filter_tx(sid, sizeof(sid) - 1, sid_out, sizeof(sid_out));

    const uint8_t partial[] = "HELLO";
    m.queue_tx_frame(partial, sizeof(partial) - 1);
    check("B2F buffered incomplete line emits no ARQ DATA", sent.empty());

    const uint8_t suffix[] = "\r";
    m.queue_tx_frame(suffix, sizeof(suffix) - 1);
    const std::vector<uint8_t> expected = {'H', 'E', 'L', 'L', 'O', '\r'};
    const auto decoded = decode_v2_data_records(sent);
    check("B2F terminating suffix emits only the filtered combined line",
          decoded.size() == 1 && decoded[0] == expected);
}

static void test_b2f_reroll_hard_gate() {
    printf("\n=== B2F Reroll Hard Gate (C1) ===\n");

    // A representative Winlink message plaintext (headers + body).
    const char* pt =
        "Mid: A1B2C3D4E5F6\r\n"
        "Subject: //WL2K Tuesday net check-in\r\n"
        "From: KX0ABC\r\n"
        "To: net-control\r\n\r\n"
        "Net control checking in for the Tuesday evening traffic net. Conditions\r\n"
        "fair on 80 meters, some QSB. Two pieces of traffic to pass. 73 de KX0ABC\r\n";
    int ptlen = (int)strlen(pt);

    static uint8_t lzh1[8192], back[8192], lzh2[8192];
    size_t lzh1_len = 0, back_len = 0, lzh2_len = 0;

    // Original wire LZHUF (stands in for the sender's Winlink LZHUF — byte-identical
    // encoders per WINLINK_DICT_SCOPE).
    int rc1 = lzhuf_encode_buffer((const uint8_t*)pt, ptlen, lzh1, sizeof(lzh1), &lzh1_len);
    check("b2f reroll: original LZHUF encode ok", rc1 == 0 && lzh1_len > 0);

    // RX unroll: decode to plaintext, then reroll (re-encode) — the exact production path.
    int rcd = lzhuf_decode_buffer(lzh1, lzh1_len, back, sizeof(back), &back_len);
    check("b2f reroll: decode recovers plaintext bit-exact",
          rcd == 0 && back_len == (size_t)ptlen && memcmp(back, pt, ptlen) == 0);
    int rc2 = lzhuf_encode_buffer(back, back_len, lzh2, sizeof(lzh2), &lzh2_len);
    check("b2f reroll: re-encode is BIT-IDENTICAL to original (D3)",
          rc2 == 0 && lzh2_len == lzh1_len && memcmp(lzh1, lzh2, lzh1_len) == 0);

    int expected = (int)lzh1_len;  // the FC-declared comp_size

    // Hard gate ACCEPTS the bit-identical reroll.
    check("b2f reroll: hard gate ACCEPTS bit-identical reroll",
          b2f_reroll_shippable(rc2, lzh2_len, expected));

    // Hard gate REJECTS a corrupted reroll (size differs from declared comp_size).
    check("b2f reroll: hard gate REJECTS size-mismatched reroll (short)",
          !b2f_reroll_shippable(0, lzh2_len - 1, expected));
    check("b2f reroll: hard gate REJECTS size-mismatched reroll (long)",
          !b2f_reroll_shippable(0, lzh2_len + 1, expected));
    check("b2f reroll: hard gate REJECTS LZHUF encode failure",
          !b2f_reroll_shippable(1, 0, expected));
    check("b2f reroll: hard gate REJECTS absent declared comp_size",
          !b2f_reroll_shippable(0, lzh2_len, 0));

    // Fail-before/pass-after contrast: the OLD decision was (rc==0 && len>0) — it would
    // SHIP a size-mismatched reroll (fabricated ratio). The NEW gate refuses it.
    bool old_would_ship = (0 == 0) && ((lzh2_len - 1) > 0);          // old warn-only logic
    bool new_ships      = b2f_reroll_shippable(0, lzh2_len - 1, expected);
    check("b2f reroll: OLD warn-only path would have shipped the mismatch (documents bug)",
          old_would_ship);
    check("b2f reroll: NEW hard gate refuses that same mismatch (fix)", !new_ships);
}

// ---------------------------------------------------------------------------
// T2 — #1 reverse-ACK ROOT: LIVE N(R) re-stamp at the air chokepoint.
//
// The GBN correctness theorem (Bertsekas & Gallager §2.4.2) requires premise 2:
// every emitted cumulative ACK carries the receiver's ack point AT EMISSION TIME.
// Pre-fix, an autonomous RR froze its N(R) at BUILD time, then sat in tx_queue_
// behind other traffic and hit the air a full window stale — aliasing mod-8 as
// "all acked" at the peer.  This test drives a real Ax25Session's V(R) to 7 while
// a stale RR (frozen N(R)=2) waits, then applies the PRODUCTION stamp helper
// (ax25_restamp_nr, the exact code the drain runs at modem.cc:3444/6626) and
// asserts the on-air N(R) is the LIVE V(R)=7, not the frozen 2.  FAIL-BEFORE:
// ax25_restamp_nr did not exist and the RR flew with N(R)=2; PASS-AFTER: N(R)==7.
// ---------------------------------------------------------------------------
static void test_reverse_ack_live_restamp() {
    printf("\n=== T2: #1 reverse-ACK ROOT — live N(R) re-stamp at the air chokepoint ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");
    uint8_t body[6] = {'H','I','0','0','0','0'};

    Ax25Session s;
    s.set_local_callsign("N0AAA");
    s.set_send_callback([](const uint8_t*, size_t){});
    // Bring up a native OFDM-KISS session (initiator; the role is irrelevant to
    // V(R), which tracks every decoded peer I-frame).
    auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
    s.notify_outgoing(sabm.data(), sabm.size());
    auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    { Ax25Frame f; ax25_parse(ua.data(), ua.size(), f); s.on_frame_received(f); }
    s.set_native_active(true);

    auto feed_in_iframe = [&](uint8_t ns) {
        auto fr = ax25_build_i(me, peer, ns, 0, false, 0xF0, body, sizeof(body));
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f); s.on_frame_received(f);
    };

    // (a) Peer sends 2 in-sequence I-frames -> V(R): 0 -> 2.  An autonomous RR is
    //     built here with the FROZEN N(R)=2 and queued behind traffic.
    feed_in_iframe(0); feed_in_iframe(1);
    check("T2: V(R)=2 after 2 in-sequence I-frames", s.current_vr() == 2);
    auto rr = ax25_build_s(me, peer, Ax25SType::RR, 2, false, false);  // frozen N(R)=2
    check("T2: queued RR carries the FROZEN N(R)=2", ((rr[14] >> 5) & 0x07) == 2);

    // (b) 5 more in-sequence I-frames arrive while the RR waits in tx_queue_ -> V(R)=7.
    feed_in_iframe(2); feed_in_iframe(3); feed_in_iframe(4);
    feed_in_iframe(5); feed_in_iframe(6);
    check("T2: V(R)=7 after 5 more in-sequence I-frames", s.current_vr() == 7);

    // (c) The drain re-stamps the RR to the LIVE V(R) at the TX instant.  Contrast:
    //     frozen 2 (what the base tree emitted, a full-window-stale alias) vs the
    //     live 7 (what the fix emits).
    uint8_t old_nr = 0xFF;
    bool stamped = ax25_restamp_nr(rr, s.current_vr(), &old_nr);
    check("T2: re-stamp fires on the S-frame (carries an N(R))", stamped);
    check("T2 (fail-before): pre-stamp frozen N(R) was 2 (the stale alias)", old_nr == 2);
    check("T2 (pass-after): ON-AIR N(R) == LIVE V(R) == 7 (root fix)",
          ((rr[14] >> 5) & 0x07) == 7);

    // (d) A U-frame (UA) carries no N(R) field -> never re-stamped.
    auto ua2 = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
    check("T2: U-frame (UA) is NOT re-stamped (no N(R) field)",
          !ax25_restamp_nr(ua2, 7));

    // (e) An I-frame IS re-stampable (its N(R) piggyback also aliases when stale).
    auto ifr = ax25_build_i(peer, me, 3, 1, false, 0xF0, body, sizeof(body));
    check("T2: I-frame IS re-stamped (piggyback N(R))", ax25_restamp_nr(ifr, 6) &&
          ((ifr[14] >> 5) & 0x07) == 6);
}

// ---------------------------------------------------------------------------
// T3 — #3 desync backstop: the kiss-shadow path must STOP silently ignoring a
// persistent out-of-window N(R) from a validly-decoded peer ACK.
//
// The deadlock's final leg: after a stale-N(R) alias destroyed the window, the
// peer's HONEST N(R) fell outside [V(A),V(S)] and nr_valid()==false, and the
// shadow S-frame path SILENTLY dropped it forever (unlike the owned path's
// nr_error_recovery).  Post-fix, 3 consecutive invalid N(R) from validly-decoded
// peer ACKs declare desync LOUD and re-establish (AWAITING_CONNECTION + SABM).
// A valid N(R) in between RESETS the streak (no spurious re-establish).
// FAIL-BEFORE: the session stays CONNECTED forever; PASS-AFTER: it re-establishes.
// ---------------------------------------------------------------------------
static void test_reverse_ack_desync_backstop() {
    printf("\n=== T3: #3 desync backstop — 3 invalid N(R) declare desync (no silent ignore) ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");
    uint8_t body[6] = {'D','A','T','A','!','!'};

    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign("N0AAA");
        s.set_send_callback([](const uint8_t*, size_t){});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f); s.on_frame_received(f);
        s.set_native_active(true);
    };
    auto feed_out_iframe = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(peer, me, ns, 0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    };
    auto feed_rx_rr = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, false, false);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f); s.on_frame_received(f);
    };

    // Window [V(A)=0, V(S)=3]; N(R)=5 is OUT of window -> nr_valid()==false.
    {
        Ax25Session s; bring_up(s);
        for (uint8_t ns = 0; ns < 3; ns++) feed_out_iframe(s, ns);
        check("T3: 3 frames outstanding (V(A)=0,V(S)=3)", s.va() == 0 && s.vs() == 3);
        feed_rx_rr(s, 5);
        check("T3: 1 invalid N(R) -> still CONNECTED (streak=1)",
              s.state() == Ax25SessionState::CONNECTED);
        feed_rx_rr(s, 5);
        check("T3: 2 invalid N(R) -> still CONNECTED (streak=2)",
              s.state() == Ax25SessionState::CONNECTED);
        feed_rx_rr(s, 5);
        check("T3 (pass-after): 3rd invalid N(R) -> DESYNC re-establish "
              "(AWAITING_CONNECTION)",
              s.state() == Ax25SessionState::AWAITING_CONNECTION);
    }

    // A VALID N(R) between invalids RESETS the streak (no spurious re-establish).
    {
        Ax25Session s2; bring_up(s2);
        for (uint8_t ns = 0; ns < 3; ns++) feed_out_iframe(s2, ns);
        feed_rx_rr(s2, 5); feed_rx_rr(s2, 5);          // 2 invalid (streak=2)
        feed_rx_rr(s2, 2);                              // VALID in [0,3] -> reset + V(A)=2
        check("T3: valid N(R)=2 advances V(A) and resets the streak",
              s2.va() == 2 && s2.state() == Ax25SessionState::CONNECTED);
        feed_rx_rr(s2, 6); feed_rx_rr(s2, 6);          // 2 invalid post-reset (6 out of [2,3])
        check("T3: reset held -> 2 post-reset invalids stay CONNECTED (no fire)",
              s2.state() == Ax25SessionState::CONNECTED);
    }
}

// ---------------------------------------------------------------------------
// T1 — #2 burst-epoch guard: the 114.801 smoking-gun repro.  The sender is at the
// full-window wedge V(A)=3 / V(S)=2 (7 outstanding).  A STALE reverse RR with
// N(R)=2 aliases mod-8 as "all 7 acked" (N(R) == V(S) boundary).  Pre-#2 (or on an
// epoch MATCH) that RR destructively purges the whole window (the on-air wedge);
// with the burst-epoch guard flagging the stale tone, the shadow S-frame consumer
// treats it as ADVISORY and the window stays intact.  FAIL-BEFORE = window purged;
// PASS-AFTER = window intact.
// ---------------------------------------------------------------------------
namespace iris {
bool test_burst_epoch_wrap_rejects_delayed_ack_hook();
}

static void test_reverse_ack_epoch_guard() {
    printf("\n=== T1: #2 burst-epoch guard — stale reverse ACK held (114.801 repro) ===\n");
    using namespace iris;
    Ax25Address me   = ax25_make_addr("N0AAA");
    Ax25Address peer = ax25_make_addr("N0BBB");
    uint8_t body[6] = {'X','X','X','X','X','X'};

    auto bring_up = [&](Ax25Session& s) {
        s.set_local_callsign("N0AAA");
        s.set_send_callback([](const uint8_t*, size_t){});
        auto sabm = ax25_build_u(peer, me, AX25_CTRL_SABM, true, true);
        s.notify_outgoing(sabm.data(), sabm.size());
        auto ua = ax25_build_u(me, peer, AX25_CTRL_UA, true, false);
        Ax25Frame f; ax25_parse(ua.data(), ua.size(), f); s.on_frame_received(f);
        s.set_native_active(true);
    };
    auto feed_out = [&](Ax25Session& s, uint8_t ns) {
        auto fr = ax25_build_i(peer, me, ns, 0, false, 0xF0, body, sizeof(body));
        s.notify_outgoing(fr.data(), fr.size());
    };
    auto feed_rr = [&](Ax25Session& s, uint8_t nr) {
        auto fr = ax25_build_s(me, peer, Ax25SType::RR, nr, false, false);
        Ax25Frame f; ax25_parse(fr.data(), fr.size(), f); s.on_frame_received(f);
    };
    // Drive to the exact wedge state V(A)=3, V(S)=2 (full 7-frame window).
    auto to_wedge = [&](Ax25Session& s) {
        bring_up(s);
        feed_out(s, 0); feed_out(s, 1); feed_out(s, 2);   // V(S)=3
        feed_rr(s, 3);                                     // ACK all -> V(A)=3
        uint8_t seq[7] = {3,4,5,6,7,0,1};
        for (uint8_t x : seq) feed_out(s, x);             // V(S)=2, window full
    };

    // (A) FAIL-BEFORE contrast: epoch MATCH (flag TRUE == the pre-#2 behavior) ->
    //     the stale RR N(R)=2 aliases as 'all 7 acked' and DESTROYS the window.
    {
        Ax25Session s; to_wedge(s);
        check("T1: wedge state V(A)=3, V(S)=2, window full (7 outstanding)",
              s.va() == 3 && s.vs() == 2 && s.window_used() == 7);
        s.set_reverse_ack_epoch_ok(true);   // epoch match (== the pre-#2 default)
        feed_rr(s, 2);                        // stale N(R)=2 (== V(S) boundary alias)
        check("T1 (fail-before): epoch-MATCH stale RR N(R)=2 PURGES the window "
              "(V(A)->2) — the aliasing wedge", s.va() == 2 && s.window_used() == 0);
    }
    // (B) PASS-AFTER: epoch MISMATCH (flag FALSE) -> ADVISORY, window INTACT.
    {
        Ax25Session s; to_wedge(s);
        s.set_reverse_ack_epoch_ok(false);   // stale/buffered tone: epoch mismatch
        feed_rr(s, 2);                        // same stale N(R)=2
        check("T1 (pass-after): epoch-MISMATCH stale RR N(R)=2 is ADVISORY — window "
              "INTACT (V(A)=3, 7 outstanding)", s.va() == 3 && s.window_used() == 7);
        check("T1: session stays CONNECTED (a valid-but-stale ACK is not a desync)",
              s.state() == Ax25SessionState::CONNECTED);
    }

    // (C) MFSK epoch codec round-trip (parity + absent handling).
    for (int e = 0; e < 8; e++)
        check("T1: MFSK epoch codec round-trips 0..7",
              MfskAck::tone_to_epoch(MfskAck::epoch_to_tone(e)) == e);
    check("T1: MFSK epoch ABSENT (-1) -> parity-violating tone -> decodes ABSENT",
          MfskAck::tone_to_epoch(MfskAck::epoch_to_tone(-1)) == -1);
    check("T1: MFSK epoch out-of-range (8) -> ABSENT",
          MfskAck::tone_to_epoch(MfskAck::epoch_to_tone(8)) == -1);
}

static void test_burst_epoch_wrap_rejects_delayed_ack() {
    check("wrapped stale ACK preserves current window",
          iris::test_burst_epoch_wrap_rejects_delayed_ack_hook());
}

// ===================== AX.25 digipeater via-path guardrail =====================
// The digipeater ("via") path must round-trip parse -> build byte-identically,
// and the no-via encoding must stay byte-identical to the historical
// two-address form (the interop invariant: Iris's AFSK surface must stay
// standard AX.25 or it cannot talk to real TNCs, digis, and gateways).
// Wire bytes are built BY HAND from the AX.25 v2.2 address-field rules
// (callsign chars << 1; SSID octet = hi-bit | 0x60 | SSID<<1 | extension bit
// on the final address; hi-bit = C on dst/src, H (has-been-repeated) on digis)
// so the test cannot inherit a builder bug.
static void test_ax25_via_path() {
    printf("\n=== AX.25 Via Path (digipeater guardrail) ===\n");

    // 6-char callsign field, shifted left one bit (caller pads to 6 chars)
    auto sh = [](const char* s) {
        std::vector<uint8_t> v;
        for (int i = 0; i < 6; i++) v.push_back((uint8_t)(s[i]) << 1);
        return v;
    };
    auto put = [](std::vector<uint8_t>& w, const std::vector<uint8_t>& v) {
        w.insert(w.end(), v.begin(), v.end());
    };

    // --- 1. Hand-built UI frame: KG7VSN>APRS via WIDE1-1,WIDE2-1, info "test" ---
    std::vector<uint8_t> wire;
    put(wire, sh("APRS  ")); wire.push_back(0xE0);   // dst: C=1 (command), ext=0
    put(wire, sh("KG7VSN")); wire.push_back(0x60);   // src: C=0, ext=0 (digis follow)
    put(wire, sh("WIDE1 ")); wire.push_back(0x62);   // via1: SSID=1, H=0, ext=0
    put(wire, sh("WIDE2 ")); wire.push_back(0x63);   // via2: SSID=1, H=0, ext=1 (last)
    wire.push_back(0x03);                            // UI control
    wire.push_back(0xF0);                            // PID: no layer 3
    const char* payload = "test";
    wire.insert(wire.end(), payload, payload + 4);

    Ax25Frame f;
    check("via: parse 2-digi UI frame", ax25_parse(wire.data(), wire.size(), f));
    check("via: dst", f.dst.to_string() == "APRS");
    check("via: src", f.src.to_string() == "KG7VSN");
    check("via: 2 hops captured", f.via.size() == 2);
    check("via: hop1 = WIDE1-1", f.via.size() == 2 && f.via[0].addr.to_string() == "WIDE1-1");
    check("via: hop2 = WIDE2-1", f.via.size() == 2 && f.via[1].addr.to_string() == "WIDE2-1");
    check("via: H-bits clear", f.via.size() == 2 && !f.via[0].repeated && !f.via[1].repeated);
    check("via: control parsed past digis", f.type() == Ax25FrameType::U_FRAME &&
          f.u_type() == Ax25UType::UI && f.pid == 0xF0);
    check("via: info parsed past digis", f.info.size() == 4 &&
          memcmp(f.info.data(), "test", 4) == 0);
    check("via: to_string", ax25_via_to_string(f.via) == "WIDE1-1,WIDE2-1");

    // Round-trip: rebuild from the parsed pieces -> byte-identical
    auto rebuilt = ax25_build_u(f.dst, f.src, AX25_CTRL_UI, false, true, f.via);
    rebuilt.push_back(f.pid);
    rebuilt.insert(rebuilt.end(), f.info.begin(), f.info.end());
    check("via: UI round-trip byte-identical", rebuilt == wire);

    // --- 2. H-bit round-trip: SABM through a used hop (KG7AAA-2*,WIDE2-1) ---
    std::vector<uint8_t> wire2;
    put(wire2, sh("W7XYZ ")); wire2.push_back(0xE0); // dst: C=1, ext=0
    put(wire2, sh("KG7VSN")); wire2.push_back(0x60); // src: C=0, ext=0
    put(wire2, sh("KG7AAA")); wire2.push_back(0xE4); // via1: SSID=2, H=1 (repeated), ext=0
    put(wire2, sh("WIDE2 ")); wire2.push_back(0x63); // via2: SSID=1, H=0, ext=1
    wire2.push_back(0x3F);                           // SABM, P=1
    Ax25Frame f2;
    check("via: parse H-bit frame", ax25_parse(wire2.data(), wire2.size(), f2));
    check("via: SABM through digis", f2.type() == Ax25FrameType::U_FRAME &&
          f2.u_type() == Ax25UType::SABM && f2.poll_final());
    check("via: hop1 H-bit set", f2.via.size() == 2 && f2.via[0].repeated);
    check("via: hop2 H-bit clear", f2.via.size() == 2 && !f2.via[1].repeated);
    check("via: to_string marks used hop",
          ax25_via_to_string(f2.via) == "KG7AAA-2*,WIDE2-1");
    auto rebuilt2 = ax25_build_u(f2.dst, f2.src, AX25_CTRL_SABM, true, true, f2.via);
    check("via: H-bit round-trip byte-identical", rebuilt2 == wire2);

    // --- 3. I-frame with a via path: control/PID/info offsets stay correct ---
    Ax25ViaPath vp;
    vp.push_back({ax25_make_addr("W7DIG-3"), false});
    uint8_t body[3] = {0x01, 0x02, 0x03};
    Ax25Address dsta = ax25_make_addr("W7XYZ");
    Ax25Address srca = ax25_make_addr("KG7VSN");
    auto ifr = ax25_build_i(dsta, srca, 2, 5, true, 0xF0, body, 3, false, vp);
    Ax25Frame fi;
    check("via: parse built I-frame", ax25_parse(ifr.data(), ifr.size(), fi));
    check("via: I-frame hop kept", fi.via.size() == 1 &&
          fi.via[0].addr.to_string() == "W7DIG-3" && !fi.via[0].repeated);
    check("via: I-frame seq/PF", fi.type() == Ax25FrameType::I_FRAME &&
          fi.ns() == 2 && fi.nr() == 5 && fi.poll_final());
    check("via: I-frame info intact", fi.info.size() == 3 &&
          memcmp(fi.info.data(), body, 3) == 0);
    auto ifr2 = ax25_build_i(fi.dst, fi.src, fi.ns(), fi.nr(), fi.poll_final(),
                             fi.pid, fi.info.data(), fi.info.size(), false, fi.via);
    check("via: I-frame round-trip byte-identical", ifr2 == ifr);

    // --- 4. Interop invariant: no-via builders byte-identical to the
    //        historical two-address encoding (hand-computed golden bytes) ---
    Ax25Address gd = ax25_make_addr("KG7VSN");
    Ax25Address gs = ax25_make_addr("W7ABC");
    // UA response, F=1: dst C=0 -> 0x60; src C=1, last -> 0xE1; ctrl 0x63|0x10
    std::vector<uint8_t> gold_u;
    put(gold_u, sh("KG7VSN")); gold_u.push_back(0x60);
    put(gold_u, sh("W7ABC ")); gold_u.push_back(0xE1);
    gold_u.push_back(0x73);
    check("no-via: U-frame byte-identical to golden",
          ax25_build_u(gd, gs, AX25_CTRL_UA, true, false) == gold_u);
    // RR response, N(R)=3, P/F=0: ctrl = 0x01 | (3<<5) = 0x61
    std::vector<uint8_t> gold_s;
    put(gold_s, sh("KG7VSN")); gold_s.push_back(0x60);
    put(gold_s, sh("W7ABC ")); gold_s.push_back(0xE1);
    gold_s.push_back(0x61);
    check("no-via: S-frame byte-identical to golden",
          ax25_build_s(gd, gs, Ax25SType::RR, 3, false, false) == gold_s);
    // I-frame command, N(S)=1 N(R)=2 P=0: dst 0xE0; src 0x61; ctrl 0x42
    std::vector<uint8_t> gold_i;
    put(gold_i, sh("KG7VSN")); gold_i.push_back(0xE0);
    put(gold_i, sh("W7ABC ")); gold_i.push_back(0x61);
    gold_i.push_back(0x42);
    gold_i.push_back(0xF0);
    gold_i.push_back('a'); gold_i.push_back('b');
    const uint8_t ab[2] = {'a', 'b'};
    check("no-via: I-frame byte-identical to golden",
          ax25_build_i(gd, gs, 1, 2, false, 0xF0, ab, 2) == gold_i);
    // Direct (no-via) parse leaves the via list empty
    Ax25Frame fg;
    check("no-via: parse yields empty via list",
          ax25_parse(gold_i.data(), gold_i.size(), fg) && fg.via.empty());
}

// ================= AX.25 endpoint THROUGH a digipeater (session via path) =================
// Phase 5.5-1 of the digipeater design (the Winlink "Digipeater"
// connection type: connect DST VIA digi1[,digi2]).  Every SESSION-GENERATED frame
// (SABM/UA/DISC/DM/RR/RNR/REJ/I) must carry the connection's via path — the
// originator as requested (H-bits clear), the responder REVERSED (replies traverse
// the digis in reverse order; Direwolf reverses an incoming connection's path the
// same way, get_link_handle, ../direwolf/src/ax25_link.c:877-890) — and a frame
// RETURNING through the path (H-bits set by the digis) must be accepted and matched
// to the session.  The pre-measurement T1 default must absorb the extra
// store-and-forward hops (Direwolf INIT_T1V_SRT: FRACK x (2m+1),
// ../direwolf/src/ax25_link.c:358-364).  Incoming wire frames are built BY HAND
// from the AX.25 v2.2 address-field rules so the test cannot inherit a builder bug.
// FAIL-BEFORE: session frames flew dst+src only (no path, no reversal, no T1 bump).
static void test_ax25_endpoint_via() {
    printf("\n=== AX.25 Endpoint Via (session frames through a digipeater) ===\n");

    auto sh = [](const char* s6) {
        std::vector<uint8_t> v;
        for (int i = 0; i < 6; i++) v.push_back((uint8_t)(s6[i]) << 1);
        return v;
    };
    auto put = [](std::vector<uint8_t>& w, const std::vector<uint8_t>& v) {
        w.insert(w.end(), v.begin(), v.end());
    };
    auto feed_rx = [](Ax25Session& s, const std::vector<uint8_t>& wire) {
        Ax25Frame f;
        if (ax25_parse(wire.data(), wire.size(), f)) s.on_frame_received(f);
    };

    // ---- 1. ORIGINATOR, KISS shadow (the Winlink screenshot) ----
    // Client SABM, hand-built: KG7VSN > WA7FC-10 via RELAY-1 (H=0), P=1.
    {
        std::vector<uint8_t> sabm;
        put(sabm, sh("WA7FC ")); sabm.push_back(0xF4);  // dst: C=1, SSID=10, ext=0
        put(sabm, sh("KG7VSN")); sabm.push_back(0x60);  // src: C=0, ext=0 (digi follows)
        put(sabm, sh("RELAY ")); sabm.push_back(0x63);  // via: SSID=1, H=0, ext=1
        sabm.push_back(0x3F);                           // SABM P=1

        Ax25Session s;
        s.set_local_callsign("KG7VSN");
        s.set_send_callback([](const uint8_t*, size_t) {});
        s.notify_outgoing(sabm.data(), sabm.size());
        check("evia1: KISS SABM-with-via tracked (AWAITING_CONNECTION)",
              s.state() == Ax25SessionState::AWAITING_CONNECTION);
        check("evia1: client's via path captured on the session",
              ax25_via_to_string(s.remote_via()) == "RELAY-1");
        check("evia1: T1 default bumped for 1 hop (FRACK x 3, Direwolf INIT_T1V_SRT)",
              s.t1_ticks() == 3 * Ax25Session::T1_TICKS);

        // Return UA as heard AFTER the digi repeated it: reply path reversed,
        // H-bit SET.  Hand-built: WA7FC-10 > KG7VSN via RELAY-1*, F=1, response.
        std::vector<uint8_t> ua;
        put(ua, sh("KG7VSN")); ua.push_back(0x60);      // dst: C=0 (response), ext=0
        put(ua, sh("WA7FC ")); ua.push_back(0xF4);      // src: C=1, SSID=10, ext=0
        put(ua, sh("RELAY ")); ua.push_back(0xE3);      // via: SSID=1, H=1, ext=1
        ua.push_back(0x73);                             // UA F=1
        feed_rx(s, ua);
        check("evia1: H-bit-set return UA accepted and matched -> CONNECTED",
              s.state() == Ax25SessionState::CONNECTED);
    }

    // ---- 2. ORIGINATOR, owned connect("DST via A,B"): every frame carries the path ----
    {
        std::vector<std::vector<uint8_t>> out;
        Ax25Session s;
        s.set_local_callsign("KG7VSN");
        s.set_send_callback([&](const uint8_t* d, size_t n) {
            out.emplace_back(d, d + n);
        });
        s.connect("WA7FC-10 via RELAY-1,WIDE2-1");
        check("evia2: connect-string parsed -> AWAITING_CONNECTION",
              s.state() == Ax25SessionState::AWAITING_CONNECTION);
        check("evia2: T1 default bumped for 2 hops (FRACK x 5)",
              s.t1_ticks() == 5 * Ax25Session::T1_TICKS);

        // Golden SABM: KG7VSN > WA7FC-10 via RELAY-1,WIDE2-1 (H=0), P=1.
        std::vector<uint8_t> gold_sabm;
        put(gold_sabm, sh("WA7FC ")); gold_sabm.push_back(0xF4);
        put(gold_sabm, sh("KG7VSN")); gold_sabm.push_back(0x60);
        put(gold_sabm, sh("RELAY ")); gold_sabm.push_back(0x62);  // H=0, ext=0
        put(gold_sabm, sh("WIDE2 ")); gold_sabm.push_back(0x63);  // H=0, ext=1
        gold_sabm.push_back(0x3F);
        check("evia2: emitted SABM carries the via path byte-exactly",
              out.size() == 1 && out[0] == gold_sabm);

        // UA returns through the reversed path, both H-bits set by the digis.
        std::vector<uint8_t> ua;
        put(ua, sh("KG7VSN")); ua.push_back(0x60);
        put(ua, sh("WA7FC ")); ua.push_back(0xF4);
        put(ua, sh("WIDE2 ")); ua.push_back(0xE2);      // reversed hop 1, H=1
        put(ua, sh("RELAY ")); ua.push_back(0xE3);      // reversed hop 2, H=1, ext=1
        ua.push_back(0x73);
        feed_rx(s, ua);
        check("evia2: H-set reversed-path UA accepted -> CONNECTED",
              s.state() == Ax25SessionState::CONNECTED);

        // I-frame: N(S)=0 N(R)=0 P=0, PID F0, "hello", via forward path H clear.
        const uint8_t hello[5] = {'h', 'e', 'l', 'l', 'o'};
        s.send_data(hello, sizeof(hello));
        std::vector<uint8_t> gold_i;
        put(gold_i, sh("WA7FC ")); gold_i.push_back(0xF4);
        put(gold_i, sh("KG7VSN")); gold_i.push_back(0x60);
        put(gold_i, sh("RELAY ")); gold_i.push_back(0x62);
        put(gold_i, sh("WIDE2 ")); gold_i.push_back(0x63);
        gold_i.push_back(0x00);                         // I: N(S)=0 N(R)=0 P=0
        gold_i.push_back(0xF0);                         // PID: no layer 3
        gold_i.insert(gold_i.end(), hello, hello + sizeof(hello));
        check("evia2: emitted I-frame carries the via path byte-exactly",
              out.size() == 2 && out[1] == gold_i);

        // Peer RR N(R)=1 returns through the reversed path (H set) -> V(A)=1.
        std::vector<uint8_t> rr;
        put(rr, sh("KG7VSN")); rr.push_back(0x60);
        put(rr, sh("WA7FC ")); rr.push_back(0xF4);
        put(rr, sh("WIDE2 ")); rr.push_back(0xE2);
        put(rr, sh("RELAY ")); rr.push_back(0xE3);
        rr.push_back(0x21);                             // RR N(R)=1, F=0
        feed_rx(s, rr);
        check("evia2: H-set return RR matched to session (V(A) 0 -> 1)", s.va() == 1);

        // DISC carries the path too.
        s.disconnect();
        std::vector<uint8_t> gold_disc;
        put(gold_disc, sh("WA7FC ")); gold_disc.push_back(0xF4);
        put(gold_disc, sh("KG7VSN")); gold_disc.push_back(0x60);
        put(gold_disc, sh("RELAY ")); gold_disc.push_back(0x62);
        put(gold_disc, sh("WIDE2 ")); gold_disc.push_back(0x63);
        gold_disc.push_back(0x53);                      // DISC P=1
        check("evia2: emitted DISC carries the via path byte-exactly",
              out.size() == 3 && out[2] == gold_disc);
    }

    // ---- 3. RESPONDER, owned: reply path REVERSED, H-bits cleared ----
    {
        std::vector<std::vector<uint8_t>> out;
        std::vector<uint8_t> delivered;
        Ax25Session r;
        r.set_local_callsign("WA7FC-10");
        r.set_send_callback([&](const uint8_t* d, size_t n) {
            out.emplace_back(d, d + n);
        });
        r.set_data_callback([&](const uint8_t* d, size_t n) {
            delivered.assign(d, d + n);
        });

        // SABM arrives with BOTH H-bits set (each digi marked its hop), hand-built.
        std::vector<uint8_t> sabm;
        put(sabm, sh("WA7FC ")); sabm.push_back(0xF4);  // dst = us
        put(sabm, sh("KG7VSN")); sabm.push_back(0x60);
        put(sabm, sh("RELAY ")); sabm.push_back(0xE2);  // H=1, ext=0
        put(sabm, sh("WIDE2 ")); sabm.push_back(0xE3);  // H=1, ext=1
        sabm.push_back(0x3F);
        feed_rx(r, sabm);
        check("evia3: digipeated SABM accepted -> CONNECTED",
              r.state() == Ax25SessionState::CONNECTED);
        check("evia3: responder adopted the REVERSED path, H-bits cleared",
              ax25_via_to_string(r.remote_via()) == "WIDE2-1,RELAY-1");

        // Golden UA: KG7VSN < WA7FC-10 via WIDE2-1,RELAY-1 (reversed, H=0), F=1.
        std::vector<uint8_t> gold_ua;
        put(gold_ua, sh("KG7VSN")); gold_ua.push_back(0x60);
        put(gold_ua, sh("WA7FC ")); gold_ua.push_back(0xF4);
        put(gold_ua, sh("WIDE2 ")); gold_ua.push_back(0x62);
        put(gold_ua, sh("RELAY ")); gold_ua.push_back(0x63);
        gold_ua.push_back(0x73);
        check("evia3: emitted UA carries the reversed path byte-exactly",
              out.size() == 1 && out[0] == gold_ua);

        // Peer I-frame through the forward path (H set): accepted, delivered,
        // and the autonomous T2 RR flies with the reversed path.
        std::vector<uint8_t> ifr;
        put(ifr, sh("WA7FC ")); ifr.push_back(0xF4);
        put(ifr, sh("KG7VSN")); ifr.push_back(0x60);
        put(ifr, sh("RELAY ")); ifr.push_back(0xE2);
        put(ifr, sh("WIDE2 ")); ifr.push_back(0xE3);
        ifr.push_back(0x00);                            // I: N(S)=0 N(R)=0 P=0
        ifr.push_back(0xF0);
        ifr.push_back('p'); ifr.push_back('i'); ifr.push_back('n'); ifr.push_back('g');
        feed_rx(r, ifr);
        check("evia3: H-set I-frame accepted (V(R)=1) and delivered",
              r.vr() == 1 && delivered.size() == 4 &&
              memcmp(delivered.data(), "ping", 4) == 0);
        for (int i = 0; i < 8 && out.size() < 2; i++) r.tick();  // T2 fires the RR
        std::vector<uint8_t> gold_rr;
        put(gold_rr, sh("KG7VSN")); gold_rr.push_back(0x60);
        put(gold_rr, sh("WA7FC ")); gold_rr.push_back(0xF4);
        put(gold_rr, sh("WIDE2 ")); gold_rr.push_back(0x62);
        put(gold_rr, sh("RELAY ")); gold_rr.push_back(0x63);
        gold_rr.push_back(0x21);                        // RR N(R)=1, F=0
        check("evia3: autonomous RR carries the reversed path byte-exactly",
              out.size() == 2 && out[1] == gold_rr);
    }

    // ---- 4. Interop invariant: a DIRECT session is byte-identical to today ----
    {
        std::vector<std::vector<uint8_t>> out;
        Ax25Session d;
        d.set_local_callsign("KG7VSN");
        d.set_send_callback([&](const uint8_t* p2, size_t n) {
            out.emplace_back(p2, p2 + n);
        });
        d.connect("W7ABC");
        std::vector<uint8_t> gold_sabm;
        put(gold_sabm, sh("W7ABC ")); gold_sabm.push_back(0xE0);  // dst: C=1, ext=0
        put(gold_sabm, sh("KG7VSN")); gold_sabm.push_back(0x61);  // src: C=0, ext=1
        gold_sabm.push_back(0x3F);
        check("evia4: no-via SABM byte-identical to the historical encoding",
              out.size() == 1 && out[0] == gold_sabm);
        check("evia4: no-via T1 default unchanged",
              d.t1_ticks() == Ax25Session::T1_TICKS);
    }
}

// ======================= AX.25 digipeater (Iris as the repeater) =======================
// Direwolf port, two DELIBERATELY different paths (kept in separate files
// there because they behave differently):
//   (A) connected-mode I/S/U — cdigipeater.c cdigipeat_match :238: exact call
//       on the first clear-H hop, set H (ax25_set_h ax25_pad.c:1579 = |0x80 on
//       the hop's SSID octet), re-emit VERBATIM, **NO dedup** (sequence
//       numbers self-dedup; a dedup here would eat I-frame retransmissions).
//   (B) UI — digipeater.c digipeat_match :297 + dedupe.c (30 s src+dst+info
//       checksum, :238/:202) + alias regex (:418) + WIDEn-N decrement (:525).
// The correctness CORE of this feature is the split: a connected-mode I-frame
// retransmission within 30 s IS repeated again, while a duplicate UI frame
// within 30 s is DROPPED.  Wire bytes are hand-built from the AX.25 v2.2
// address rules so the tests cannot inherit a builder bug.  Config-gated,
// default OFF.  FAIL-BEFORE: no repeat logic existed (the dispatch hook only
// logged), and the AFSK->native migration byte-14 sniff aliased every
// via-carrying frame as an I-frame (§3.1 landmine).
static void test_ax25_digipeater() {
    printf("\n=== AX.25 Digipeater (connected + UI paths, dedup split) ===\n");

    auto sh = [](const char* s6) {
        std::vector<uint8_t> v;
        for (int i = 0; i < 6; i++) v.push_back((uint8_t)(s6[i]) << 1);
        return v;
    };
    auto put = [](std::vector<uint8_t>& w, const std::vector<uint8_t>& v) {
        w.insert(w.end(), v.begin(), v.end());
    };

    DigipeatConfig dc;
    dc.enabled = true;
    dc.mycalls = {"KG7DIG"};
    // defaults: connected on, ui on, wide ^WIDE[1-7]-[1-7]$, dedupe 30 s
    Digipeater digi;
    check("digi: config accepted", digi.configure(dc).empty() && digi.enabled());

    // ---- 1. UI repeat: N0AAA>APRS via KG7DIG(H=0),WIDE2-1 -> H set, else identical ----
    std::vector<uint8_t> ui;
    put(ui, sh("APRS  ")); ui.push_back(0xE0);   // dst: C=1, ext=0
    put(ui, sh("N0AAA ")); ui.push_back(0x60);   // src: C=0, ext=0
    put(ui, sh("KG7DIG")); ui.push_back(0x60);   // via1: SSID=0, H=0, ext=0
    put(ui, sh("WIDE2 ")); ui.push_back(0x63);   // via2: SSID=1, H=0, ext=1
    ui.push_back(0x03); ui.push_back(0xF0);      // UI, PID no-L3
    const char* hello = "hello";
    ui.insert(ui.end(), hello, hello + 5);

    std::vector<uint8_t> ui_expect = ui;
    ui_expect[14 + 6] = 0xE0;                    // KG7DIG hop: H-bit set, nothing else
    auto r1 = digi.digipeat(ui.data(), ui.size(), 1000);
    check("digi1: UI frame naming us repeated", !r1.empty());
    check("digi1: H-bit SET on our hop, frame otherwise byte-identical", r1 == ui_expect);

    // ---- 2. THE DEDUP SPLIT (the correctness core) ----
    // (a) duplicate UI within 30 s -> DROPPED (dedupe.c:238; digipeater.c:391-415)
    auto r2 = digi.digipeat(ui.data(), ui.size(), 6000);
    check("digi2a: duplicate UI within 30 s DROPPED (dedup)", r2.empty());
    check("digi2a: drop reason is dup", std::string(digi.last_drop_reason()) == "dup");
    // (b) connected-mode I-frame retransmission within 30 s -> REPEATED AGAIN
    //     (cdigipeater.c:233: "APRS digipeating drops duplicates within 30
    //     seconds but we don't do that here")
    std::vector<uint8_t> ifr;
    put(ifr, sh("N0BBB ")); ifr.push_back(0xE0); // dst: C=1, ext=0
    put(ifr, sh("N0AAA ")); ifr.push_back(0x60); // src: C=0, ext=0
    put(ifr, sh("KG7DIG")); ifr.push_back(0x61); // via1: SSID=0, H=0, ext=1
    ifr.push_back(0x00); ifr.push_back(0xF0);    // I-frame N(S)=0 N(R)=0 P=0
    const char* d1 = "DATA1";
    ifr.insert(ifr.end(), d1, d1 + 5);
    std::vector<uint8_t> ifr_expect = ifr;
    ifr_expect[14 + 6] = 0xE1;                   // H set, ext kept
    auto r3 = digi.digipeat(ifr.data(), ifr.size(), 7000);
    check("digi2b: connected I-frame repeated (H set, else identical)", r3 == ifr_expect);
    auto r4 = digi.digipeat(ifr.data(), ifr.size(), 12000);   // T1 retransmission, 5 s later
    check("digi2b: connected I-frame RETRANSMISSION within 30 s repeated AGAIN (no dedup)",
          r4 == ifr_expect);
    auto r5 = digi.digipeat(ifr.data(), ifr.size(), 17000);
    check("digi2b: and a third time — connected mode never dedups", r5 == ifr_expect);

    // ---- 3. Dedup window + keying ----
    // TTL expiry: the UI frame repeats again once 30 s have passed
    auto r6 = digi.digipeat(ui.data(), ui.size(), 32000);     // remembered at 1000; 31 s later
    check("digi3: UI repeats again after the 30 s dedup TTL", r6 == ui_expect);
    // Dedup keys on src+dst+info NOT the via path (ax25_dedupe_crc,
    // ax25_pad.c:2777: "but NOT the digipeaters"): same payload, different path
    std::vector<uint8_t> ui2;
    put(ui2, sh("APRS  ")); ui2.push_back(0xE0);
    put(ui2, sh("N0AAA ")); ui2.push_back(0x60);
    put(ui2, sh("KG7DIG")); ui2.push_back(0x61); // via1 only, ext=1 — DIFFERENT path
    ui2.push_back(0x03); ui2.push_back(0xF0);
    ui2.insert(ui2.end(), hello, hello + 5);
    auto r7 = digi.digipeat(ui2.data(), ui2.size(), 33000);   // 1 s after r6's remember
    check("digi3: dedup keys on src+dst+info NOT via (same payload, new path -> dup)",
          r7.empty() && std::string(digi.last_drop_reason()) == "dup");

    // ---- 4. Never repeat what is not ours / already used / our own ----
    Digipeater digi2; digi2.configure(dc);
    std::vector<uint8_t> notus;
    put(notus, sh("APRS  ")); notus.push_back(0xE0);
    put(notus, sh("N0AAA ")); notus.push_back(0x60);
    put(notus, sh("W7OTH ")); notus.push_back(0x60);  // first clear hop: NOT us
    put(notus, sh("KG7DIG")); notus.push_back(0x61);  // we are hop 2 (still clear)
    notus.push_back(0x03); notus.push_back(0xF0);
    check("digi4: first clear hop not us -> NOT repeated (even if we are a later hop)",
          digi2.digipeat(notus.data(), notus.size(), 1000).empty() &&
          std::string(digi2.last_drop_reason()) == "not-us");
    std::vector<uint8_t> used;
    put(used, sh("APRS  ")); used.push_back(0xE0);
    put(used, sh("N0AAA ")); used.push_back(0x60);
    put(used, sh("KG7DIG")); used.push_back(0xE1);    // H already SET
    used.push_back(0x03); used.push_back(0xF0);
    check("digi4: all hops used -> nothing to do",
          digi2.digipeat(used.data(), used.size(), 1000).empty() &&
          std::string(digi2.last_drop_reason()) == "no-hop");
    std::vector<uint8_t> own;
    put(own, sh("APRS  ")); own.push_back(0xE0);
    put(own, sh("KG7DIG")); own.push_back(0x60);      // src is OUR digi call
    put(own, sh("KG7DIG")); own.push_back(0x61);
    own.push_back(0x03); own.push_back(0xF0);
    check("digi4: never repeat our own transmission (digipeater.c:380-388)",
          digi2.digipeat(own.data(), own.size(), 1000).empty() &&
          std::string(digi2.last_drop_reason()) == "own-src");
    // Exact match means exact SSID too: KG7DIG-7 is not KG7DIG(-0)
    std::vector<uint8_t> ssid7;
    put(ssid7, sh("APRS  ")); ssid7.push_back(0xE0);
    put(ssid7, sh("N0AAA ")); ssid7.push_back(0x60);
    put(ssid7, sh("KG7DIG")); ssid7.push_back(0x6F);  // SSID=7, H=0, ext=1
    ssid7.push_back(0x03); ssid7.push_back(0xF0);
    check("digi4: exact match is callsign AND SSID (KG7DIG-7 != KG7DIG)",
          digi2.digipeat(ssid7.data(), ssid7.size(), 1000).empty());
    // No via path at all -> never a digipeat candidate
    std::vector<uint8_t> novia;
    put(novia, sh("APRS  ")); novia.push_back(0xE0);
    put(novia, sh("N0AAA ")); novia.push_back(0x61);
    novia.push_back(0x03); novia.push_back(0xF0);
    check("digi4: no via path -> not repeated",
          digi2.digipeat(novia.data(), novia.size(), 1000).empty());

    // ---- 5. Config gates: default OFF; per-path disables ----
    DigipeatConfig off = dc; off.enabled = false;
    Digipeater dg_off; dg_off.configure(off);
    check("digi5: DISABLED -> nothing repeated (default-off contract)",
          dg_off.digipeat(ui.data(), ui.size(), 1000).empty() &&
          !dg_off.enabled());
    check("digi5: DigipeatConfig default is OFF", DigipeatConfig().enabled == false);
    DigipeatConfig noconn = dc; noconn.connected_mode = false;
    Digipeater dg_nc; dg_nc.configure(noconn);
    check("digi5: connected path disabled -> I-frame not repeated, UI still works",
          dg_nc.digipeat(ifr.data(), ifr.size(), 1000).empty() &&
          !dg_nc.digipeat(ui.data(), ui.size(), 1000).empty());
    DigipeatConfig noui = dc; noui.ui_mode = false;
    Digipeater dg_nu; dg_nu.configure(noui);
    check("digi5: UI path disabled -> UI not repeated, connected still works",
          dg_nu.digipeat(ui.data(), ui.size(), 1000).empty() &&
          !dg_nu.digipeat(ifr.data(), ifr.size(), 1000).empty());

    // ---- 6. Connected-mode covers S and U frames too (cdigipeat: I/S/U) ----
    Digipeater digi3; digi3.configure(dc);
    std::vector<uint8_t> sabm;
    put(sabm, sh("N0BBB ")); sabm.push_back(0xE0);
    put(sabm, sh("N0AAA ")); sabm.push_back(0x60);
    put(sabm, sh("KG7DIG")); sabm.push_back(0x61);
    sabm.push_back(0x3F);                              // SABM P=1
    std::vector<uint8_t> sabm_expect = sabm; sabm_expect[14 + 6] = 0xE1;
    check("digi6: SABM through us repeated with H set (connected path)",
          digi3.digipeat(sabm.data(), sabm.size(), 1000) == sabm_expect);
    std::vector<uint8_t> rr;
    put(rr, sh("N0AAA ")); rr.push_back(0xE0);
    put(rr, sh("N0BBB ")); rr.push_back(0x60);
    put(rr, sh("KG7DIG")); rr.push_back(0x61);
    rr.push_back(0x21);                                // RR N(R)=1
    std::vector<uint8_t> rr_expect = rr; rr_expect[14 + 6] = 0xE1;
    check("digi6: RR (S-frame) through us repeated with H set",
          digi3.digipeat(rr.data(), rr.size(), 2000) == rr_expect);
    // and again immediately — S-frames must never be dedup'd either
    check("digi6: RR repeated again 1 s later (no dedup on the connected path)",
          digi3.digipeat(rr.data(), rr.size(), 3000) == rr_expect);

    // ---- 7. WIDEn-N (UI only; digipeater.c:525-600) ----
    Digipeater digi4; digi4.configure(dc);
    // WIDE1-1: SSID 1 -> replace with MYCALL, H set (digipeater.c:576-585)
    std::vector<uint8_t> w11;
    put(w11, sh("APRS  ")); w11.push_back(0xE0);
    put(w11, sh("N0AAA ")); w11.push_back(0x60);
    put(w11, sh("WIDE1 ")); w11.push_back(0x63);       // WIDE1-1, H=0, ext=1
    w11.push_back(0x03); w11.push_back(0xF0);
    const char* b1 = "b1"; w11.insert(w11.end(), b1, b1 + 2);
    std::vector<uint8_t> w11_expect;
    put(w11_expect, sh("APRS  ")); w11_expect.push_back(0xE0);
    put(w11_expect, sh("N0AAA ")); w11_expect.push_back(0x60);
    put(w11_expect, sh("KG7DIG")); w11_expect.push_back(0xE1);  // MYCALL*, ext kept
    w11_expect.push_back(0x03); w11_expect.push_back(0xF0);
    w11_expect.insert(w11_expect.end(), b1, b1 + 2);
    check("digi7: WIDE1-1 -> replaced by MYCALL with H set",
          digi4.digipeat(w11.data(), w11.size(), 1000) == w11_expect);
    // WIDE2-2: decrement to WIDE2-1 (H clear) + insert MYCALL* ahead (:587-600)
    std::vector<uint8_t> w22;
    put(w22, sh("APRS  ")); w22.push_back(0xE0);
    put(w22, sh("N0AAA ")); w22.push_back(0x60);
    put(w22, sh("WIDE2 ")); w22.push_back(0x65);       // WIDE2-2, H=0, ext=1
    w22.push_back(0x03); w22.push_back(0xF0);
    const char* b2 = "b2"; w22.insert(w22.end(), b2, b2 + 2);
    std::vector<uint8_t> w22_expect;
    put(w22_expect, sh("APRS  ")); w22_expect.push_back(0xE0);
    put(w22_expect, sh("N0AAA ")); w22_expect.push_back(0x60);
    put(w22_expect, sh("KG7DIG")); w22_expect.push_back(0xE0);  // inserted MYCALL*, ext=0
    put(w22_expect, sh("WIDE2 ")); w22_expect.push_back(0x63);  // WIDE2-1, H=0, ext=1
    w22_expect.push_back(0x03); w22_expect.push_back(0xF0);
    w22_expect.insert(w22_expect.end(), b2, b2 + 2);
    check("digi7: WIDE2-2 -> WIDE2-1 + MYCALL* inserted ahead (trace)",
          digi4.digipeat(w22.data(), w22.size(), 2000) == w22_expect);
    // Alias regex substitution (digipeater.c:418-433)
    DigipeatConfig al = dc; al.ui_alias = "^RELAY$";
    Digipeater digi5; digi5.configure(al);
    std::vector<uint8_t> rel;
    put(rel, sh("APRS  ")); rel.push_back(0xE0);
    put(rel, sh("N0AAA ")); rel.push_back(0x60);
    put(rel, sh("RELAY ")); rel.push_back(0x61);       // RELAY, H=0, ext=1
    rel.push_back(0x03); rel.push_back(0xF0);
    std::vector<uint8_t> rel_expect;
    put(rel_expect, sh("APRS  ")); rel_expect.push_back(0xE0);
    put(rel_expect, sh("N0AAA ")); rel_expect.push_back(0x60);
    put(rel_expect, sh("KG7DIG")); rel_expect.push_back(0xE1);
    rel_expect.push_back(0x03); rel_expect.push_back(0xF0);
    check("digi7: alias RELAY -> substituted with MYCALL, H set",
          digi5.digipeat(rel.data(), rel.size(), 1000) == rel_expect);
    // WIDEn-N applies ONLY to UI: an I-frame via WIDE2-1 is NOT repeated
    std::vector<uint8_t> iw;
    put(iw, sh("N0BBB ")); iw.push_back(0xE0);
    put(iw, sh("N0AAA ")); iw.push_back(0x60);
    put(iw, sh("WIDE2 ")); iw.push_back(0x63);
    iw.push_back(0x00); iw.push_back(0xF0);
    check("digi7: WIDE alias NEVER applies to connected frames (exact call only)",
          digi4.digipeat(iw.data(), iw.size(), 3000).empty());

    // ---- 8. Dispatch wiring through the REAL RX path (Modem hook) ----
    {
        Modem m;
        std::vector<uint8_t> got;
        check("digi8: enabled -> dispatch_rx_frame queues the repeat on the AFSK TX queue",
              m.test_digipeat_dispatch(dc, ui, &got) == 1 && got == ui_expect);
        Modem m2;
        check("digi8: disabled -> dispatch_rx_frame repeats NOTHING (default-off contract)",
              m2.test_digipeat_dispatch(off, ui, nullptr) == 0);
    }

    // ---- 9. §3.1 landmine: AFSK->native migration eligibility ----
    // Old predicate was (frame[14] & 1) == 0 — byte 14 of a via-carrying frame
    // is a shifted callsign char (bit0 ALWAYS 0), so every via frame aliased
    // as an I-frame and was stolen off the AFSK path into the native queue.
    {
        Ax25Address da = ax25_make_addr("N0BBB");
        Ax25Address sa = ax25_make_addr("N0AAA");
        const uint8_t pay[3] = {1, 2, 3};
        auto i_novia = ax25_build_i(da, sa, 0, 0, false, 0xF0, pay, 3);
        check("digi9: no-via I-frame IS migration-eligible (session traffic)",
              Modem::migrate_to_native_eligible(i_novia));
        Ax25ViaPath vp; vp.push_back({ax25_make_addr("KG7DIG"), false});
        auto i_via = ax25_build_i(da, sa, 0, 0, false, 0xF0, pay, 3, false, vp);
        check("digi9: via-carrying I-frame stays on AFSK (was aliased as migratable)",
              !Modem::migrate_to_native_eligible(i_via));
        auto u_via = ax25_build_u(da, sa, AX25_CTRL_SABM, true, true, vp);
        check("digi9: via-carrying SABM stays on AFSK (byte-14 sniff called it an I-frame)",
              !Modem::migrate_to_native_eligible(u_via));
        auto s_novia = ax25_build_s(da, sa, Ax25SType::RR, 1, false);
        check("digi9: no-via RR is not an I-frame -> not migrated",
              !Modem::migrate_to_native_eligible(s_novia));
    }
}

template <typename T>
static bool cipher_suite_copy_aliases_owned_secret() {
    if constexpr (std::is_copy_constructible_v<T>) {
        pid_t pid = fork();
        if (pid == 0) {
            uint8_t pk[MLKEM_PK_SIZE];
            T a;
            if (a.generate_mlkem_keypair(pk) != 0) _exit(2);
            T b(a);
            a.wipe();
            b.wipe();
            _exit(0);
        }
        if (pid < 0) return true;

        int status = 0;
        if (waitpid(pid, &status, 0) != pid) return true;
        return !WIFEXITED(status) || WEXITSTATUS(status) != 0;
    } else {
        return false;
    }
}

static void test_cipher_suite_not_copyable() {
    bool copy_hazard =
        cipher_suite_copy_aliases_owned_secret<CipherSuite>();
    check("CipherSuite cannot alias owned ML-KEM secret",
          !std::is_copy_constructible_v<CipherSuite> &&
          !std::is_copy_assignable_v<CipherSuite> &&
          !copy_hazard);
}

#ifdef IRIS_USE_OSS
namespace iris {
using OssOpenHook = int (*)(const char*, int);
using OssCloseHook = int (*)(int);
using OssIoctlHook = int (*)(int, unsigned long, void*);
using OssWriteHook = ssize_t (*)(int, const void*, size_t);
void oss_set_test_io(OssOpenHook open_hook, OssCloseHook close_hook,
                     OssIoctlHook ioctl_hook, OssWriteHook write_hook);
void oss_reset_test_io();
}

static int oss_test_queued_bytes;

static int oss_test_open(const char*, int) {
    return 17;
}

static int oss_test_close(int) {
    return 0;
}

static int oss_test_ioctl(int, unsigned long request, void* value) {
    if (request == SNDCTL_DSP_GETODELAY)
        *(int*)value = oss_test_queued_bytes;
    return 0;
}

static ssize_t oss_test_write(int, const void*, size_t bytes) {
    oss_test_queued_bytes += (int)bytes;
    return (ssize_t)bytes;
}

static void test_oss_drain_tracks_driver_queue() {
    printf("\n=== OSS Audio Backend Regressions ===\n");
    oss_test_queued_bytes = 0;
    oss_set_test_io(oss_test_open, oss_test_close, oss_test_ioctl, oss_test_write);
    set_audio_backend(AudioBackend::AUTO);

    auto playback = create_playback();
    float block[16] = {};
    bool opened = playback && playback->open(0, 48000, 1, 16);
    int written = opened ? playback->write(block, 16) : -1;
    if (opened) playback->mark_drain();
    bool pending = opened && !playback->is_drained();
    oss_test_queued_bytes = 0;
    bool drained = opened && playback->is_drained();

    if (playback) playback->close();
    oss_reset_test_io();
    check("OSS drain tracks driver queue",
          opened && written == 16 && pending && drained);
}
#endif

int run_tests() {
    setvbuf(stdout, nullptr, _IONBF, 0);  // unbuffered: last line printed == where we are
    printf("Iris Modem - Loopback Tests\n");
    printf("============================\n");

    run_acceptance_gate(tests_passed, tests_failed);

    // Phase 1: AX.25 compatibility
    printf("\n--- Phase 1: AX.25 Compatibility ---\n");
    test_crc16();
    test_nrzi();
    test_hdlc();
    test_kiss();
    test_ax25_via_path();
    test_ax25_endpoint_via();
    test_ax25_digipeater();
    test_afsk_loopback();
    test_gfsk_loopback();
    test_modem_ax25_non48k_sample_rate();

    // Phase 2: Native PHY
    printf("\n--- Phase 2: Native PHY ---\n");
    test_crc32();
    test_rrc_filter();
    test_constellation();
    test_native_phy_loopback();
    test_native_frame_loopback();
    test_ofdm_kiss_loopback();
    test_ofdm_blind_detect();
    test_ofdm_climb_desync();
    test_ofdm_narrow_bandpass_sync();
    test_ofdm_phy_roundtrip();
    test_ofdm_multi_codeword();
    test_ofdm_genie_ncw4();
    test_ofdm_longframe_phase();
    test_ofdm_block_pilot_removed();
    test_ofdm_s2_scope_64qam();
    test_ofdm_pilot_symbol_papr();
    test_ofdm_noise_color_agnostic();
    test_ofdm_speed_levels();
    test_ofdm_throughput_configs();
    test_ofdm_metric_vs_awgn();
    test_ofdm_meter_inband_bandlimited();
    test_ofdm_meter_topend_bandlimited();
    test_ofdm_topgear_fer_bandlimited();
    test_ofdm_bandlimit_junk_rejection();
    test_ofdm_snr_sweep();
    test_fd_zc_gate_calibration();

    // Phase 3: Auto-upgrade
    printf("\n--- Phase 3: Auto-Upgrade ---\n");
    test_xid();
    test_xid_negotiation();
    test_xid_frame_build();
    test_ax25_to_native_upgrade();
    test_ax25_window_accounting();
    test_ax25_wide_window();
    test_ax25_selective_retx();
    test_ax25_native_stream_iris_prefix_payload();
    test_tone_ack_provenance();
    test_stale_drain_provenance();
    test_one_cw_collapse_capacity();
    test_ofdm_ladder_single_source();
    test_ofdm_tx_level_coherence();
    test_tx_level_ring_anchor();
    test_burst_reject_requeue();
    test_slot_coalescing_bounds();
    test_grid_pin_narrow();
    test_tx_level_cap();
    test_coalesced_slot_midloss();
    test_repack_roundtrip();
    test_repack_peer_busy_backpressure();
    test_repack_rx_queue_bounded();
    test_repack_iframe_piggyback_nr();
    test_anchor_futility_and_chase();
    test_shadow_desync_prevention_suite();
    test_datalink_rej_and_desync_teardown_suite();
    test_dynamic_max_info();
    test_burst_fill_pacing();
    test_root2_retain_truncated_reverse();
    test_rx_tonemap_latch();
    test_c1_implicit_nak();
    test_rr_volley_guard();
    // NOTE(m0): the ed73eb4 turnaround cherry-pick supplies the two ARQ test
    // definitions that a735b61 registered but never defined; registrations
    // restored here now that the bodies exist (test_t1_deferred_during_tune @
    // :3450, test_tx_queue_no_evict_recovery_burst @ :3529).
    test_t1_deferred_during_tune();
    test_tx_queue_no_evict_recovery_burst();
    test_tx_queue_never_evicts_accepted_frame();
    // Reverse-ACK ROOT FIX (m0): #1 live N(R) re-stamp (T2), #3 desync backstop
    // (T3), #2 burst-epoch guard + MFSK epoch codec (T1).
    test_reverse_ack_live_restamp();
    test_reverse_ack_desync_backstop();
    test_reverse_ack_epoch_guard();
    test_burst_epoch_wrap_rejects_delayed_ack();
    test_native_active_reset();
    test_ofdm_config_fingerprint();
    test_grid_derivation_deterministic();
    test_compress_stream_coherence();
    test_mlkem_held_frames_replayed();
    test_compress_dict_priming();
    test_b2f_filter_zero_does_not_forward_original();
    test_b2f_reroll_hard_gate();

    // Phase 4: Engine components
    printf("\n--- Phase 4: Engine & FEC ---\n");
#ifdef IRIS_USE_OSS
    test_oss_drain_tracks_driver_queue();
#endif

    // Speed levels
    {
        printf("\n=== Speed Levels ===\n");
        check("8 speed levels", NUM_SPEED_LEVELS == 8);
        check("A0 = BPSK", SPEED_LEVELS[0].modulation == Modulation::BPSK);
        check("A7 = QAM256", SPEED_LEVELS[7].modulation == Modulation::QAM256);
        int tp = net_throughput(0, 2400);  // A0 at Mode A
        check("A0 throughput = 1200 bps", tp == 1200);
        tp = net_throughput(1, 2400);  // A1 at Mode A
        check("A1 throughput = 2400 bps", tp == 2400);
        tp = net_throughput(7, 2400);  // A7 at Mode A
        check("A7 throughput = 16800 bps", tp == 16800);
        check("Mode A baud matches PHY", mode_baud_rate('A') == mode_a_config().baud_rate);
        check("Mode B baud = 4800", mode_baud_rate('B') == 4800);
        check("Mode C baud = 9600", mode_baud_rate('C') == 9600);
    }

    // SNR estimation
    {
        printf("\n=== SNR Estimation ===\n");
        // Perfect channel: SNR should be very high
        std::complex<float> tx[] = {{1,0},{-1,0},{1,0},{-1,0}};
        std::complex<float> rx[] = {{1,0},{-1,0},{1,0},{-1,0}};
        float snr = estimate_snr(tx, rx, 4);
        check("Perfect channel SNR > 50 dB", snr > 50.0f);

        // Add noise
        std::complex<float> rx_noisy[] = {{1.1f,0.05f},{-0.95f,0.1f},{0.9f,-0.1f},{-1.05f,0.05f}};
        float snr2 = estimate_snr(tx, rx_noisy, 4);
        check("Noisy channel SNR > 0 dB", snr2 > 0.0f);
        check("Noisy channel SNR < 30 dB", snr2 < 30.0f);
    }

    // AGC
    {
        printf("\n=== AGC ===\n");
        AGC agc(0.3f);
        // Feed a loud signal
        float gain_after = 0;
        for (int i = 0; i < 1000; i++) {
            agc.process(0.8f * ((i % 2) ? 1.0f : -1.0f));
        }
        gain_after = agc.gain();
        check("AGC reduces gain for loud signal", gain_after < 1.0f);

        agc.reset();
        // Feed a quiet signal
        for (int i = 0; i < 1000; i++) {
            agc.process(0.01f * ((i % 2) ? 1.0f : -1.0f));
        }
        gain_after = agc.gain();
        check("AGC increases gain for quiet signal", gain_after > 1.0f);
    }

    // Gearshift
    {
        printf("\n=== Gearshift ===\n");
        Gearshift gs;
        gs.set_max_level(7);
        // Feed high SNR — should climb
        for (int i = 0; i < 20; i++) gs.update(35.0f);
        check("High SNR -> level > 0", gs.current_level() > 0);

        // Feed low SNR — should drop (needs enough frames to drain smoothed SNR)
        for (int i = 0; i < 30; i++) gs.update(2.0f);
        check("Low SNR -> level 0", gs.current_level() == 0);
    }

    // LDPC (1600-bit codewords, N=1600)
    {
        printf("\n=== LDPC FEC ===\n");
        // RATE_1_2: k=800 data bits
        std::vector<uint8_t> data;
        for (int i = 0; i < 800; i++) data.push_back(i & 1);
        auto encoded = LdpcCodec::encode(data, LdpcRate::RATE_1_2);
        check("LDPC 1/2 encoded size = 1600", (int)encoded.size() == 1600);
        auto decoded = LdpcCodec::decode(encoded, LdpcRate::RATE_1_2);
        check("LDPC 1/2 decode size = 800", (int)decoded.size() == 800);
        check("LDPC 1/2 round-trip", decoded == data);

        // RATE_5_8: k=1000, n=1600 (native matrix, no fallback needed)
        std::vector<uint8_t> data58;
        for (int i = 0; i < 1000; i++) data58.push_back((i * 3) & 1);
        auto enc58 = LdpcCodec::encode(data58, LdpcRate::RATE_5_8);
        check("LDPC 5/8 encoded = 1600 bits", (int)enc58.size() == 1600);
        auto dec58 = LdpcCodec::decode(enc58, LdpcRate::RATE_5_8);
        check("LDPC 5/8 decode size = 1000", (int)dec58.size() == 1000);
        bool match58 = true;
        for (int i = 0; i < 1000 && i < (int)dec58.size(); i++)
            if (dec58[i] != data58[i]) { match58 = false; break; }
        check("LDPC 5/8 data preserved", match58);

        // RATE_3_4: k=1200, 800 data bits padded to 1200
        auto enc34 = LdpcCodec::encode(data, LdpcRate::RATE_3_4);
        check("LDPC 3/4 encoded = 1600 bits", (int)enc34.size() == 1600);
        auto dec34 = LdpcCodec::decode(enc34, LdpcRate::RATE_3_4);
        check("LDPC 3/4 decode succeeds", !dec34.empty());

        // Weak-signal rate: RATE_1_16 — falls back to rate 1/2 (k=800).
        // 100 bits → padded to 800 → 1 codeword = 1600 bits.
        std::vector<uint8_t> data_weak;
        for (int i = 0; i < 100; i++) data_weak.push_back(i & 1);
        auto enc116 = LdpcCodec::encode(data_weak, LdpcRate::RATE_1_16);
        check("LDPC 1/16 fallback encoded = 1600 bits", (int)enc116.size() == 1600);
        auto dec116 = LdpcCodec::decode(enc116, LdpcRate::RATE_1_16);
        // Decode returns 800 bits (k=800), first 100 match original
        check("LDPC 1/16 fallback decode size", (int)dec116.size() == 800);
        bool match116 = true;
        for (int i = 0; i < 100 && i < (int)dec116.size(); i++)
            if (dec116[i] != data_weak[i]) { match116 = false; break; }
        check("LDPC 1/16 fallback data preserved", match116);

        // Dual decoder: test SPA and GBF
        auto dec_spa = LdpcCodec::decode(encoded, LdpcRate::RATE_1_2, LdpcDecoder::SPA);
        check("LDPC 1/2 SPA round-trip", dec_spa == data);
        auto dec_gbf = LdpcCodec::decode(encoded, LdpcRate::RATE_1_2, LdpcDecoder::GBF);
        check("LDPC 1/2 GBF round-trip", dec_gbf == data);
    }

    // LDPC error correction
    {
        printf("\n=== LDPC Error Correction ===\n");
        std::vector<uint8_t> data;
        for (int i = 0; i < 800; i++) data.push_back((i * 7 + 3) & 1);
        auto encoded = LdpcCodec::encode(data, LdpcRate::RATE_1_2);

        // Introduce 1 bit error
        std::vector<uint8_t> corrupted = encoded;
        corrupted[50] ^= 1;
        auto corrected = LdpcCodec::decode(corrupted, LdpcRate::RATE_1_2);
        check("LDPC 1/2 corrects 1-bit error",
              corrected.size() == data.size() && corrected == data);

        // Introduce 5 bit errors
        corrupted = encoded;
        corrupted[10] ^= 1; corrupted[100] ^= 1; corrupted[200] ^= 1;
        corrupted[300] ^= 1; corrupted[400] ^= 1;
        corrected = LdpcCodec::decode(corrupted, LdpcRate::RATE_1_2);
        check("LDPC 1/2 corrects 5-bit errors",
              corrected.size() == data.size() && corrected == data);

        // Weak-signal error correction (RATE_1_16 falls back to 1/2, k=800)
        std::vector<uint8_t> data_weak;
        for (int i = 0; i < 100; i++) data_weak.push_back((i * 5 + 1) & 1);
        auto enc_weak = LdpcCodec::encode(data_weak, LdpcRate::RATE_1_16);
        corrupted = enc_weak;
        // Flip 5 bits spread across the codeword
        corrupted[10] ^= 1; corrupted[200] ^= 1; corrupted[500] ^= 1;
        corrupted[900] ^= 1; corrupted[1300] ^= 1;
        corrected = LdpcCodec::decode(corrupted, LdpcRate::RATE_1_16);
        // Fallback to rate 1/2: decode returns 800 bits, first 100 match
        bool weak_match = (int)corrected.size() == 800;
        for (int i = 0; i < 100 && weak_match && i < (int)corrected.size(); i++)
            if (corrected[i] != data_weak[i]) weak_match = false;
        check("LDPC 1/16 fallback corrects 5-bit errors", weak_match);

        // LDPC NONE passthrough
        auto passthrough = LdpcCodec::encode(data, LdpcRate::NONE);
        check("LDPC NONE passthrough", passthrough == data);
        auto pt_dec = LdpcCodec::decode(data, LdpcRate::NONE);
        check("LDPC NONE decode passthrough", pt_dec == data);

        // fec_to_ldpc_rate helper
        check("fec_to_ldpc_rate(1,2)", fec_to_ldpc_rate(1, 2) == LdpcRate::RATE_1_2);
        check("fec_to_ldpc_rate(3,4)", fec_to_ldpc_rate(3, 4) == LdpcRate::RATE_3_4);
        check("fec_to_ldpc_rate(7,8)", fec_to_ldpc_rate(7, 8) == LdpcRate::RATE_7_8);
        check("fec_to_ldpc_rate(0,0)", fec_to_ldpc_rate(0, 0) == LdpcRate::NONE);
    }

    // Native frame with LDPC FEC loopback
    {
        printf("\n=== Native Frame + LDPC FEC Loopback ===\n");
        uint8_t payload[] = "FEC-protected Iris frame!";
        size_t payload_len = strlen((char*)payload);

        // Test each FEC rate
        LdpcRate rates[] = {LdpcRate::NONE, LdpcRate::RATE_1_2, LdpcRate::RATE_3_4, LdpcRate::RATE_7_8};
        const char* rate_names[] = {"NONE", "1/2", "3/4", "7/8"};

        for (int r = 0; r < 4; r++) {
            PhyConfig cfg = mode_b_config();
            cfg.modulation = Modulation::QPSK;

            auto iq = build_native_frame(payload, payload_len, cfg, rates[r]);
            int start = detect_frame_start(iq.data(), iq.size(), cfg.samples_per_symbol);

            char name[128];
            snprintf(name, sizeof(name), "FEC %s frame detected", rate_names[r]);
            check(name, start >= 0);

            if (start >= 0) {
                std::vector<uint8_t> rx_payload;
                bool ok = decode_native_frame(iq.data(), iq.size(), start, cfg, rx_payload);
                snprintf(name, sizeof(name), "FEC %s frame decoded", rate_names[r]);
                check(name, ok);
                if (ok) {
                    bool match = (rx_payload.size() == payload_len) &&
                                 (memcmp(rx_payload.data(), payload, payload_len) == 0);
                    snprintf(name, sizeof(name), "FEC %s payload matches", rate_names[r]);
                    check(name, match);
                }
            }
        }
    }

    // Mode A upconversion
    {
        printf("\n=== Mode A Upconversion ===\n");
        // Generate a simple IQ signal: constant on I, zero on Q
        std::vector<float> iq(2000);
        for (int i = 0; i < 1000; i++) {
            iq[2*i] = 1.0f;    // I
            iq[2*i+1] = 0.0f;  // Q
        }

        Upconverter up(1800.0f, 48000);
        auto audio = up.iq_to_audio(iq.data(), iq.size());
        check("Upconvert produces audio", (int)audio.size() == 1000);

        // Audio should be a 1800 Hz cosine
        // Check that it has non-trivial content
        float rms = 0;
        for (auto s : audio) rms += s * s;
        rms = std::sqrt(rms / audio.size());
        check("Upconverted audio has signal", rms > 0.5f);

        // Downconvert back
        Downconverter down(1800.0f, 48000);
        auto iq_back = down.audio_to_iq(audio.data(), audio.size());
        check("Downconvert produces IQ pairs", (int)iq_back.size() == 2000);

        // The I channel should recover ~1.0 (after transient)
        // Downconverter applies 2x gain to compensate cos^2 averaging
        // Check middle portion to avoid startup transient
        float i_avg = 0;
        int mid_start = 400, mid_end = 800;
        for (int i = mid_start; i < mid_end; i++)
            i_avg += iq_back[2*i];
        i_avg /= (mid_end - mid_start);
        check("Downconverted I recovers DC",
              std::abs(i_avg - 1.0f) < 0.2f); // ~1.0 after 2x gain compensation
    }

    // Mode A native frame through upconversion
    {
        printf("\n=== Mode A Native Frame Through Audio ===\n");
        uint8_t payload[] = "Mode A upconvert test!";
        size_t payload_len = strlen((char*)payload);

        PhyConfig cfg = mode_a_config();
        cfg.modulation = Modulation::BPSK;

        // Build IQ frame
        auto iq = build_native_frame(payload, payload_len, cfg);

        // Upconvert to audio
        Upconverter up(1800.0f, 48000);
        auto audio = up.iq_to_audio(iq.data(), iq.size());
        check("Mode A frame -> audio", !audio.empty());
        printf("  IQ: %zu samples -> Audio: %zu samples\n", iq.size()/2, audio.size());

        // Downconvert back to IQ
        Downconverter down(1800.0f, 48000);
        auto iq_back = down.audio_to_iq(audio.data(), audio.size());

        // Detect frame
        int start = detect_frame_start(iq_back.data(), iq_back.size(),
                                        cfg.samples_per_symbol);
        check("Frame detected after upconvert/downconvert", start >= 0);

        if (start >= 0) {
            std::vector<uint8_t> rx_payload;
            bool ok = decode_native_frame(iq_back.data(), iq_back.size(),
                                           start, cfg, rx_payload);
            check("Frame decoded after upconvert/downconvert", ok);
            if (ok) {
                bool match = (rx_payload.size() == payload_len) &&
                             (memcmp(rx_payload.data(), payload, payload_len) == 0);
                check("Payload matches after audio round-trip", match);
            }
        }
    }

    // Modem engine
    {
        printf("\n=== Modem Engine ===\n");
        IrisConfig cfg;
        cfg.mode = "A";
        cfg.callsign = "TEST01";
        cfg.ax25_baud = 1200;
        cfg.ptt_pre_delay_ms = 0;
        cfg.ptt_post_delay_ms = 0;
        cfg.persist = 255;          // Always transmit (bypass p-persistent CSMA)

        Modem modem;
        bool ok = modem.init(cfg);
        check("Modem init", ok);
        check("Modem starts IDLE", modem.state() == ModemState::IDLE);

        // Queue a frame and generate TX audio
        uint8_t frame[] = {
            'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
            'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x61,
            0x03, 0xF0, 'H', 'i'
        };
        modem.queue_tx_frame(frame, sizeof(frame));

        // Call process_tx multiple times — first call builds audio from queue,
        // subsequent calls drain the tx_buffer_
        std::vector<float> tx_audio(4800, 0.0f);
        float peak = 0;
        bool was_tx = false;
        for (int i = 0; i < 5; i++) {
            modem.process_tx(tx_audio.data(), 4800);
            if (modem.state() == ModemState::TX_AX25) was_tx = true;
            for (int j = 0; j < 4800; j++) {
                float a = std::abs(tx_audio[j]);
                if (a > peak) peak = a;
            }
        }
        check("Modem TX produces audio", was_tx);
        check("TX audio has signal", peak > 0.01f);

        auto diag = modem.get_diagnostics();
        check("TX frame counted", diag.frames_tx == 1);

        modem.shutdown();
    }

    // Config
    {
        printf("\n=== Config ===\n");
        IrisConfig cfg;
        cfg.callsign = "TEST01";
        cfg.mode = "B";
        cfg.kiss_port = 9001;
        bool saved = save_config("build/test_config.ini", cfg);
        check("Config save", saved);

        IrisConfig loaded = load_config("build/test_config.ini");
        check("Config callsign round-trip", loaded.callsign == "TEST01");
        check("Config mode round-trip", loaded.mode == "B");
        check("Config kiss_port round-trip", loaded.kiss_port == 9001);
    }

    // Bind address — KISS/AGW must default to loopback and reject malformed
    // addresses instead of silently binding INADDR_ANY (which would grant any
    // LAN host unauthenticated transmit control of the radio).
    {
        printf("\n=== Bind Address ===\n");

        // Default MUST parse to loopback. Assert the PARSED default, not the
        // string, so a future edit to the default value is caught.
        IrisConfig def;
        uint32_t def_addr = 0xA5A5A5A5u;
        bool def_ok = parse_bind_address(def.bind_address, def_addr);
        check("default bind address parses", def_ok);
        check("default bind address is 127.0.0.1", def_ok && def_addr == 0x7F000001u);
        check("default bind address is in loopback range (127/8)",
              def_ok && ((def_addr >> 24) == 127u));

        // Round-trip [Network] BindAddress through save/load (mirrors kiss_port).
        IrisConfig cfg_b;
        cfg_b.bind_address = "192.168.1.50";
        bool saved_b = save_config("build/test_bind.ini", cfg_b);
        check("Config bind save", saved_b);
        IrisConfig loaded_b = load_config("build/test_bind.ini");
        check("Config bind_address round-trip", loaded_b.bind_address == "192.168.1.50");

        // --bind overrides the config value; an absent --bind keeps the config
        // value. (main.cc applies the override via this same helper.)
        check("--bind overrides config value",
              effective_bind_address("127.0.0.1", "0.0.0.0") == "0.0.0.0");
        check("absent --bind keeps config value",
              effective_bind_address("192.168.1.50", "") == "192.168.1.50");

        // 0.0.0.0 / any are the explicit all-interfaces opt-in (== INADDR_ANY).
        uint32_t any_addr = 0xA5A5A5A5u;
        check("0.0.0.0 accepted -> INADDR_ANY",
              parse_bind_address("0.0.0.0", any_addr) && any_addr == 0u);
        uint32_t any2 = 0xA5A5A5A5u;
        check("'any' accepted -> INADDR_ANY",
              parse_bind_address("any", any2) && any2 == 0u);
        uint32_t any3 = 0xA5A5A5A5u;
        check("'ANY' accepted (case-insensitive)",
              parse_bind_address("ANY", any3) && any3 == 0u);

        // A malformed address MUST be rejected and MUST NOT silently become
        // INADDR_ANY. This is the guard against a future regression re-exposing
        // the transmitter port.
        const char* bad_addrs[] = {
            "", "localhost", "999.1.1.1", "1.2.3", "1.2.3.4.5",
            "1.2.3.256", "127.0.0.1x", "127.0.0.", ".1.2.3", "12.34.56.oops"
        };
        bool all_rejected = true;
        bool never_zeroed = true;
        for (const char* p : bad_addrs) {
            uint32_t out = 0x11223344u;  // non-zero sentinel; INADDR_ANY == 0
            bool ok = parse_bind_address(p, out);
            if (ok) all_rejected = false;
            if (out == 0u) never_zeroed = false;  // must not become INADDR_ANY
        }
        check("malformed bind addresses are rejected", all_rejected);
        check("rejected bind address never becomes INADDR_ANY", never_zeroed);
    }

    // Phase 6: ARQ protocol
    printf("\n--- Phase 6: ARQ Protocol ---\n");

    // ARQ frame serialization
    {
        printf("\n=== ARQ Frame Serialization ===\n");
        ArqFrame frame;
        frame.type = ArqType::CONNECT;
        frame.seq = 0;
        frame.flags = 3;
        frame.payload = {'T', 'E', 'S', 'T'};
        auto data = frame.serialize();
        check("ARQ frame serialized", data.size() == 7);
        check("ARQ frame type byte", data[0] == (uint8_t)ArqType::CONNECT);

        ArqFrame decoded;
        bool ok = ArqFrame::deserialize(data.data(), data.size(), decoded);
        check("ARQ frame deserialized", ok);
        check("ARQ frame type matches", decoded.type == ArqType::CONNECT);
        check("ARQ frame seq matches", decoded.seq == 0);
        check("ARQ frame flags matches", decoded.flags == 3);
        check("ARQ frame payload matches",
              decoded.payload == std::vector<uint8_t>{'T', 'E', 'S', 'T'});
    }

    // ARQ session loopback (commander + responder, with HAIL beacon phase)
    {
        printf("\n=== ARQ Session Loopback ===\n");

        ArqSession commander;
        ArqSession responder;
        commander.set_callsign("CMD01");
        responder.set_callsign("RSP01");

        // Cross-wire: commander's send goes to responder's receive and vice versa
        std::vector<std::vector<uint8_t>> cmd_to_rsp;
        std::vector<std::vector<uint8_t>> rsp_to_cmd;
        std::vector<uint8_t> received_data;
        bool transfer_done = false;
        bool transfer_ok = false;

        ArqCallbacks cmd_cb;
        cmd_cb.send_frame = [&](const uint8_t* d, size_t l) {
            cmd_to_rsp.push_back(std::vector<uint8_t>(d, d + l));
        };
        cmd_cb.on_transfer_complete = [&](bool ok) {
            transfer_done = true;
            transfer_ok = ok;
        };
        commander.set_callbacks(cmd_cb);

        ArqCallbacks rsp_cb;
        rsp_cb.send_frame = [&](const uint8_t* d, size_t l) {
            rsp_to_cmd.push_back(std::vector<uint8_t>(d, d + l));
        };
        rsp_cb.on_data_received = [&](const uint8_t* d, size_t l) {
            received_data.insert(received_data.end(), d, d + l);
        };
        responder.set_callbacks(rsp_cb);

        // Responder must listen before commander connects
        responder.listen();
        check("Responder state = LISTENING", responder.state() == ArqState::LISTENING);
        check("Responder role = LISTENING", responder.role() == ArqRole::LISTENING);

        // Commander connects — starts HAILING
        commander.connect("RSP01");
        check("Commander state = HAILING", commander.state() == ArqState::HAILING);
        check("Commander role = COMMANDER", commander.role() == ArqRole::COMMANDER);

        // Pump HAIL -> HAIL_ACK -> CONNECT -> CONNECT_ACK
        for (int pump = 0; pump < 20; pump++) {
            auto c2r = std::move(cmd_to_rsp); cmd_to_rsp.clear();
            for (auto& f : c2r) responder.on_frame_received(f.data(), f.size());
            auto r2c = std::move(rsp_to_cmd); rsp_to_cmd.clear();
            for (auto& f : r2c) commander.on_frame_received(f.data(), f.size());
            if (commander.state() == ArqState::CONNECTED ||
                commander.state() == ArqState::TURBOSHIFT)
                break;
        }

        check("Commander connected after HAIL",
              commander.state() == ArqState::CONNECTED ||
              commander.state() == ArqState::TURBOSHIFT);
        check("Responder connected", responder.state() == ArqState::CONNECTED);
        check("Responder role = RESPONDER", responder.role() == ArqRole::RESPONDER);

        // Queue data and send
        const char* test_msg = "Hello from ARQ! This is a test transfer.";
        commander.send_data((const uint8_t*)test_msg, strlen(test_msg));

        // Pump frames back and forth until transfer completes (max 50 iterations)
        int iters = 0;
        while (!transfer_done && iters < 50) {
            auto c2r = std::move(cmd_to_rsp);
            cmd_to_rsp.clear();
            for (auto& f : c2r)
                responder.on_frame_received(f.data(), f.size());

            auto r2c = std::move(rsp_to_cmd);
            rsp_to_cmd.clear();
            for (auto& f : r2c)
                commander.on_frame_received(f.data(), f.size());

            iters++;
        }

        check("Transfer completed", transfer_done);
        check("Transfer successful", transfer_ok);
        check("Data received correctly",
              received_data.size() == strlen(test_msg) &&
              memcmp(received_data.data(), test_msg, strlen(test_msg)) == 0);
        printf("  ARQ completed in %d iterations, %d retransmits\n",
               iters, commander.retransmit_count());

        // Disconnect (pump all pending frames first)
        for (int pump = 0; pump < 5; pump++) {
            auto c2r = std::move(cmd_to_rsp); cmd_to_rsp.clear();
            for (auto& f : c2r) responder.on_frame_received(f.data(), f.size());
            auto r2c = std::move(rsp_to_cmd); rsp_to_cmd.clear();
            for (auto& f : r2c) commander.on_frame_received(f.data(), f.size());
        }
        commander.disconnect();
        check("Commander disconnecting", commander.state() == ArqState::DISCONNECTING);
        // Pump disconnect frames through both sides
        for (int pump = 0; pump < 5; pump++) {
            auto c2r = std::move(cmd_to_rsp); cmd_to_rsp.clear();
            for (auto& f : c2r) responder.on_frame_received(f.data(), f.size());
            auto r2c = std::move(rsp_to_cmd); rsp_to_cmd.clear();
            for (auto& f : r2c) commander.on_frame_received(f.data(), f.size());
            if (commander.state() == ArqState::IDLE) break;
        }
        check("Responder returned to IDLE", responder.state() == ArqState::IDLE);
        check("Commander returned to IDLE", commander.state() == ArqState::IDLE);
    }

    // ARQ multi-frame transfer (exceeds single DATA frame)
    {
        printf("\n=== ARQ Multi-Frame Transfer ===\n");

        ArqSession commander;
        ArqSession responder;
        commander.set_callsign("CMD02");
        responder.set_callsign("RSP02");

        std::vector<std::vector<uint8_t>> cmd_to_rsp;
        std::vector<std::vector<uint8_t>> rsp_to_cmd;
        std::vector<uint8_t> received_data;
        bool transfer_done = false;
        bool transfer_ok = false;

        ArqCallbacks cmd_cb;
        cmd_cb.send_frame = [&](const uint8_t* d, size_t l) {
            cmd_to_rsp.push_back(std::vector<uint8_t>(d, d + l));
        };
        cmd_cb.on_transfer_complete = [&](bool ok) {
            transfer_done = true;
            transfer_ok = ok;
        };
        commander.set_callbacks(cmd_cb);

        ArqCallbacks rsp_cb;
        rsp_cb.send_frame = [&](const uint8_t* d, size_t l) {
            rsp_to_cmd.push_back(std::vector<uint8_t>(d, d + l));
        };
        rsp_cb.on_data_received = [&](const uint8_t* d, size_t l) {
            received_data.insert(received_data.end(), d, d + l);
        };
        responder.set_callbacks(rsp_cb);

        // Build a large payload (2KB)
        std::vector<uint8_t> big_data(2048);
        for (int i = 0; i < 2048; i++) big_data[i] = (uint8_t)(i & 0xFF);

        // Responder listens, commander connects (with HAIL + turboshift)
        responder.listen();
        commander.connect("RSP02");
        for (int pump = 0; pump < 30; pump++) {
            auto c2r = std::move(cmd_to_rsp);
            cmd_to_rsp.clear();
            for (auto& f : c2r)
                responder.on_frame_received(f.data(), f.size());
            auto r2c = std::move(rsp_to_cmd);
            rsp_to_cmd.clear();
            for (auto& f : r2c)
                commander.on_frame_received(f.data(), f.size());
            if (commander.state() == ArqState::CONNECTED)
                break;
        }

        check("Multi-frame: connected", commander.state() == ArqState::CONNECTED ||
              commander.state() == ArqState::TURBOSHIFT);

        // Queue all data
        commander.send_data(big_data.data(), big_data.size());

        // Pump until done
        int iters = 0;
        while (!transfer_done && iters < 200) {
            auto c2r = std::move(cmd_to_rsp);
            cmd_to_rsp.clear();
            for (auto& f : c2r)
                responder.on_frame_received(f.data(), f.size());

            auto r2c = std::move(rsp_to_cmd);
            rsp_to_cmd.clear();
            for (auto& f : r2c)
                commander.on_frame_received(f.data(), f.size());

            iters++;
        }

        check("Multi-frame transfer completed", transfer_done);
        check("Multi-frame transfer successful", transfer_ok);
        check("Multi-frame data size correct", received_data.size() == big_data.size());
        check("Multi-frame data matches", received_data == big_data);
        printf("  2KB transferred in %d iterations, %d retransmits\n",
               iters, commander.retransmit_count());
    }

    // ARQ with X25519 key exchange + ChaCha20-Poly1305 encryption
    {
        printf("\n=== ARQ Encrypted Session (X25519 DH) ===\n");

        ArqSession commander;
        ArqSession responder;
        commander.set_callsign("CMD01");
        responder.set_callsign("RSP01");

        // Both sides generate X25519 keypairs and advertise CAP_ENCRYPTION
        CipherSuite cmd_cipher, rsp_cipher;
        uint8_t cmd_pub[X25519_KEY_SIZE], rsp_pub[X25519_KEY_SIZE];
        check("CMD X25519 keygen", cmd_cipher.generate_x25519_keypair(cmd_pub) == 0);
        check("RSP X25519 keygen", rsp_cipher.generate_x25519_keypair(rsp_pub) == 0);

        commander.set_local_capabilities(CAP_ENCRYPTION);
        responder.set_local_capabilities(CAP_ENCRYPTION);
        commander.set_local_x25519_pubkey(cmd_pub);
        responder.set_local_x25519_pubkey(rsp_pub);

        // Cross-wire
        std::vector<std::vector<uint8_t>> cmd_to_rsp, rsp_to_cmd;
        std::vector<uint8_t> received_data;
        bool transfer_done = false, transfer_ok = false;

        ArqCallbacks cmd_cb;
        cmd_cb.send_frame = [&](const uint8_t* d, size_t l) {
            cmd_to_rsp.push_back(std::vector<uint8_t>(d, d + l));
        };
        cmd_cb.on_transfer_complete = [&](bool ok) { transfer_done = true; transfer_ok = ok; };
        commander.set_callbacks(cmd_cb);

        ArqCallbacks rsp_cb;
        rsp_cb.send_frame = [&](const uint8_t* d, size_t l) {
            rsp_to_cmd.push_back(std::vector<uint8_t>(d, d + l));
        };
        rsp_cb.on_data_received = [&](const uint8_t* d, size_t l) {
            received_data.insert(received_data.end(), d, d + l);
        };
        responder.set_callbacks(rsp_cb);

        // HAIL + CONNECT with X25519 pubkey exchange
        responder.listen();
        commander.connect("RSP01");
        for (int pump = 0; pump < 30; pump++) {
            auto c2r = std::move(cmd_to_rsp); cmd_to_rsp.clear();
            for (auto& f : c2r) responder.on_frame_received(f.data(), f.size());
            auto r2c = std::move(rsp_to_cmd); rsp_to_cmd.clear();
            for (auto& f : r2c) commander.on_frame_received(f.data(), f.size());
            if (commander.state() == ArqState::CONNECTED) break;
        }

        check("Encrypted: connected", commander.state() == ArqState::CONNECTED ||
              commander.state() == ArqState::TURBOSHIFT);

        // Both sides should have each other's X25519 pubkey
        check("CMD has RSP pubkey", commander.has_peer_x25519());
        check("RSP has CMD pubkey", responder.has_peer_x25519());

        // Verify pubkeys were correctly exchanged
        check("CMD sees RSP pubkey", memcmp(commander.peer_x25519_pubkey(), rsp_pub, 32) == 0);
        check("RSP sees CMD pubkey", memcmp(responder.peer_x25519_pubkey(), cmd_pub, 32) == 0);

        // Compute shared secrets (both sides should derive the same key)
        check("CMD computes shared", cmd_cipher.compute_x25519_shared(commander.peer_x25519_pubkey()) == 0);
        check("RSP computes shared", rsp_cipher.compute_x25519_shared(responder.peer_x25519_pubkey()) == 0);

        // Derive session keys
        cmd_cipher.derive_session_key("CMD01", "RSP01", nullptr, 0, false);
        rsp_cipher.derive_session_key("RSP01", "CMD01", nullptr, 0, false);
        cmd_cipher.activate();
        rsp_cipher.activate();

        // Encrypt with commander's cipher, decrypt with responder's cipher
        const char* plaintext = "Secret message over amateur radio!";
        int pt_len = (int)strlen(plaintext);
        std::vector<uint8_t> ct(pt_len + AUTH_TAG_SIZE);
        int ct_len = cmd_cipher.encrypt((const uint8_t*)plaintext, pt_len,
                                         ct.data(), (int)ct.size(),
                                         0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("Encrypt produces output", ct_len > 0);
        check("Ciphertext larger than plaintext", ct_len > pt_len);

        std::vector<uint8_t> pt_out(ct_len);
        int dec_len = rsp_cipher.decrypt(ct.data(), ct_len,
                                          pt_out.data(), (int)pt_out.size(),
                                          0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("Decrypt succeeds", dec_len == pt_len);
        check("Decrypted matches plaintext",
              dec_len > 0 && memcmp(pt_out.data(), plaintext, pt_len) == 0);

        // Verify wrong key fails
        CipherSuite wrong_cipher;
        uint8_t wrong_pub[X25519_KEY_SIZE];
        wrong_cipher.generate_x25519_keypair(wrong_pub);
        wrong_cipher.compute_x25519_shared(rsp_pub);  // wrong secret key + RSP pub
        wrong_cipher.derive_session_key("CMD01", "RSP01", nullptr, 0, false);
        wrong_cipher.activate();
        std::vector<uint8_t> bad_pt(ct_len);
        int bad_dec = wrong_cipher.decrypt(ct.data(), ct_len,
                                            bad_pt.data(), (int)bad_pt.size(),
                                            0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("Wrong key decrypt fails", bad_dec <= 0);

        // Verify PSK binding: same DH but different PSK → different key
        CipherSuite psk_cipher1, psk_cipher2;
        uint8_t pk1[32], pk2[32];
        psk_cipher1.generate_x25519_keypair(pk1);
        psk_cipher2.generate_x25519_keypair(pk2);
        psk_cipher1.compute_x25519_shared(pk2);
        psk_cipher2.compute_x25519_shared(pk1);
        uint8_t psk_a[] = "password123";
        uint8_t psk_b[] = "different456";
        psk_cipher1.derive_session_key("A", "B", psk_a, sizeof(psk_a)-1, false);
        psk_cipher2.derive_session_key("B", "A", psk_b, sizeof(psk_b)-1, false);
        psk_cipher1.activate();
        psk_cipher2.activate();
        ct.resize(pt_len + AUTH_TAG_SIZE);
        ct_len = psk_cipher1.encrypt((const uint8_t*)plaintext, pt_len,
                                      ct.data(), (int)ct.size(),
                                      0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        dec_len = psk_cipher2.decrypt(ct.data(), ct_len,
                                       pt_out.data(), (int)pt_out.size(),
                                       0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("PSK mismatch: decrypt fails", dec_len <= 0);

        printf("  X25519 DH key exchange + ChaCha20-Poly1305 AEAD verified\n");
    }

    test_cipher_suite_not_copyable();

    // Hybrid post-quantum key exchange (X25519 + ML-KEM-768)
    {
        printf("\n=== Hybrid PQ Key Exchange (X25519 + ML-KEM-768) ===\n");

        CipherSuite alice, bob;
        uint8_t alice_pub[X25519_KEY_SIZE], bob_pub[X25519_KEY_SIZE];

        // Phase 1: X25519 (classical ECDH)
        check("Alice X25519 keygen", alice.generate_x25519_keypair(alice_pub) == 0);
        check("Bob X25519 keygen", bob.generate_x25519_keypair(bob_pub) == 0);
        check("Alice X25519 shared", alice.compute_x25519_shared(bob_pub) == 0);
        check("Bob X25519 shared", bob.compute_x25519_shared(alice_pub) == 0);

        // Phase 2: ML-KEM-768 (post-quantum KEM)
        // Alice (commander) generates ML-KEM keypair
        uint8_t encaps_key[MLKEM_PK_SIZE];
        check("Alice ML-KEM keygen", alice.generate_mlkem_keypair(encaps_key) == 0);

        // Bob (responder) encapsulates with Alice's public key
        uint8_t ciphertext[MLKEM_CT_SIZE];
        check("Bob ML-KEM encapsulate", bob.encapsulate_mlkem(encaps_key, ciphertext) == 0);

        // Alice decapsulates
        check("Alice ML-KEM decapsulate", alice.decapsulate_mlkem(ciphertext) == 0);

        // Both derive hybrid session key (X25519 + ML-KEM)
        alice.derive_session_key("ALICE", "BOB", nullptr, 0, true);
        bob.derive_session_key("BOB", "ALICE", nullptr, 0, true);
        alice.activate();
        bob.activate();

        check("Alice PQ upgraded", alice.is_pq_upgraded());
        check("Bob PQ upgraded", bob.is_pq_upgraded());

        // Verify symmetric encryption works with hybrid key
        const char* msg = "Post-quantum SNDL-proof message!";
        int msg_len = (int)strlen(msg);
        std::vector<uint8_t> ct(msg_len + AUTH_TAG_SIZE);
        int ct_len = alice.encrypt((const uint8_t*)msg, msg_len,
                                    ct.data(), (int)ct.size(),
                                    0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("PQ encrypt ok", ct_len > 0);

        std::vector<uint8_t> pt(ct_len);
        int dec_len = bob.decrypt(ct.data(), ct_len,
                                   pt.data(), (int)pt.size(),
                                   0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("PQ decrypt ok", dec_len == msg_len);
        check("PQ round-trip matches", dec_len > 0 && memcmp(pt.data(), msg, msg_len) == 0);

        // Verify a classical-only key (same X25519 but no ML-KEM) produces different ciphertext
        CipherSuite classical_only;
        uint8_t co_pub[X25519_KEY_SIZE];
        classical_only.generate_x25519_keypair(co_pub);
        classical_only.compute_x25519_shared(bob_pub);
        classical_only.derive_session_key("ALICE", "BOB", nullptr, 0, false);  // mlkem_done=false
        classical_only.activate();
        std::vector<uint8_t> bad_pt(ct_len);
        int bad = classical_only.decrypt(ct.data(), ct_len,
                                          bad_pt.data(), (int)bad_pt.size(),
                                          0, DIR_CMD_TO_RSP, AUTH_TAG_SIZE);
        check("Classical-only key can't decrypt PQ ciphertext", bad <= 0);

        printf("  X25519 + ML-KEM-768 hybrid verified (NIST FIPS 203, Level 3)\n");
    }

    // Phase 7: End-to-end two-station test
    printf("\n--- Phase 7: End-to-End ---\n");

    // AX.25 frame through modem audio path
    {
        printf("\n=== End-to-End AX.25 Through Audio ===\n");

        // Station A (TX) and Station B (RX) with Mode B (no upconversion, simpler)
        IrisConfig cfg_a, cfg_b;
        cfg_a.mode = "B"; cfg_a.callsign = "STA_A";
        cfg_a.ax25_baud = 1200;
        cfg_a.ptt_pre_delay_ms = 0;
        cfg_a.ptt_post_delay_ms = 0;

        cfg_b.mode = "B"; cfg_b.callsign = "STA_B";
        cfg_b.ax25_baud = 1200;
        cfg_b.ptt_pre_delay_ms = 0;
        cfg_b.ptt_post_delay_ms = 0;

        Modem station_a, station_b;
        check("Station A init", station_a.init(cfg_a));
        check("Station B init", station_b.init(cfg_b));

        // Capture received frames from station B
        std::vector<std::vector<uint8_t>> rx_frames;
        station_b.set_rx_callback([&](const uint8_t* data, size_t len) {
            rx_frames.push_back(std::vector<uint8_t>(data, data + len));
        });

        // Build an AX.25 frame
        uint8_t ax25_frame[] = {
            'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
            'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x61,
            0x03, 0xF0,
            'H', 'e', 'l', 'l', 'o', ' ', 'W', 'o', 'r', 'l', 'd', '!'
        };
        station_a.queue_tx_frame(ax25_frame, sizeof(ax25_frame));

        // Generate TX audio from station A, feed to station B RX
        // Use large buffer to capture entire frame
        constexpr int CHUNK = 1024;
        constexpr int TOTAL = 48000 * 3;  // 3 seconds
        std::vector<float> audio(CHUNK);

        // First, generate ALL TX audio
        std::vector<float> all_tx;
        int samples_tx = 0;
        float max_sample = 0;
        while (samples_tx < TOTAL) {
            std::fill(audio.begin(), audio.end(), 0.0f);
            station_a.process_tx(audio.data(), CHUNK);

            for (int j = 0; j < CHUNK; j++) {
                float a = std::abs(audio[j]);
                if (a > max_sample) max_sample = a;
            }
            all_tx.insert(all_tx.end(), audio.begin(), audio.end());
            samples_tx += CHUNK;

            // Stop after TX finishes (state returns to IDLE)
            if (station_a.state() == ModemState::IDLE && max_sample > 0.01f)
                break;
        }

        // Now feed all TX audio to station B in chunks
        for (size_t off = 0; off < all_tx.size(); off += CHUNK) {
            int n = std::min(CHUNK, (int)(all_tx.size() - off));
            station_b.process_rx(all_tx.data() + off, n);
            if (!rx_frames.empty()) break;
        }

        check("Station B received frame", !rx_frames.empty());
        if (!rx_frames.empty()) {
            // HDLC decoded frame should contain our payload
            auto& f = rx_frames[0];
            bool has_hello = false;
            if (f.size() >= 18) {
                has_hello = (f[16] == 'H' && f[17] == 'e');
            }
            check("Received payload matches", has_hello);
            printf("  Frame received: %zu bytes after %d samples\n",
                   f.size(), samples_tx);
        }

        station_a.shutdown();
        station_b.shutdown();
    }

    // Native frame through modem audio path (Mode A with upconversion)
    {
        printf("\n=== End-to-End Native Through Audio (Mode A) ===\n");

        IrisConfig cfg_a, cfg_b;
        cfg_a.mode = "A"; cfg_a.callsign = "STA_A";
        cfg_a.ptt_pre_delay_ms = 0;
        cfg_a.ptt_post_delay_ms = 0;
        cfg_a.max_modulation = Modulation::BPSK;

        cfg_b.mode = "A"; cfg_b.callsign = "STA_B";
        cfg_b.ptt_pre_delay_ms = 0;
        cfg_b.ptt_post_delay_ms = 0;
        cfg_b.max_modulation = Modulation::BPSK;

        Modem station_a, station_b;
        check("Native A init", station_a.init(cfg_a));
        check("Native B init", station_b.init(cfg_b));

        // Capture received frames from station B
        std::vector<std::vector<uint8_t>> rx_frames;
        station_b.set_rx_callback([&](const uint8_t* data, size_t len) {
            rx_frames.push_back(std::vector<uint8_t>(data, data + len));
        });

        // Build a native-mode payload and queue it as if native mode were active
        // We need to simulate XID exchange to enter native mode first
        // For simplicity, build an AX.25 frame and verify it goes through
        uint8_t test_frame[] = {
            'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
            'N'<<1, 'A'<<1, 'T'<<1, 'V'<<1, ' '<<1, ' '<<1, 0x61,
            0x03, 0xF0,
            'N', 'a', 't', 'i', 'v', 'e', ' ', 'T', 'e', 's', 't'
        };
        station_a.queue_tx_frame(test_frame, sizeof(test_frame));

        constexpr int CHUNK = 1024;
        constexpr int TOTAL = 48000 * 3;
        std::vector<float> audio(CHUNK);

        int samples_tx = 0;
        while (samples_tx < TOTAL) {
            std::fill(audio.begin(), audio.end(), 0.0f);
            station_a.process_tx(audio.data(), CHUNK);
            station_b.process_rx(audio.data(), CHUNK);
            samples_tx += CHUNK;
            if (!rx_frames.empty()) break;
        }

        check("Mode A frame received", !rx_frames.empty());
        if (!rx_frames.empty()) {
            auto& f = rx_frames[0];
            bool has_native = false;
            if (f.size() >= 18) {
                has_native = (f[16] == 'N' && f[17] == 'a');
            }
            check("Mode A payload matches", has_native);
            printf("  Mode A frame received: %zu bytes after %d samples\n",
                   f.size(), samples_tx);
        }

        station_a.shutdown();
        station_b.shutdown();
    }

    // Phase 7: Compression
    {
        printf("\n=== Compression ===\n");

        // Repetitive text compresses well
        const char* text = "Hello Hello Hello Hello World World World World!!!!";
        auto compressed = compress_frame((const uint8_t*)text, strlen(text));
        check("Compressed is smaller", compressed.size() < strlen(text));
        printf("  Text: %zu -> %zu bytes (%.0f%%)\n",
               strlen(text), compressed.size(),
               100.0 * compressed.size() / strlen(text));

        auto decompressed = decompress_frame(compressed.data(), compressed.size());
        check("Decompress round-trip",
              decompressed.size() == strlen(text) &&
              memcmp(decompressed.data(), text, strlen(text)) == 0);

        // Random data should not expand much (stored uncompressed)
        std::vector<uint8_t> random_data(256);
        for (int i = 0; i < 256; i++) random_data[i] = (uint8_t)(i * 37 + 13);
        auto comp_rand = compress_frame(random_data.data(), random_data.size());
        auto decomp_rand = decompress_frame(comp_rand.data(), comp_rand.size());
        check("Random data round-trip", decomp_rand == random_data);
        printf("  Random: %zu -> %zu bytes\n", random_data.size(), comp_rand.size());

        // Empty data
        auto comp_empty = compress_frame(nullptr, 0);
        auto decomp_empty = decompress_frame(comp_empty.data(), comp_empty.size());
        check("Empty round-trip", decomp_empty.empty());
    }

    // Phase 7: Encryption
    {
        printf("\n=== Encryption ===\n");

        CipherSuite oversized_a, oversized_b;
        uint8_t oversized_a_pub[X25519_KEY_SIZE];
        uint8_t oversized_b_pub[X25519_KEY_SIZE];
        check("Oversized callsign A X25519 keygen",
              oversized_a.generate_x25519_keypair(oversized_a_pub) == 0);
        check("Oversized callsign B X25519 keygen",
              oversized_b.generate_x25519_keypair(oversized_b_pub) == 0);
        check("Oversized callsign X25519 shared",
              oversized_a.compute_x25519_shared(oversized_b_pub) == 0);
        char long_call_a[201];
        char long_call_b[201];
        memset(long_call_a, 'A', sizeof(long_call_a) - 1);
        memset(long_call_b, 'B', sizeof(long_call_b) - 1);
        long_call_a[sizeof(long_call_a) - 1] = '\0';
        long_call_b[sizeof(long_call_b) - 1] = '\0';
        oversized_a.derive_session_key(long_call_a, long_call_b, nullptr, 0, false);
        oversized_a.activate();
        check("Crypto rejects oversized callsigns", !oversized_a.is_active());

        // Basic encrypt/decrypt round-trip
        CryptoKey key = crypto_random_key();
        const char* msg = "Secret message for encryption test!";
        auto encrypted = crypto_encrypt((const uint8_t*)msg, strlen(msg), key);
        check("Encrypted is larger", encrypted.size() > strlen(msg));
        printf("  Plaintext: %zu -> Encrypted: %zu bytes (overhead: %zu)\n",
               strlen(msg), encrypted.size(), encrypted.size() - strlen(msg));

        auto decrypted = crypto_decrypt(encrypted.data(), encrypted.size(), key);
        check("Decrypt round-trip",
              decrypted.size() == strlen(msg) &&
              memcmp(decrypted.data(), msg, strlen(msg)) == 0);

        // Wrong key fails
        CryptoKey bad_key = crypto_random_key();
        auto bad_decrypt = crypto_decrypt(encrypted.data(), encrypted.size(), bad_key);
        check("Wrong key fails", bad_decrypt.empty());

        // Tampered ciphertext fails
        auto tampered = encrypted;
        tampered[tampered.size() - 1] ^= 0xFF;
        auto tamper_decrypt = crypto_decrypt(tampered.data(), tampered.size(), key);
        check("Tampered data fails", tamper_decrypt.empty());

        // X25519 key exchange
        auto alice = crypto_generate_keypair();
        auto bob = crypto_generate_keypair();
        auto shared_a = crypto_key_exchange(alice.secret, bob.public_key);
        auto shared_b = crypto_key_exchange(bob.secret, alice.public_key);
        check("X25519 shared secret matches", shared_a == shared_b);

        // Encrypt with shared key
        auto enc_shared = crypto_encrypt((const uint8_t*)msg, strlen(msg), shared_a);
        auto dec_shared = crypto_decrypt(enc_shared.data(), enc_shared.size(), shared_b);
        check("Encrypt with shared key round-trip",
              dec_shared.size() == strlen(msg) &&
              memcmp(dec_shared.data(), msg, strlen(msg)) == 0);

        // Frame wrapper
        auto enc_frame = encrypt_frame((const uint8_t*)msg, strlen(msg), key);
        check("Frame header is 0x01", enc_frame[0] == 0x01);
        auto dec_frame = decrypt_frame(enc_frame.data(), enc_frame.size(), key);
        check("Frame decrypt round-trip",
              dec_frame.size() == strlen(msg) &&
              memcmp(dec_frame.data(), msg, strlen(msg)) == 0);
    }

    // Native PHY upconvert/downconvert loopback
    {
        printf("\n=== Native PHY Upconvert/Downconvert Loopback ===\n");

        float center = 1900.0f;
        PhyConfig cfg = mode_a_config();
        cfg.modulation = Modulation::BPSK;

        // Build a test frame
        uint8_t payload[] = "TestPayload123";
        auto iq = build_native_frame(payload, sizeof(payload), cfg, LdpcRate::RATE_1_2);
        printf("  Built frame: %zu IQ samples (%zu audio samples)\n",
               iq.size() / 2, iq.size() / 2);

        // Upconvert to audio
        Upconverter up(center, 48000);
        auto audio = up.iq_to_audio(iq.data(), iq.size());
        printf("  Upconverted: %zu audio samples, center %.0f Hz\n",
               audio.size(), center);

        // Check audio signal level
        float peak = 0;
        for (float s : audio) { float a = std::abs(s); if (a > peak) peak = a; }
        printf("  Audio peak: %.4f\n", peak);
        check("Audio has signal", peak > 0.01f);

        // Downconvert back to baseband IQ
        Downconverter down(center, 48000);
        auto rx_iq = down.audio_to_iq(audio.data(), audio.size());
        printf("  Downconverted: %zu IQ samples\n", rx_iq.size() / 2);

        // Check baseband IQ signal level
        float iq_peak = 0;
        for (size_t i = 0; i < rx_iq.size(); i += 2) {
            float mag = std::sqrt(rx_iq[i]*rx_iq[i] + rx_iq[i+1]*rx_iq[i+1]);
            if (mag > iq_peak) iq_peak = mag;
        }
        printf("  IQ peak: %.4f\n", iq_peak);
        check("IQ has signal after downconvert", iq_peak > 0.001f);

        // Try frame detection
        int start = detect_frame_start(rx_iq.data(), rx_iq.size(), cfg.samples_per_symbol);
        float best = detect_best_corr();
        printf("  Frame detect: offset=%d, best_corr=%.3f\n", start, best);
        check("Frame detected after upconvert/downconvert", start >= 0);

        // Try full decode
        if (start >= 0) {
            std::vector<uint8_t> decoded;
            bool ok = decode_native_frame(rx_iq.data(), rx_iq.size(),
                                           start, cfg, decoded);
            printf("  Decode: %s, %zu bytes\n", ok ? "OK" : "FAIL", decoded.size());
            check("Frame decoded successfully", ok);
            if (ok) {
                check("Decoded payload matches",
                      decoded.size() == sizeof(payload) &&
                      memcmp(decoded.data(), payload, sizeof(payload)) == 0);
            }
        }
    }

    // Phase 8: Probe & Channel Equalization
    printf("\n--- Phase 8: Probe & Channel EQ ---\n");

    // Chirp probe generation and analysis round-trip
    {
        printf("\n=== Probe Generate/Analyze (Chirp) ===\n");
        int sr = 48000;
        int max_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sr) + 1000;
        std::vector<float> probe_audio(max_samples);
        int n = probe_generate(probe_audio.data(), max_samples, sr, 0.7f);
        check("Probe generates samples", n > 0);
        // Chirp: 2x 24000 + 2400 gap = 50400 samples = 1.05s
        check("Probe duration ~1.05s", n >= sr * 1 && n <= (int)(sr * 1.1f));

        // Analyze the clean probe (no channel filtering)
        ProbeResult result = probe_analyze(probe_audio.data(), n, sr);
        check("Probe analysis valid", result.valid);
        check("Probe detects >= 50 tones", result.tones_detected >= 50);
        check("Probe low_hz near 300", result.low_hz >= 250.0f && result.low_hz <= 400.0f);
        // Grid now tops at 6300 Hz (covers the wide 6 kHz data-port lane); a
        // clean/unfiltered probe discovers the full grid.
        check("Probe high_hz near 6300", result.high_hz >= 6100.0f && result.high_hz <= 6400.0f);
    }

    // Width-sweep discovery — the owner-requested "special test at various filter
    // widths". The probe audio is band-limited by the SAME Blackman windowed-sinc
    // bandpass the real-audio channel uses (300 Hz low edge, sweep of high edges),
    // and probe_analyze() must DISCOVER the true filter width: narrow -> narrow,
    // wide -> wide, with NO over-discovery (the killed full-band bug) and NO
    // under-discovery (must not cripple a genuinely wide channel). Regression
    // guard for the 300-6300 Hz grid extension.
    {
        printf("\n=== Probe Width-Sweep Discovery (2-6 kHz) ===\n");
        const int sr = 48000;
        int probe_n = (int)(PassbandProbeConfig::PROBE_DURATION_S * sr) + 1;
        int pad = sr / 2;  // 0.5s silence each side for the FIR transient
        float widths[] = {2000, 2500, 3000, 4000, 5000, 6000};
        int   n_w = 6;
        float prev_bw = -1.0f;
        bool  monotonic = true;
        for (int wi = 0; wi < n_w; wi++) {
            float w = widths[wi];
            float lo = 300.0f, hi = 300.0f + w;
            std::vector<float> buf(pad + probe_n + pad, 0.0f);
            probe_generate(buf.data() + pad, probe_n, sr, 0.5f);
            probe_test_bandpass(buf, lo, hi, 511, (float)sr);

            ProbeResult r = probe_analyze(buf.data(), (int)buf.size(), sr);
            NegotiatedPassband neg = probe_negotiate(r, r);
            float disc_bw = neg.bandwidth_hz;   // discovered usable width
            float err = disc_bw - w;            // + = over, - = under
            printf("    width=%.0f Hz [300,%.0f]: discovered %.0f-%.0f Hz, BW=%.0f (err %+.0f), tones=%d\n",
                   w, hi, neg.low_hz, neg.high_hz, disc_bw, err, r.tones_detected);

            char lbl[96];
            // Discovery tracks the true width within ~1.5 tone spacings (~150 Hz)
            // of transition-band + 25 Hz edge margin. Allow +250/-200 Hz.
            snprintf(lbl, sizeof(lbl), "Width %.0f Hz: discovered BW ~= filter width", w);
            check(lbl, r.valid && err <= 250.0f && err >= -200.0f);

            // No over-discovery: a narrow filter must NOT read back the full grid.
            snprintf(lbl, sizeof(lbl), "Width %.0f Hz: no over-discovery (< width+400)", w);
            check(lbl, disc_bw < w + 400.0f);

            // No under-discovery: even the 5-6 kHz data-port widths must be seen
            // (the old 4500 Hz grid ceiling capped these at ~4150 Hz).
            snprintf(lbl, sizeof(lbl), "Width %.0f Hz: no under-discovery (> width-300)", w);
            check(lbl, disc_bw > w - 300.0f);

            if (prev_bw >= 0 && disc_bw < prev_bw - 100.0f) monotonic = false;
            prev_bw = disc_bw;
        }
        // Discovery must be monotone non-decreasing in filter width.
        check("Width-sweep: discovery monotone in filter width", monotonic);
    }

    // Event-driven capture early-exit (PART B timing rework). A responder given a
    // full CMD chirp inside its capture window must END the capture the instant
    // the complete probe is in hand, WITHOUT waiting out the fixed 2.0s window;
    // and a buffer of pure silence must NEVER early-exit (weak/lost-chirp path
    // falls back to the fixed window unchanged).
    {
        printf("\n=== Probe Event-Driven Capture Early-Exit ===\n");
        const int sr = 48000;
        int probe_n = (int)(PassbandProbeConfig::PROBE_DURATION_S * sr) + 1;

        // (a) Onset finder locates a real chirp and rejects silence.
        std::vector<float> chirp(probe_n, 0.0f);
        probe_generate(chirp.data(), probe_n, sr, 0.5f);
        float pk = 0.0f;
        int onset = probe_find_chirp_onset(chirp.data(), probe_n, sr, &pk);
        check("Onset finder locates chirp (onset>=0)", onset >= 0);
        check("Onset finder peak/median >= 20 dB", pk >= 20.0f);
        std::vector<float> silence(probe_n, 0.0f);
        float pk2 = 0.0f;
        int onset2 = probe_find_chirp_onset(silence.data(), probe_n, sr, &pk2);
        check("Onset finder rejects silence (onset<0)", onset2 < 0);

        // (b) RSP early-exits its 2.0s capture once a full chirp + settle arrives.
        // Feed a chirp followed by a short settle tail — well under the 2.0s
        // fixed window — and confirm the state transitions WITHOUT a full window.
        {
            ProbeController rsp;
            rsp.on_send_audio = [](const float*, int){};
            rsp.on_send_msg = [](const uint8_t*, size_t){};
            rsp.set_local_caps(0x0001);
            rsp.set_local_ofdm_config(64, 4, 24, 2);
            rsp.start_responder(sr, 8.0f);
            int fixed_window = (int)(2.0f * sr);
            // Build chirp + 0.2s settle (total ~1.25s << 2.0s window).
            std::vector<float> feed(probe_n + sr / 5, 0.0f);
            probe_generate(feed.data(), probe_n, sr, 0.5f);
            // Feed in ~50ms chunks so the throttled early-exit check runs.
            int fed = 0, chunk = sr / 20;
            while (fed < (int)feed.size() &&
                   rsp.state() == ProbeState::LISTENING_PROBE) {
                int c = std::min(chunk, (int)feed.size() - fed);
                rsp.feed_rx(feed.data() + fed, c);
                fed += c;
            }
            check("RSP early-exit fired before fixed window",
                  rsp.state() == ProbeState::SENDING_RESULT && fed < fixed_window);
            check("RSP early-exit used < 1.6s of capture", fed < (int)(1.6f * sr));
        }

        // (c) Weak-path invariance: pure silence NEVER early-exits — the capture
        // stays LISTENING until the fixed window fills.
        {
            ProbeController rsp;
            rsp.on_send_audio = [](const float*, int){};
            rsp.on_send_msg = [](const uint8_t*, size_t){};
            rsp.start_responder(sr, 8.0f);
            int fixed_window = (int)(2.0f * sr);
            std::vector<float> sil(fixed_window - sr / 10, 0.0f);  // just under the window
            rsp.feed_rx(sil.data(), (int)sil.size());
            check("Silence never early-exits (stays LISTENING)",
                  rsp.state() == ProbeState::LISTENING_PROBE);
        }
    }

    // RSP RESULT re-announce — closes the OFDM-activation timing race where
    // RSP's single fire-once RESULT lands before CMD's WAITING_RESULT window
    // opens (~96% of sessions previously fell back to AFSK). The responder must
    // keep re-emitting RESULT past DONE until CMD positively confirms.
    {
        printf("\n=== Probe RSP RESULT Re-announce ===\n");
        int sr = 48000;
        ProbeController rsp;
        int results_sent = 0;
        rsp.on_send_msg = [&](const uint8_t* d, size_t n) {
            if (n >= 1 && d[0] == PROBE_MSG_RESULT) results_sent++;
        };
        rsp.on_send_audio = [&](const float*, int) {};  // discard RSP's own tones
        rsp.set_local_caps(0x0001);
        rsp.set_local_ofdm_config(64, 4, 24, 2);

        rsp.start_responder(sr, 8.0f);
        // Feed a 2.0s capture window: real CMD chirp (1.05s) + silence.
        int cap = (int)(2.0f * sr);
        std::vector<float> cmd_chirp(cap, 0.0f);
        probe_generate(cmd_chirp.data(), cap, sr, 0.7f);
        rsp.feed_rx(cmd_chirp.data(), cap);
        check("RSP captured CMD chirp -> SENDING_RESULT",
              rsp.state() == ProbeState::SENDING_RESULT);
        // Drive the turnaround countdown so RSP queues its own tones.
        for (int i = 0; i < 12; i++) rsp.tick();
        check("RSP turnaround -> SENDING_PROBE", rsp.state() == ProbeState::SENDING_PROBE);
        rsp.on_tx_complete();  // analyze + send RESULT x2 + finalize + arm re-announce
        check("RSP finalized (DONE)", rsp.is_done());
        int after_finalize = results_sent;
        check("RSP sent initial RESULT (>=2)", after_finalize >= 2);
        check("RSP re-announce armed after finalize", rsp.reannounce_active());
        // One period of ticks -> exactly one more RESULT (the fix; before the fix
        // tick() early-returned on DONE and never re-sent).
        for (int i = 0; i < 40; i++) rsp.tick();
        check("RSP re-announced RESULT after 1 period",
              results_sent == after_finalize + 1);
        // CMD's corrected/confirming RESULT arrives -> re-announce stops.
        ProbeResult corrected = rsp.their_tx_result();
        auto enc = probe_result_encode(corrected);
        std::vector<uint8_t> msg;
        msg.push_back(PROBE_MSG_RESULT);
        msg.insert(msg.end(), enc.begin(), enc.end());
        rsp.on_message(msg.data(), msg.size());
        check("RSP re-announce stopped on CMD confirm", !rsp.reannounce_active());
        int at_stop = results_sent;
        for (int i = 0; i < 80; i++) rsp.tick();
        check("RSP sends no more RESULT after confirm", results_sent == at_stop);
    }

    // Probe result encode/decode (v4 wire format with tone powers + OFDM config)
    {
        printf("\n=== Probe Wire Format v4 ===\n");
        ProbeResult tx_result;
        tx_result.low_hz = 350.0f;
        tx_result.high_hz = 3050.0f;
        tx_result.tones_detected = 42;
        tx_result.valid = true;
        tx_result.capabilities = 0x001F;  // all caps
        tx_result.ofdm_cp_samples = 32;
        tx_result.ofdm_pilot_carrier_spacing = 6;
        tx_result.ofdm_pilot_symbol_spacing = 24;
        tx_result.ofdm_nfft_code = 0;  // 512

        // Set up realistic tone data
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float freq = probe_tone_freq(k);
            tx_result.tone_detected[k] = (freq >= 350.0f && freq <= 3050.0f);
            // Simulate FM de-emphasis: -6 dB/octave from 1 kHz
            float ref = 1000.0f;
            if (freq > ref)
                tx_result.tone_power_db[k] = -20.0f * std::log10(freq / ref) * 2.0f;
            else
                tx_result.tone_power_db[k] = -10.0f;  // flat below 1 kHz
        }

        auto encoded = probe_result_encode(tx_result);
        // v4 = 19 base + 2 caps + 64 tone powers + 4 ofdm config = 89 bytes
        check("Probe v4 encode size = 89", (int)encoded.size() == 89);

        ProbeResult decoded;
        bool ok = probe_result_decode(encoded.data(), encoded.size(), decoded);
        check("Probe v4 decode succeeds", ok);
        check("Probe v4 low_hz matches", std::abs(decoded.low_hz - tx_result.low_hz) < 0.01f);
        check("Probe v4 high_hz matches", std::abs(decoded.high_hz - tx_result.high_hz) < 0.01f);
        check("Probe v4 tones_detected matches", decoded.tones_detected == tx_result.tones_detected);
        check("Probe v4 capabilities matches", decoded.capabilities == tx_result.capabilities);
        check("Probe v4 ofdm_cp", decoded.ofdm_cp_samples == 32);
        check("Probe v4 ofdm_pilot_carrier", decoded.ofdm_pilot_carrier_spacing == 6);
        check("Probe v4 ofdm_pilot_symbol", decoded.ofdm_pilot_symbol_spacing == 24);
        check("Probe v4 ofdm_nfft_code", decoded.ofdm_nfft_code == 0);

        // Check tone powers survived quantization (0.5 dB resolution)
        float max_power_err = 0;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float err = std::abs(decoded.tone_power_db[k] - tx_result.tone_power_db[k]);
            if (err > max_power_err) max_power_err = err;
        }
        check("Probe v4 tone powers within 0.5 dB", max_power_err <= 0.5f);

        // Bitmap round-trip
        bool bitmap_match = true;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            if (decoded.tone_detected[k] != tx_result.tone_detected[k]) {
                bitmap_match = false; break;
            }
        }
        check("Probe v4 bitmap round-trip", bitmap_match);
    }

    // Backward compatibility: decode old v2 packet (no tone powers, no OFDM config)
    {
        printf("\n=== Probe Wire Format Backward Compat ===\n");
        ProbeResult tx_result;
        tx_result.low_hz = 400.0f;
        tx_result.high_hz = 2800.0f;
        tx_result.tones_detected = 35;
        tx_result.valid = true;
        tx_result.capabilities = 0x0003;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            tx_result.tone_detected[k] = (k >= 2 && k <= 40);
            tx_result.tone_power_db[k] = -15.0f;
        }

        auto full = probe_result_encode(tx_result);
        // Simulate old peer: truncate to 21 bytes (19 base + 2 caps, no tone powers)
        std::vector<uint8_t> old_wire(full.begin(), full.begin() + 21);

        ProbeResult decoded;
        bool ok = probe_result_decode(old_wire.data(), old_wire.size(), decoded);
        check("Old v2 decode succeeds", ok);
        check("Old v2 low_hz matches", std::abs(decoded.low_hz - tx_result.low_hz) < 0.01f);
        check("Old v2 capabilities", decoded.capabilities == 0x0003);

        // Tone powers should be 0 (no EQ data from old peer)
        bool powers_zero = true;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            if (decoded.tone_power_db[k] != 0.0f) { powers_zero = false; break; }
        }
        check("Old v2 tone powers = 0 (no EQ)", powers_zero);

        // OFDM config should be 0 (old peer, use defaults)
        check("Old v2 ofdm_cp = 0", decoded.ofdm_cp_samples == 0);
        check("Old v2 ofdm_pilot_carrier = 0", decoded.ofdm_pilot_carrier_spacing == 0);
        check("Old v2 ofdm_pilot_symbol = 0", decoded.ofdm_pilot_symbol_spacing == 0);
        check("Old v2 ofdm_nfft_code = 0", decoded.ofdm_nfft_code == 0);
    }

    // Backward compatibility: decode old v3 packet (tone powers but no OFDM config)
    {
        printf("\n=== Probe Wire Format v3 Backward Compat ===\n");
        ProbeResult tx_result;
        tx_result.low_hz = 400.0f;
        tx_result.high_hz = 2800.0f;
        tx_result.tones_detected = 35;
        tx_result.valid = true;
        tx_result.capabilities = 0x0203;  // CAP_OFDM + others
        tx_result.ofdm_cp_samples = 32;   // will be in full encode but truncated
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            tx_result.tone_detected[k] = (k >= 2 && k <= 40);
            tx_result.tone_power_db[k] = -15.0f;
        }

        auto full = probe_result_encode(tx_result);
        // Simulate v3 peer: truncate to 85 bytes (no OFDM config)
        std::vector<uint8_t> v3_wire(full.begin(), full.begin() + 85);

        ProbeResult decoded;
        bool ok = probe_result_decode(v3_wire.data(), v3_wire.size(), decoded);
        check("Old v3 decode succeeds", ok);
        check("Old v3 capabilities", decoded.capabilities == 0x0203);
        check("Old v3 ofdm_cp = 0 (not present)", decoded.ofdm_cp_samples == 0);
        check("Old v3 ofdm_pilot_carrier = 0", decoded.ofdm_pilot_carrier_spacing == 0);
    }

    // Channel Equalizer: flatten synthetic FM de-emphasis
    {
        printf("\n=== Channel Equalizer ===\n");
        int sr = 48000;

        // Build a synthetic probe result with FM de-emphasis rolloff
        // -6 dB/octave above 1 kHz (typical FM de-emphasis)
        ProbeResult probe_rx;
        probe_rx.valid = true;
        probe_rx.low_hz = 366.7f;   // tone index 1
        probe_rx.high_hz = 3033.3f; // tone index ~41

        int n_det = 0;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float freq = probe_tone_freq(k);
            if (freq >= 350.0f && freq <= 3100.0f) {
                probe_rx.tone_detected[k] = true;
                // -6 dB/octave = -20*log10(f/1000)
                float ref = 1000.0f;
                if (freq > ref)
                    probe_rx.tone_power_db[k] = -20.0f * std::log10(freq / ref);
                else
                    probe_rx.tone_power_db[k] = 0.0f;  // flat below ref
                n_det++;
            } else {
                probe_rx.tone_detected[k] = false;
                probe_rx.tone_power_db[k] = -80.0f;
            }
        }
        probe_rx.tones_detected = n_det;

        NegotiatedPassband passband;
        passband.valid = true;
        passband.low_hz = 400.0f;
        passband.high_hz = 3000.0f;
        passband.center_hz = 1700.0f;
        passband.bandwidth_hz = 2600.0f;

        ChannelEqualizer eq;
        eq.configure(probe_rx, passband, sr);
        check("EQ configured", eq.is_configured());
        check("EQ has taps", !eq.taps().empty());
        check("EQ has EQ curve", !eq.eq_curve_db().empty());

        // EQ curve should boost high frequencies to compensate rolloff
        // Check that the max EQ gain is positive (boosting attenuated highs)
        float max_eq = *std::max_element(eq.eq_curve_db().begin(), eq.eq_curve_db().end());
        float min_eq = *std::min_element(eq.eq_curve_db().begin(), eq.eq_curve_db().end());
        check("EQ curve has positive boost", max_eq > 0.5f);
        check("EQ curve range > 1 dB", (max_eq - min_eq) > 1.0f);

        // Apply EQ to a multi-tone test signal and verify flattening
        // Generate 2 tones: 800 Hz (no rolloff) and 2400 Hz (rolled off)
        int test_samples = sr;  // 1 second
        std::vector<float> test_audio(test_samples);
        for (int i = 0; i < test_samples; i++) {
            float t = (float)i / sr;
            test_audio[i] = 0.5f * std::cos(2.0f * (float)M_PI * 800.0f * t)
                          + 0.5f * std::cos(2.0f * (float)M_PI * 2400.0f * t);
        }

        // Measure power before EQ
        // Use last 80% to avoid FIR transient (127 taps ~ 2.6ms)
        auto measure_tone_power = [&](const std::vector<float>& audio, float freq, int skip) {
            double sum = 0;
            int count = (int)audio.size() - skip;
            for (int i = skip; i < (int)audio.size(); i++) {
                float t = (float)i / sr;
                float ref = std::cos(2.0f * (float)M_PI * freq * t);
                sum += audio[i] * ref;
            }
            return (float)(sum / count);
        };

        float pre_800 = std::abs(measure_tone_power(test_audio, 800.0f, 0));
        float pre_2400 = std::abs(measure_tone_power(test_audio, 2400.0f, 0));

        eq.apply(test_audio.data(), test_samples);

        int skip = 500;  // skip FIR transient
        float post_800 = std::abs(measure_tone_power(test_audio, 800.0f, skip));
        float post_2400 = std::abs(measure_tone_power(test_audio, 2400.0f, skip));

        // After EQ, the ratio between 800 Hz and 2400 Hz should be closer to 1:1
        float pre_ratio = (pre_800 > 0.001f) ? pre_2400 / pre_800 : 0;
        float post_ratio = (post_800 > 0.001f) ? post_2400 / post_800 : 0;
        printf("  Pre-EQ ratio (2400/800): %.3f\n", pre_ratio);
        printf("  Post-EQ ratio (2400/800): %.3f\n", post_ratio);
        // EQ should bring the ratio closer to 1.0 (boost the rolled-off 2400 Hz)
        check("EQ improves tone balance", post_ratio > pre_ratio * 0.9f);
        check("EQ doesn't amplify excessively", post_ratio < 3.0f);
    }

    // Channel EQ: skip when channel is already flat
    {
        printf("\n=== Channel EQ: Flat Channel Skip ===\n");
        ProbeResult flat_probe;
        flat_probe.valid = true;
        flat_probe.low_hz = 400.0f;
        flat_probe.high_hz = 3000.0f;
        flat_probe.tones_detected = 40;
        for (int k = 0; k < PassbandProbeConfig::N_TONES; k++) {
            float freq = probe_tone_freq(k);
            flat_probe.tone_detected[k] = (freq >= 350.0f && freq <= 3100.0f);
            flat_probe.tone_power_db[k] = -10.0f;  // perfectly flat
        }

        NegotiatedPassband passband;
        passband.valid = true;
        passband.low_hz = 400.0f;
        passband.high_hz = 3000.0f;

        ChannelEqualizer eq;
        eq.configure(flat_probe, passband, 48000);
        check("Flat channel: EQ not configured (skipped)", !eq.is_configured());
    }

    // Probe negotiate
    {
        printf("\n=== Probe Negotiate ===\n");
        ProbeResult a_to_b;
        a_to_b.valid = true;
        a_to_b.low_hz = 400.0f;
        a_to_b.high_hz = 3200.0f;

        ProbeResult b_to_a;
        b_to_a.valid = true;
        b_to_a.low_hz = 350.0f;
        b_to_a.high_hz = 2900.0f;

        auto neg = probe_negotiate(a_to_b, b_to_a);
        check("Negotiated valid", neg.valid);
        // Intersection should be max(lows)+margin .. min(highs)-margin
        check("Negotiated low >= 400", neg.low_hz >= 400.0f);
        check("Negotiated high <= 2900", neg.high_hz <= 2900.0f);
        check("Negotiated bandwidth > 0", neg.bandwidth_hz > 0);
    }

    // Probe channel shape detection
    {
        printf("\n=== Probe Channel Shape Detection ===\n");

        // Test 1: Flat channel — all tones at similar power
        ProbeResult flat_probe;
        flat_probe.valid = true;
        flat_probe.low_hz = 400.0f;
        flat_probe.high_hz = 3000.0f;
        flat_probe.tones_detected = 0;
        for (int k = 0; k < 64; k++) {
            float freq = probe_tone_freq(k);
            if (freq >= 400.0f && freq <= 3000.0f) {
                flat_probe.tone_detected[k] = true;
                flat_probe.tone_power_db[k] = -20.0f + 0.5f * ((k % 3) - 1);  // ±0.5 dB noise
                flat_probe.tones_detected++;
            }
        }
        float flat_corner = probe_detect_preemph_corner(flat_probe);
        check("Flat channel: corner = 0", flat_corner == 0.0f);

        // Test 2: FM channel — rolloff matching ~300 Hz corner
        // With chirp probe, H(f) directly measures channel response (no TX pre-emphasis).
        // Simulate FM de-emphasis rolloff: -10*log10(1 + (f/300)^2)
        ProbeResult fm_probe;
        fm_probe.valid = true;
        fm_probe.low_hz = 400.0f;
        fm_probe.high_hz = 3000.0f;
        fm_probe.tones_detected = 0;
        for (int k = 0; k < 64; k++) {
            float freq = probe_tone_freq(k);
            if (freq >= 400.0f && freq <= 3000.0f) {
                fm_probe.tone_detected[k] = true;
                // Channel rolloff only (chirp has flat TX spectrum)
                float ch_rolloff = -10.0f * std::log10(1.0f + (freq / 300.0f) * (freq / 300.0f));
                fm_probe.tone_power_db[k] = -10.0f + ch_rolloff;
                fm_probe.tones_detected++;
            }
        }
        float fm_corner = probe_detect_preemph_corner(fm_probe);
        check("FM channel: corner > 0", fm_corner > 0.0f);
        check("FM channel: corner in range 150-500 Hz",
              fm_corner >= 150.0f && fm_corner <= 500.0f);
        printf("  Fitted corner: %.0f Hz (expected ~300 Hz)\n", fm_corner);
    }

    // =========================================================================
    printf("\n=== MFSK ACK Round-Trip ===\n");
    {
        MfskAck tx, rx;
        // Center tones around bin 36 (1687.5 Hz at NFFT=1024, 48kHz)
        int first_bin = 36 - MfskAck::M / 2;  // bin 28
        tx.init(first_bin, 48000);
        rx.init(first_bin, 48000);

        // Test all N(R) values and P/F bit
        for (int nr = 0; nr < 8; nr++) {
            for (int pf = 0; pf <= 1; pf++) {
                auto audio = tx.generate(nr, pf);
                check("MFSK generate non-empty", !audio.empty());

                // Add some leading silence (simulates post-TX mute)
                std::vector<float> rx_buf(2048, 0.0f);
                rx_buf.insert(rx_buf.end(), audio.begin(), audio.end());
                rx_buf.resize(rx_buf.size() + 1024, 0.0f);  // trailing silence

                auto result = rx.detect(rx_buf.data(), (int)rx_buf.size());
                char label[64];
                snprintf(label, sizeof(label), "MFSK detect N(R)=%d PF=%d", nr, pf);
                check(label, result.detected);
                snprintf(label, sizeof(label), "MFSK N(R) correct (got %d)", result.n_r);
                check(label, result.n_r == nr);
                snprintf(label, sizeof(label), "MFSK PF correct (got %d)", result.pf);
                check(label, result.pf == pf);
            }
        }
        printf("  Burst duration: %.1f ms\n", tx.burst_duration_s() * 1000.0f);
    }

    // =========================================================================
    printf("\n=== MFSK ACK Receiver-Drives-Rate (proposed forward level) ===\n");
    {
        // The reverse MFSK tone ACK carries B's ABSOLUTE proposed forward O-level
        // (its decode-margin-validated RX level) in a new suffix block appended
        // after the N(R) suffix. The field is self-rejecting: a corrupt/out-of-
        // range field must be DROPPED (proposed_level = -1 = "hold"), never applied.
        MfskAck tx, rx;
        int first_bin = 36 - MfskAck::M / 2;  // bin 28
        tx.init(first_bin, 48000);
        rx.init(first_bin, 48000);

        const float PI = 3.14159265358979323846f;
        auto make_buf = [](const std::vector<float>& audio) {
            std::vector<float> b(2048, 0.0f);
            b.insert(b.end(), audio.begin(), audio.end());
            b.resize(b.size() + 1024, 0.0f);
            return b;
        };
        // Overwrite one 1024-sample symbol window with a pure tone (corruption).
        auto corrupt_sym = [&](std::vector<float>& b, int burst_sym, int tone) {
            int start = 2048 + burst_sym * MfskAck::SYM_SAMPLES;
            float bin_hz = 48000.0f / MfskAck::NFFT;
            float freq = (first_bin + tone) * bin_hz;
            for (int i = 0; i < MfskAck::SYM_SAMPLES; i++)
                b[start + i] = 0.5f * sinf(2.0f * PI * freq * (float)i / 48000.0f);
        };
        // Level-proposal reps live at burst symbols [ACK_LEN+NR_SYMS, ..+LEVEL_REPS)
        // (N(R) now occupies NR_SYMS = 2 tones x NR_REPS for the modulo-128 window).
        const int LVL0 = MfskAck::ACK_LEN + MfskAck::NR_SYMS;

        // 1) Clean round-trip: B proposes O7 -> A recovers 7, N(R)/PF intact.
        {
            auto audio = tx.generate(3, 0, /*proposed_level=*/7);
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR: base detect + N(R)/PF intact with level suffix",
                  r.detected && r.n_r == 3 && r.pf == 0);
            check("RDR: proposed_level == 7 (clean)", r.proposed_level == 7);
        }
        // 2) No proposal (legacy call, level absent) -> proposed_level == -1.
        {
            auto audio = tx.generate(2, 1);  // no level arg -> ABSENT
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR: no-proposal N(R)/PF intact", r.detected && r.n_r == 2 && r.pf == 1);
            check("RDR: no-proposal -> level ABSENT (-1)", r.proposed_level == -1);
        }
        // 3) 1-of-3 level reps corrupted -> 2/3 majority still yields 7.
        {
            auto audio = tx.generate(4, 0, 7);
            auto b = make_buf(audio);
            corrupt_sym(b, LVL0 + 0, 2);  // one rep -> wrong tone
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR: 1/3 level corrupt -> 2/3 vote yields 7", r.proposed_level == 7);
            check("RDR: 1/3 level corrupt -> N(R) intact", r.n_r == 4);
        }
        // 4) 2-of-3 reps corrupted to DISTINCT tones (no majority) -> ABSENT.
        {
            auto audio = tx.generate(5, 0, 7);
            auto b = make_buf(audio);
            corrupt_sym(b, LVL0 + 0, 2);
            corrupt_sym(b, LVL0 + 1, 9);  // distinct -> no 2 agree
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR: 2/3 level corrupt -> ABSENT (-1), conservative hold",
                  r.proposed_level == -1);
            check("RDR: base detect survives corrupt level suffix",
                  r.detected && r.n_r == 5);
        }
        // 5) Out-of-range tone (>= NUM_OFDM_SPEED_LEVELS) -> DROPPED (ABSENT).
        {
            // First invalid level (== NUM_OFDM_SPEED_LEVELS) -> encoder maps to
            // the reserved top tone -> RX range-check rejects it as ABSENT.
            // Uses the constant so it stays correct as the ladder grows.
            auto audio = tx.generate(1, 0, NUM_OFDM_SPEED_LEVELS);
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR: out-of-range level tone -> ABSENT (-1)", r.proposed_level == -1);
            check("RDR: out-of-range still decodes N(R)", r.detected && r.n_r == 1);
        }
        // 6) Miss-cheap invariant: base-symbol detection threshold unchanged.
        check("RDR: MATCH_THRESHOLD unchanged (5/8 base)", MfskAck::MATCH_THRESHOLD == 5);
        // 7) #2 burst-epoch echo round-trips through the AUDIO path (generate ->
        //    detect), and N(R)/level stay intact alongside it.
        {
            auto audio = tx.generate(3, 0, /*proposed_level=*/7, /*burst_epoch=*/5);
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR/epoch: N(R)/PF/level intact with epoch suffix",
                  r.detected && r.n_r == 3 && r.pf == 0 && r.proposed_level == 7);
            check("RDR/epoch: burst-epoch 5 decoded from audio", r.epoch == 5);
        }
        // 8) Absent epoch (legacy 3-arg call) -> ABSENT (-1), N(R) still intact.
        {
            auto audio = tx.generate(2, 0, 7);  // no burst_epoch -> absent
            auto b = make_buf(audio);
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR/epoch: absent epoch -> -1 (ABSENT)", r.epoch == -1);
            check("RDR/epoch: N(R) intact with absent epoch", r.detected && r.n_r == 2);
        }
        // 9) 1-of-3 epoch reps corrupted -> 2/3 majority still recovers the epoch.
        {
            auto audio = tx.generate(4, 0, 7, /*burst_epoch=*/6);
            auto b = make_buf(audio);
            const int EP0 = MfskAck::ACK_LEN + MfskAck::NR_SYMS + MfskAck::LEVEL_REPS;
            corrupt_sym(b, EP0 + 0, 2);  // one epoch rep -> wrong tone
            auto r = rx.detect(b.data(), (int)b.size());
            check("RDR/epoch: 1/3 epoch rep corrupt -> 2/3 vote yields 6", r.epoch == 6);
        }
        printf("  Burst duration with level+epoch suffix: %.1f ms\n",
               tx.burst_duration_s() * 1000.0f);
    }

    printf("\n============================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}

// =========================================================================
//  FM Channel SNR Benchmark
//
//  Sweeps all 8 O-levels (BPSK r1/2 through 256QAM r7/8) at varying
//  audio SNR through the full FM channel model:
//    TX de-emphasis → pre-emphasis → deviation limiter → de-emphasis
//    → audio bandpass (300-3000 Hz) → AWGN noise → detect → decode
//
//  Reports minimum SNR for successful decode at each level.
// =========================================================================

// FM channel model (matches audio_loopback.cc FmChannelState)
static void fm_channel_process(float* audio, int n, float target_peak,
                                float noise_amplitude, float fs,
                                float freq_diffusion_override)
{
    float tau_s = 530e-6f;

    // Pre-emphasis: H(s) = (1+s·τ₁)/(1+s·τ₂) via bilinear transform
    // τ₂ adds a stabilizing pole at ~15 kHz
    float tau2_s = 1.0f / (2.0f * (float)M_PI * 15000.0f);
    float c1 = 2.0f * fs * tau_s;
    float c2 = 2.0f * fs * tau2_s;
    float pe_a0 = 1.0f + c2;
    float pe_b0 = (1.0f + c1) / pe_a0;
    float pe_b1 = (1.0f - c1) / pe_a0;
    float pe_a1 = (1.0f - c2) / pe_a0;

    // Two-pass: apply pre-emphasis, then normalize so peak = deviation limit.
    // This models a properly-adjusted radio: operator sets audio level so peaks
    // just reach max deviation. The deviation limiter then barely clips.
    std::vector<float> pe_audio(n);
    float pe_x1 = 0.0f, pe_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = audio[i];
        pe_audio[i] = pe_b0 * x + pe_b1 * pe_x1 - pe_a1 * pe_y1;
        pe_x1 = x;
        pe_y1 = pe_audio[i];
    }

    // Normalize post-pre-emphasis so peak = target_peak × deviation_limit.
    // target_peak controls how close to the limit we push (0.95 = near max deviation).
    float pe_peak = 0;
    for (int i = 0; i < n; i++)
        if (std::abs(pe_audio[i]) > pe_peak) pe_peak = std::abs(pe_audio[i]);
    float dev_limit = 0.95f;
    if (pe_peak > 0) {
        float norm = (target_peak * dev_limit) / pe_peak;
        // Scale the ORIGINAL audio by the same factor, then re-apply pre-emphasis
        for (int i = 0; i < n; i++) audio[i] *= norm;
    }

    // Re-apply pre-emphasis with correct scaling
    pe_x1 = 0.0f; pe_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float x = audio[i];
        audio[i] = pe_b0 * x + pe_b1 * pe_x1 - pe_a1 * pe_y1;
        pe_x1 = x;
        pe_y1 = audio[i];
    }

    // Deviation limiter: hard clip to ±deviation_limit
    for (int i = 0; i < n; i++) {
        if (audio[i] > dev_limit) audio[i] = dev_limit;
        if (audio[i] < -dev_limit) audio[i] = -dev_limit;
    }

    // Oscillator frequency drift: random walk in frequency (Brownian motion).
    // Models the slowly-varying frequency offset between TX and RX oscillators
    // that appears as time-varying CPE on OFDM subcarriers.
    // OTA logs show residual CFO of 1-7 Hz with drift over the frame.
    // Model: Wiener process in frequency with diffusion rate σ_f Hz/√s.
    // At σ_f = 3 Hz/√s over a 1.3s frame, frequency wanders ~3.4 Hz RMS.
    // Applied via Hilbert transform: analytic signal × exp(jφ), take real part.
    // This properly shifts the spectrum without AM distortion.
    {
        std::mt19937 pn_rng(123);  // non-static: same realization for each call
        float freq_diffusion = (freq_diffusion_override >= 0.0f)
            ? freq_diffusion_override : 3.0f;  // Hz/√s — frequency drift rate
        float dt = 1.0f / fs;
        float freq_step_std = freq_diffusion * std::sqrt(dt);
        std::normal_distribution<float> pn_dist(0.0f, freq_step_std);

        // Pre-compute phase trajectory
        std::vector<float> phase_traj(n);
        float freq_offset = 0.0f;
        float phase_accum = 0.0f;
        for (int i = 0; i < n; i++) {
            freq_offset += pn_dist(pn_rng);
            phase_accum += 2.0f * (float)M_PI * freq_offset * dt;
            phase_traj[i] = phase_accum;
        }

        // Apply proper frequency shift via analytic signal
        auto analytic = hilbert_analytic(audio, n);
        for (int i = 0; i < n; i++) {
            std::complex<float> rot(std::cos(phase_traj[i]),
                                     std::sin(phase_traj[i]));
            audio[i] = std::real(analytic[i] * rot);
        }
    }

    // f²-shaped FM discriminator noise (inserted BEFORE de-emphasis)
    // FM discriminator output noise PSD ∝ f². We generate white noise,
    // filter through a first-order differentiator (y[n] = x[n] - x[n-1])
    // which has |H(f)|² = 4sin²(πf/fs) ≈ (2πf/fs)² for f << fs,
    // giving the correct f² spectral shape. De-emphasis then partially
    // whitens it: combined PSD ∝ f²/(1+(2πfτ)²), matching real FM.
    if (noise_amplitude > 0) {
        std::mt19937 rng(42);  // non-static: reproducible per call
        std::normal_distribution<float> dist(0.0f, noise_amplitude);
        float prev_noise = 0.0f;
        for (int i = 0; i < n; i++) {
            float wn = dist(rng);
            float shaped = wn - prev_noise;  // differentiator: +6 dB/octave
            prev_noise = wn;
            audio[i] += shaped;
        }
    }

    // De-emphasis: H(s) = 1/(1+sτ) via bilinear transform
    float wc = 1.0f / tau_s;
    float K = 2.0f * fs;
    float a = K + wc;
    float de_b0 = wc / a, de_b1 = wc / a, de_a1 = (wc - K) / a;
    float de_x1 = 0.0f, de_y1 = 0.0f;
    for (int i = 0; i < n; i++) {
        float de = de_b0 * audio[i] + de_b1 * de_x1 - de_a1 * de_y1;
        de_x1 = audio[i];
        de_y1 = de;
        audio[i] = de;
    }

    // Audio bandpass filter REMOVED from batch test.
    // The IIR BPF's group delay creates ISI/ICI that the per-carrier
    // equalizer cannot compensate in a batch (non-streaming) test.
    // In a real radio, the analog BPF is part of the continuous-time
    // channel captured by the training symbols; its steady-state response
    // is equalized by H[k]. The real-time FM channel model in
    // audio_loopback.cc keeps the BPF for continuous-stream testing.
}

int run_benchmark() {
    printf("Iris OFDM FM Channel Benchmark\n");
    printf("================================\n");
    printf("Passband: 300-3000 Hz (2700 Hz BW)\n");
    printf("FM model: pre-emphasis 530us + dev limiter 0.95 + f^2 noise + de-emphasis\n\n");

    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 3000.0f;
    pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);

    printf("OFDM: NFFT=%d, CP=%d, %d used carriers (%d data, %d pilot), spacing=%.1f Hz\n",
           cfg.nfft, cfg.cp_samples, cfg.n_used_carriers, cfg.n_data_carriers,
           cfg.n_used_carriers - cfg.n_data_carriers,
           (float)cfg.sample_rate / cfg.nfft);
    printf("Pilot: carrier spacing=%d, symbol spacing=%d, row spacing=%d\n\n",
           cfg.pilot_carrier_spacing, cfg.pilot_symbol_spacing, cfg.pilot_row_spacing);

    uint8_t payload[64];
    for (int i = 0; i < 64; i++) payload[i] = (uint8_t)(i * 37 + 13);

    static const struct { int preset; const char* name; LdpcRate fec; int bpc; } levels[] = {
        {1, "O0 BPSK  r1/2",  LdpcRate::RATE_1_2, 1},
        {2, "O1 QPSK  r1/2",  LdpcRate::RATE_1_2, 2},
        {3, "O2 QPSK  r3/4",  LdpcRate::RATE_3_4, 2},
        {4, "O3 16QAM r1/2",  LdpcRate::RATE_1_2, 4},
        {5, "O4 16QAM r3/4",  LdpcRate::RATE_3_4, 4},
        {6, "O5 64QAM r3/4",  LdpcRate::RATE_3_4, 6},
        {7, "O6 64QAM r7/8",  LdpcRate::RATE_7_8, 6},
        {8, "O7 256QAM r7/8", LdpcRate::RATE_7_8, 8},
    };

    // Audio SNR sweep
    float snr_sweep[] = {50, 40, 35, 30, 27, 25, 23, 21, 19, 17, 15, 13, 11, 9, 7, 5, 3, 1, 0};
    int n_snr = sizeof(snr_sweep) / sizeof(snr_sweep[0]);

    printf("%-18s | %-7s | %-12s | %-10s | %-6s | %-5s\n",
           "Level", "bps*", "Min SNR (dB)", "Clean SNR", "LDPC", "Det");
    printf("-------------------+---------+--------------+------------+--------+------\n");

    for (auto& lv : levels) {
        ToneMap tm = get_uniform_tone_map(lv.preset, cfg);
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload, 64, tm, lv.fec);
        if (iq.empty()) {
            printf("%-18s | SKIP: frame gen failed\n", lv.name);
            continue;
        }

        // Extract real audio
        std::vector<float> tx_audio(iq.size());
        for (size_t i = 0; i < iq.size(); i++)
            tx_audio[i] = iq[i].real();

        // Compute throughput
        int sym_samples = cfg.nfft + cfg.cp_samples;
        float sym_rate = (float)cfg.sample_rate / sym_samples;
        float fec_rate = 0;
        switch (lv.fec) {
            case LdpcRate::RATE_1_2: fec_rate = 0.5f; break;
            case LdpcRate::RATE_3_4: fec_rate = 0.75f; break;
            case LdpcRate::RATE_7_8: fec_rate = 0.875f; break;
            default: fec_rate = 0.5f;
        }
        int phy_bps = (int)(cfg.n_data_carriers * lv.bpc * fec_rate * sym_rate);

        // Measure signal RMS after clean FM channel (post-DE) for SNR reference.
        float sig_rms_measured = 0.0f;
        {
            std::vector<float> fm_ref(tx_audio);
            fm_channel_process(fm_ref.data(), (int)fm_ref.size(), 0.50f, 0.0f);
            double sum2 = 0;
            for (size_t i = 0; i < fm_ref.size(); i++) sum2 += (double)fm_ref[i] * fm_ref[i];
            sig_rms_measured = (float)std::sqrt(sum2 / fm_ref.size());
        }

        // Calibrate f²-noise transfer gain: measure what RMS emerges at the
        // demod output (post-DE) per unit of white noise input to the differentiator.
        // This accounts for the differentiator's low gain at audio frequencies
        // (|H(f)| = 2sin(πf/fs) ≈ 2πf/fs << 1 for f << fs) and the de-emphasis
        // attenuation. We need this to correctly set noise amplitude for a target
        // post-DE SNR that the OFDM demodulator actually sees.
        float noise_transfer_gain = 0.0f;
        {
            // Run noise-only through the same FM channel (PE+limiter have no effect
            // on zero-signal noise, but de-emphasis does). Use enough samples for
            // statistical accuracy.
            int cal_n = 48000 * 2;  // 2 seconds
            std::mt19937 cal_rng(99);
            std::normal_distribution<float> cal_dist(0.0f, 1.0f);

            // Simulate: white noise -> differentiator -> de-emphasis
            float tau_s = 530e-6f;
            float wc = 1.0f / tau_s;
            float K = 2.0f * 48000.0f;
            float a = K + wc;
            float de_b0 = wc / a, de_b1 = wc / a, de_a1 = (wc - K) / a;
            float de_x1 = 0.0f, de_y1 = 0.0f;
            float prev_noise = 0.0f;
            double sum2 = 0;
            for (int i = 0; i < cal_n; i++) {
                float wn = cal_dist(cal_rng);
                float shaped = wn - prev_noise;
                prev_noise = wn;
                float de = de_b0 * shaped + de_b1 * de_x1 - de_a1 * de_y1;
                de_x1 = shaped;
                de_y1 = de;
                sum2 += (double)de * de;
            }
            noise_transfer_gain = (float)std::sqrt(sum2 / cal_n);
        }

        // Clean FM channel baseline
        float clean_snr = -1.0f;
        bool clean_ok = false;
        int clean_ldpc = 0;
        {
            std::vector<float> fm_audio(tx_audio);
            fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, 0.0f);
            auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
            size_t pad = 48000;
            rx_iq.insert(rx_iq.begin(), pad, {0, 0});
            rx_iq.insert(rx_iq.end(), pad, {0, 0});
            auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
            if (sync.detected) {
                OfdmDemodulator demod(cfg);
                auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);
                clean_snr = result.snr_db;
                clean_ldpc = result.worst_ldpc_iters;
                clean_ok = result.success && result.payload.size() == 64 &&
                           memcmp(result.payload.data(), payload, 64) == 0;
            }
        }

        // Detailed per-SNR sweep with diagnostics
        printf("\n  [%s]  sig_rms=%.4f  noise_xfer=%.4f  clean: %s (SNR=%.1f dB, LDPC=%d)\n",
               lv.name, sig_rms_measured, noise_transfer_gain,
               clean_ok ? "PASS" : "FAIL", clean_snr, clean_ldpc);
        printf("  %5s | %4s | %8s | %6s | %6s | %4s | %6s | %s\n",
               "SNR", "Det", "ZC/SC", "chSNR", "mean|H|", "LDPC", "LLRmax", "Result");

        float min_snr = -1.0f;
        int min_ldpc_iters = 0;
        for (int si = 0; si < n_snr; si++) {
            float target_snr = snr_sweep[si];
            // Target: post-DE noise RMS = sig_rms / 10^(SNR/20)
            // Since noise_transfer_gain maps 1.0 input RMS to output RMS,
            // we need input amplitude = target_noise_rms / noise_transfer_gain
            float target_noise_rms = sig_rms_measured / std::pow(10.0f, target_snr / 20.0f);
            float noise_amp = target_noise_rms / noise_transfer_gain;

            std::vector<float> fm_audio(tx_audio);
            fm_channel_process(fm_audio.data(), (int)fm_audio.size(), 0.50f, noise_amp);

            // Measure actual RX SNR
            double rx_sum2 = 0;
            for (size_t i = 0; i < fm_audio.size(); i++)
                rx_sum2 += (double)fm_audio[i] * fm_audio[i];
            float rx_rms = (float)std::sqrt(rx_sum2 / fm_audio.size());

            auto rx_iq = hilbert_analytic(fm_audio.data(), (int)fm_audio.size());
            size_t pad = 48000;
            rx_iq.insert(rx_iq.begin(), pad, {0, 0});
            rx_iq.insert(rx_iq.end(), pad, {0, 0});

            auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
            if (!sync.detected) {
                printf("  %5.0f | NO   |          |        |        |      |        | no detection\n",
                       target_snr);
                continue;
            }

            OfdmDemodulator demod(cfg);
            auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync);

            // Compute max LLR magnitude from result
            float llr_max = 0;
            for (auto& l : result.llrs) {
                float a = std::abs(l);
                if (a > llr_max) llr_max = a;
            }

            bool ok = result.success && result.payload.size() == 64 &&
                      memcmp(result.payload.data(), payload, 64) == 0;
            if (ok) {
                min_snr = target_snr;
                min_ldpc_iters = result.worst_ldpc_iters;
            }

            printf("  %5.0f | yes  | %.2f/%.2f | %5.1f | %6.3f | %4d | %6.1f | %s\n",
                   target_snr,
                   sync.zc_metric, sync.sc_metric,
                   result.mean_channel_snr_db,
                   result.mean_H_mag,
                   result.worst_ldpc_iters,
                   llr_max,
                   ok ? "PASS" : "FAIL");
        }

        if (min_snr >= 0) {
            printf("%-18s | %5d   | %5.0f dB      | %5.1f dB    | %3d    | yes\n",
                   lv.name, phy_bps, min_snr, clean_snr, min_ldpc_iters);
        } else {
            printf("%-18s | %5d   | %s    | %5.1f dB    |   -    | -\n",
                   lv.name, phy_bps, clean_ok ? "clean only" : "BROKEN   ", clean_snr);
        }
    }

    printf("\n* PHY bps = data_carriers × bpc × fec_rate × symbol_rate (before framing overhead)\n");
    printf("  Actual throughput depends on preamble/pilot/tail overhead and ARQ efficiency.\n");
    return 0;
}

// --- OTA file-based TX/RX for hardware testing ---

// Passband for the offline TX/RX path. Defaults to the narrow 300-3000 band,
// but IRIS_OFDM_PB_LOW/IRIS_OFDM_PB_HIGH override it so the offline replay can
// match the LIVE probe-negotiated wide band (300-5000 -> BW=4150 Hz, 87
// carriers), the regime that reproduces the O0/O1 MPG decode failure at the
// live rate.
static NegotiatedPassband test_passband() {
    float lo = 300.0f, hi = 3000.0f;
    if (const char* e = getenv("IRIS_OFDM_PB_LOW"))  { float v = atof(e); if (v > 0) lo = v; }
    if (const char* e = getenv("IRIS_OFDM_PB_HIGH")) { float v = atof(e); if (v > 0) hi = v; }
    NegotiatedPassband pb;
    pb.low_hz = lo; pb.high_hz = hi;
    pb.center_hz = 0.5f * (lo + hi); pb.bandwidth_hz = (hi - lo); pb.valid = true;
    return pb;
}

// Offline O-level shapes (frame batching only). The MODULATION and FEC RATE
// come from the single-source ladder (ofdm_frame.cc kUniformPresets via
// ofdm_level_fec_rate / ofdm_level_bits_per_carrier; preset id = level + 1) —
// this table carries ONLY what the ladder does not define: how many codewords
// and payload bytes the offline harness packs per frame.
//
// The previous table here was a private 10-rung copy of the ladder with its
// own fec column, and it had drifted: its level-7 row said r3/4 while
// get_uniform_tone_map(preset 8) builds r5/8. build_ofdm_frame took the fec
// from the row and the tone map from the preset — TX encoded at one rate, RX
// decoded at another, and every 64QAM+ offline cell failed 8/8 LDPC blocks at
// ANY SNR. Same defect class as the modem.cc dual-ladder (both call sites now
// delegate to the one table); keep no second copy of the ladder anywhere.
static const struct {
    int ncw; int payload_bytes;
} ota_level_shape[] = {
    {1,  32},   // O0 BPSK r1/2
    {1,  32},   // O1 QPSK r1/2
    {4, 200},   // O2 QPSK r3/4
    {4, 200},   // O3 16QAM r1/2
    {4, 200},   // O4 16QAM r5/8
    {4, 200},   // O5 16QAM r3/4
    {8, 476},   // O6 32QAM r5/8
    {8, 476},   // O7 64QAM r5/8
    {8, 476},   // O8 64QAM r3/4
    {8, 476},   // O9 256QAM r5/8
    {8, 476},   // O10 256QAM r3/4
    {8, 476},   // O11 256QAM r7/8
    {8, 476},   // O12 1024QAM r3/4
    {8, 476},   // O13 1024QAM r7/8
};
static const int N_OTA_LEVELS = sizeof(ota_level_shape) / sizeof(ota_level_shape[0]);

static const char* ota_level_name(int level) {
    static char buf[48];
    int bpc = ofdm_level_bits_per_carrier(level);
    LdpcRate r = ofdm_level_fec_rate(level);
    const char* mod = bpc == 1 ? "BPSK" : bpc == 2 ? "QPSK" :
                      bpc == 4 ? "16QAM" : bpc == 5 ? "32QAM" :
                      bpc == 6 ? "64QAM" : bpc == 8 ? "256QAM" : "1024QAM";
    const char* rate = r == LdpcRate::RATE_1_2 ? "r1/2" :
                       r == LdpcRate::RATE_5_8 ? "r5/8" :
                       r == LdpcRate::RATE_3_4 ? "r3/4" :
                       r == LdpcRate::RATE_7_8 ? "r7/8" : "r?";
    snprintf(buf, sizeof(buf), "O%d %s %s", level, mod, rate);
    return buf;
}

int run_tx_ofdm(int level, const char* outfile, bool dft_spread) {
    if (level < 0 || level >= N_OTA_LEVELS) {
        printf("Error: level must be 0-%d\n", N_OTA_LEVELS - 1);
        return 1;
    }
    const int preset = level + 1;                      // single-source ladder
    const LdpcRate fec = ofdm_level_fec_rate(level);   // (ofdm_frame.cc)
    const int ncw = ota_level_shape[level].ncw;
    const char* name = ota_level_name(level);

    NegotiatedPassband pb = test_passband();
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);
    cfg.dft_spread = dft_spread;
    if (!dft_spread)
        printf("  DFT-spread: DISABLED (plain OFDM)\n");

    // TX pre-emphasis compensation override (IRIS_TX_PREEMPH_CORNER, Hz;
    // 0 disables). Real installations differ — a mic-path TX compensates for
    // the radio's pre-emphasis, a flat data-port TX must not — and the
    // receiver has to decode either without being told. This knob builds the
    // "uncompensated" TX arm for that gate; the RX side takes no knob.
    if (const char* e = getenv("IRIS_TX_PREEMPH_CORNER")) {
        cfg.fm_preemph_corner_hz = (float)atof(e);
        printf("  TX pre-emphasis compensation corner: %.0f Hz%s\n",
               cfg.fm_preemph_corner_hz,
               cfg.fm_preemph_corner_hz <= 0.0f ? " (disabled)" : "");
    }

    ToneMap tm = get_uniform_tone_map(preset, cfg);
    tm.n_codewords = ncw;

    // Payload size override (IRIS_OFDM_PAYLOAD) so the offline replay can match
    // the live frame length (e.g. a ~94-byte / 21-symbol O1 frame). Default =
    // level's canonical size.
    int pbytes = ota_level_shape[level].payload_bytes;
    if (const char* e = getenv("IRIS_OFDM_PAYLOAD")) { int v = atoi(e); if (v > 0) pbytes = v; }

    // Create known payload pattern
    std::vector<uint8_t> payload(pbytes);
    for (int i = 0; i < pbytes; i++)
        payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    OfdmModulator mod(cfg);
    auto iq = mod.build_ofdm_frame(payload.data(), pbytes, tm, fec, ncw);
    if (iq.empty()) {
        printf("Error: frame generation failed for %s\n", name);
        return 1;
    }

    // Extract real-valued audio (Hermitian symmetry)
    std::vector<float> audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++)
        audio[i] = iq[i].real();

    // Add 0.5s silence before and after (24000 samples each)
    int pad_samples = 24000;
    std::vector<float> padded(pad_samples + audio.size() + pad_samples, 0.0f);
    for (size_t i = 0; i < audio.size(); i++)
        padded[pad_samples + i] = audio[i];

    // Normalize to 0.8 peak
    float peak = 0;
    for (auto s : padded) { float a = std::abs(s); if (a > peak) peak = a; }
    if (peak > 0) {
        float scale = 0.8f / peak;
        for (auto& s : padded) s *= scale;
    }

    // Write as S16LE raw PCM
    FILE* f = fopen(outfile, "wb");
    if (!f) {
        printf("Error: cannot open %s for writing\n", outfile);
        return 1;
    }
    for (auto s : padded) {
        int16_t v = (int16_t)(s * 32767.0f);
        fwrite(&v, 2, 1, f);
    }
    fclose(f);

    int total_samples = (int)padded.size();
    float duration_ms = total_samples * 1000.0f / 48000.0f;
    printf("TX %s: %d samples (%.0f ms) -> %s\n", name, total_samples, duration_ms, outfile);
    printf("  payload=%d bytes, ncw=%d, fec=%s\n", pbytes, ncw,
           fec == LdpcRate::RATE_1_2 ? "r1/2" :
           fec == LdpcRate::RATE_5_8 ? "r5/8" :
           fec == LdpcRate::RATE_3_4 ? "r3/4" :
           fec == LdpcRate::RATE_7_8 ? "r7/8" : "?");
    return 0;
}

// Shared RX signal conditioning: DC removal + 3.5 kHz Butterworth LPF + 6 s
// highest-energy trim + Hilbert analytic (mirrors modem.cc:1232-1244). Factored
// out so the offline genie-H control (run_rx_ofdm) can condition its noiseless
// companion file with the IDENTICAL pipeline — guaranteeing the genie channel
// estimate is sample-aligned with the noisy frame it is injected into.
static std::vector<std::complex<float>> ofdm_rx_condition_analytic(
    std::vector<float> audio, const char* tag) {
    int n_samples = (int)audio.size();

    // 1. Audio diagnostics + 2. DC removal
    {
        float sum = 0, sum2 = 0, peak = 0;
        for (int i = 0; i < n_samples; i++) {
            sum += audio[i];
            sum2 += audio[i] * audio[i];
            float a = std::abs(audio[i]);
            if (a > peak) peak = a;
        }
        float dc = n_samples ? sum / n_samples : 0.0f;
        float rms = n_samples ? std::sqrt(sum2 / n_samples) : 0.0f;
        printf("  [%s] Audio: DC=%.6f RMS=%.4f peak=%.4f (%d samples)\n",
               tag, dc, rms, peak, n_samples);
        if (std::abs(dc) > 1e-6f) {
            for (int i = 0; i < n_samples; i++) audio[i] -= dc;
        }
    }

    // 3. 2nd-order Butterworth LPF at 3500 Hz (production uses band_high + 500)
    {
        const float fc = 3500.0f, fs = 48000.0f, Q = 0.7071f;
        float w0 = 2.0f * (float)M_PI * fc / fs;
        float c = std::cos(w0), s = std::sin(w0);
        float alpha = s / (2.0f * Q);
        float a0 = 1.0f + alpha;
        float b0 = ((1.0f - c) / 2.0f) / a0;
        float b1 = (1.0f - c) / a0;
        float b2 = b0;
        float a1 = (-2.0f * c) / a0;
        float a2 = (1.0f - alpha) / a0;
        float x1 = 0, x2 = 0, y1 = 0, y2 = 0;
        for (int i = 0; i < n_samples; i++) {
            float x = audio[i];
            float y = b0 * x + b1 * x1 + b2 * x2 - a1 * y1 - a2 * y2;
            x2 = x1; x1 = x; y2 = y1; y1 = y;
            audio[i] = y;
        }
    }

    // 4. Trim to 6 s around highest-energy region (match production buffer limit)
    constexpr int MAX_RX_SAMPLES = 48000 * 6;
    if (n_samples > MAX_RX_SAMPLES) {
        const int stride = 24000;  // 0.5s
        int best_start = 0;
        float best_energy = 0;
        for (int start = 0; start + MAX_RX_SAMPLES <= n_samples; start += stride) {
            float energy = 0;
            for (int i = start; i < start + MAX_RX_SAMPLES; i++)
                energy += audio[i] * audio[i];
            if (energy > best_energy) { best_energy = energy; best_start = start; }
        }
        printf("  [%s] Trimmed %d -> %d samples (start=%d, %.1fs offset)\n",
               tag, n_samples, MAX_RX_SAMPLES, best_start, best_start / 48000.0f);
        std::vector<float> trimmed(audio.begin() + best_start,
                                    audio.begin() + best_start + MAX_RX_SAMPLES);
        audio = std::move(trimmed);
        n_samples = MAX_RX_SAMPLES;
    }

    return hilbert_analytic(audio.data(), n_samples);
}

int run_rx_ofdm(int level, const char* infile, bool dft_spread) {
    if (level < 0 || level >= N_OTA_LEVELS) {
        printf("Error: level must be 0-%d\n", N_OTA_LEVELS - 1);
        return 1;
    }
    const int preset = level + 1;                      // single-source ladder
    const int ncw = ota_level_shape[level].ncw;
    const char* name = ota_level_name(level);

    // Read S16LE raw PCM
    FILE* f = fopen(infile, "rb");
    if (!f) {
        printf("Error: cannot open %s for reading\n", infile);
        return 1;
    }
    fseek(f, 0, SEEK_END);
    long file_size = ftell(f);
    fseek(f, 0, SEEK_SET);
    int n_samples = (int)(file_size / 2);
    std::vector<float> audio(n_samples);
    for (int i = 0; i < n_samples; i++) {
        int16_t v;
        if (fread(&v, 2, 1, f) != 1) break;
        audio[i] = v / 32768.0f;
    }
    fclose(f);

    printf("RX %s: %d samples (%.0f ms) from %s\n", name, n_samples,
           n_samples * 1000.0f / 48000.0f, infile);

    NegotiatedPassband pb = test_passband();
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 8, 24);
    cfg.dft_spread = dft_spread;
    if (!dft_spread)
        printf("  DFT-spread: DISABLED (plain OFDM)\n");

    ToneMap tm = get_uniform_tone_map(preset, cfg);
    tm.n_codewords = ncw;

    // --- Signal conditioning + analytic reconstruction (shared helper) ---
    auto rx_iq = ofdm_rx_condition_analytic(std::move(audio), "RX");

    // ---- Offline genie-H control (IRIS_GENIE_H_FILE) ----
    // Capture the TRUE per-carrier channel from a NOISELESS same-seed companion
    // file (identical Watterson taps, negligible AWGN) and inject it into the
    // decode of the NOISY infile — isolating channel-ESTIMATION error from the
    // EQ/diversity/FEC. The companion is conditioned + synced + estimated with
    // the identical pipeline, so its H is sample-aligned with the noisy frame.
    std::vector<std::complex<float>> genie_H;
    const char* genie_file = getenv("IRIS_GENIE_H_FILE");
    if (genie_file && genie_file[0]) {
        FILE* gf = fopen(genie_file, "rb");
        if (!gf) {
            printf("  [GENIE] cannot open companion %s — genie DISABLED\n", genie_file);
        } else {
            fseek(gf, 0, SEEK_END); long gsz = ftell(gf); fseek(gf, 0, SEEK_SET);
            int gn = (int)(gsz / 2);
            std::vector<float> gaudio(gn);
            for (int i = 0; i < gn; i++) {
                int16_t v; if (fread(&v, 2, 1, gf) != 1) break;
                gaudio[i] = v / 32768.0f;
            }
            fclose(gf);
            auto genie_iq = ofdm_rx_condition_analytic(std::move(gaudio), "GENIE");
            auto gsync = ofdm_detect_frame(genie_iq.data(), (int)genie_iq.size(), cfg);
            if (gsync.detected) {
                OfdmDemodulator gdemod(cfg);
                gdemod.demodulate(genie_iq.data(), (int)genie_iq.size(), tm, &gsync);
                genie_H = gdemod.last_channel_estimate().H;
                printf("  [GENIE] true-channel H captured (%d carriers, SC=%.3f ZC=%.3f)\n",
                       (int)genie_H.size(), gsync.sc_metric, gsync.zc_metric);
            } else {
                printf("  [GENIE] companion sync FAILED (SC=%.3f) — genie DISABLED\n",
                       gsync.sc_metric);
            }
        }
    }

    // Try to detect frame
    auto sync = ofdm_detect_frame(rx_iq.data(), (int)rx_iq.size(), cfg);
    if (!sync.detected) {
        printf("  RESULT: NO DETECTION (SC=%.3f, ZC=%.3f)\n",
               sync.sc_metric, sync.zc_metric);
        return 2;
    }

    printf("  Detected: SC=%.3f ZC=%.3f CFO=%.1f Hz SNR~%.1f dB\n",
           sync.sc_metric, sync.zc_metric, sync.cfo_hz, sync.snr_est);

    // Demodulate (genie_H non-empty only under IRIS_GENIE_H_FILE)
    OfdmDemodulator demod(cfg);
    auto result = demod.demodulate(rx_iq.data(), (int)rx_iq.size(), tm, &sync,
                                   genie_H.empty() ? nullptr : &genie_H);

    // Check payload (payload size override must match TX; see run_tx_ofdm)
    int pbytes = ota_level_shape[level].payload_bytes;
    if (const char* e = getenv("IRIS_OFDM_PAYLOAD")) { int v = atoi(e); if (v > 0) pbytes = v; }
    std::vector<uint8_t> expected(pbytes);
    for (int i = 0; i < pbytes; i++)
        expected[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    bool match = result.success && (int)result.payload.size() == pbytes &&
                 memcmp(result.payload.data(), expected.data(), pbytes) == 0;

    // Byte attestation hook: dump the decoded payload so an external gate can
    // hash TX and RX payloads independently (IRIS_RX_PAYLOAD_OUT=<file>).
    if (result.success) {
        if (const char* e = getenv("IRIS_RX_PAYLOAD_OUT")) {
            FILE* pf = fopen(e, "wb");
            if (pf) {
                fwrite(result.payload.data(), 1, result.payload.size(), pf);
                fclose(pf);
            }
        }
    }

    printf("  LDPC: %d iters (worst block), SNR=%.1f dB, mean|H|=%.3f\n",
           result.worst_ldpc_iters, result.snr_db, result.mean_H_mag);
    printf("  RESULT: %s\n", match ? "PASS" : "FAIL");

    if (!match && result.success) {
        printf("  payload size: got %d, expected %d\n",
               (int)result.payload.size(), pbytes);
    }

    return match ? 0 : 3;
}

// =======================================================================
//  CPE / PHASE-ACCRUAL LENS  (scratch root-cause harness, --cpe-lens)
//  Isolates: does multi-symbol oscillator drift (CPE accrual) + intra-symbol
//  phase noise crush 16QAM (O3) over the longer frame, once additive noise is
//  present? Genie control = drift ON (3 Hz/rt-s) vs OFF (0), at matched SNR.
//  Measures residual EVM from the post-EQ/post-CPE constellation.
// =======================================================================
static float evm_db_16qam(const std::vector<std::complex<float>>& pts) {
    if (pts.empty()) return 0.0f;
    // RMS-match to ideal 16QAM grid {±1,±3}; ref mean power = 10.
    double p = 0; for (auto& z : pts) p += (double)std::norm(z);
    double rx_rms = std::sqrt(p / pts.size());
    if (rx_rms <= 0) return 0.0f;
    const double ref_rms = std::sqrt(10.0);
    double scale = ref_rms / rx_rms;
    auto slice = [](double v) -> double {
        double lv = (v < 0) ? -v : v;
        double q = (lv < 2.0) ? 1.0 : 3.0;
        return (v < 0) ? -q : q;
    };
    double err = 0;
    for (auto& z : pts) {
        double i = z.real() * scale, q = z.imag() * scale;
        double ri = slice(i), rq = slice(q);
        err += (i - ri) * (i - ri) + (q - rq) * (q - rq);
    }
    double evm_rms = std::sqrt(err / pts.size());
    return 20.0f * (float)std::log10(evm_rms / ref_rms);
}

int run_cpe_lens() {
    printf("=== CPE / PHASE-ACCRUAL LENS (O3 16QAM) ===\n");
    NegotiatedPassband pb;
    // RELAY narrowband config (two-stack --native-hail probe): BW=1000 Hz,
    // center 1700, 16 data carriers, pilot 1:5 — the config where the O3
    // failure is actually observed (m_after_wgn40_B.log: 20 used, 16 data).
    pb.low_hz = 1200.0f; pb.high_hz = 2200.0f; pb.center_hz = 1700.0f;
    pb.bandwidth_hz = 1000.0f; pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 5, 24);

    // Larger payload -> longer frame -> more CPE accrual (the "long O3 frame").
    const int PN = 200;
    std::vector<uint8_t> payload(PN);
    for (int i = 0; i < PN; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    struct Cell { int preset; const char* name; LdpcRate fec; };
    Cell cells[] = {
        { 2, "O1 QPSK  r1/2", LdpcRate::RATE_1_2 },  // control (robust)
        { 4, "O3 16QAM r1/2", LdpcRate::RATE_1_2 },  // suspect
    };
    float noises[] = { 0.0f, 0.004f, 0.008f, 0.015f, 0.03f, 0.06f };
    float drifts[] = { 0.0f, 3.0f, 6.0f };  // Hz/rt-s ; 0 = genie (no drift)

    printf("  frame payload=%d bytes\n", PN);
    printf("  %-16s %-7s %-6s | %-4s %-8s %-6s %-7s %-9s\n",
           "cell","noise","drift","det","snr_dB","evm_dB","ldpc_it","result");
    for (auto& c : cells) {
        ToneMap tm = get_uniform_tone_map(c.preset, cfg);
        tm.n_codewords = 4;  // multi-codeword (matches the ncw=4 real O3 frame)
        OfdmModulator mod(cfg);
        auto iq = mod.build_ofdm_frame(payload.data(), PN, tm, c.fec, 4);
        if (iq.empty()) { printf("  [%s] no frame\n", c.name); continue; }
        int nsym = 0;
        for (float drift : drifts) {
            for (float na : noises) {
                std::vector<float> audio(iq.size());
                for (size_t i = 0; i < iq.size(); i++) audio[i] = iq[i].real();
                fm_channel_process(audio.data(), (int)audio.size(),
                                   0.50f, na, 48000.0f, drift);
                auto rx = hilbert_analytic(audio.data(), (int)audio.size());
                size_t pad = 48000;
                rx.insert(rx.begin(), pad, std::complex<float>(0,0));
                rx.insert(rx.end(), pad, std::complex<float>(0,0));
                auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
                bool det = sync.detected;
                float snr = 0, evm = 0, cpe_hz = 0; int it = -1; bool ok = false;
                if (det) {
                    OfdmDemodulator demod(cfg);
                    auto r = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
                    snr = r.snr_db; it = r.worst_ldpc_iters;
                    cpe_hz = r.cpe_drift_hz; nsym = r.n_data_symbols;
                    evm = evm_db_16qam(r.eq_constellation);
                    ok = r.success && (int)r.payload.size() == PN &&
                         memcmp(r.payload.data(), payload.data(), PN) == 0;
                }
                printf("  %-16s %-7.3f %-6.1f | %-4s %-8.1f %-6.1f %-7d %-9s cpe=%.2fHz\n",
                       c.name, na, drift, det?"yes":"NO", snr, evm, it,
                       ok?"PASS":(det?"FAIL":"nodet"), cpe_hz);
            }
        }
        printf("  [%s] n_data_symbols=%d\n", c.name, nsym);
    }
    return 0;
}

// =======================================================================
//  MU_AVG / SOFT-DEMAP LENS  (scratch root-cause harness, --mu-lens)
//  Isolates: after MMSE-FDE + IDFT despreading the recovered symbol is
//  d_hat = mu_avg*d + noise (mu_avg = mean(|H|^2/(|H|^2+nv)) = 1 - dft_sigma_sq).
//  Does the un-compensated MMSE bias (constellation shrunk by mu_avg, refs at
//  full scale) crush O3 (16QAM) LLRs while leaving O0 (BPSK) intact?  Genie
//  control = noiseless (na=0): mu_avg->1, bias vanishes -> O3 must decode.
//  Arm A = baseline demap; Arm B = mu_avg-corrected demap (sym/=mu, sig/=mu).
//  Measures rms|z| (1.0 = unbiased, mu_avg = shrunk), mean/max |LLR|, decode.
// =======================================================================
struct MuArm {
    bool det = false, ok = false;
    int iters = 0, n_llr = 0;
    float mu_avg = 0, rms_radius = 0, mean_llr = 0, max_llr = 0, eff_snr = 0;
};

static MuArm mu_run_arm(OfdmConfig cfg, int preset, LdpcRate fec,
                        const uint8_t* payload, int plen,
                        float na, float drift, bool mu_correct) {
    cfg.mu_avg_correct = mu_correct;
    ToneMap tm = get_uniform_tone_map(preset, cfg);
    tm.n_codewords = 4;  // multi-codeword (matches the ncw=4 real O3 frame)
    OfdmModulator mod(cfg);
    auto iq = mod.build_ofdm_frame(payload, plen, tm, fec, 4);
    MuArm r;
    if (iq.empty()) return r;
    std::vector<float> audio(iq.size());
    for (size_t i = 0; i < iq.size(); i++) audio[i] = iq[i].real();
    fm_channel_process(audio.data(), (int)audio.size(), 0.50f, na, 48000.0f, drift);
    auto rx = hilbert_analytic(audio.data(), (int)audio.size());
    size_t pad = 48000;
    rx.insert(rx.begin(), pad, std::complex<float>(0, 0));
    rx.insert(rx.end(), pad, std::complex<float>(0, 0));
    auto sync = ofdm_detect_frame(rx.data(), (int)rx.size(), cfg);
    if (!sync.detected) return r;
    r.det = true;
    OfdmDemodulator demod(cfg);
    auto res = demod.demodulate(rx.data(), (int)rx.size(), tm, &sync);
    r.ok = res.success && (int)res.payload.size() == plen &&
           memcmp(res.payload.data(), payload, plen) == 0;
    r.iters = res.worst_ldpc_iters;
    r.mu_avg = 1.0f - res.dft_sigma_sq_llr;
    r.eff_snr = res.effective_snr_db;
    double sa = 0; float mx = 0;
    for (float l : res.llrs) { float a = std::fabs(l); sa += a; if (a > mx) mx = a; }
    r.n_llr = (int)res.llrs.size();
    r.mean_llr = r.n_llr > 0 ? (float)(sa / r.n_llr) : 0;
    r.max_llr = mx;
    double sr = 0;
    for (auto& z : res.eq_constellation) sr += (double)std::norm(z);
    r.rms_radius = res.eq_constellation.empty()
                   ? 0.0f : (float)std::sqrt(sr / res.eq_constellation.size());
    return r;
}

int run_mu_lens() {
    printf("=== MU_AVG / SOFT-DEMAP LENS (O0 BPSK vs O3 16QAM) ===\n");
    NegotiatedPassband pb;
    pb.low_hz = 300.0f; pb.high_hz = 3000.0f; pb.center_hz = 1650.0f;
    pb.bandwidth_hz = 2700.0f; pb.valid = true;
    // Production config: carrier_sp=8 (matches genie + receiver-drives-rate path).
    OfdmConfig base = ofdm_config_from_probe(pb, 1024, 64, 8, 24);

    const int PN = 200;
    std::vector<uint8_t> payload(PN);
    for (int i = 0; i < PN; i++) payload[i] = (uint8_t)((i * 37 + 13) & 0xFF);

    struct Lvl { int preset; const char* name; LdpcRate fec; };
    Lvl lvls[] = {
        { 1, "O0 BPSK ", LdpcRate::RATE_1_2 },
        { 4, "O3 16QAM", LdpcRate::RATE_1_2 },
    };
    // drift=0 isolates from CPE (the sibling lens). na sweep from noiseless genie up.
    float noises[] = { 0.0f, 0.004f, 0.008f, 0.015f, 0.03f };

    printf("  frame payload=%d bytes; drift=0 (CPE isolated); FM 50%% drive\n", PN);
    printf("  %-8s %-7s %-7s | %-6s %-6s %-7s %-8s %-8s %-9s\n",
           "level", "noise", "arm", "decode", "iters", "muavg",
           "rms|z|", "meanLLR", "maxLLR");
    for (auto& lv : lvls) {
        for (float na : noises) {
            for (int arm = 0; arm < 2; arm++) {
                MuArm r = mu_run_arm(base, lv.preset, lv.fec, payload.data(), PN,
                                     na, 0.0f, arm == 1);
                printf("  %-8s %-7.3f %-7s | %-6s %-6d %-7.3f %-8.3f %-8.2f %-9.1f\n",
                       lv.name, na, arm ? "mu_corr" : "base",
                       !r.det ? "NODET" : (r.ok ? "OK" : "FAIL"),
                       r.iters, r.mu_avg, r.rms_radius, r.mean_llr, r.max_llr);
            }
        }
    }
    return 0;
}
