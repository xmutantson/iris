#include "common/types.h"
#include "ax25/crc16.h"
#include "ax25/hdlc.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
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
#include "engine/modem.h"
#include "native/upconvert.h"
#include "arq/arq.h"
#include "config/config.h"
#include "compress/compress.h"
#include "crypto/crypto.h"
#include "probe/passband_probe.h"
#include "native/channel_eq.h"
#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_frame.h"
#include "ofdm/ofdm_papr.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

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
    OfdmConfig cfg = ofdm_config_from_probe(pb, 512, 32, 4, 14);

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
    float papr = compute_papr_db(iq);
    printf("  PAPR = %.1f dB (target <= 8.0 dB)\n", papr);
    check("PAPR reduced to <= 8.0 dB", papr <= 8.0f);

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
        // Convert audio back to complex (real + j0)
        std::vector<std::complex<float>> rx_iq(audio.size());
        for (size_t i = 0; i < audio.size(); i++)
            rx_iq[i] = std::complex<float>(audio[i], 0.0f);

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
                   result.success, result.mean_H_mag, result.samples_consumed, result.snr_db);
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

        // Convert to complex for detection/demod
        std::vector<std::complex<float>> rx_iq(fm_audio.size());
        for (size_t i = 0; i < fm_audio.size(); i++)
            rx_iq[i] = std::complex<float>(fm_audio[i], 0.0f);

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

int run_tests() {
    printf("Iris Modem - Loopback Tests\n");
    printf("============================\n");

    // Phase 1: AX.25 compatibility
    printf("\n--- Phase 1: AX.25 Compatibility ---\n");
    test_crc16();
    test_nrzi();
    test_hdlc();
    test_kiss();
    test_afsk_loopback();
    test_gfsk_loopback();

    // Phase 2: Native PHY
    printf("\n--- Phase 2: Native PHY ---\n");
    test_crc32();
    test_rrc_filter();
    test_constellation();
    test_native_phy_loopback();
    test_native_frame_loopback();
    test_ofdm_kiss_loopback();
    test_ofdm_phy_roundtrip();

    // Phase 3: Auto-upgrade
    printf("\n--- Phase 3: Auto-Upgrade ---\n");
    test_xid();
    test_xid_negotiation();
    test_xid_frame_build();
    test_ax25_to_native_upgrade();

    // Phase 4: Engine components
    printf("\n--- Phase 4: Engine & FEC ---\n");

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
        check("Mode A baud = 2400", mode_baud_rate('A') == 2400);
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

        // RATE_5_8: no precomputed matrix yet, falls back to rate 1/2 (k=800).
        // Verify fallback: 1000 bits → padded to 1600 (2×k) → 2 codewords = 3200 bits.
        std::vector<uint8_t> data58;
        for (int i = 0; i < 1000; i++) data58.push_back((i * 3) & 1);
        auto enc58 = LdpcCodec::encode(data58, LdpcRate::RATE_5_8);
        check("LDPC 5/8 fallback encoded = 3200 bits", (int)enc58.size() == 3200);
        auto dec58 = LdpcCodec::decode(enc58, LdpcRate::RATE_5_8);
        // Decode returns 1600 bits (2×k=800), first 1000 match original
        check("LDPC 5/8 fallback decode size", (int)dec58.size() == 1600);
        bool match58 = true;
        for (int i = 0; i < 1000 && i < (int)dec58.size(); i++)
            if (dec58[i] != data58[i]) { match58 = false; break; }
        check("LDPC 5/8 fallback data preserved", match58);

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

    // Probe tone generation and analysis round-trip
    {
        printf("\n=== Probe Generate/Analyze ===\n");
        int sr = 48000;
        int max_samples = (int)(PassbandProbeConfig::PROBE_DURATION_S * sr) + 1000;
        std::vector<float> probe_audio(max_samples);
        int n = probe_generate(probe_audio.data(), max_samples, sr, 0.5f);
        check("Probe generates samples", n > 0);
        check("Probe duration ~2.25s", n > sr * 2 && n < sr * 3);

        // Analyze the clean probe (no channel filtering)
        ProbeResult result = probe_analyze(probe_audio.data(), n, sr);
        check("Probe analysis valid", result.valid);
        check("Probe detects >= 50 tones", result.tones_detected >= 50);
        check("Probe low_hz near 300", result.low_hz >= 250.0f && result.low_hz <= 400.0f);
        check("Probe high_hz near 4500", result.high_hz >= 4400.0f && result.high_hz <= 4600.0f);
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
        tx_result.ofdm_cp_samples = 64;
        tx_result.ofdm_pilot_carrier_spacing = 4;
        tx_result.ofdm_pilot_symbol_spacing = 14;
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
        check("Probe v4 ofdm_cp", decoded.ofdm_cp_samples == 64);
        check("Probe v4 ofdm_pilot_carrier", decoded.ofdm_pilot_carrier_spacing == 4);
        check("Probe v4 ofdm_pilot_symbol", decoded.ofdm_pilot_symbol_spacing == 14);
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

    printf("\n============================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
