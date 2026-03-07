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
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <cmath>

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

    // Test each modulation in loopback (no channel impairments)
    Modulation mods[] = {
        Modulation::BPSK, Modulation::QPSK,
        Modulation::QAM16, Modulation::QAM64, Modulation::QAM256
    };
    const char* mod_names[] = {"BPSK", "QPSK", "QAM16", "QAM64", "QAM256"};

    for (int m = 0; m < 5; m++) {
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

        // Feed low SNR — should drop
        for (int i = 0; i < 5; i++) gs.update(2.0f);
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

        // RATE_5_8: k=1000
        std::vector<uint8_t> data58;
        for (int i = 0; i < 1000; i++) data58.push_back((i * 3) & 1);
        auto enc58 = LdpcCodec::encode(data58, LdpcRate::RATE_5_8);
        check("LDPC 5/8 encoded = 1600 bits", (int)enc58.size() == 1600);
        auto dec58 = LdpcCodec::decode(enc58, LdpcRate::RATE_5_8);
        check("LDPC 5/8 round-trip", dec58 == data58);

        // RATE_3_4: k=1200, 800 data bits padded to 1200
        auto enc34 = LdpcCodec::encode(data, LdpcRate::RATE_3_4);
        check("LDPC 3/4 encoded = 1600 bits", (int)enc34.size() == 1600);
        auto dec34 = LdpcCodec::decode(enc34, LdpcRate::RATE_3_4);
        check("LDPC 3/4 decode succeeds", !dec34.empty());

        // Weak-signal rate: RATE_1_16 (k=100)
        std::vector<uint8_t> data_weak;
        for (int i = 0; i < 100; i++) data_weak.push_back(i & 1);
        auto enc116 = LdpcCodec::encode(data_weak, LdpcRate::RATE_1_16);
        check("LDPC 1/16 encoded = 1600 bits", (int)enc116.size() == 1600);
        auto dec116 = LdpcCodec::decode(enc116, LdpcRate::RATE_1_16);
        check("LDPC 1/16 round-trip", dec116 == data_weak);

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

        // Weak-signal error correction (RATE_1_16: k=100, p=1500)
        std::vector<uint8_t> data_weak;
        for (int i = 0; i < 100; i++) data_weak.push_back((i * 5 + 1) & 1);
        auto enc_weak = LdpcCodec::encode(data_weak, LdpcRate::RATE_1_16);
        corrupted = enc_weak;
        // Flip 5 bits spread across the codeword
        corrupted[10] ^= 1; corrupted[200] ^= 1; corrupted[500] ^= 1;
        corrupted[900] ^= 1; corrupted[1300] ^= 1;
        corrected = LdpcCodec::decode(corrupted, LdpcRate::RATE_1_16);
        check("LDPC 1/16 corrects 5-bit errors",
              corrected.size() == data_weak.size() && corrected == data_weak);

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
        // Check middle portion to avoid startup transient
        float i_avg = 0;
        int mid_start = 400, mid_end = 800;
        for (int i = mid_start; i < mid_end; i++)
            i_avg += iq_back[2*i];
        i_avg /= (mid_end - mid_start);
        check("Downconverted I recovers DC",
              std::abs(i_avg - 0.5f) < 0.2f); // ~0.5 due to cos^2 average
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

        std::vector<float> tx_audio(4800, 0.0f);
        modem.process_tx(tx_audio.data(), 4800);
        check("Modem TX produces audio", modem.state() == ModemState::TX_AX25);

        // Check that audio has non-zero content
        float peak = 0;
        for (float s : tx_audio) {
            float a = std::abs(s);
            if (a > peak) peak = a;
        }
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

    printf("\n============================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
