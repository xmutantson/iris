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

int main() {
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

    printf("\n============================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
