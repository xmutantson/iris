#include "common/types.h"
#include "ax25/crc16.h"
#include "ax25/hdlc.h"
#include "ax25/afsk.h"
#include "ax25/gfsk.h"
#include "kiss/kiss.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

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

// --- Test CRC-16 ---
static void test_crc16() {
    printf("\n=== CRC-16 CCITT ===\n");
    // Standard test vector: "123456789" -> 0x906E (CRC-16/X.25 used by AX.25)
    const uint8_t data[] = "123456789";
    uint16_t crc = crc16_ccitt(data, 9);
    printf("    CRC value: 0x%04X\n", crc);
    check("CRC of '123456789' == 0x906E", crc == 0x906E);

    // Empty data
    uint16_t empty_crc = crc16_ccitt(nullptr, 0);
    check("CRC of empty == 0xFFFF ^ 0xFFFF == 0x0000",
          empty_crc == 0x0000);  // 0xFFFF ^ 0xFFFF
}

// --- Test NRZI ---
static void test_nrzi() {
    printf("\n=== NRZI Encode/Decode ===\n");
    std::vector<uint8_t> input = {0, 1, 0, 0, 1, 1, 0, 1};
    auto encoded = nrzi_encode(input);
    auto decoded = nrzi_decode(encoded);
    check("NRZI round-trip", decoded == input);
}

// --- Test HDLC ---
static void test_hdlc() {
    printf("\n=== HDLC Encode/Decode ===\n");

    // A simple test frame
    uint8_t frame[] = {0xAA, 0x55, 0x01, 0x02, 0x03, 0x04, 0x05};
    size_t frame_len = sizeof(frame);

    // Encode
    auto bits = hdlc_encode(frame, frame_len, 4, 2);
    check("HDLC encode produces bits", bits.size() > 0);

    // Decode: first NRZI-decode (hdlc_encode already applies NRZI)
    auto nrzi_bits = nrzi_decode(bits);

    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : nrzi_bits) {
        if (decoder.push_bit(b)) {
            found = true;
            break;
        }
    }

    check("HDLC decode found frame", found);
    if (found) {
        const auto& decoded = decoder.frame();
        bool match = (decoded.size() == frame_len) &&
                     (memcmp(decoded.data(), frame, frame_len) == 0);
        check("HDLC decoded frame matches original", match);
    }
}

// --- Test AFSK 1200 loopback ---
static void test_afsk_loopback() {
    printf("\n=== AFSK 1200 Loopback ===\n");

    // Build a small AX.25 frame
    uint8_t frame[] = {
        // Dest: CQ    (padded to 6, shifted left 1)
        'C'<<1, 'Q'<<1, ' '<<1, ' '<<1, ' '<<1, ' '<<1, 0x60,
        // Src: TEST
        'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x61,
        // Control + PID
        0x03, 0xF0,
        // Payload
        'H', 'e', 'l', 'l', 'o'
    };
    size_t frame_len = sizeof(frame);

    // HDLC encode (includes NRZI)
    auto bits = hdlc_encode(frame, frame_len, 8, 4);
    printf("  HDLC bits: %zu\n", bits.size());

    // AFSK modulate
    AfskModulator mod;
    auto samples = mod.modulate(bits);
    printf("  Audio samples: %zu (%.1f ms)\n", samples.size(),
           samples.size() * 1000.0 / SAMPLE_RATE);

    // AFSK demodulate
    AfskDemodulator demod;
    auto rx_nrzi = demod.demodulate(samples.data(), samples.size());
    printf("  Demodulated NRZI bits: %zu\n", rx_nrzi.size());

    // NRZI decode
    auto rx_bits = nrzi_decode(rx_nrzi);

    // HDLC decode
    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : rx_bits) {
        if (decoder.push_bit(b)) {
            found = true;
            break;
        }
    }

    check("AFSK loopback: frame decoded", found);
    if (found) {
        const auto& decoded = decoder.frame();
        bool match = (decoded.size() == frame_len) &&
                     (memcmp(decoded.data(), frame, frame_len) == 0);
        check("AFSK loopback: payload matches", match);
        if (!match) {
            printf("    Expected %zu bytes, got %zu bytes\n",
                   frame_len, decoded.size());
        }
    }
}

// --- Test GFSK 9600 loopback ---
static void test_gfsk_loopback() {
    printf("\n=== GFSK 9600 Loopback ===\n");

    uint8_t frame[] = {
        'T'<<1, 'E'<<1, 'S'<<1, 'T'<<1, ' '<<1, ' '<<1, 0x60,
        'I'<<1, 'R'<<1, 'I'<<1, 'S'<<1, ' '<<1, ' '<<1, 0x61,
        0x03, 0xF0,
        'G', 'F', 'S', 'K', ' ', 't', 'e', 's', 't'
    };
    size_t frame_len = sizeof(frame);

    auto bits = hdlc_encode(frame, frame_len, 8, 4);
    printf("  HDLC bits: %zu\n", bits.size());

    GfskModulator mod;
    auto samples = mod.modulate(bits);
    printf("  Audio samples: %zu (%.1f ms)\n", samples.size(),
           samples.size() * 1000.0 / SAMPLE_RATE);

    GfskDemodulator demod;
    auto rx_nrzi = demod.demodulate(samples.data(), samples.size());
    printf("  Demodulated NRZI bits: %zu\n", rx_nrzi.size());

    auto rx_bits = nrzi_decode(rx_nrzi);

    HdlcDecoder decoder;
    bool found = false;
    for (uint8_t b : rx_bits) {
        if (decoder.push_bit(b)) {
            found = true;
            break;
        }
    }

    check("GFSK loopback: frame decoded", found);
    if (found) {
        const auto& decoded = decoder.frame();
        bool match = (decoded.size() == frame_len) &&
                     (memcmp(decoded.data(), frame, frame_len) == 0);
        check("GFSK loopback: payload matches", match);
        if (!match) {
            printf("    Expected %zu bytes, got %zu bytes\n",
                   frame_len, decoded.size());
        }
    }
}

// --- Test KISS codec ---
static void test_kiss() {
    printf("\n=== KISS Codec ===\n");

    uint8_t frame[] = {0x01, 0xC0, 0xDB, 0x55, 0xAA};  // includes FEND and FESC bytes
    size_t frame_len = sizeof(frame);

    // Encode
    auto kiss_frame = KissCodec::encode(frame, frame_len);
    check("KISS encode produces output", kiss_frame.size() > 0);

    // Check escaping: FEND (0xC0) and FESC (0xDB) should be escaped
    check("KISS frame starts with FEND", kiss_frame.front() == KISS_FEND);
    check("KISS frame ends with FEND", kiss_frame.back() == KISS_FEND);

    // Decode
    KissCodec codec;
    bool got_frame = false;
    uint8_t rx_port = 0xFF;
    uint8_t rx_cmd = 0xFF;
    std::vector<uint8_t> rx_data;

    codec.set_callback([&](uint8_t port, uint8_t cmd, const uint8_t* data, size_t len) {
        got_frame = true;
        rx_port = port;
        rx_cmd = cmd;
        rx_data.assign(data, data + len);
    });

    codec.feed(kiss_frame.data(), kiss_frame.size());

    check("KISS decode: got frame", got_frame);
    check("KISS decode: port == 0", rx_port == 0);
    check("KISS decode: cmd == DATA", rx_cmd == KISS_CMD_DATA);
    check("KISS decode: payload matches",
          rx_data.size() == frame_len &&
          memcmp(rx_data.data(), frame, frame_len) == 0);
}

int main() {
    printf("Iris Modem — Loopback Tests\n");
    printf("============================\n");

    test_crc16();
    test_nrzi();
    test_hdlc();
    test_kiss();
    test_afsk_loopback();
    test_gfsk_loopback();

    printf("\n============================\n");
    printf("Results: %d passed, %d failed\n", tests_passed, tests_failed);
    return tests_failed > 0 ? 1 : 0;
}
