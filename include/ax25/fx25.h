#ifndef IRIS_FX25_H
#define IRIS_FX25_H

// FX.25 Forward Error Correction for AX.25
// Implements the FX.25 specification using Reed-Solomon codes over GF(2^8).
// RS codec based on Phil Karn KA9Q's GPL code.

#include <cstdint>
#include <cstddef>
#include <vector>

namespace iris {

// --- Constants ---

constexpr int FX25_BLOCK_SIZE = 255;    // RS block size (2^8 - 1)
constexpr int FX25_MAX_DATA   = 239;    // Max data bytes (RS(255,239))
constexpr int FX25_MAX_CHECK  = 64;     // Max check bytes (RS(255,191))
constexpr int FX25_CTAG_MIN   = 0x01;
constexpr int FX25_CTAG_MAX   = 0x0B;

// FX.25 mode: number of RS check bytes requested (0 = auto/off)
constexpr int FX25_MODE_16 = 16;
constexpr int FX25_MODE_32 = 32;
constexpr int FX25_MODE_64 = 64;

// Correlation tag match tolerance (max bit errors)
constexpr int FX25_CTAG_TOLERANCE = 8;

// --- Correlation Tag Table ---

struct Fx25Tag {
    uint64_t value;         // 64-bit correlation tag
    int k_data_radio;       // Data bytes sent over radio
    int k_data_rs;          // Data bytes for RS codec (= block_size - nroots)
    int nroots;             // Number of RS check bytes
    int rs_codec_id;        // Which RS codec (0=16-check, 1=32-check, 2=64-check)
};

// Returns tag info for ctag_num (1..11), or nullptr if invalid
const Fx25Tag* fx25_get_tag(int ctag_num);

// Returns the correlation tag value for ctag_num
uint64_t fx25_get_ctag_value(int ctag_num);

// Pick best correlation tag for given FX.25 mode and data length
// fx_mode: 16, 32, or 64 (number of check bytes)
// dlen: actual data length after HDLC bit-stuffing (bytes)
// Returns ctag_num (1..11) or -1 if no suitable format
int fx25_pick_tag(int fx_mode, int dlen);

// --- Reed-Solomon Codec ---

struct RsCodec {
    int mm;             // Symbol size (bits) = 8
    int nn;             // Block size = 255
    int nroots;         // Number of check symbols
    int kk;             // Data symbols = nn - nroots
    uint8_t* alpha_to;  // Log lookup table
    uint8_t* index_of;  // Antilog lookup table
    uint8_t* genpoly;   // Generator polynomial
    int fcr;            // First consecutive root
    int prim;           // Primitive element
    int iprim;          // prim-th root of 1
};

// Initialize the three RS codecs (call once at startup)
void fx25_init();

// Get RS codec for a given ctag_num
RsCodec* fx25_get_rs(int ctag_num);

// RS encode: data[0..kk-1] -> check[0..nroots-1]
void fx25_rs_encode(RsCodec* rs, const uint8_t* data, uint8_t* check);

// RS decode: block[0..254] (data + zero-fill + check) in-place
// Returns number of corrected errors, or -1 if uncorrectable
int fx25_rs_decode(RsCodec* rs, uint8_t* block);

// --- HDLC Bit-Stuffing to Byte Array (for FX.25 TX) ---

// Stuff an AX.25 frame (with FCS already appended) into a byte array
// suitable for RS encoding. Output: [0x7E][bit-stuffed data+FCS][0x7E][0x7E padding...]
// Returns number of bytes used (up to the trailing flag), or -1 if won't fit.
int fx25_hdlc_stuff(const uint8_t* frame_with_fcs, int flen,
                    uint8_t* out, int out_size);

// Unstuff HDLC from byte array (reverse of fx25_hdlc_stuff)
// Input: data portion of RS codeblock (starts with 0x7E flag)
// Output: frame bytes including FCS
// Returns frame length including FCS, or 0 on error
int fx25_hdlc_unstuff(const uint8_t* data, int dlen,
                      uint8_t* frame_buf, int frame_buf_size);

// --- FX.25 TX Encoder ---

// Encode an AX.25 frame as FX.25 NRZI bit stream ready for AFSK modulation.
// frame: raw AX.25 frame (addresses + control + PID + info, no FCS)
// len: frame length
// fx_mode: 16, 32, or 64 (RS check bytes)
// preamble_flags: number of 0x7E preamble flags
// Returns NRZI-encoded bits, or empty vector on failure.
std::vector<uint8_t> fx25_encode(const uint8_t* frame, size_t len,
                                  int fx_mode = FX25_MODE_16,
                                  int preamble_flags = 4);

// Encode FX.25 frame to raw bits (pre-NRZI). Appends to out_bits.
// Returns false if encoding fails (frame too large, etc.).
bool fx25_encode_raw(std::vector<uint8_t>& out_bits, const uint8_t* frame, size_t len,
                     int fx_mode = FX25_MODE_16, int preamble_flags = 4);

// --- FX.25 RX Decoder ---

// Stateful bit-by-bit FX.25 receiver.
// Feed NRZI-decoded data bits. When a complete FX.25 frame is decoded,
// frame() returns the extracted AX.25 frame (without FCS).
class Fx25Decoder {
public:
    Fx25Decoder();

    // Feed one data bit (already NRZI-decoded). Returns true if frame ready.
    bool push_bit(int bit);

    // Get last decoded frame (valid after push_bit returns true)
    const std::vector<uint8_t>& frame() const { return frame_; }

    // Number of RS errors corrected in last frame (-1 if none decoded yet)
    int last_rs_errors() const { return last_rs_errors_; }

    void reset();

private:
    enum State { FX_TAG, FX_DATA, FX_CHECK };

    State state_;
    uint64_t accum_;        // 64-bit shift register for correlation tag
    int ctag_num_;          // Matched correlation tag number
    int k_data_radio_;      // Expected data bytes
    int nroots_;            // Expected check bytes
    int coffs_;             // Check byte offset in block (= kk = 255 - nroots)
    int dlen_;              // Accumulated data bytes
    int clen_;              // Accumulated check bytes
    uint8_t imask_;         // Current bit mask within byte
    uint8_t block_[FX25_BLOCK_SIZE + 1]; // RS codeblock + fence byte

    std::vector<uint8_t> frame_;    // Last decoded AX.25 frame
    int last_rs_errors_;
};

// Find correlation tag match in 64-bit accumulator.
// Returns ctag_num (1..11) if match within tolerance, or -1.
int fx25_tag_match(uint64_t accum);

} // namespace iris

#endif // IRIS_FX25_H
