#include "ax25/fx25.h"
#include "ax25/crc16.h"
#include "ax25/hdlc.h"
#include "common/logging.h"
#include <cstring>
#include <cassert>
#include <algorithm>

namespace iris {

// ============================================================================
// Correlation Tag Table (from FX.25 specification)
// ============================================================================

// Three RS codecs: 16-check, 32-check, 64-check
// All use GF(2^8) with generator polynomial 0x11d
static const Fx25Tag tag_table[] = {
    // ctag 0: reserved (not used)
    { 0x566ED2717946107EULL,   0,   0,  0, -1 },
    // ctag 1: RS(255,239) 16 check bytes, full size
    { 0xB74DB7DF8A532F3EULL, 255, 239, 16,  0 },
    // ctag 2: RS(144,128) shortened from RS(255,239)
    { 0x26FF60A600CC8FDEULL, 144, 239, 16,  0 },
    // ctag 3: RS(80,64) shortened from RS(255,239)
    { 0xC7DC0508F3D9B09EULL,  80, 239, 16,  0 },
    // ctag 4: RS(48,32) shortened from RS(255,239)
    { 0x8F056EB4369660EEULL,  48, 239, 16,  0 },
    // ctag 5: RS(255,223) 32 check bytes, full size
    { 0x6E260B1AC5835FAEULL, 255, 223, 32,  1 },
    // ctag 6: RS(160,128) shortened from RS(255,223)
    { 0xFF94DC634F1CFF4EULL, 160, 223, 32,  1 },
    // ctag 7: RS(96,64) shortened from RS(255,223)
    { 0x1EB7B9CDBC09C00EULL,  96, 223, 32,  1 },
    // ctag 8: RS(64,32) shortened from RS(255,223)
    { 0xDBF869BD2DBB1776ULL,  64, 223, 32,  1 },
    // ctag 9: RS(255,191) 64 check bytes, full size
    { 0x3ADB0C13DEAE2836ULL, 255, 191, 64,  2 },
    // ctag 10: RS(192,128) shortened from RS(255,191)
    { 0xAB69DB6A543188D6ULL, 192, 191, 64,  2 },
    // ctag 11: RS(128,64) shortened from RS(255,191)
    { 0x4A4ABEC4A724B796ULL, 128, 191, 64,  2 },
};

const Fx25Tag* fx25_get_tag(int ctag_num) {
    if (ctag_num < FX25_CTAG_MIN || ctag_num > FX25_CTAG_MAX) return nullptr;
    return &tag_table[ctag_num];
}

uint64_t fx25_get_ctag_value(int ctag_num) {
    if (ctag_num < FX25_CTAG_MIN || ctag_num > FX25_CTAG_MAX) return 0;
    return tag_table[ctag_num].value;
}

// ============================================================================
// Reed-Solomon Codec (GF(2^8), polynomial 0x11d)
// Based on Phil Karn KA9Q's GPL code
// ============================================================================

static RsCodec rs_codecs[3];  // 16-check, 32-check, 64-check
static bool rs_initialized = false;

static inline int modnn(const RsCodec* rs, int x) {
    while (x >= rs->nn) {
        x -= rs->nn;
        x = (x >> rs->mm) + (x & rs->nn);
    }
    return x;
}

static RsCodec* init_rs_codec(int symsize, int gfpoly, int fcr, int prim, int nroots) {
    RsCodec* rs = new RsCodec();
    rs->mm = symsize;
    rs->nn = (1 << symsize) - 1;
    rs->nroots = nroots;
    rs->kk = rs->nn - nroots;
    rs->fcr = fcr;
    rs->prim = prim;

    rs->alpha_to = new uint8_t[rs->nn + 1];
    rs->index_of = new uint8_t[rs->nn + 1];
    rs->genpoly = new uint8_t[nroots + 1];

    // Generate Galois field lookup tables
    int sr = 1;
    rs->index_of[0] = rs->nn;  // log(0) = -inf, represented as nn (A0)
    rs->alpha_to[rs->nn] = 0;  // alpha^(-inf) = 0
    for (int i = 0; i < rs->nn; i++) {
        rs->index_of[sr] = i;
        rs->alpha_to[i] = sr;
        sr <<= 1;
        if (sr & (1 << symsize))
            sr ^= gfpoly;
        sr &= rs->nn;
    }

    // Find prim-th root of 1 (iprim)
    for (int i = 1; i < rs->nn; i++) {
        if (modnn(rs, prim * i) == 1) {
            rs->iprim = i;
            break;
        }
    }

    // Generate RS generator polynomial
    rs->genpoly[0] = 1;
    for (int i = 0; i < nroots; i++) {
        rs->genpoly[i + 1] = 1;
        for (int j = i; j > 0; j--) {
            if (rs->genpoly[j] != 0) {
                rs->genpoly[j] = rs->genpoly[j - 1] ^
                    rs->alpha_to[modnn(rs, rs->index_of[rs->genpoly[j]] + fcr + i)];
            } else {
                rs->genpoly[j] = rs->genpoly[j - 1];
            }
        }
        rs->genpoly[0] = rs->alpha_to[modnn(rs, rs->index_of[rs->genpoly[0]] + fcr + i)];
    }

    // Convert generator poly to index form
    for (int i = 0; i <= nroots; i++)
        rs->genpoly[i] = rs->index_of[rs->genpoly[i]];

    return rs;
}

void fx25_init() {
    if (rs_initialized) return;

    // GF(2^8), primitive polynomial x^8+x^4+x^3+x^2+1 = 0x11d
    // FCR=1, PRIM=1 (standard for FX.25)
    static const int nroots_table[3] = { 16, 32, 64 };

    for (int i = 0; i < 3; i++) {
        RsCodec* codec = init_rs_codec(8, 0x11d, 1, 1, nroots_table[i]);
        rs_codecs[i] = *codec;
        delete codec;
    }
    rs_initialized = true;
}

RsCodec* fx25_get_rs(int ctag_num) {
    if (!rs_initialized) fx25_init();
    const Fx25Tag* tag = fx25_get_tag(ctag_num);
    if (!tag || tag->rs_codec_id < 0) return nullptr;
    return &rs_codecs[tag->rs_codec_id];
}

// RS systematic encoder
void fx25_rs_encode(RsCodec* rs, const uint8_t* data, uint8_t* bb) {
    int nn = rs->nn;
    int nroots = rs->nroots;
    int kk = nn - nroots;
    uint8_t* alpha_to = rs->alpha_to;
    uint8_t* index_of = rs->index_of;
    uint8_t* genpoly = rs->genpoly;

    memset(bb, 0, nroots);

    for (int i = 0; i < kk; i++) {
        uint8_t feedback = index_of[data[i] ^ bb[0]];
        if (feedback != (uint8_t)nn) {  // nn = A0 = log(0)
            for (int j = 1; j < nroots; j++)
                bb[j] ^= alpha_to[modnn(rs, feedback + genpoly[nroots - j])];
        }
        memmove(&bb[0], &bb[1], nroots - 1);
        if (feedback != (uint8_t)nn)
            bb[nroots - 1] = alpha_to[modnn(rs, feedback + genpoly[0])];
        else
            bb[nroots - 1] = 0;
    }
}

// RS decoder (Berlekamp-Massey + Chien search + Forney)
int fx25_rs_decode(RsCodec* rs, uint8_t* data) {
    int nn = rs->nn;
    int nroots = rs->nroots;
    int fcr = rs->fcr;
    int prim = rs->prim;
    int iprim = rs->iprim;
    uint8_t* alpha_to = rs->alpha_to;
    uint8_t* index_of = rs->index_of;

    uint8_t s[FX25_MAX_CHECK];
    uint8_t lambda[FX25_MAX_CHECK + 1];
    uint8_t b[FX25_MAX_CHECK + 1];
    uint8_t t[FX25_MAX_CHECK + 1];
    uint8_t omega[FX25_MAX_CHECK + 1];
    uint8_t root[FX25_MAX_CHECK];
    uint8_t reg[FX25_MAX_CHECK + 1];
    int loc[FX25_MAX_CHECK];
    int count;

    // Compute syndromes
    for (int i = 0; i < nroots; i++)
        s[i] = data[0];

    for (int j = 1; j < nn; j++) {
        for (int i = 0; i < nroots; i++) {
            if (s[i] == 0)
                s[i] = data[j];
            else
                s[i] = data[j] ^ alpha_to[modnn(rs, index_of[s[i]] + (fcr + i) * prim)];
        }
    }

    // Check for zero syndrome (no errors)
    int syn_error = 0;
    for (int i = 0; i < nroots; i++) {
        syn_error |= s[i];
        s[i] = index_of[s[i]];
    }
    if (!syn_error) return 0;

    // Initialize lambda
    memset(&lambda[1], 0, nroots);
    lambda[0] = 1;

    for (int i = 0; i < nroots + 1; i++)
        b[i] = index_of[lambda[i]];

    // Berlekamp-Massey algorithm
    int r = 0, el = 0;
    while (++r <= nroots) {
        uint8_t discr_r = 0;
        for (int i = 0; i < r; i++) {
            if ((lambda[i] != 0) && (s[r - i - 1] != (uint8_t)nn))
                discr_r ^= alpha_to[modnn(rs, index_of[lambda[i]] + s[r - i - 1])];
        }
        discr_r = index_of[discr_r];

        if (discr_r == (uint8_t)nn) {
            memmove(&b[1], b, nroots);
            b[0] = nn;
        } else {
            t[0] = lambda[0];
            for (int i = 0; i < nroots; i++) {
                if (b[i] != (uint8_t)nn)
                    t[i + 1] = lambda[i + 1] ^ alpha_to[modnn(rs, discr_r + b[i])];
                else
                    t[i + 1] = lambda[i + 1];
            }
            if (2 * el <= r - 1) {
                el = r - el;
                for (int i = 0; i <= nroots; i++)
                    b[i] = (lambda[i] == 0) ? (uint8_t)nn :
                            (uint8_t)modnn(rs, index_of[lambda[i]] - discr_r + nn);
            } else {
                memmove(&b[1], b, nroots);
                b[0] = nn;
            }
            memcpy(lambda, t, (nroots + 1));
        }
    }

    // Convert lambda to index form and find degree
    int deg_lambda = 0;
    for (int i = 0; i < nroots + 1; i++) {
        lambda[i] = index_of[lambda[i]];
        if (lambda[i] != (uint8_t)nn) deg_lambda = i;
    }

    // Chien search for roots
    memcpy(&reg[1], &lambda[1], nroots);
    count = 0;
    for (int i = 1, k = iprim - 1; i <= nn; i++, k = modnn(rs, k + iprim)) {
        uint8_t q = 1;
        for (int j = deg_lambda; j > 0; j--) {
            if (reg[j] != (uint8_t)nn) {
                reg[j] = (uint8_t)modnn(rs, reg[j] + j);
                q ^= alpha_to[reg[j]];
            }
        }
        if (q != 0) continue;
        root[count] = i;
        loc[count] = k;
        if (++count == deg_lambda) break;
    }

    if (deg_lambda != count) return -1;  // Uncorrectable

    // Compute omega(x)
    int deg_omega = 0;
    for (int i = 0; i < nroots; i++) {
        uint8_t tmp = 0;
        int jmax = (deg_lambda < i) ? deg_lambda : i;
        for (int j = jmax; j >= 0; j--) {
            if ((s[i - j] != (uint8_t)nn) && (lambda[j] != (uint8_t)nn))
                tmp ^= alpha_to[modnn(rs, s[i - j] + lambda[j])];
        }
        if (tmp != 0) deg_omega = i;
        omega[i] = index_of[tmp];
    }
    omega[nroots] = nn;

    // Apply error corrections (Forney algorithm)
    for (int j = count - 1; j >= 0; j--) {
        uint8_t num1 = 0;
        for (int i = deg_omega; i >= 0; i--) {
            if (omega[i] != (uint8_t)nn)
                num1 ^= alpha_to[modnn(rs, omega[i] + i * root[j])];
        }
        uint8_t num2 = alpha_to[modnn(rs, root[j] * (fcr - 1) + nn)];
        uint8_t den = 0;
        // lambda[i+1] for i even is formal derivative
        int jlim = std::min(deg_lambda, nroots - 1) & ~1;
        for (int i = jlim; i >= 0; i -= 2) {
            if (lambda[i + 1] != (uint8_t)nn)
                den ^= alpha_to[modnn(rs, lambda[i + 1] + i * root[j])];
        }
        if (den == 0) return -1;

        if (num1 != 0) {
            data[loc[j]] ^= alpha_to[modnn(rs,
                index_of[num1] + index_of[num2] + nn - index_of[den])];
        }
    }

    return count;
}

// ============================================================================
// Correlation Tag Matching
// ============================================================================

static int popcount64(uint64_t x) {
#if defined(__GNUC__) || defined(__clang__)
    return __builtin_popcountll(x);
#else
    // Portable popcount
    x = x - ((x >> 1) & 0x5555555555555555ULL);
    x = (x & 0x3333333333333333ULL) + ((x >> 2) & 0x3333333333333333ULL);
    x = (x + (x >> 4)) & 0x0F0F0F0F0F0F0F0FULL;
    return (int)((x * 0x0101010101010101ULL) >> 56);
#endif
}

int fx25_tag_match(uint64_t accum) {
    int best_tag = -1;
    int best_errors = FX25_CTAG_TOLERANCE + 1;

    for (int i = FX25_CTAG_MIN; i <= FX25_CTAG_MAX; i++) {
        int errors = popcount64(accum ^ tag_table[i].value);
        if (errors < best_errors) {
            best_errors = errors;
            best_tag = i;
        }
    }

    return (best_errors <= FX25_CTAG_TOLERANCE) ? best_tag : -1;
}

// ============================================================================
// Tag Selection for TX
// ============================================================================

int fx25_pick_tag(int fx_mode, int dlen) {
    // Order of preference: smallest format that fits
    // For each check-byte count, try from smallest to largest
    static const int order_16[] = { 4, 3, 2, 1, -1 };  // 32, 64, 128, 239 data bytes
    static const int order_32[] = { 8, 7, 6, 5, -1 };  // 32, 64, 128, 223 data bytes
    static const int order_64[] = { 11, 10, 9, -1 };    // 64, 128, 191 data bytes

    const int* order;
    switch (fx_mode) {
        case 16: order = order_16; break;
        case 32: order = order_32; break;
        case 64: order = order_64; break;
        default: return -1;
    }

    for (int i = 0; order[i] >= 0; i++) {
        const Fx25Tag& tag = tag_table[order[i]];
        // k_data_radio minus nroots gives actual data capacity
        // But k_data_radio already IS the data capacity (nroots are separate)
        if (dlen <= tag.k_data_radio - tag.nroots) {
            return order[i];
        }
    }
    return -1;
}

// ============================================================================
// HDLC Bit-Stuffing to Byte Array (for RS encoding)
// ============================================================================

// Macro to put one bit into byte-packed output
#define PUT_BIT(value) do { \
    if (olen >= osize_bits) return -1; \
    if (value) out[olen >> 3] |= (1 << (olen & 7)); \
    olen++; \
} while(0)

int fx25_hdlc_stuff(const uint8_t* frame_fcs, int flen,
                    uint8_t* out, int out_size) {
    const uint8_t flag = 0x7E;
    memset(out, 0, out_size);

    // Start with flag
    out[0] = flag;
    int olen = 8;  // bits
    int osize_bits = out_size * 8;
    int ones = 0;

    // Bit-stuff the frame data (including FCS)
    for (int i = 0; i < flen; i++) {
        for (uint8_t imask = 1; imask != 0; imask <<= 1) {
            int v = (frame_fcs[i] & imask) ? 1 : 0;
            PUT_BIT(v);
            if (v) {
                ones++;
                if (ones == 5) {
                    PUT_BIT(0);  // stuff bit
                    ones = 0;
                }
            } else {
                ones = 0;
            }
        }
    }

    // Trailing flag
    for (uint8_t imask = 1; imask != 0; imask <<= 1) {
        PUT_BIT(flag & imask);
    }

    int ret = (olen + 7) / 8;  // Bytes used including any partial byte

    // Fill remainder with flag patterns (may not be byte-aligned)
    uint8_t fmask = 1;
    while (olen < osize_bits) {
        PUT_BIT(flag & fmask);
        fmask = (fmask << 1) | (fmask >> 7);  // rotate
    }

    return ret;
}

#undef PUT_BIT

int fx25_hdlc_unstuff(const uint8_t* data, int dlen,
                      uint8_t* frame_buf, int frame_buf_size) {
    if (dlen < 1 || data[0] != 0x7E) return 0;

    // Skip leading flag bytes
    const uint8_t* pin = data;
    int ilen = dlen;
    while (ilen > 0 && *pin == 0x7E) {
        ilen--;
        pin++;
    }

    uint8_t pat_det = 0;
    uint8_t oacc = 0;
    int obit = 0;
    int frame_len = 0;

    for (int i = 0; i < ilen; i++) {
        for (uint8_t imask = 0x01; imask != 0; imask <<= 1) {
            uint8_t dbit = (pin[i] & imask) ? 1 : 0;

            pat_det >>= 1;
            pat_det |= (dbit << 7);

            if (pat_det == 0xFE) return 0;  // Seven 1s = invalid

            if (dbit) {
                oacc >>= 1;
                oacc |= 0x80;
            } else {
                if (pat_det == 0x7E) {
                    // End flag
                    if (obit == 7)
                        return frame_len;
                    else
                        return 0;  // Not whole bytes
                } else if ((pat_det >> 2) == 0x1F) {
                    continue;  // Stuffed bit — discard
                }
                oacc >>= 1;
            }

            obit++;
            if (obit & 8) {
                obit = 0;
                if (frame_len >= frame_buf_size) return 0;
                frame_buf[frame_len++] = oacc;
            }
        }
    }

    return 0;  // No terminating flag found
}

// ============================================================================
// FX.25 TX Encoder
// ============================================================================

std::vector<uint8_t> fx25_encode(const uint8_t* frame, size_t len,
                                  int fx_mode, int preamble_flags) {
    if (!rs_initialized) fx25_init();

    // Append FCS to frame
    uint8_t fbuf[FX25_MAX_DATA + 2];
    if (len + 2 > sizeof(fbuf)) return {};
    memcpy(fbuf, frame, len);
    uint16_t fcs = crc16_ccitt(frame, len);
    fbuf[len] = fcs & 0xFF;
    fbuf[len + 1] = (fcs >> 8) & 0xFF;
    int flen = (int)len + 2;

    // HDLC bit-stuff into byte array
    uint8_t data[FX25_MAX_DATA + 1];
    const uint8_t fence = 0xAA;
    data[FX25_MAX_DATA] = fence;
    int dlen = fx25_hdlc_stuff(fbuf, flen, data, FX25_MAX_DATA);
    assert(data[FX25_MAX_DATA] == fence);
    if (dlen < 0) {
        IRIS_LOG("FX.25 TX: frame too large (%d bytes)", flen);
        return {};
    }

    // Pick correlation tag
    int ctag_num = fx25_pick_tag(fx_mode, dlen);
    if (ctag_num < FX25_CTAG_MIN || ctag_num > FX25_CTAG_MAX) {
        IRIS_LOG("FX.25 TX: no suitable tag for mode=%d dlen=%d", fx_mode, dlen);
        return {};
    }

    const Fx25Tag* tag = fx25_get_tag(ctag_num);
    uint64_t ctag_value = tag->value;
    int k_data_radio = tag->k_data_radio - tag->nroots;
    int nroots = tag->nroots;

    // Zero-fill data beyond what we used (for shortened codes)
    int shorten_by = FX25_MAX_DATA - k_data_radio;
    if (shorten_by > 0)
        memset(data + k_data_radio, 0, shorten_by);

    // RS encode
    uint8_t check[FX25_MAX_CHECK + 1];
    check[FX25_MAX_CHECK] = fence;
    RsCodec* rs = fx25_get_rs(ctag_num);
    fx25_rs_encode(rs, data, check);
    assert(check[FX25_MAX_CHECK] == fence);

    IRIS_LOG("FX.25 TX: ctag=0x%02X, data=%d, check=%d, frame=%zu",
             ctag_num, k_data_radio, nroots, len);

    // Build raw bit stream (pre-NRZI)
    int total_bits = preamble_flags * 8 + 64 + k_data_radio * 8 + nroots * 8;
    std::vector<uint8_t> bits;
    bits.reserve(total_bits);

    auto emit_byte = [&bits](uint8_t byte) {
        for (int i = 0; i < 8; i++)
            bits.push_back((byte >> i) & 1);
    };

    for (int i = 0; i < preamble_flags; i++)
        emit_byte(0x7E);

    for (int k = 0; k < 8; k++)
        emit_byte((ctag_value >> (k * 8)) & 0xFF);

    for (int i = 0; i < k_data_radio; i++)
        emit_byte(data[i]);

    for (int i = 0; i < nroots; i++)
        emit_byte(check[i]);

    return nrzi_encode(bits);
}

bool fx25_encode_raw(std::vector<uint8_t>& out_bits, const uint8_t* frame, size_t len,
                     int fx_mode, int preamble_flags) {
    // Same as fx25_encode but appends raw (pre-NRZI) bits to out_bits.
    if (!rs_initialized) fx25_init();

    uint8_t fbuf[FX25_MAX_DATA + 2];
    if (len + 2 > sizeof(fbuf)) return false;
    memcpy(fbuf, frame, len);
    uint16_t fcs = crc16_ccitt(frame, len);
    fbuf[len] = fcs & 0xFF;
    fbuf[len + 1] = (fcs >> 8) & 0xFF;
    int flen = (int)len + 2;

    uint8_t data[FX25_MAX_DATA + 1];
    const uint8_t fence = 0xAA;
    data[FX25_MAX_DATA] = fence;
    int dlen = fx25_hdlc_stuff(fbuf, flen, data, FX25_MAX_DATA);
    assert(data[FX25_MAX_DATA] == fence);
    if (dlen < 0) return false;

    int ctag_num = fx25_pick_tag(fx_mode, dlen);
    if (ctag_num < FX25_CTAG_MIN || ctag_num > FX25_CTAG_MAX) return false;

    const Fx25Tag* tag = fx25_get_tag(ctag_num);
    uint64_t ctag_value = tag->value;
    int k_data_radio = tag->k_data_radio - tag->nroots;
    int nroots = tag->nroots;

    int shorten_by = FX25_MAX_DATA - k_data_radio;
    if (shorten_by > 0)
        memset(data + k_data_radio, 0, shorten_by);

    uint8_t check[FX25_MAX_CHECK + 1];
    check[FX25_MAX_CHECK] = fence;
    RsCodec* rs = fx25_get_rs(ctag_num);
    fx25_rs_encode(rs, data, check);
    assert(check[FX25_MAX_CHECK] == fence);

    auto emit_byte = [&out_bits](uint8_t byte) {
        for (int i = 0; i < 8; i++)
            out_bits.push_back((byte >> i) & 1);
    };

    for (int i = 0; i < preamble_flags; i++)
        emit_byte(0x7E);

    for (int k = 0; k < 8; k++)
        emit_byte((ctag_value >> (k * 8)) & 0xFF);

    for (int i = 0; i < k_data_radio; i++)
        emit_byte(data[i]);

    for (int i = 0; i < nroots; i++)
        emit_byte(check[i]);

    return true;
}

// ============================================================================
// FX.25 RX Decoder
// ============================================================================

Fx25Decoder::Fx25Decoder() { reset(); }

void Fx25Decoder::reset() {
    state_ = FX_TAG;
    accum_ = 0;
    ctag_num_ = -1;
    k_data_radio_ = 0;
    nroots_ = 0;
    coffs_ = 0;
    dlen_ = 0;
    clen_ = 0;
    imask_ = 0x01;
    memset(block_, 0, sizeof(block_));
    frame_.clear();
    last_rs_errors_ = -1;
}

bool Fx25Decoder::push_bit(int bit) {
    if (!rs_initialized) fx25_init();

    switch (state_) {
    case FX_TAG:
        accum_ >>= 1;
        if (bit) accum_ |= 1ULL << 63;
        {
            int c = fx25_tag_match(accum_);
            if (c >= FX25_CTAG_MIN && c <= FX25_CTAG_MAX) {
                const Fx25Tag* tag = fx25_get_tag(c);
                ctag_num_ = c;
                k_data_radio_ = tag->k_data_radio - tag->nroots;
                nroots_ = tag->nroots;
                coffs_ = tag->k_data_rs;  // = 255 - nroots
                // Bounds check: reject corrupted tag that would overflow block_[]
                if (k_data_radio_ <= 0 || k_data_radio_ > FX25_BLOCK_SIZE ||
                    coffs_ + nroots_ > FX25_BLOCK_SIZE) {
                    break;
                }
                imask_ = 0x01;
                dlen_ = 0;
                clen_ = 0;
                memset(block_, 0, FX25_BLOCK_SIZE + 1);
                block_[FX25_BLOCK_SIZE] = 0x55;  // fence
                state_ = FX_DATA;
            }
        }
        break;

    case FX_DATA:
        if (dlen_ >= FX25_BLOCK_SIZE) { state_ = FX_TAG; accum_ = 0; break; }
        if (bit) block_[dlen_] |= imask_;
        imask_ <<= 1;
        if (imask_ == 0) {
            imask_ = 0x01;
            dlen_++;
            if (dlen_ >= k_data_radio_)
                state_ = FX_CHECK;
        }
        break;

    case FX_CHECK:
        if (bit) block_[coffs_ + clen_] |= imask_;
        imask_ <<= 1;
        if (imask_ == 0) {
            imask_ = 0x01;
            clen_++;
            if (clen_ >= nroots_) {
                // Complete RS block received — decode
                assert(block_[FX25_BLOCK_SIZE] == 0x55);

                RsCodec* rs = fx25_get_rs(ctag_num_);
                int derrors = fx25_rs_decode(rs, block_);

                if (derrors >= 0) {
                    // RS decode succeeded — extract HDLC frame
                    uint8_t frame_buf[FX25_MAX_DATA + 1];
                    int frame_len = fx25_hdlc_unstuff(block_, k_data_radio_,
                                                       frame_buf, FX25_MAX_DATA);

                    // Minimum: 14 (two addresses) + 1 (control) + 2 (FCS) = 17
                    // Also validate frame_len fits in buffer
                    if (frame_len >= 17 && frame_len <= FX25_MAX_DATA) {
                        // Verify FCS
                        uint16_t rx_fcs = frame_buf[frame_len - 2] |
                                          (frame_buf[frame_len - 1] << 8);
                        uint16_t calc_fcs = crc16_ccitt(frame_buf, frame_len - 2);

                        if (rx_fcs == calc_fcs) {
                            // Strip FCS, deliver frame
                            frame_.assign(frame_buf, frame_buf + frame_len - 2);
                            last_rs_errors_ = derrors;

                            IRIS_LOG("FX.25 RX: decoded frame %d bytes, %d RS corrections",
                                     (int)frame_.size(), derrors);

                            // Reset state
                            ctag_num_ = -1;
                            accum_ = 0;
                            state_ = FX_TAG;
                            return true;
                        }
                    }
                }

                if (derrors < 0) {
                    IRIS_LOG("FX.25 RX: RS decode failed (uncorrectable)");
                } else {
                    IRIS_LOG("FX.25 RX: RS OK but HDLC/FCS check failed");
                }

                // Reset to tag search
                ctag_num_ = -1;
                accum_ = 0;
                state_ = FX_TAG;
            }
        }
        break;
    }

    return false;
}

} // namespace iris
