/*
 * Iris adaptive dual-algorithm compression (PPMd8 + zstd).
 * Ported from Mercury's cl_compressor.
 *
 * TX: entropy test -> try PPMd/zstd -> pick smallest (including raw).
 * RX: parse header -> decompress with indicated algorithm.
 *
 * Streaming mode: PPMd model carries across batches (skip Ppmd8_Init on
 * warm model), zstd uses ZSTD_CCtx_refPrefix/ZSTD_DCtx_refPrefix with
 * a 32KB sliding window of previous raw data.
 */

#include "compress/compress.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

// PPMd8
extern "C" {
#include "Ppmd8.h"
}

// zstd
#define ZSTD_STATIC_LINKING_ONLY
extern "C" {
#include "zstd.h"
}

namespace iris {

// PPMd model order and memory
static constexpr int PPMD_ORDER = 6;
static constexpr int PPMD_MEM_SIZE = (1 << 21);  // 2 MB

// CRC16-MODBUS for streaming desync detection
static uint16_t compress_crc16(const uint8_t* data, int len) {
    uint16_t crc = 0xFFFF;
    for (int j = 0; j < len; j++) {
        crc ^= data[j];
        for (int i = 0; i < 8; i++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1;
    }
    return crc;
}

// PPMd allocator
static void* SzAlloc(ISzAllocPtr, size_t size) { return malloc(size); }
static void SzFree(ISzAllocPtr, void* address) { free(address); }
static const ISzAlloc g_Alloc = { SzAlloc, SzFree };

// PPMd byte-stream adapters
struct CByteOutBuf {
    IByteOut vt;
    uint8_t* buf;
    int capacity;
    int pos;
    int overflow;
};

static void ByteOutBuf_Write(const IByteOut* pp, Byte b) {
    CByteOutBuf* p = (CByteOutBuf*)(void*)pp;
    if (p->pos < p->capacity)
        p->buf[p->pos++] = b;
    else
        p->overflow = 1;
}

struct CByteInBuf {
    IByteIn vt;
    const uint8_t* buf;
    int size;
    int pos;
};

static Byte ByteInBuf_Read(const IByteIn* pp) {
    CByteInBuf* p = (CByteInBuf*)(void*)pp;
    if (p->pos < p->size)
        return p->buf[p->pos++];
    return 0;
}

// ---- Compressor ----

Compressor::Compressor()
    : ppmd_ctx_(nullptr), zstd_cctx_(nullptr), zstd_dctx_(nullptr),
      workspace_(nullptr), workspace_size_(0), initialized_(false),
      streaming_active_(false), stream_batch_count_(0), ppmd_model_warm_(false),
      zstd_prefix_(nullptr), zstd_prefix_len_(0),
      pending_raw_(nullptr), pending_raw_len_(0), pending_raw_capacity_(0) {}

Compressor::~Compressor() { deinit(); }

void Compressor::init() {
    if (initialized_) return;

    CPpmd8* p = (CPpmd8*)malloc(sizeof(CPpmd8));
    if (!p) return;
    Ppmd8_Construct(p);
    if (!Ppmd8_Alloc(p, PPMD_MEM_SIZE, &g_Alloc)) {
        free(p);
        return;
    }
    ppmd_ctx_ = p;

    zstd_cctx_ = ZSTD_createCCtx();
    zstd_dctx_ = ZSTD_createDCtx();
    if (!zstd_cctx_ || !zstd_dctx_) { deinit(); return; }

    ZSTD_CCtx_setParameter((ZSTD_CCtx*)zstd_cctx_, ZSTD_c_compressionLevel, 3);

    workspace_size_ = COMPRESS_WORKSPACE_SIZE;
    workspace_ = (uint8_t*)malloc(workspace_size_);
    if (!workspace_) { deinit(); return; }

    initialized_ = true;
    printf("[COMPRESS] Initialized: PPMd8 (order %d, %d KB) + zstd (level 3)\n",
           PPMD_ORDER, PPMD_MEM_SIZE / 1024);
    fflush(stdout);
}

void Compressor::deinit() {
    streaming_disable();
    if (ppmd_ctx_) {
        Ppmd8_Free((CPpmd8*)ppmd_ctx_, &g_Alloc);
        free(ppmd_ctx_);
        ppmd_ctx_ = nullptr;
    }
    if (zstd_cctx_) { ZSTD_freeCCtx((ZSTD_CCtx*)zstd_cctx_); zstd_cctx_ = nullptr; }
    if (zstd_dctx_) { ZSTD_freeDCtx((ZSTD_DCtx*)zstd_dctx_); zstd_dctx_ = nullptr; }
    if (workspace_) { free(workspace_); workspace_ = nullptr; workspace_size_ = 0; }
    initialized_ = false;
}

// ---- Streaming context management ----

void Compressor::streaming_enable() {
    if (!initialized_ || streaming_active_) return;
    zstd_prefix_ = (uint8_t*)malloc(ZSTD_PREFIX_CAPACITY);
    zstd_prefix_len_ = 0;
    pending_raw_capacity_ = COMPRESS_WORKSPACE_SIZE;
    pending_raw_ = (uint8_t*)malloc(pending_raw_capacity_);
    pending_raw_len_ = 0;
    stream_batch_count_ = 0;
    ppmd_model_warm_ = false;
    streaming_active_ = true;
}

void Compressor::streaming_disable() {
    if (!streaming_active_) return;
    streaming_active_ = false;
    if (ppmd_ctx_)
        Ppmd8_Init((CPpmd8*)ppmd_ctx_, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
    ppmd_model_warm_ = false;
    stream_batch_count_ = 0;
    free(zstd_prefix_); zstd_prefix_ = nullptr; zstd_prefix_len_ = 0;
    free(pending_raw_); pending_raw_ = nullptr; pending_raw_len_ = 0; pending_raw_capacity_ = 0;
}

void Compressor::streaming_reset() {
    if (!streaming_active_) return;
    if (ppmd_ctx_)
        Ppmd8_Init((CPpmd8*)ppmd_ctx_, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
    ppmd_model_warm_ = false;
    zstd_prefix_len_ = 0;
    pending_raw_len_ = 0;
    stream_batch_count_ = 0;
}

void Compressor::set_pending_raw(const uint8_t* data, int len) {
    if (!streaming_active_ || !pending_raw_ || len <= 0) return;
    if (len > pending_raw_capacity_) len = pending_raw_capacity_;
    memcpy(pending_raw_, data, len);
    pending_raw_len_ = len;
}

void Compressor::commit_pending() {
    if (!streaming_active_ || pending_raw_len_ <= 0) return;
    streaming_commit(pending_raw_, pending_raw_len_);
    pending_raw_len_ = 0;
}

void Compressor::clear_pending() { pending_raw_len_ = 0; }

void Compressor::streaming_commit(const uint8_t* raw_data, int raw_len) {
    if (!streaming_active_ || !zstd_prefix_ || raw_len <= 0) return;

    if (zstd_prefix_len_ + raw_len <= ZSTD_PREFIX_CAPACITY) {
        memcpy(zstd_prefix_ + zstd_prefix_len_, raw_data, raw_len);
        zstd_prefix_len_ += raw_len;
    } else {
        int total_needed = zstd_prefix_len_ + raw_len;
        int drop = total_needed - ZSTD_PREFIX_CAPACITY;
        if (drop >= zstd_prefix_len_) {
            int offset = raw_len - ZSTD_PREFIX_CAPACITY;
            if (offset < 0) offset = 0;
            memcpy(zstd_prefix_, raw_data + offset, raw_len - offset);
            zstd_prefix_len_ = raw_len - offset;
        } else {
            memmove(zstd_prefix_, zstd_prefix_ + drop, zstd_prefix_len_ - drop);
            zstd_prefix_len_ -= drop;
            memcpy(zstd_prefix_ + zstd_prefix_len_, raw_data, raw_len);
            zstd_prefix_len_ += raw_len;
        }
    }
    ppmd_model_warm_ = true;
    stream_batch_count_++;
}

// ---- Shannon entropy ----

float Compressor::quick_entropy(const uint8_t* data, int len) {
    if (len <= 0) return 8.0f;
    int freq[256] = {0};
    for (int i = 0; i < len; i++) freq[data[i]]++;
    float entropy = 0.0f;
    float inv_len = 1.0f / (float)len;
    for (int i = 0; i < 256; i++) {
        if (freq[i] == 0) continue;
        float p = (float)freq[i] * inv_len;
        entropy -= p * log2f(p);
    }
    return entropy;
}

// ---- PPMd compress/decompress ----

int Compressor::ppmd_compress(const uint8_t* in, int in_len, uint8_t* out, int out_cap) {
    if (!ppmd_ctx_ || in_len <= 0) return -1;
    CPpmd8* p = (CPpmd8*)ppmd_ctx_;

    if (!streaming_active_ || !ppmd_model_warm_)
        Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);

    CByteOutBuf outStream;
    outStream.vt.Write = ByteOutBuf_Write;
    outStream.buf = out;
    outStream.capacity = out_cap;
    outStream.pos = 0;
    outStream.overflow = 0;

    p->Stream.Out = &outStream.vt;
    Ppmd8_Init_RangeEnc(p);

    for (int i = 0; i < in_len; i++)
        Ppmd8_EncodeSymbol(p, in[i]);
    Ppmd8_Flush_RangeEnc(p);

    if (outStream.overflow) return -1;
    return outStream.pos;
}

int Compressor::ppmd_decompress(const uint8_t* in, int in_len, int orig_len,
                                 uint8_t* out, int out_cap) {
    if (!ppmd_ctx_ || in_len <= 0 || orig_len <= 0 || orig_len > out_cap) return -1;
    CPpmd8* p = (CPpmd8*)ppmd_ctx_;

    if (!streaming_active_ || !ppmd_model_warm_)
        Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);

    CByteInBuf inStream;
    inStream.vt.Read = ByteInBuf_Read;
    inStream.buf = in;
    inStream.size = in_len;
    inStream.pos = 0;

    p->Stream.In = &inStream.vt;
    if (!Ppmd8_Init_RangeDec(p)) return -1;

    for (int i = 0; i < orig_len; i++) {
        int sym = Ppmd8_DecodeSymbol(p);
        if (sym < 0) return -1;
        out[i] = (uint8_t)sym;
    }
    return orig_len;
}

// ---- zstd compress/decompress ----

int Compressor::zstd_compress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap) {
    if (!zstd_cctx_ || in_len <= 0) return -1;
    if (streaming_active_ && zstd_prefix_ && zstd_prefix_len_ > 0)
        ZSTD_CCtx_refPrefix((ZSTD_CCtx*)zstd_cctx_, zstd_prefix_, zstd_prefix_len_);
    size_t result = ZSTD_compress2((ZSTD_CCtx*)zstd_cctx_, out, out_cap, in, in_len);
    if (ZSTD_isError(result)) return -1;
    return (int)result;
}

int Compressor::zstd_decompress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap) {
    if (!zstd_dctx_ || in_len <= 0) return -1;
    if (streaming_active_ && zstd_prefix_ && zstd_prefix_len_ > 0)
        ZSTD_DCtx_refPrefix((ZSTD_DCtx*)zstd_dctx_, zstd_prefix_, zstd_prefix_len_);
    size_t result = ZSTD_decompressDCtx((ZSTD_DCtx*)zstd_dctx_, out, out_cap, in, in_len);
    if (ZSTD_isError(result)) return -1;
    return (int)result;
}

// ---- Block compress (TX) ----

int Compressor::compress_block(const uint8_t* in, int in_len, uint8_t* out, int out_capacity) {
    if (!initialized_ || in_len <= 0 || !workspace_) return 0;

    int hdr_size = get_header_size();
    float entropy = quick_entropy(in, in_len);

    int best_algo = COMPRESS_ALGO_RAW;
    int best_comp_size = in_len;
    int best_offset = 0;
    bool best_is_raw = true;
    int half = workspace_size_ / 2;

    // Streaming with warm model: PPMd only (avoid model desync)
    if (streaming_active_ && ppmd_model_warm_) {
        if (entropy < ENTROPY_SKIP_ALL) {
            int ps = ppmd_compress(in, in_len, workspace_ + half, half);
            if (ps > 0 && ps < best_comp_size) {
                best_algo = COMPRESS_ALGO_PPMD;
                best_comp_size = ps;
                best_offset = half;
                best_is_raw = false;
            }
        }
    } else {
        if (entropy <= ENTROPY_SKIP_ALL) {
            int zs = zstd_compress_buf(in, in_len, workspace_, half);
            if (zs > 0 && zs < best_comp_size) {
                best_algo = COMPRESS_ALGO_ZSTD;
                best_comp_size = zs;
                best_offset = 0;
                best_is_raw = false;
            }
        }
        if (entropy < ENTROPY_ZSTD_ONLY) {
            int ps = ppmd_compress(in, in_len, workspace_ + half, half);
            if (ps > 0 && ps < best_comp_size) {
                best_algo = COMPRESS_ALGO_PPMD;
                best_comp_size = ps;
                best_offset = half;
                best_is_raw = false;
            }
        }
    }

    int raw_total = hdr_size + in_len;
    int compressed_total = hdr_size + best_comp_size;

    if (!best_is_raw && compressed_total >= raw_total) {
        best_algo = COMPRESS_ALGO_RAW;
        best_comp_size = in_len;
        best_is_raw = true;
        if (streaming_active_ && ppmd_model_warm_)
            streaming_reset();
    }

    int total = hdr_size + best_comp_size;
    if (total > out_capacity) {
        if (streaming_active_ && ppmd_model_warm_)
            streaming_reset();
        return -1;
    }

    // Write header
    out[0] = (uint8_t)(best_algo |
        (streaming_active_ && stream_batch_count_ > 0 ? COMPRESS_FLAG_STREAMING : 0));
    out[1] = (uint8_t)(best_comp_size & 0xFF);
    out[2] = (uint8_t)((best_comp_size >> 8) & 0xFF);
    out[3] = (uint8_t)(in_len & 0xFF);
    out[4] = (uint8_t)((in_len >> 8) & 0xFF);

    if (streaming_active_) {
        uint16_t crc = compress_crc16(in, in_len);
        out[5] = (uint8_t)(crc & 0xFF);
        out[6] = (uint8_t)((crc >> 8) & 0xFF);
    }

    // Write payload
    if (best_is_raw)
        memcpy(out + hdr_size, in, in_len);
    else
        memcpy(out + hdr_size, workspace_ + best_offset, best_comp_size);

    if (best_algo != COMPRESS_ALGO_RAW) {
        last_ratio_ = (float)in_len / (float)best_comp_size;
        printf("[COMPRESS] %d -> %d bytes (%s%s, entropy=%.1f, ratio=%.1fx)\n",
               in_len, total,
               best_algo == COMPRESS_ALGO_PPMD ? "PPMd" : "zstd",
               (streaming_active_ && stream_batch_count_ > 0) ? "+stream" : "",
               entropy, last_ratio_);
        fflush(stdout);
    } else {
        last_ratio_ = 0;
    }

    return total;
}

// ---- Block decompress (RX) ----

int Compressor::decompress_block(const uint8_t* in, int in_len, uint8_t* out, int out_capacity) {
    int hdr_size = get_header_size();
    if (!initialized_ || in_len < hdr_size) return -1;

    int algo = in[0] & COMPRESS_ALGO_MASK;
    bool is_streaming_frame = (in[0] & COMPRESS_FLAG_STREAMING) != 0;
    int comp_size = in[1] | (in[2] << 8);
    int orig_size = in[3] | (in[4] << 8);

    // RX-side desync detection: warm model receiving non-streaming frame
    if (streaming_active_ && !is_streaming_frame && ppmd_model_warm_) {
        printf("[COMPRESS-RX] Desync: warm model but non-streaming frame received, resetting\n");
        fflush(stdout);
        streaming_reset();
    }

    // RX-side desync: streaming frame received but model is cold
    if (streaming_active_ && is_streaming_frame && !ppmd_model_warm_ && stream_batch_count_ == 0) {
        printf("[COMPRESS-RX] Desync: streaming frame but cold model, resetting\n");
        fflush(stdout);
        streaming_reset();
    }

    if (comp_size < 0 || orig_size < 0 || orig_size > out_capacity) return -1;
    if (hdr_size + comp_size > in_len) return -1;

    const uint8_t* payload = in + hdr_size;
    int result = -1;

    if (algo == COMPRESS_ALGO_RAW) {
        if (comp_size != orig_size) return -1;
        memcpy(out, payload, orig_size);
        result = orig_size;
    } else if (algo == COMPRESS_ALGO_PPMD) {
        result = ppmd_decompress(payload, comp_size, orig_size, out, out_capacity);
        if (result != orig_size) {
            printf("[COMPRESS-RX] PPMd decompress failed: got %d, expected %d\n",
                   result, orig_size);
            fflush(stdout);
            if (streaming_active_) streaming_reset();
            return -1;
        }
    } else if (algo == COMPRESS_ALGO_ZSTD) {
        result = zstd_decompress_buf(payload, comp_size, out, out_capacity);
        if (result != orig_size) {
            printf("[COMPRESS-RX] zstd decompress failed: got %d, expected %d\n",
                   result, orig_size);
            fflush(stdout);
            if (streaming_active_) streaming_reset();
            return -1;
        }
    } else {
        printf("[COMPRESS-RX] Unknown algo %d\n", algo);
        fflush(stdout);
        return -1;
    }

    // CRC16 verify (streaming mode)
    if (streaming_active_ && result > 0) {
        uint16_t expected_crc = in[5] | (in[6] << 8);
        uint16_t actual_crc = compress_crc16(out, result);
        if (actual_crc != expected_crc) {
            printf("[COMPRESS-RX] CRC16 mismatch: expected 0x%04X, got 0x%04X, resetting stream\n",
                   expected_crc, actual_crc);
            fflush(stdout);
            streaming_reset();
            return -1;
        }
    }

    return result;
}

// ---- Legacy simple API ----

std::vector<uint8_t> compress(const uint8_t* data, size_t len) {
    if (len == 0) return {};
    size_t bound = ZSTD_compressBound(len);
    std::vector<uint8_t> out(bound);
    size_t result = ZSTD_compress(out.data(), bound, data, len, 1);
    if (ZSTD_isError(result)) return {};
    out.resize(result);
    return out;
}

std::vector<uint8_t> decompress(const uint8_t* data, size_t len) {
    if (len == 0) return {};
    unsigned long long decom_size = ZSTD_getFrameContentSize(data, len);
    if (decom_size == ZSTD_CONTENTSIZE_ERROR ||
        decom_size == ZSTD_CONTENTSIZE_UNKNOWN) {
        decom_size = len * 16;
        if (decom_size > 1024 * 1024) decom_size = 1024 * 1024;
    }
    std::vector<uint8_t> out((size_t)decom_size);
    size_t result = ZSTD_decompress(out.data(), out.size(), data, len);
    if (ZSTD_isError(result)) return {};
    out.resize(result);
    return out;
}

std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len) {
    if (len == 0) return {0x00};
    auto compressed = compress(data, len);
    if (!compressed.empty() && compressed.size() < len) {
        std::vector<uint8_t> out;
        out.reserve(1 + compressed.size());
        out.push_back(0x01);
        out.insert(out.end(), compressed.begin(), compressed.end());
        return out;
    }
    std::vector<uint8_t> out;
    out.reserve(1 + len);
    out.push_back(0x00);
    out.insert(out.end(), data, data + len);
    return out;
}

std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len) {
    if (len < 1) return {};
    if (data[0] == 0x00) return std::vector<uint8_t>(data + 1, data + len);
    if (data[0] == 0x01) return decompress(data + 1, len - 1);
    return {};
}

} // namespace iris
