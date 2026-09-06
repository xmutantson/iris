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
#include "compress/winlink_dict.h"
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
    int overread;
};

static Byte ByteInBuf_Read(const IByteIn* pp) {
    CByteInBuf* p = (CByteInBuf*)(void*)pp;
    if (p->pos < p->size)
        return p->buf[p->pos++];
    p->overread = 1;
    return 0;
}

// ---- Compressor ----

Compressor::Compressor()
    : ppmd_ctx_(nullptr), zstd_cctx_(nullptr), zstd_dctx_(nullptr),
      workspace_(nullptr), workspace_size_(0), initialized_(false),
      codec_failed_(false),
      streaming_active_(false), stream_batch_count_(0), ppmd_model_warm_(false),
      ppmd_model_initialized_(false),
      zstd_prefix_(nullptr), zstd_prefix_len_(0),
      pending_raw_(nullptr), pending_raw_len_(0), pending_raw_capacity_(0),
      // Dict priming is DEFAULT-ON. IRIS_NO_DICT (build/runtime kill-switch) forces
      // the cold path so the stream is byte-identical to the pre-dict baseline (used
      // by the no-regression A/B and as a field escape hatch). Disabling on EITHER
      // peer cleanly degrades to cold on both (both prime identically or neither does).
      dict_priming_enabled_(getenv("IRIS_NO_DICT") == nullptr),
      dict_primed_(false) {}

Compressor::~Compressor() { deinit(); }

void Compressor::init() {
    if (initialized_) return;
    codec_failed_ = false;

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
    codec_failed_ = false;
}

// ---- Streaming context management ----

void Compressor::streaming_enable() {
    if (!initialized_ || streaming_active_) return;
    zstd_prefix_ = (uint8_t*)malloc(ZSTD_PREFIX_CAPACITY);
    zstd_prefix_len_ = 0;
    pending_raw_capacity_ = COMPRESS_WORKSPACE_SIZE;
    pending_raw_ = (uint8_t*)malloc(pending_raw_capacity_);
    pending_raw_len_ = 0;
    if (!zstd_prefix_ || !pending_raw_) {
        free(zstd_prefix_); zstd_prefix_ = nullptr;
        free(pending_raw_); pending_raw_ = nullptr;
        pending_raw_capacity_ = 0;
        codec_failed_ = true;
        return;
    }
    stream_batch_count_ = 0;
    ppmd_model_warm_ = false;
    // The PPMd model is NOT Ppmd8_Init()-ed here — it is lazily initialized on the
    // first ppmd_compress/decompress call (or after a reset). Mark it uninitialized
    // so the first PPMd use Init()s before encoding/decoding.
    ppmd_model_initialized_ = false;
    dict_primed_ = false;
    streaming_active_ = true;

    // Winlink dict priming: warm the brand-new stream with the firmware-baked
    // dictionary so the FIRST small message compresses as if warm. Done AFTER
    // streaming_active_=true (prime_with_dict drives decompress_block /
    // streaming_commit, which require an active stream). Role-independent and
    // deterministic — see prime_with_dict().
    if (dict_priming_enabled_ && !prime_with_dict())
        codec_failed_ = true;
}

// Role-independent in-place priming. BOTH peers run the IDENTICAL sequence on the
// IDENTICAL firmware-baked bytes, so the PPMd model + zstd prefix end in the SAME
// state on TX and RX (the bit-exact agreement the streaming CRC16 demands):
//
//   1. decompress_block(WINLINK_DICT_COMPRESSED) — a single deterministic forward
//      pass over the dict symbols through the SHARED PPMd model (PPMd8 uses one model
//      object for enc+dec, and encoding symbol X and decoding symbol X advance the
//      model identically). This is why decompress-on-both-sides is exact: the TX's
//      eventual ppmd_compress and the RX's ppmd_decompress both continue from the same
//      model state.
//   2. streaming_commit(WINLINK_DICT_RAW) — loads the raw dict into the zstd prefix
//      (32 KB back-reference window) AND sets ppmd_model_warm_.
//
// The dict bytes are NEVER transmitted (baked into both ends). Any failure latches
// the codec failed for this session; only a newly negotiated session may restart it.
int Compressor::prime_with_dict() {
    if (!streaming_active_ || !zstd_prefix_ || !workspace_) return 0;
    if (WINLINK_DICT_COMPRESSED_LEN == 0 || WINLINK_DICT_RAW_LEN == 0) return 0;

    // (1) Warm the PPMd model via a forward decode of the baked compressed dict.
    //     workspace_ is the scratch sink (>= raw dict length; dict is < 4 KB).
    if ((int)WINLINK_DICT_RAW_LEN > workspace_size_) return 0;  // safety; dict is tiny
    int d = decompress_block(WINLINK_DICT_COMPRESSED, (int)WINLINK_DICT_COMPRESSED_LEN,
                             workspace_, workspace_size_);
    if (d != (int)WINLINK_DICT_RAW_LEN) {
        // Self-inconsistent firmware is a negotiated-codec failure, not authority
        // to silently select a different representation.
        printf("[DICT] Prime decode failed (d=%d, expected %u)\n",
               d, WINLINK_DICT_RAW_LEN);
        fflush(stdout);
        return 0;
    }

    // (2) Load the raw dict into the zstd prefix window (and mark the model warm).
    streaming_commit((const uint8_t*)WINLINK_DICT_RAW, (int)WINLINK_DICT_RAW_LEN);

    // The dict batch is a "warm-up" the wire never saw. stream_batch_count_ is left as
    // streaming_commit set it (==1) so the first REAL frame carries
    // COMPRESS_FLAG_STREAMING — both peers are primed identically, so this is correct
    // and symmetric.
    dict_primed_ = true;

    printf("[DICT] Primed: raw=%u compressed=%u (zstd prefix=%d, PPMd %s)\n",
           WINLINK_DICT_RAW_LEN, WINLINK_DICT_COMPRESSED_LEN, zstd_prefix_len_,
           ppmd_model_initialized_ ? "warm" : "cold(zstd-only dict)");
    fflush(stdout);
    return 1;
}

void Compressor::streaming_disable() {
    if (!streaming_active_) return;
    streaming_active_ = false;
    if (ppmd_ctx_)
        Ppmd8_Init((CPpmd8*)ppmd_ctx_, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
    ppmd_model_warm_ = false;
    ppmd_model_initialized_ = false;
    dict_primed_ = false;
    stream_batch_count_ = 0;
    free(zstd_prefix_); zstd_prefix_ = nullptr; zstd_prefix_len_ = 0;
    free(pending_raw_); pending_raw_ = nullptr; pending_raw_len_ = 0; pending_raw_capacity_ = 0;
}

void Compressor::streaming_reset() {
    if (!streaming_active_) return;
    if (ppmd_ctx_)
        Ppmd8_Init((CPpmd8*)ppmd_ctx_, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
    ppmd_model_warm_ = false;
    // The explicit Ppmd8_Init above leaves the model in a valid RESTART state, but
    // the streaming contract is "next streaming PPMd batch starts fresh": mark it
    // uninitialized so the next ppmd_compress/decompress Re-Init()s deterministically.
    ppmd_model_initialized_ = false;
    // A reset is a desync RECOVERY: the stream drops to the cold/un-primed state and
    // STAYS cold for the rest of the session (re-priming one side without the peer
    // re-priming in lock-step would re-desync). "Fail-safe, never corrupt" prefers
    // the lower-ratio cold path.
    dict_primed_ = false;
    zstd_prefix_len_ = 0;
    pending_raw_len_ = 0;
    stream_batch_count_ = 0;
}

// Reset ONLY the PPMd streaming model (leave the zstd prefix / committed window
// intact). Used when a streaming batch is carried by zstd/raw rather than PPMd: the
// PPMd model must not carry symbols across a non-PPMd batch, because TX and RX
// diverge there (TX may have speculatively run ppmd_compress to compare; RX only ran
// zstd). Resetting the PPMd model on BOTH sides at every non-PPMd batch keeps the
// model in lock-step: any PPMd batch is then either the fresh start of a PPMd run
// (both sides just reset) or a continuation of an all-PPMd run (both carried
// identically). The zstd prefix is unaffected and keeps warming.
void Compressor::ppmd_model_reset() {
    if (!streaming_active_) return;
    // Mark uninitialized so the next ppmd_compress/decompress Re-Init()s fresh and
    // deterministically on both sides. No Ppmd8_Init here — it is lazy.
    ppmd_model_initialized_ = false;
}

void Compressor::set_pending_raw(const uint8_t* data, int len) {
    if (!streaming_active_ || !pending_raw_ || len <= 0) return;
    if (len > pending_raw_capacity_) {
        data += len - pending_raw_capacity_;
        len = pending_raw_capacity_;
    }
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
    stream_batch_count_ = 1;
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

    // ROOT-CAUSE GUARD (C0): skip Ppmd8_Init ONLY when the PPMd model is actually
    // initialized. ppmd_model_warm_ is NOT a safe gate: streaming_commit() sets
    // warm=true after ANY committed batch (including a zstd-only batch, or a
    // dict-primed batch, that never exercised the PPMd model), so a warm model can be
    // one that was only Ppmd8_Construct()+Alloc()-ed, never RestartModel()-ed.
    // Encoding into that uninitialized model dereferences uninitialized context
    // pointers -> segfault. Gating on ppmd_model_initialized_ guarantees the model is
    // RESTART-initialized before the first symbol.
    if (!streaming_active_ || !ppmd_model_initialized_) {
        Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
        ppmd_model_initialized_ = true;
    }

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

    // ROOT-CAUSE GUARD (C0, RX mirror of ppmd_compress): skip Ppmd8_Init only when the
    // PPMd model is actually initialized. The RX side NEVER speculatively runs PPMd (it
    // only decodes the chosen algo), so after a zstd/dict batch the RX PPMd model is
    // guaranteed uninitialized while ppmd_model_warm_ is true (set by streaming_commit).
    // Decoding a later PPMd frame against that model would read uninitialized context ->
    // segfault. Gate on ppmd_model_initialized_, not warm.
    if (!streaming_active_ || !ppmd_model_initialized_) {
        Ppmd8_Init(p, PPMD_ORDER, PPMD8_RESTORE_METHOD_RESTART);
        ppmd_model_initialized_ = true;
    }

    CByteInBuf inStream;
    inStream.vt.Read = ByteInBuf_Read;
    inStream.buf = in;
    inStream.size = in_len;
    inStream.pos = 0;
    inStream.overread = 0;

    p->Stream.In = &inStream.vt;
    if (!Ppmd8_Init_RangeDec(p) || inStream.overread) return -1;

    for (int i = 0; i < orig_len; i++) {
        int sym = Ppmd8_DecodeSymbol(p);
        if (sym < 0 || inStream.overread) return -1;
        out[i] = (uint8_t)sym;
    }
    return orig_len;
}

// ---- zstd compress/decompress ----

int Compressor::zstd_compress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap) {
    if (!zstd_cctx_ || in_len <= 0) return -1;
    if (streaming_active_ && zstd_prefix_ && zstd_prefix_len_ > 0 &&
        ZSTD_isError(ZSTD_CCtx_refPrefix((ZSTD_CCtx*)zstd_cctx_, zstd_prefix_,
                                        zstd_prefix_len_)))
        return -1;
    size_t result = ZSTD_compress2((ZSTD_CCtx*)zstd_cctx_, out, out_cap, in, in_len);
    if (ZSTD_isError(result)) return -1;
    return (int)result;
}

int Compressor::zstd_decompress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap) {
    if (!zstd_dctx_ || in_len <= 0) return -1;
    if (streaming_active_ && zstd_prefix_ && zstd_prefix_len_ > 0 &&
        ZSTD_isError(ZSTD_DCtx_refPrefix((ZSTD_DCtx*)zstd_dctx_, zstd_prefix_,
                                        zstd_prefix_len_)))
        return -1;
    size_t result = ZSTD_decompressDCtx((ZSTD_DCtx*)zstd_dctx_, out, out_cap, in, in_len);
    if (ZSTD_isError(result)) return -1;
    return (int)result;
}

// ---- Block compress (TX) ----

int Compressor::compress_block(const uint8_t* in, int in_len, uint8_t* out, int out_capacity) {
    if (!initialized_ || codec_failed_ || in_len <= 0 || !workspace_ || !in || !out ||
        static_cast<size_t>(in_len) > COMPRESS_MAX_RECORD_SIZE) return -1;

    int hdr_size = get_header_size();
    float entropy = quick_entropy(in, in_len);

    int best_algo = COMPRESS_ALGO_RAW;
    int best_comp_size = in_len;
    int best_offset = 0;
    bool best_is_raw = true;
    int half = workspace_size_ / 2;

    // Streaming with an INITIALIZED PPMd model: PPMd only (avoid model desync).
    // Gate on ppmd_model_initialized_, NOT ppmd_model_warm_: warm is set after ANY
    // committed batch (zstd-only or dict-primed), but PPMd-only mode is only safe once
    // PPMd has actually carried a batch and initialized the model on BOTH sides. While
    // the stream is warm but PPMd has not yet carried (e.g. a leading zstd run or a
    // dict-primed stream whose dict blob was zstd), stay in the cold try-both branch so
    // PPMd is only adopted when it wins — at which point the model is initialized in
    // lock-step on TX and RX. (ppmd_model_reset() drops this flag whenever a non-PPMd
    // batch is committed.)
    if (streaming_active_ && ppmd_model_initialized_) {
        if (entropy < ENTROPY_SKIP_ALL) {
            int ps = ppmd_compress(in, in_len, workspace_ + half, half);
            if (ps <= 0) {
                codec_failed_ = true;
                return -1;
            }
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
            if (zs <= 0) {
                codec_failed_ = true;
                return -1;
            }
            if (zs > 0 && zs < best_comp_size) {
                best_algo = COMPRESS_ALGO_ZSTD;
                best_comp_size = zs;
                best_offset = 0;
                best_is_raw = false;
            }
        }
        if (entropy < ENTROPY_ZSTD_ONLY) {
            int ps = ppmd_compress(in, in_len, workspace_ + half, half);
            if (ps <= 0) {
                codec_failed_ = true;
                return -1;
            }
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
        if (streaming_active_)
            ppmd_model_reset();
    }

    int total = hdr_size + best_comp_size;
    if (total > out_capacity) {
        if (streaming_active_)
            codec_failed_ = true;
        return -1;
    }

    // Streaming: a zstd-carried batch (or RAW) must NOT leave the PPMd model
    // "carrying". The cold try-both branch above may have speculatively run
    // ppmd_compress (advancing the TX model) before zstd won; the RX runs zstd-only
    // and never touches its PPMd model. Drop the PPMd model on BOTH sides at every
    // non-PPMd batch so the next PPMd batch Re-Init()s fresh in lock-step. (RAW wins
    // are handled by the selection above, but ppmd_model_reset() here
    // is idempotent; the zstd-wins path — which previously left the model "warm" ->
    // PPMd-only mode against a model the RX never initialized -> segfault — is the one
    // this covers.) The zstd prefix is untouched.
    if (streaming_active_ && best_algo != COMPRESS_ALGO_PPMD)
        ppmd_model_reset();

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
    if (!initialized_ || codec_failed_ || !in || !out || in_len < hdr_size) return -1;

    int algo = in[0] & COMPRESS_ALGO_MASK;
    bool is_streaming_frame = (in[0] & COMPRESS_FLAG_STREAMING) != 0;
    int comp_size = in[1] | (in[2] << 8);
    int orig_size = in[3] | (in[4] << 8);

    if (comp_size < 0 || orig_size < 0 || orig_size > out_capacity) return -1;
    // A block is one complete record. Concatenation and trailing bytes are not
    // silently accepted because they would make the transform boundary depend
    // on the caller's slot size.
    if (hdr_size + comp_size != in_len) return -1;

    // RX-side desync detection: warm model receiving non-streaming frame
    if (streaming_active_ && !is_streaming_frame && ppmd_model_warm_) {
        printf("[COMPRESS-RX] Desync: warm model but non-streaming frame received\n");
        fflush(stdout);
        return -1;
    }

    // RX-side desync: streaming frame received but model is cold
    if (streaming_active_ && is_streaming_frame && !ppmd_model_warm_ && stream_batch_count_ == 0) {
        printf("[COMPRESS-RX] Desync: streaming frame but cold model\n");
        fflush(stdout);
        return -1;
    }

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
            codec_failed_ = true;
            return -1;
        }
    } else if (algo == COMPRESS_ALGO_ZSTD) {
        result = zstd_decompress_buf(payload, comp_size, out, out_capacity);
        if (result != orig_size) {
            printf("[COMPRESS-RX] zstd decompress failed: got %d, expected %d\n",
                   result, orig_size);
            fflush(stdout);
            codec_failed_ = true;
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
            printf("[COMPRESS-RX] CRC16 mismatch: expected 0x%04X, got 0x%04X\n",
                   expected_crc, actual_crc);
            fflush(stdout);
            codec_failed_ = true;
            return -1;
        }
    }

    // RX mirror of compress_block's post-batch PPMd model reset: any NON-PPMd
    // streaming batch (zstd OR raw) leaves the RX PPMd model untouched (only zstd /
    // memcpy ran). The TX resets its PPMd model after every non-PPMd chosen algo, so
    // the RX MUST do the same on every non-PPMd decoded algo to stay in lock-step.
    // Otherwise ppmd_model_warm_ could stay true while the model is uninitialized (or
    // diverged), and the next PPMd frame's warm path would decode into an
    // uninitialized/diverged model (segfault or desync). PPMd frames legitimately
    // carry the model — they are the only algo that must NOT reset it.
    if (streaming_active_ && result > 0 && algo != COMPRESS_ALGO_PPMD)
        ppmd_model_reset();

    return result;
}

bool Compressor::declared_record_sizes(const uint8_t* in, size_t in_len,
                                       size_t& encoded_size,
                                       size_t& original_size) const {
    encoded_size = 0;
    original_size = 0;
    const size_t header = static_cast<size_t>(get_header_size());
    if (!initialized_ || !in || in_len < header) return false;
    const uint8_t allowed = COMPRESS_ALGO_MASK | COMPRESS_FLAG_STREAMING;
    if ((in[0] & ~allowed) != 0 ||
        (in[0] & COMPRESS_ALGO_MASK) > COMPRESS_ALGO_ZSTD) return false;
    const size_t compressed = static_cast<size_t>(in[1]) |
                              (static_cast<size_t>(in[2]) << 8);
    const size_t original = static_cast<size_t>(in[3]) |
                            (static_cast<size_t>(in[4]) << 8);
    if (compressed > COMPRESS_MAX_RECORD_SIZE ||
        original > COMPRESS_MAX_RECORD_SIZE ||
        compressed > SIZE_MAX - header) return false;
    encoded_size = header + compressed;
    original_size = original;
    return true;
}

v2::RecordTransformResult Compressor::compress_record(const uint8_t* in,
                                                       size_t in_len) {
    v2::RecordTransformResult result;
    if (!in || in_len == 0 || in_len > COMPRESS_MAX_RECORD_SIZE) {
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::OriginalSizeLimitExceeded;
        return result;
    }
    result.produced_bytes.resize(in_len + static_cast<size_t>(get_header_size()));
    const int count = compress_block(in, static_cast<int>(in_len),
                                     result.produced_bytes.data(),
                                     static_cast<int>(result.produced_bytes.size()));
    if (count <= 0) {
        result.produced_bytes.clear();
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::TransformFailure;
        return result;
    }
    result.produced_bytes.resize(static_cast<size_t>(count));
    result.input_bytes_consumed = in_len;
    result.status = v2::TransformStatus::Produced;
    return result;
}

v2::RecordTransformResult Compressor::decompress_record(const uint8_t* in,
                                                         size_t in_len) {
    v2::RecordTransformResult result;
    size_t encoded = 0, original = 0;
    if (!declared_record_sizes(in, in_len, encoded, original)) {
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::TransformFailure;
        return result;
    }
    if (encoded != in_len) {
        result.status = v2::TransformStatus::Failed;
        result.error = encoded > in_len
            ? v2::EnvelopeValidationError::IncompleteExtentCoverage
            : v2::EnvelopeValidationError::TransformFailure;
        return result;
    }
    result.produced_bytes.resize(original);
    // The legacy codec cannot represent empty input blocks. Keep a non-null
    // destination for defensive C APIs even when a malformed peer declares 0.
    uint8_t sink = 0;
    uint8_t* destination = result.produced_bytes.empty()
        ? &sink : result.produced_bytes.data();
    const int count = decompress_block(in, static_cast<int>(in_len), destination,
                                       static_cast<int>(original));
    if (count < 0 || static_cast<size_t>(count) != original) {
        result.produced_bytes.clear();
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::TransformFailure;
        return result;
    }
    result.input_bytes_consumed = in_len;
    result.status = v2::TransformStatus::Produced;
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
    if (decom_size > 1024 * 1024) return {};
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
