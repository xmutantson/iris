#ifndef IRIS_COMPRESS_H
#define IRIS_COMPRESS_H

#include <vector>
#include <cstdint>
#include <cstddef>
#include "v2/encoded_record.h"

namespace iris {

// Compression algorithm IDs
static constexpr uint8_t COMPRESS_ALGO_RAW  = 0x00;
static constexpr uint8_t COMPRESS_ALGO_PPMD = 0x01;
static constexpr uint8_t COMPRESS_ALGO_ZSTD = 0x02;
static constexpr uint8_t COMPRESS_ALGO_MASK = 0x03;
static constexpr uint8_t COMPRESS_FLAG_STREAMING = 0x04;

// Entropy thresholds (bits per byte, 0.0 = constant, 8.0 = random)
static constexpr float ENTROPY_SKIP_ALL  = 7.5f;  // Incompressible — send raw
static constexpr float ENTROPY_ZSTD_ONLY = 6.0f;  // Mixed — try zstd only
                                                    // Below 6.0: try both PPMd and zstd

// Header sizes
static constexpr int COMPRESS_HEADER_SIZE        = 7;  // Streaming: algo(1)+comp(2)+orig(2)+crc16(2)
static constexpr int COMPRESS_HEADER_SIZE_LEGACY  = 5;  // Non-streaming: algo(1)+comp(2)+orig(2)

static constexpr int COMPRESS_WORKSPACE_SIZE = 65536;  // 64 KB workspace
static constexpr int ZSTD_PREFIX_CAPACITY    = 32768;  // 32 KB sliding window
static constexpr size_t COMPRESS_MAX_RECORD_SIZE = 65535;

// Adaptive dual-algorithm compressor (PPMd8 + zstd) with streaming context.
// Ported from Mercury's cl_compressor.
class Compressor {
public:
    Compressor();
    ~Compressor();

    void init();
    void deinit();

    // Compress a batch block. Returns total bytes written (header + payload), or -1 on error.
    int compress_block(const uint8_t* in, int in_len, uint8_t* out, int out_capacity);

    // Decompress a batch block. Returns decompressed bytes, or -1 on error.
    int decompress_block(const uint8_t* in, int in_len, uint8_t* out, int out_capacity);

    // Complete-record APIs. Capacity comes from the validated original-size
    // field, never from an encoded-size expansion guess.
    v2::RecordTransformResult compress_record(const uint8_t* in, size_t in_len);
    v2::RecordTransformResult decompress_record(const uint8_t* in, size_t in_len);
    bool declared_record_sizes(const uint8_t* in, size_t in_len,
                               size_t& encoded_size, size_t& original_size) const;

    // Streaming context management
    void streaming_enable();
    void streaming_disable();
    void streaming_reset();
    void streaming_commit(const uint8_t* raw_data, int raw_len);
    void ppmd_model_reset();   // Reset ONLY the PPMd model (keep zstd prefix) — see .cc
    void set_pending_raw(const uint8_t* data, int len);
    void commit_pending();
    void clear_pending();
    bool is_streaming() const { return streaming_active_; }
    int get_header_size() const { return streaming_active_ ? COMPRESS_HEADER_SIZE : COMPRESS_HEADER_SIZE_LEGACY; }
    bool is_initialized() const { return initialized_; }
    bool is_healthy() const { return initialized_ && !codec_failed_; }
    float last_ratio() const { return last_ratio_; }  // Last compression ratio (0 if no compression)

    // ---- Winlink dictionary priming (universal static dict) ----
    // Enable/disable dict priming. Default-ON. When disabled, streaming_enable()
    // does NOT prime and the cold path is byte-identical to the pre-dict baseline.
    // Must be set IDENTICALLY on both peers (a build/runtime posture). PRE-SHIP: no
    // wire version bit — the dict is firmware-baked and both ends rebuild on change.
    void set_dict_priming(bool on) { dict_priming_enabled_ = on; }
    bool dict_priming() const { return dict_priming_enabled_; }
    // True once this stream has been primed with the baked dict (cold otherwise).
    bool dict_primed() const { return dict_primed_; }

    // Shannon entropy (bits per byte)
    static float quick_entropy(const uint8_t* data, int len);

private:
    // Author: xmutantson. RC4 acceptance: observe installed callbacks/state only.
    friend class AcceptanceArqHarness;

    // Role-independent in-place priming: decompress the firmware-baked compressed
    // dict (one deterministic forward pass through the shared PPMd model, identical
    // on TX and RX) then load the raw dict into the zstd prefix. Zero wire cost.
    // Called from streaming_enable() when dict_priming_enabled_. Returns 1 on
    // success (primed), 0 on failure/disabled (stream stays cold).
    int prime_with_dict();

    int ppmd_compress(const uint8_t* in, int in_len, uint8_t* out, int out_cap);
    int ppmd_decompress(const uint8_t* in, int in_len, int orig_len, uint8_t* out, int out_cap);
    int zstd_compress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap);
    int zstd_decompress_buf(const uint8_t* in, int in_len, uint8_t* out, int out_cap);

    void* ppmd_ctx_;
    void* zstd_cctx_;
    void* zstd_dctx_;
    uint8_t* workspace_;
    int workspace_size_;
    bool initialized_;
    // Sticky for the lifetime of the negotiated codec session.  A transform or
    // dictionary failure may only be cleared by deinit()+init() as part of a
    // newly negotiated session; unilateral streaming reset is not recovery.
    bool codec_failed_;

    // Streaming state
    bool streaming_active_;
    int stream_batch_count_;
    bool ppmd_model_warm_;         // A streaming batch has been COMMITTED since the last
                                   // reset (zstd OR PPMd). Governs PPMd-only mode, the
                                   // raw-win/doesn't-fit reset, and RX desync detection.
    bool ppmd_model_initialized_;  // The PPMd model has ACTUALLY been Ppmd8_Init()-ed (and
                                   // not reset) since streaming began. ONLY this flag may
                                   // gate the Init-skip in ppmd_compress/ppmd_decompress —
                                   // ppmd_model_warm_ can be true after a zstd-only or
                                   // dict-primed committed batch that never touched the PPMd
                                   // model, which would otherwise encode/decode into an
                                   // uninitialized model (segfault). See ppmd_compress().
    uint8_t* zstd_prefix_;
    int zstd_prefix_len_;
    uint8_t* pending_raw_;
    int pending_raw_len_;
    int pending_raw_capacity_;
    float last_ratio_ = 0;  // Last compression ratio (in_len / compressed_len)

    // Winlink dict priming
    bool dict_priming_enabled_;    // Posture: prime new streams with the baked dict
    bool dict_primed_;             // This stream was primed (reset to cold on desync)
};

// Legacy simple API (wraps Compressor for backward compatibility)
std::vector<uint8_t> compress(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress(const uint8_t* data, size_t len);
std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len);

} // namespace iris

#endif
