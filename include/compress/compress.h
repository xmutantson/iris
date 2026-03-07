#ifndef IRIS_COMPRESS_H
#define IRIS_COMPRESS_H

#include <vector>
#include <cstdint>
#include <cstddef>

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

    // Streaming context management
    void streaming_enable();
    void streaming_disable();
    void streaming_reset();
    void streaming_commit(const uint8_t* raw_data, int raw_len);
    void set_pending_raw(const uint8_t* data, int len);
    void commit_pending();
    void clear_pending();
    bool is_streaming() const { return streaming_active_; }
    int get_header_size() const { return streaming_active_ ? COMPRESS_HEADER_SIZE : COMPRESS_HEADER_SIZE_LEGACY; }
    bool is_initialized() const { return initialized_; }

    // Shannon entropy (bits per byte)
    static float quick_entropy(const uint8_t* data, int len);

private:
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

    // Streaming state
    bool streaming_active_;
    int stream_batch_count_;
    bool ppmd_model_warm_;
    uint8_t* zstd_prefix_;
    int zstd_prefix_len_;
    uint8_t* pending_raw_;
    int pending_raw_len_;
    int pending_raw_capacity_;
};

// Legacy simple API (wraps Compressor for backward compatibility)
std::vector<uint8_t> compress(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress(const uint8_t* data, size_t len);
std::vector<uint8_t> compress_frame(const uint8_t* data, size_t len);
std::vector<uint8_t> decompress_frame(const uint8_t* data, size_t len);

} // namespace iris

#endif
