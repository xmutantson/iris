#ifndef IRIS_FRAME_H
#define IRIS_FRAME_H

#include "native/phy.h"
#include "fec/ldpc.h"
#include "common/types.h"
#include <vector>
#include <cstdint>
#include <complex>

namespace iris {

// Kalman filter state trace for GUI visualization
// Downsampled to max ~512 points for display
struct KalmanTracePoint {
    float phase;    // radians
    float freq;     // rad/symbol
    float accel;    // rad/symbol²
    bool is_pilot;  // true if this symbol was a pilot measurement
};

struct KalmanTrace {
    std::vector<KalmanTracePoint> fwd;       // forward Kalman estimates
    std::vector<KalmanTracePoint> smoothed;  // RTS smoothed estimates
    int total_symbols = 0;                   // original symbol count before downsampling
    int downsample_factor = 1;
};

// CRC-32 for native frames
uint32_t crc32(const uint8_t* data, size_t len);

// Frame structure:
//   [Preamble: 63-symbol m-sequence (BPSK)]
//   [Sync word: 16-symbol Barker-like (BPSK)]
//   [Header: 32 bits BPSK — modulation, payload_len, fec_rate(4b), reserved(4b), header_crc8]
//   [Payload: N symbols at configured modulation, LDPC-protected]
//     Known BPSK pilots inserted every PILOT_SPACING symbols (QAM16+ always, BPSK/QPSK on long frames)
//   [CRC-32: 32 bits appended to payload before FEC encoding]

// Mid-frame pilot spacing (symbols between pilots, 0 = disabled)
// Pilots are known +1 BPSK symbols inserted after constellation mapping.
// BPSK/QPSK uses spacing 8 (~12.5% overhead) because VV4 second-pass
// phase correction fails on FM channels (all variants tested OTA), so
// these modulations rely entirely on pilot-only Kalman + RTS smoother.
// Denser pilots halve the interpolation midpoint phase error variance
// (σ²_mid ≈ D·S/4), giving ~3 dB effective phase-noise improvement.
// QAM16+ keeps spacing 16 (~6% overhead) since it uses DD second pass.
constexpr int PILOT_SPACING_QAM = 16;   // QAM16 and above: 1 pilot every 16 symbols (~6% overhead)
constexpr int PILOT_SPACING_LOW = 8;    // BPSK/QPSK: 1 pilot every 8 symbols (~12.5% overhead)

// Returns the pilot spacing for the given modulation
inline int pilot_spacing_for(int mod_int) {
    return (mod_int >= 2) ? PILOT_SPACING_QAM : PILOT_SPACING_LOW;  // mod>=2 is QAM16+
}

// Legacy constant for backward reference
constexpr int PILOT_SPACING = 16;  // default (widest; PILOT_SPACING_LOW=8 is tighter)

// Tail pilots: known +1 BPSK symbols appended after the last data symbol.
// These give the RTS backward smoother measurement data at the frame tail,
// fixing the asymmetry where smoothed[N-1] == forward[N-1] (no future data).
// Without these, forward Kalman drift at the tail propagates uncorrected,
// causing the last LDPC block to fail on long frames (8000+ symbols).
constexpr int N_TAIL_PILOTS = 4;

// Preamble m-sequence (length 63, LFSR polynomial x^6+x+1)
std::vector<std::complex<float>> generate_preamble();

// 16-symbol sync word
std::vector<std::complex<float>> generate_sync_word();

// Header: encodes modulation type, payload byte count, and FEC rate
// Format: [4 bits modulation][12 bits payload_len][4 bits fec_rate][4 bits reserved][8 bits CRC-8]
std::vector<uint8_t> encode_header(Modulation mod, uint16_t payload_len, LdpcRate fec = LdpcRate::NONE, bool harq_flag = false);
bool decode_header(const std::vector<uint8_t>& bits, Modulation& mod, uint16_t& payload_len, LdpcRate& fec, bool& harq_flag);

// Build a complete native frame from payload bytes
// Returns IQ samples ready for transmission
std::vector<float> build_native_frame(const uint8_t* payload, size_t len,
                                       const PhyConfig& config,
                                       LdpcRate fec = LdpcRate::RATE_1_2);

// Frame detector: correlate against preamble/sync to find frame start
// Returns sample offset of frame start, or -1 if not found
int detect_frame_start(const float* iq_samples, size_t count, int sps);

// Get the best normalized correlation from the last detect_frame_start() call
float detect_best_corr();

// Decode a native frame from IQ samples starting at frame_start
// Returns true if frame decoded successfully, fills payload
bool decode_native_frame(const float* iq_samples, size_t count,
                          int frame_start, const PhyConfig& default_config,
                          std::vector<uint8_t>& payload);

// Returns true if last decode_native_frame failed due to buffer overflow
// (not enough IQ data), as opposed to a real CRC/decode error.
bool decode_was_overflow();

// Returns IQ pairs consumed by last successful decode (for buffer drain)
size_t decode_consumed_iq();

// Returns SNR (dB) estimated from last decoded frame's phase-corrected preamble
float decode_snr_db();
float decode_snr_preamble_db();  // Preamble-only SNR (for peer feedback)

// Get the Kalman trace from the last decoded frame (for GUI visualization)
const KalmanTrace& decode_kalman_trace();

// Get channel gain from last decoded frame's preamble (for auto-tune)
float decode_channel_gain();

// Chase combining (CC-HARQ): accumulate LLRs across retransmissions.
// Enable before ARQ retransmit of the same frame; disable/clear on new frame.
void chase_enable();    // Enable Chase combining mode
void chase_disable();   // Disable and clear stored LLRs
void chase_clear();     // Clear stored LLRs (keep enabled)
int chase_combine_count();  // Number of successful combines so far

// --- Per-symbol soft HARQ with piggybacked retransmissions ---

// Region of LDPC codeword bits to retransmit
struct HarqRetxRegion {
    uint8_t block_index;   // LDPC block within original frame (0-based)
    uint16_t bit_start;    // Start bit position within codeword (0-1599)
    uint16_t bit_count;    // Number of bits to retransmit
};

// Retransmit descriptor: packed into payload prefix of HARQ frames
struct HarqRetxDescriptor {
    uint8_t original_seq;                    // ARQ sequence of original failed frame
    std::vector<HarqRetxRegion> regions;     // Which regions to retransmit
    std::vector<uint8_t> retx_bits;          // The actual encoded bits to retransmit
};

// Serialize descriptor + retx bits into bytes for payload prefix
std::vector<uint8_t> serialize_retx_descriptor(const HarqRetxDescriptor& desc);

// Deserialize from payload prefix. Returns bytes consumed, 0 on error.
size_t deserialize_retx_descriptor(const uint8_t* data, size_t len, HarqRetxDescriptor& desc);

// Result of per-block HARQ decode
struct HarqDecodeResult {
    bool any_failed = false;                          // true if at least one block failed
    bool all_failed = false;                          // true if every block failed
    int num_blocks = 0;
    std::vector<LdpcCodec::BlockResult> blocks;       // per-block decode results
    std::vector<float> stored_llrs;                   // full frame LLRs (for combining)
    std::vector<float> sym_phase_var;                 // per-data-symbol P00 (for region selection)
    Modulation mod = Modulation::BPSK;
    LdpcRate fec = LdpcRate::NONE;
    uint16_t payload_len = 0;
    bool harq_flag = false;                           // frame had HARQ retx prefix
    HarqRetxDescriptor retx_desc;                     // parsed retx descriptor (if harq_flag)
    std::vector<uint8_t> new_data;                    // new data portion (if harq_flag)
    std::vector<uint8_t> payload;                     // full reassembled payload (if all blocks OK)
};

// Build a HARQ frame: retx descriptor + retx bits + new data, all FEC-protected
std::vector<float> build_native_frame_harq(const uint8_t* new_payload, size_t new_len,
                                            const HarqRetxDescriptor& retx_desc,
                                            const PhyConfig& config,
                                            LdpcRate fec = LdpcRate::RATE_1_2);

// Decode with per-block results for HARQ (does not use global chase state)
HarqDecodeResult decode_native_frame_harq(const float* iq_samples, size_t count,
                                           int frame_start, const PhyConfig& default_config);

// Get per-data-symbol phase variance from last decode (Kalman P00)
const std::vector<float>& decode_sym_phase_var();

// Was the last decoded frame a HARQ frame (retx prefix present)?
bool decode_last_harq_flag();

} // namespace iris

#endif
