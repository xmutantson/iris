#ifndef IRIS_OFDM_DEMOD_H
#define IRIS_OFDM_DEMOD_H

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_mod.h"    // ToneMap
#include "native/frame.h"   // KalmanTrace
#include "fec/ldpc.h"
#include <complex>
#include <vector>
#include <cstdint>

namespace iris {

// Result of OFDM frame demodulation
struct OfdmDemodResult {
    enum class Completion : uint8_t {
        NeedMoreSamples = 0,
        RejectedCandidate,
        CompleteValidatedFrame,
    };

    bool success = false;          // CRC passed
    Completion completion = Completion::RejectedCandidate;
    bool complete_boundary_validated = false;
    bool sync_agreement = false;
    bool payload_validated = false;
    bool cfo_resolved = false;
    OfdmEstimatorValidity estimator_validity =
        OfdmEstimatorValidity::Unavailable;
    std::vector<uint8_t> payload;  // Decoded payload bytes (without CRC or length prefix)

    // Frame parameters (from pre-negotiated tone_map, not a header)
    LdpcRate fec_rate = LdpcRate::RATE_1_2;
    uint16_t payload_len = 0;      // Extracted from 2-byte length prefix after LDPC decode
    int demodulated_level = -1;    // Uniform-map O-level actually demodulated
    int demodulated_n_codewords = 0;

    // Diagnostics
    float snr_db = 0;             // From sync
    float cfo_hz = 0;             // Estimated CFO
    float cpe_drift_hz = 0;       // Residual CFO from CPE slope feedback
    float mean_channel_snr_db = 0; // From channel estimation
    float effective_snr_db = 0;    // Harmonic mean SNR from dft_sigma_sq (for gearshift)
    float payload_residual_snr_db = 0; // Independent post-EQ decision residual
    bool payload_residual_snr_valid = false;
    float dft_sigma_sq_llr = 0;    // The LLR-scaling sigma^2 used by soft-demap (mu_avg = 1 - this)
    float mean_H_mag = 0;         // Mean |H| (quality gate diagnostic)
    int n_data_symbols = 0;
    int n_ldpc_blocks = 0;
    int worst_ldpc_iters = 0;      // Worst-case LDPC iterations across all blocks
    // Input-relative exclusive end of the decoded frame.  This already includes
    // frame_start; callers must never add a sync/timing offset again.
    int consumed_from_input_start = 0;
    std::uint64_t additional_samples_required = 0;
    float pair_coherence = 0;

    // Per-carrier SNR (for waterfilling feedback)
    std::vector<float> snr_per_carrier;

    // Trial-owned channel estimate.  Live hypothesis search never publishes to
    // the demodulator's persistent last-estimate cache; commit chooses exactly
    // one result and publishes/consumes this matching snapshot.
    OfdmChannelEst channel_estimate;

    // Equalized constellation points (for GUI scatter plot)
    std::vector<std::complex<float>> eq_constellation;

    // Soft LLRs (for HARQ Chase combining)
    std::vector<float> llrs;

    // Per-coded-bit phase variance (for HARQ region selection)
    std::vector<float> sym_phase_var;

    // Per-LDPC-block decode results (for HARQ selective retransmit)
    std::vector<LdpcCodec::BlockResult> block_results;

    // Kalman filter trace (for GUI 3D viewer and CSV logging)
    KalmanTrace kalman_trace;
};

// Complete payload validation resolves a CFO hypothesis, but it authorizes
// training-derived SNR only when an independent post-EQ payload residual
// cross-checks every estimate that may reach rate control or meters.
void ofdm_authorize_payload_estimator(OfdmDemodResult& result);

class OfdmDemodulator {
public:
    explicit OfdmDemodulator(const OfdmConfig& config);

    // Main entry: demodulate one OFDM frame from baseband IQ.
    // tone_map: REQUIRED — pre-negotiated config (no per-frame header).
    // pre_sync: if non-null, skip internal Schmidl-Cox detection and use this.
    // genie_H: DIAGNOSTIC control only (offline genie-H fork). If non-null and
    //   sized == n_used_carriers, the preamble-derived channel estimate is
    //   OVERRIDDEN with this true per-carrier channel vector right after the
    //   preamble-to-data phase cal — bypassing channel-ESTIMATION error while
    //   keeping the SAME EQ/LLR/FEC/phase-tracking. Used by the offline replay
    //   harness to prove whether the estimator (vs the EQ/diversity) is the
    //   binding term on the MPG freq-selective boss. Never set on the live path.
    OfdmDemodResult demodulate(const std::complex<float>* iq, int n_samples,
                                const ToneMap& tone_map,
                                const OfdmSyncResult* pre_sync = nullptr,
                                const std::vector<std::complex<float>>* genie_H = nullptr);

    // Live search entry: selected hypotheses are trial-local and may not mutate
    // the persistent last-channel cache.  The returned result owns its estimate.
    OfdmDemodResult demodulate_trial(
        const std::complex<float>* iq, int n_samples, const ToneMap& tone_map,
        const OfdmSyncResult& selected_sync);

    // Single post-search publication of the committed result's owned snapshot.
    void commit_channel_estimate(const OfdmChannelEst& estimate) {
        channel_est_ = estimate;
    }

    // Get the last channel estimate (for waterfilling updates)
    const OfdmChannelEst& last_channel_estimate() const { return channel_est_; }

    // Reassemble a frame payload from LDPC-decoded data bits. Each block is
    // [2B len][chunk][4B CRC-32] padded to the code's k data bits; a frame is
    // 1..n concatenated blocks. Single source shared by the main decode path
    // and the Chase-combining re-decode (engine/modem.cc) — the Chase path
    // previously reconstructed with single-block logic, which on a multi-block
    // frame CRC-validates block 0 alone and returns a TRUNCATED payload as
    // success. Returns true and fills payload_out only if every block's
    // length is in range and its CRC-32 validates.
    static bool extract_payload_blocks(const std::vector<uint8_t>& decoded_bits,
                                       LdpcRate rate,
                                       int expected_codeword_count,
                                       size_t exact_information_bit_length,
                                       std::vector<uint8_t>& payload_out);

private:
    OfdmDemodResult demodulate_selected(
        const std::complex<float>* iq, int n_samples, const ToneMap& tone_map,
        const OfdmSyncResult& selected_sync,
        const std::vector<std::complex<float>>* genie_H);

    OfdmConfig config_;
    OfdmSyncWorkspace sync_workspace_;
    OfdmChannelEst channel_est_;

    // Extract data carriers from one OFDM symbol (after FFT, skip pilots)
    // Writes into pre-allocated output buffer, returns count of data carriers written.
    int extract_data_carriers(const std::complex<float>* symbol_freq, int n_used,
                              std::vector<std::complex<float>>& out);

    // MMSE equalize data carriers using channel estimate (writes into out)
    void equalize_mmse(const std::vector<std::complex<float>>& data_carriers, int n_data,
                       const OfdmChannelEst& est,
                       std::vector<std::complex<float>>& out);

    // Demap equalized symbols to soft LLRs per carrier (mixed modulation)
    void demap_to_llrs(const std::vector<std::complex<float>>& eq_carriers, int n_data,
                       const ToneMap& tone_map,
                       const OfdmChannelEst& est,
                       std::vector<float>& llrs,
                       float& dft_sigma_sq_out,
                       float& metric_sigma_sq_out);
};

// S2 continuous per-carrier channel-tracking scope predicate (single source of
// truth for ofdm_demod.cc's `s2_scope`). True for QPSK through 64QAM
// (max_bpc 2-6): 16QAM+ use per-symbol BPS so they are immune to the S1
// per-carrier phase fold; QPSK (E5) relies on the lindet detrend for the same
// safety and NEEDS tracking most — a preamble-frozen H is what fails the QPSK
// rungs under a moving multipath notch, and it is the error floor at threshold
// AWGN SNR. BPSK (bpc 1) stays frozen (no per-symbol loop, robust-rung role);
// 256QAM+ (bpc>=8) stays frozen: its tiny decision regions cannot absorb
// per-carrier EMA noise and it is not on the supported burst path yet.
// Exposed so the scope contract is unit-tested.
bool ofdm_s2_track_in_scope(int max_bpc);

} // namespace iris
#endif
