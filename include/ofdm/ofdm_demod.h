#ifndef IRIS_OFDM_DEMOD_H
#define IRIS_OFDM_DEMOD_H

#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_sync.h"
#include "ofdm/ofdm_mod.h"    // ToneMap
#include "fec/ldpc.h"
#include <complex>
#include <vector>
#include <cstdint>

namespace iris {

// Result of OFDM frame demodulation
struct OfdmDemodResult {
    bool success = false;          // CRC passed
    std::vector<uint8_t> payload;  // Decoded payload bytes (without CRC or length prefix)

    // Frame parameters (from pre-negotiated tone_map, not a header)
    LdpcRate fec_rate = LdpcRate::RATE_1_2;
    uint16_t payload_len = 0;      // Extracted from 2-byte length prefix after LDPC decode

    // Diagnostics
    float snr_db = 0;             // From sync
    float cfo_hz = 0;             // Estimated CFO
    float mean_channel_snr_db = 0; // From channel estimation
    float mean_H_mag = 0;         // Mean |H| (quality gate diagnostic)
    int n_data_symbols = 0;
    int n_ldpc_blocks = 0;
    int worst_ldpc_iters = 0;      // Worst-case LDPC iterations across all blocks
    int samples_consumed = 0;      // Total samples used by this frame

    // Per-carrier SNR (for waterfilling feedback)
    std::vector<float> snr_per_carrier;

    // Equalized constellation points (for GUI scatter plot)
    std::vector<std::complex<float>> eq_constellation;

    // Soft LLRs (for HARQ Chase combining)
    std::vector<float> llrs;

    // Per-coded-bit phase variance (for HARQ region selection)
    std::vector<float> sym_phase_var;

    // Per-LDPC-block decode results (for HARQ selective retransmit)
    std::vector<LdpcCodec::BlockResult> block_results;
};

class OfdmDemodulator {
public:
    explicit OfdmDemodulator(const OfdmConfig& config);

    // Main entry: demodulate one OFDM frame from baseband IQ.
    // tone_map: REQUIRED — pre-negotiated config (no per-frame header).
    // pre_sync: if non-null, skip internal Schmidl-Cox detection and use this.
    OfdmDemodResult demodulate(const std::complex<float>* iq, int n_samples,
                                const ToneMap& tone_map,
                                const OfdmSyncResult* pre_sync = nullptr);

    // Get the last channel estimate (for waterfilling updates)
    const OfdmChannelEst& last_channel_estimate() const { return channel_est_; }

private:
    OfdmConfig config_;
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
                       std::vector<float>& llrs);
};

} // namespace iris
#endif
