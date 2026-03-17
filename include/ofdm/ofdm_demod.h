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
    std::vector<uint8_t> payload;  // Decoded payload bytes (without CRC)

    // Header fields
    uint8_t tone_map_id = 0;
    LdpcRate fec_rate = LdpcRate::RATE_1_2;
    uint16_t payload_len = 0;
    int nfft_mode = 0;
    bool harq_flag = false;

    // Diagnostics
    float snr_db = 0;             // From sync
    float cfo_hz = 0;             // Estimated CFO
    float mean_channel_snr_db = 0; // From channel estimation
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
    // Maps per-carrier noise_var/|H|² to each coded bit position.
    std::vector<float> sym_phase_var;

    // Per-LDPC-block decode results (for HARQ selective retransmit)
    std::vector<LdpcCodec::BlockResult> block_results;
};

class OfdmDemodulator {
public:
    explicit OfdmDemodulator(const OfdmConfig& config);

    // Main entry: demodulate one OFDM frame from baseband IQ.
    // iq points to the start of the search window.
    // n_samples is the number of available samples.
    // tone_map: if non-null, use this tone map (from negotiation).
    //           if null, decode header to determine tone map.
    // pre_sync: if non-null, skip internal Schmidl-Cox detection and use this.
    OfdmDemodResult demodulate(const std::complex<float>* iq, int n_samples,
                                const ToneMap* tone_map = nullptr,
                                const OfdmSyncResult* pre_sync = nullptr);

    // Get the last channel estimate (for waterfilling updates)
    const OfdmChannelEst& last_channel_estimate() const { return channel_est_; }

private:
    OfdmConfig config_;
    OfdmChannelEst channel_est_;

    // Decode the 3-symbol BPSK header
    bool decode_header(const std::complex<float>* header_freq_symbols,
                       int n_header_symbols,
                       uint8_t& tone_map_id, LdpcRate& fec_rate,
                       uint16_t& payload_len, int& nfft_mode, bool& harq_flag);

    // Extract data carriers from one OFDM symbol (after FFT, skip pilots)
    std::vector<std::complex<float>> extract_data_carriers(
        const std::complex<float>* symbol_freq, int n_used);

    // MMSE equalize data carriers using channel estimate
    std::vector<std::complex<float>> equalize_mmse(
        const std::vector<std::complex<float>>& data_carriers,
        const OfdmChannelEst& est);

    // Demap equalized symbols to soft LLRs per carrier (mixed modulation)
    void demap_to_llrs(const std::vector<std::complex<float>>& eq_carriers,
                       const ToneMap& tone_map,
                       const OfdmChannelEst& est,
                       std::vector<float>& llrs);
};

} // namespace iris
#endif
