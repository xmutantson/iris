#ifndef IRIS_OFDM_MOD_H
#define IRIS_OFDM_MOD_H

#include "ofdm/ofdm_config.h"
#include "fec/ldpc.h"
#include "native/constellation.h"
#include "native/nuc_tables.h"
#include <vector>
#include <complex>
#include <cstdint>

namespace iris {

// Per-subcarrier bit loading descriptor for OFDM data symbols.
// Either waterfilled (per-carrier SNR adaptive) or uniform preset.
struct ToneMap {
    std::vector<uint8_t> bits_per_carrier;  // one entry per data carrier: 0,1,2,4,6,8
    int n_data_carriers = 0;
    int total_bits_per_symbol = 0;  // sum of bits_per_carrier
    LdpcRate fec_rate = LdpcRate::RATE_1_2;
    int nfft = 512;
    uint8_t tone_map_id = 0;  // 0=waterfill, 1-8=uniform presets
    bool use_nuc = false;     // Use Non-Uniform Constellations (ATSC 3.0 optimized)
    int n_codewords = 1;      // LDPC blocks per frame (>1 = multi-codeword)
};

// Build a uniform tone map (all data carriers same modulation).
// Preset IDs: 1=BPSK r1/2, 2=QPSK r1/2, 3=QPSK r3/4, 4=16QAM r1/2,
//             5=16QAM r3/4, 6=64QAM r3/4, 7=64QAM r7/8, 8=256QAM r7/8
ToneMap make_uniform_tone_map(uint8_t preset_id, int n_data_carriers, int nfft);

// Modulation enum for bits_per_carrier lookup
Modulation bits_to_modulation(int bpc);

class OfdmModulator {
public:
    explicit OfdmModulator(const OfdmConfig& config);

    // Generate Schmidl-Cox training symbol (identical halves, even subcarriers only)
    // Returns time-domain IQ samples (cp_samples + nfft)
    std::vector<std::complex<float>> generate_training_symbol_1();

    // Generate full channel estimation training symbol (all used subcarriers = +1)
    // Returns time-domain IQ samples (cp_samples + nfft)
    std::vector<std::complex<float>> generate_training_symbol_2();

    // Generate one OFDM data symbol from per-subcarrier complex symbols.
    // freq_symbols: n_used_carriers complex values (pilots + data interleaved)
    std::vector<std::complex<float>> generate_data_symbol(
        const std::vector<std::complex<float>>& freq_symbols);

    // Generate a block-pilot symbol (all used subcarriers = known +1)
    std::vector<std::complex<float>> generate_pilot_symbol();

    // Build complete OFDM frame from payload bytes.
    // n_codewords: number of LDPC blocks (1 = legacy, >1 = multi-codeword).
    // Returns baseband IQ samples (complex) ready for upconversion.
    std::vector<std::complex<float>> build_ofdm_frame(
        const uint8_t* payload, size_t len,
        const ToneMap& tone_map, LdpcRate fec,
        int n_codewords = 1);

private:
    OfdmConfig config_;

    // IFFT + CP insertion for one OFDM symbol.
    // freq_bins: nfft-length array with subcarrier values placed at correct bins.
    std::vector<std::complex<float>> symbol_to_time(
        const std::vector<std::complex<float>>& freq_bins);

    // Soft clip the output to reduce PAPR.
    // Returns number of samples clipped.
    int soft_clip(std::vector<std::complex<float>>& samples, float clip_ratio = 3.0f);

    // OFDM header: encode metadata into BPSK bits for 3 header symbols.
    std::vector<uint8_t> encode_ofdm_header(uint8_t tone_map_id, LdpcRate fec,
                                             uint16_t payload_len, int nfft_mode,
                                             bool harq_flag);
};

} // namespace iris

#endif // IRIS_OFDM_MOD_H
