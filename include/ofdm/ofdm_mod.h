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
    std::vector<uint8_t> bits_per_carrier;  // one per data carrier: 0,1,2,4,5,6,8,10
    int n_data_carriers = 0;
    int total_bits_per_symbol = 0;  // sum of bits_per_carrier
    LdpcRate fec_rate = LdpcRate::RATE_1_2;
    int nfft = 1024;
    uint8_t tone_map_id = 0;  // 0=waterfill, 1-14=uniform presets
    bool use_nuc = false;     // Use Non-Uniform Constellations (ATSC 3.0 optimized)
    int n_codewords = 1;      // LDPC blocks per frame (>1 = multi-codeword)
};

// Build a uniform tone map (all data carriers same modulation).
// Preset IDs: 1=BPSK r1/2, 2=QPSK r1/2, 3=QPSK r3/4, 4=16QAM r1/2,
//   5=16QAM r5/8, 6=16QAM r3/4, 7=32QAM r5/8, 8=64QAM r5/8, 9=64QAM r3/4,
//   10=256QAM r5/8, 11=256QAM r3/4, 12=256QAM r7/8, 13=1024QAM r3/4,
//   14=1024QAM r7/8. (preset_id = O-level + 1; keep in sync with
//   kUniformPresets[] in ofdm_frame.cc.)
ToneMap make_uniform_tone_map(uint8_t preset_id, int n_data_carriers, int nfft);

// Modulation enum for bits_per_carrier lookup
Modulation bits_to_modulation(int bpc);

class OfdmModulator {
public:
    explicit OfdmModulator(const OfdmConfig& config);

    // Generate one OFDM data symbol from per-subcarrier complex symbols.
    // freq_symbols: n_used_carriers complex values (pilots + data interleaved)
    std::vector<std::complex<float>> generate_data_symbol(
        const std::vector<std::complex<float>>& freq_symbols);

    // Generate the known reference symbol used for pilot rows and the leading
    // noise symbol: constant-modulus Zadoff-Chu
    // phases (root = ofdm_pilot_zc_root(n_used), distinct from the preamble
    // root) x the TX de-emphasis gain on every used carrier. Constant
    // modulus keeps H[k] = Y[k]/X[k] well-conditioned; the pseudorandom ZC
    // phase keeps the symbol's PAPR near the data symbols' instead of the
    // 10*log10(n_used) impulse an all-ones comb produces (which an FM
    // deviation limiter clips first). The demodulator derotates pilot-row
    // observations by conj(ZC) before the CPE/H updates — both ends derive
    // the sequence from the shared ofdm_pilot_zc_root rule.
    // Returns time-domain IQ samples (cp_samples + nfft).
    std::vector<std::complex<float>> generate_pilot_symbol();

    // Build complete OFDM frame from payload bytes.
    // ToneMap is the sole owner of FEC and codeword count.
    // Returns baseband IQ samples (complex) ready for upconversion.
    std::vector<std::complex<float>> build_ofdm_frame(
        const uint8_t* payload, size_t len, const ToneMap& tone_map);

    // Compatibility boundaries for older callers. The historical four-argument
    // form means one codeword and records that choice in the caller's map before
    // serialization. Redundant five-argument values are assertions only.
    std::vector<std::complex<float>> build_ofdm_frame(
        const uint8_t* payload, size_t len,
        ToneMap& tone_map, LdpcRate fec);
    std::vector<std::complex<float>> build_ofdm_frame(
        const uint8_t* payload, size_t len,
        const ToneMap& tone_map, LdpcRate fec);
    std::vector<std::complex<float>> build_ofdm_frame(
        const uint8_t* payload, size_t len,
        const ToneMap& tone_map, LdpcRate fec,
        int n_codewords);

private:
    OfdmConfig config_;

    // IFFT + CP insertion for one OFDM symbol.
    // freq_bins: nfft-length array with subcarrier values placed at correct bins.
    std::vector<std::complex<float>> symbol_to_time(
        const std::vector<std::complex<float>>& freq_bins);

    // Soft clip the output to reduce PAPR.
    // Returns number of samples clipped.
    int soft_clip(std::vector<std::complex<float>>& samples, float clip_ratio = 3.0f);

    // OFDM header: encode metadata into BPSK bits (currently disabled, n_header_symbols=0).
    std::vector<uint8_t> encode_ofdm_header(uint8_t tone_map_id, LdpcRate fec,
                                             uint16_t payload_len, int nfft_mode,
                                             bool harq_flag);
};

} // namespace iris

#endif // IRIS_OFDM_MOD_H
