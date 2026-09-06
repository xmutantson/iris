#ifndef IRIS_OFDM_SYNC_H
#define IRIS_OFDM_SYNC_H

#include "ofdm/ofdm_config.h"
#include <cstdint>
#include <complex>
#include <deque>
#include <limits>
#include <vector>

namespace iris {

enum class OfdmSyncStatus : std::uint8_t {
    NoCandidate = 0,
    NeedContext,
    Ready,
    Rejected,
};

enum class OfdmEstimatorValidity : std::uint8_t {
    Unavailable = 0,
    CoherentPairUnresolved,
    PayloadValidatedForSelectedCfo,
    PayloadValidatedEstimateWithheld,
};

// Result of frame detection
struct OfdmSyncResult {
    OfdmSyncStatus status = OfdmSyncStatus::NoCandidate;
    bool detected = false;
    bool candidate_present = false; // Chronological SC region was examined
    int frame_start = -1;        // Sample offset of first training symbol CP start
    float schmidl_metric = 0;    // Peak metric value [0,1] (Schmidl-Cox autocorrelation)
    // The repeated training pair measures CFO modulo Fs/(NFFT+CP).  Detection
    // populates cfo_principal_hz and leaves cfo_resolved false.  cfo_hz is the
    // current trial value; it is a publishable total only after complete frame
    // validation sets cfo_resolved.
    float cfo_principal_hz = 0;
    float cfo_hz = 0;
    float cfo_alias_spacing_hz = 0;
    float cfo_training_metric = 0;
    int cfo_ambiguity_index = 0;
    bool cfo_hypothesis_selected = false;
    bool cfo_resolved = false;
    float pair_coherence = 0;
    OfdmEstimatorValidity estimator_validity =
        OfdmEstimatorValidity::Unavailable;
    float snr_est = 0;           // SNR estimate from training symbols (dB)
    float zc_metric = 0;         // ZC cross-correlation peak [0,1]
    float sc_metric = 0;         // Schmidl-Cox autocorrelation peak [0,1]
    float sc_peak_raw = 0;       // Raw squared SC metric M=|P|^2/(A*R) at best d,
                                 // ALWAYS set (even on a no-detect) for diagnostics.
    int timing_interval_begin = -1; // Inclusive alternatives in this SC region
    int timing_interval_end = -1;   // Inclusive alternatives in this SC region
    int candidate_support_begin = -1; // Settling + repeated-training support
    int candidate_support_end = -1;   // Exclusive rejected-region boundary
    int required_right_context = 0;   // Exclusive extent used to commit timing
    int search_examined_through = -1; // Last timing offset examined; -1 if none
};

// Instance-owned detector scratch/reference cache.  The fingerprint used by
// the implementation includes every input that changes the training waveform
// (FFT/CP/rate, carrier-bin layout and FM pre-emphasis).  Keeping this on the
// receiver avoids process-global mutable DSP scratch and cross-receiver cache
// aliasing.
struct OfdmSyncWorkspace {
    std::uint64_t waveform_fingerprint = 0;
    std::vector<std::complex<float>> zc_td;
    std::vector<std::complex<float>> fft_train1;
    std::vector<std::complex<float>> fft_train2;
    std::vector<float> sc_profile;
    std::vector<float> timing_profile;
    float zc_energy = 0.0f;
    std::uint64_t no_detection_count = 0;

    void reset();
};

enum class OfdmAcquisitionStatus : std::uint8_t {
    NeedContext = 0,
    Ready,
    Rejected,
    Validated,
};

struct OfdmIncompleteShapeWork {
    std::size_t cursor = 0;
    std::uint64_t required_right_context = 0;
};

// Absolute, receiver-owned work item.  Timing quality is deliberately kept
// separate from validation state: a later high-quality item cannot overtake an
// unresolved earlier one.
struct OfdmAcquisitionCandidate {
    std::uint64_t id = 0;
    std::uint64_t capture_epoch = 0;
    std::uint64_t absolute_start = 0;
    std::uint64_t timing_begin = 0;
    std::uint64_t timing_end = 0;
    std::uint64_t support_begin = 0;
    std::uint64_t support_end = 0;
    std::uint64_t required_right_context = 0;
    std::size_t cfo_trial_cursor = 0;
    std::size_t shape_trial_cursor = 0;
    std::size_t next_untried_shape_cursor = 0;
    std::size_t earliest_incomplete_shape_cursor =
        std::numeric_limits<std::size_t>::max();
    std::uint64_t incomplete_shape_required_right_context = 0;
    std::vector<OfdmIncompleteShapeWork> incomplete_shape_work;
    OfdmAcquisitionStatus status = OfdmAcquisitionStatus::NeedContext;
    OfdmSyncResult sync;
};

// Bookkeeping for chronological, resumable acquisition.  DSP discovery is
// performed by ofdm_detect_frame; this class owns absolute stream identity,
// unresolved work/cursors, and the only safe retirement watermark.
class OfdmAcquisitionState {
public:
    static constexpr std::size_t kMaxRetainedCandidates = 64;

    OfdmAcquisitionState();
    void reset(std::uint64_t new_buffer_origin = 0,
               bool advance_capture_epoch = true);
    std::uint64_t capture_epoch() const { return capture_epoch_; }
    std::uint64_t buffer_origin() const { return buffer_origin_; }
    std::uint64_t searched_watermark() const { return searched_watermark_; }
    const std::deque<OfdmAcquisitionCandidate>& candidates() const {
        return candidates_;
    }

    void note_searched_through(std::uint64_t absolute_sample);
    // Returns null when the finite queue is full.  The caller must yield while
    // retaining its buffer; it must never discard unrecorded work.
    OfdmAcquisitionCandidate* remember(const OfdmSyncResult& sync);
    OfdmAcquisitionCandidate* earliest_unresolved();
    OfdmAcquisitionCandidate* find(std::uint64_t id);
    void require_context(std::uint64_t id, std::uint64_t absolute_end);
    void mark_ready(std::uint64_t id);
    void mark_rejected(std::uint64_t id);
    void mark_validated(std::uint64_t id);
    void set_trial_cursors(std::uint64_t id, std::size_t cfo_cursor,
                           std::size_t shape_cursor);
    void advance_cfo_trial(std::uint64_t id, std::size_t cfo_cursor);
    void remember_incomplete_shape(std::uint64_t id, std::size_t shape_cursor,
                                   std::uint64_t absolute_end);
    void resolve_shape_trial(std::uint64_t id, std::size_t shape_cursor);

    std::uint64_t safe_retirement_watermark(std::uint64_t frontend_overlap) const;
    void retire_prefix(std::uint64_t absolute_end);

private:
    std::uint64_t capture_epoch_ = 1;
    std::uint64_t buffer_origin_ = 0;
    std::uint64_t searched_watermark_ = 0;
    std::uint64_t next_candidate_id_ = 1;
    std::deque<OfdmAcquisitionCandidate> candidates_;
};

// ---------------------------------------------------------------------------
// Zadoff-Chu preamble generation (shared by TX and RX)
// ---------------------------------------------------------------------------

// Generate a Zadoff-Chu sequence of the given length.
// ZC: x[n] = exp(-j * pi * root * n * (n+1) / length), n = 0..length-1.
// Root must be coprime to length. Default root=7 works well for length~50-100.
std::vector<std::complex<float>> generate_zc_sequence(int root, int length);

// Generate a time-domain ZC training symbol for the given OFDM config.
// Steps:
//   1. Generate ZC sequence of length n_used_carriers
//   2. Place into correct FFT bins (config.used_carrier_bins)
//   3. IFFT to get nfft time-domain samples
//   4. Scale by nfft to match data symbol amplitude
// Returns nfft samples WITHOUT cyclic prefix (caller adds CP).
std::vector<std::complex<float>> generate_zc_training_symbol(const OfdmConfig& config,
                                                              int root = 7);

// ZC root for the pilot-row / noise reference symbol. Shared by the
// modulator (symbol synthesis), the demodulator (pilot-row derotation) and
// the ACE/ICF pilot restore — ONE derivation, both ends.
//
// The pilot reference must be constant-modulus with pseudorandom phase: the
// least-squares channel estimate divides by X[k], so |X[k]| = const keeps it
// well-conditioned, and a zero-phase comb (the old all-ones pilot) is a time-
// domain impulse with PAPR ~ 10*log10(n_used) that an FM deviation limiter
// clips first. Every deployed OFDM system does the same: 802.11 LTF uses a
// fixed low-PAPR sequence (IEEE 802.11-2020 17.3.3), LTE UL DMRS uses
// Zadoff-Chu (TS 36.211 5.5.1), DVB-T pilots are PRBS-modulated
// (EN 300 744 4.5.2).
//
// Rule: first r >= max(2, (7*n_used+50)/100) with gcd(r, n_used) == 1 and
// r != 7 (7 is the preamble root; a distinct root keeps the pilot rows out
// of the preamble's ZC timing/FD-ZC correlators). INTEGER-ONLY on purpose:
// selecting the root by a runtime float argmin over candidate PAPRs would
// make the wire format depend on platform rounding (x86 vs ARM, fast-math),
// and conjugate root pairs tie exactly — the two ends could pick different
// roots, a split-brain the config fingerprint cannot catch. Offline study
// (nfft=1024, 48 kHz, de-emphasis corner 300 Hz cap 10): narrow n_used=57 ->
// root 4, symbol PAPR 6.6 dB; wide6k n_used=127 -> root 9, 7.3 dB; worst
// case over n_used 40..140 and start bins 5..15: 9.7 dB. The old all-ones
// symbol measures 19.2 / 22.2 dB on the same grids.
int ofdm_pilot_zc_root(int n_used);

// Codeword-count-bound tail reference. Each legal C selects a distinct ZC root,
// also distinct from the preamble and dense-pilot roots. This makes the final
// known symbol an in-band declaration of both the candidate boundary and C.
// Returns zero when the carrier count cannot represent every legal C=1..8.
int ofdm_tail_zc_root(int n_used, int n_codewords);

// Detect OFDM frame using hybrid Schmidl-Cox + ZC detection.
// Schmidl-Cox autocorrelation (train1 vs train2) for channel-invariant detection,
// then ZC cross-correlation for precise timing refinement.
// Returns detection result with timing, CFO, and SNR estimates.
OfdmSyncResult ofdm_detect_frame(const std::complex<float>* iq, int n_samples,
                                  const OfdmConfig& config,
                                  OfdmSyncWorkspace* workspace = nullptr);

// Build the finite ambiguity bank for one detected time candidate.  Every
// integer m for which principal + m*Fs/(NFFT+CP) lies inside +/-150 Hz plus a
// 5 Hz estimator-uncertainty margin is retained.  Each returned trial carries
// timing refined against CFO-corrected known training, but remains unresolved
// until complete LDPC/per-block CRC and boundary validation succeeds.
std::vector<OfdmSyncResult> ofdm_cfo_hypotheses(
    const std::complex<float>* iq, int n_samples, const OfdmConfig& config,
    const OfdmSyncResult& principal_sync);

// Refine one bank entry against the CFO-corrected known training pair.  Kept
// separate so a bounded live scheduler pays for only its current cursor entry.
OfdmSyncResult ofdm_refine_cfo_hypothesis(
    const std::complex<float>* iq, int n_samples, const OfdmConfig& config,
    const OfdmSyncResult& hypothesis);

// Correct CFO on a block of IQ samples (in-place rotation).
// For each sample n: iq[n] *= exp(-j * 2*pi * cfo_hz * n / sample_rate)
void ofdm_correct_cfo(std::complex<float>* iq, int n_samples,
                       float cfo_hz, int sample_rate);

// Channel estimation result
struct OfdmChannelEst {
    std::vector<std::complex<float>> H;      // Channel response per used carrier
    std::vector<float> snr_per_carrier;       // SNR estimate per used carrier (linear)
    std::vector<float> noise_var;             // Noise variance per used carrier (for MMSE)
    float mean_snr_db = 0;                    // Average SNR across carriers
    // AWGN noise floor measured from guard/out-of-band FFT bins. Used by the
    // post-EQ gearshift metric (effective_snr_db) as a bias-free per-frame
    // noise estimate. Independent of the per-carrier noise_var above (which
    // can be confused by H(f) shape on frequency-selective channels).
    float noise_var_frame = 1e-6f;
};

// Estimate channel from training symbol (ZC-based).
// iq_symbol points to the start of the training symbol (after CP removal).
// Performs NFFT-point FFT, divides by known ZC frequency-domain sequence,
// estimates H and noise.
OfdmChannelEst ofdm_estimate_channel(const std::complex<float>* iq_symbol,
                                      const OfdmConfig& config);

// Per-carrier noise variance sigma^2(k) from the two (identical) preamble
// training symbols: after removing the measured common inter-symbol rotation,
// Y2 - Y1 at each used bin is pure noise with E|diff|^2 = 2 sigma^2(k).
// Frequency-smoothed (boxcar) because every physical noise-coloring mechanism
// (triangular FM discriminator noise, de-emphasis, IF shaping) is smooth in f.
// Y1/Y2 are the full nfft-point FFTs of the two training symbol bodies taken
// from the SAME CFO-corrected buffer. Returns TRUE sigma^2 on the received-
// audio scale (no preamble-boost compensation applies — see the definition).
std::vector<float> ofdm_noise_from_training_pair(
    const std::complex<float>* Y1, const std::complex<float>* Y2,
    const OfdmConfig& config);

// Fine CFO estimation from training symbol.
// After coarse CFO correction, the channel estimate H[k] for each used
// subcarrier should align with the known ZC phase. Any residual CFO
// manifests as a common phase rotation across subcarriers.
// Returns residual CFO in Hz (add to coarse CFO for total correction).
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config);

// ---------------------------------------------------------------------------
// RX front-end: analytic (Hilbert) conversion + in-band selection
// ---------------------------------------------------------------------------
// Edge behavior of the RX band-limit (see ofdm_analytic_bandlimit):
//   OFF       — plain Hilbert transform, no band-limit (byte-identical to the
//               historical IRIS_RX_BANDLIMIT=0 path: DC left intact).
//   BRICKWALL — hard 0/1 bin mask at the used band ± 1 carrier (the legacy
//               zero-margin edge; byte-identical to the pre-taper live path).
//               Kept for A/B diagnosis only: its sinc kernel rings across the
//               capture buffer and installs a ~42 dB training-pair self-noise
//               floor that caps the SNR meter's top end.
//   TAPERED   — raised-cosine transition band + odd-reflected, tapered guard
//               extensions (discarded after filtering). The shipping edge.
enum class RxBandlimitEdge { OFF, BRICKWALL, TAPERED };

// Convert a real audio capture buffer to the analytic signal, band-limited to
// the negotiated OFDM band. Returns exactly n analytic samples aligned 1:1
// with the input (guard extensions are internal and discarded).
// trans_lo/hi_carriers: raised-cosine transition width, in carrier spacings,
// below/above the flat band edges (TAPERED only). guard_samples: reflected
// guard extension length per side (TAPERED only). trans_profile_pow: exponent
// on the raised-cosine profile (1 = Hann-shaped roll-off, tails ~1/t^3;
// 2 = squared, C1-smooth at both transition ends, tails ~1/t^5 — matters on
// the narrow grid where DC caps the low-side width). The low-side flat
// extension includes a quarter-carrier calibration shoulder for the current
// carrier geometry. Defaults are the shipping values; they are parameters so
// tests can sweep them.
std::vector<std::complex<float>> ofdm_analytic_bandlimit(
    const float* audio, int n, const OfdmConfig& cfg, RxBandlimitEdge edge,
    float trans_lo_carriers = 3.0f, float trans_hi_carriers = 8.0f,
    int guard_samples = 1024, float trans_profile_pow = 2.0f,
    float lo_flat_ext_carriers = 3.25f);

} // namespace iris

#endif // IRIS_OFDM_SYNC_H
