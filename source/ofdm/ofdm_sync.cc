#include "ofdm/ofdm_sync.h"
#include "common/fft.h"
#include "common/logging.h"
#include <cmath>
#include <algorithm>
#include <numeric>
#include <cstdlib>
#include <cstring>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

// ---------------------------------------------------------------------------
// Zadoff-Chu sequence generation
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> generate_zc_sequence(int root, int length)
{
    while (length > 1 && std::gcd(root, length) != 1) {
        ++root;
    }
    std::vector<std::complex<float>> seq(length);
    for (int n = 0; n < length; ++n) {
        // x[n] = exp(-j * pi * root * n * (n+1) / length)
        float phase = -(float)M_PI * root * n * (n + 1) / (float)length;
        seq[n] = std::complex<float>(std::cos(phase), std::sin(phase));
    }
    return seq;
}

// ---------------------------------------------------------------------------
// Generate time-domain ZC training symbol (nfft samples, no CP)
// Applies Hermitian symmetry so the IFFT output is REAL-VALUED.
// This is essential because OFDM audio is real — without Hermitian symmetry,
// only the real part of a complex ZC symbol survives through audio, halving
// the effective power at each used carrier and destroying detection/channel est.
// ---------------------------------------------------------------------------
std::vector<std::complex<float>> generate_zc_training_symbol(const OfdmConfig& config,
                                                              int root)
{
    const int nfft = config.nfft;
    const int n_used = config.n_used_carriers;

    // Generate ZC sequence in frequency domain for used carriers
    auto zc_freq = generate_zc_sequence(root, n_used);

    // Place into FFT bins WITH Hermitian symmetry: X[N-k] = conj(X[k])
    // Used carrier bins are all in the positive-frequency half (< nfft/2),
    // so mirrors land in the upper half with no overlap.
    // TX de-emphasis: attenuate higher carriers so radio pre-emphasis
    // produces a flat signal entering the deviation limiter.  Same curve as
    // data symbols and pilots — ensures channel estimate H[k] matches.
    std::vector<std::complex<float>> X(nfft, std::complex<float>(0.0f, 0.0f));
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        // FM TX de-emphasis gain (real scalar, doesn't break Hermitian symmetry)
        float g = 1.0f;
        if (config.fm_preemph_corner_hz > 0.0f) {
            float fhz = (float)bin * (float)config.sample_rate / (float)nfft;
            g = 1.0f / std::sqrt(1.0f + (fhz / config.fm_preemph_corner_hz)
                                       * (fhz / config.fm_preemph_corner_hz));
            if (g < 1.0f / config.fm_preemph_gain_cap)
                g = 1.0f / config.fm_preemph_gain_cap;
        }
        X[bin] = zc_freq[i] * g;
        // Hermitian mirror for real-valued IFFT output
        if (bin > 0 && bin < nfft / 2) {
            X[nfft - bin] = std::conj(zc_freq[i]) * g;
        }
    }

    // IFFT to time domain — output is real-valued due to Hermitian symmetry
    ifft_complex(X.data(), nfft);

    // Scale by nfft to match data symbol amplitude from symbol_to_time().
    // ifft_complex divides by N, so ×N recovers unit-power-per-carrier scaling.
    // This keeps preamble and data at similar amplitudes (preamble also gets
    // PREAMBLE_BOOST in build_ofdm_frame for detection headroom).
    float scale = (float)nfft;
    for (int n = 0; n < nfft; ++n) {
        X[n] *= scale;
    }

    return X;
}

// ---------------------------------------------------------------------------
// Pilot-row ZC root (see ofdm_sync.h for the derivation + rationale).
// Integer-only so both ends of the link derive the identical root from the
// negotiated n_used_carriers — no float in the wire format.
// ---------------------------------------------------------------------------
int ofdm_pilot_zc_root(int n_used)
{
    if (n_used < 3) return 1;
    auto gcd_int = [](int a, int b) { while (b) { int t = a % b; a = b; b = t; } return a; };
    int start = (7 * n_used + 50) / 100;
    if (start < 2) start = 2;
    for (int r = start; r < n_used; ++r) {
        if (r == 7) continue;  // preamble root — keep pilot rows uncorrelated
        if (gcd_int(r, n_used) == 1) return r;
    }
    return 1;  // unreachable for n_used >= 3 (n_used-1 is always coprime)
}

int ofdm_tail_zc_root(int n_used, int n_codewords)
{
    if (n_used < 3 || n_codewords < 1 || n_codewords > 8) return 0;
    auto gcd_int = [](int a, int b) {
        while (b) { int t = a % b; a = b; b = t; }
        return a;
    };
    const int pilot_root = ofdm_pilot_zc_root(n_used);
    int ordinal = 0;
    int selected_root = 0;
    for (int r = 1; r < n_used; ++r) {
        if (r == 7 || r == pilot_root || gcd_int(r, n_used) != 1) continue;
        ++ordinal;
        if (ordinal == n_codewords) selected_root = r;
    }
    // Do not admit a carrier layout unless every legal codeword count has a
    // distinct tail declaration.  Checking only the requested ordinal would
    // let a narrow layout build C=1 while making the rest of the legal shape
    // enumeration impossible at the receiver.
    return ordinal >= 8 ? selected_root : 0;
}

// ---------------------------------------------------------------------------
// Hybrid Schmidl-Cox + ZC frame detection
// ---------------------------------------------------------------------------
// Schmidl-Cox autocorrelation for DETECTION (channel-invariant):
//   Correlates train1 body with train2 body (identical ZC sequences).
//   Both pass through the same channel, so distortion cancels out.
//   Works regardless of radio frequency response differences.
// ZC cross-correlation for TIMING REFINEMENT (sharp peak):
//   After SC detection, ZC xcorr pinpoints exact frame start within ±cp.
// ---------------------------------------------------------------------------

// SC detection threshold.  SC metric for identical training symbols:
//   M ≈ (SNR/(SNR+1))² → ~0.82 at 10 dB, ~0.64 at 5 dB, ~0.25 at 0 dB.
// Noise-only: M ≈ 1/nfft ≈ 0.002.  Data symbols: ~0.01-0.05 (no repetition).
// SC is a COARSE pre-filter — rejects obvious silence/noise.
// FD-ZC (loose admission 0.70) is the secondary gate. The sync word after the
// preamble is the real discriminator (deterministic bit-check, no threshold).
// Threshold 0.25 passes signals down to ~0 dB SNR.  False triggers that
// pass SC+FD-ZC are caught by the sync word check (random bits → ~50% BER).
static constexpr float SC_DETECTION_THRESHOLD = 0.25f;  // coarse pre-filter only

void OfdmSyncWorkspace::reset()
{
    waveform_fingerprint = 0;
    zc_td.clear();
    fft_train1.clear();
    fft_train2.clear();
    sc_profile.clear();
    timing_profile.clear();
    zc_energy = 0.0f;
    no_detection_count = 0;
}

OfdmAcquisitionState::OfdmAcquisitionState() = default;

void OfdmAcquisitionState::reset(std::uint64_t new_buffer_origin,
                                 bool advance_capture_epoch)
{
    if (advance_capture_epoch) {
        ++capture_epoch_;
        if (capture_epoch_ == 0) capture_epoch_ = 1;
    }
    buffer_origin_ = new_buffer_origin;
    searched_watermark_ = new_buffer_origin;
    candidates_.clear();
    next_candidate_id_ = 1;
}

void OfdmAcquisitionState::note_searched_through(std::uint64_t absolute_sample)
{
    searched_watermark_ = std::max(searched_watermark_, absolute_sample);
}

OfdmAcquisitionCandidate* OfdmAcquisitionState::remember(
    const OfdmSyncResult& sync)
{
    if ((!sync.detected && !sync.candidate_present) || sync.frame_start < 0)
        return nullptr;
    auto absolute = [&](int relative) {
        const std::uint64_t offset =
            static_cast<std::uint64_t>(std::max(0, relative));
        const std::uint64_t limit = std::numeric_limits<std::uint64_t>::max();
        return offset > limit - buffer_origin_ ? limit : buffer_origin_ + offset;
    };
    const std::uint64_t start = absolute(sync.frame_start);
    const std::uint64_t timing_begin = absolute(
        sync.timing_interval_begin >= 0 ? sync.timing_interval_begin
                                        : sync.frame_start);
    const std::uint64_t timing_end = absolute(
        sync.timing_interval_end >= 0 ? sync.timing_interval_end
                                      : sync.frame_start);

    // Re-observation after added right context refines the same absolute timing
    // item.  Keep its cursors/status; never append a duplicate that can starve
    // later work.
    for (auto& candidate : candidates_) {
        if (candidate.capture_epoch != capture_epoch_ ||
            candidate.status == OfdmAcquisitionStatus::Rejected ||
            candidate.status == OfdmAcquisitionStatus::Validated)
            continue;
        const bool timing_overlaps = timing_begin <= candidate.timing_end &&
                                     candidate.timing_begin <= timing_end;
        if (timing_overlaps || start == candidate.absolute_start) {
            candidate.absolute_start = start;
            candidate.timing_begin = std::min(candidate.timing_begin, timing_begin);
            candidate.timing_end = std::max(candidate.timing_end, timing_end);
            candidate.support_begin = std::min(candidate.support_begin, absolute(
                sync.candidate_support_begin >= 0
                    ? sync.candidate_support_begin : sync.frame_start));
            candidate.support_end = std::max(candidate.support_end, absolute(
                sync.candidate_support_end >= 0
                    ? sync.candidate_support_end : sync.frame_start + 1));
            candidate.required_right_context = std::max(
                candidate.required_right_context,
                absolute(sync.required_right_context));
            candidate.sync = sync;
            return &candidate;
        }
    }

    // Resolved records no longer protect samples or represent schedulable
    // work. Drop only those records before applying the finite queue bound;
    // retaining them until their old support end passes could fill the queue
    // while the receiver advances one safe timing alternative at a time.
    candidates_.erase(
        std::remove_if(candidates_.begin(), candidates_.end(),
                       [](const OfdmAcquisitionCandidate& candidate) {
                           return candidate.status ==
                                      OfdmAcquisitionStatus::Rejected ||
                                  candidate.status ==
                                      OfdmAcquisitionStatus::Validated;
                       }),
        candidates_.end());
    if (candidates_.size() >= kMaxRetainedCandidates)
        return nullptr;

    OfdmAcquisitionCandidate candidate;
    candidate.id = next_candidate_id_++;
    candidate.capture_epoch = capture_epoch_;
    candidate.absolute_start = start;
    candidate.timing_begin = timing_begin;
    candidate.timing_end = timing_end;
    candidate.support_begin = absolute(sync.candidate_support_begin >= 0
        ? sync.candidate_support_begin : sync.frame_start);
    candidate.support_end = absolute(sync.candidate_support_end >= 0
        ? sync.candidate_support_end : sync.frame_start + 1);
    candidate.required_right_context = absolute(sync.required_right_context);
    candidate.status = OfdmAcquisitionStatus::Ready;
    candidate.sync = sync;

    auto insert_at = std::upper_bound(
        candidates_.begin(), candidates_.end(), candidate.absolute_start,
        [](std::uint64_t value, const OfdmAcquisitionCandidate& item) {
            return value < item.absolute_start;
        });
    insert_at = candidates_.insert(insert_at, std::move(candidate));
    return &*insert_at;
}

OfdmAcquisitionCandidate* OfdmAcquisitionState::earliest_unresolved()
{
    for (auto& candidate : candidates_)
        if (candidate.status == OfdmAcquisitionStatus::NeedContext ||
            candidate.status == OfdmAcquisitionStatus::Ready)
            return &candidate;
    return nullptr;
}

OfdmAcquisitionCandidate* OfdmAcquisitionState::find(std::uint64_t id)
{
    for (auto& candidate : candidates_)
        if (candidate.id == id) return &candidate;
    return nullptr;
}

void OfdmAcquisitionState::require_context(std::uint64_t id,
                                           std::uint64_t absolute_end)
{
    if (auto* candidate = find(id)) {
        candidate->required_right_context = std::max(
            candidate->required_right_context, absolute_end);
        candidate->status = OfdmAcquisitionStatus::NeedContext;
    }
}

void OfdmAcquisitionState::mark_ready(std::uint64_t id)
{
    if (auto* candidate = find(id))
        candidate->status = OfdmAcquisitionStatus::Ready;
}

void OfdmAcquisitionState::mark_rejected(std::uint64_t id)
{
    if (auto* candidate = find(id))
        candidate->status = OfdmAcquisitionStatus::Rejected;
}

void OfdmAcquisitionState::mark_validated(std::uint64_t id)
{
    if (auto* candidate = find(id))
        candidate->status = OfdmAcquisitionStatus::Validated;
}

void OfdmAcquisitionState::set_trial_cursors(std::uint64_t id,
                                             std::size_t cfo_cursor,
                                             std::size_t shape_cursor)
{
    if (auto* candidate = find(id)) {
        candidate->cfo_trial_cursor = cfo_cursor;
        candidate->next_untried_shape_cursor = std::max(
            candidate->next_untried_shape_cursor, shape_cursor);
        candidate->shape_trial_cursor = std::min(
            candidate->next_untried_shape_cursor,
            candidate->earliest_incomplete_shape_cursor);
    }
}

void OfdmAcquisitionState::advance_cfo_trial(std::uint64_t id,
                                             std::size_t cfo_cursor)
{
    if (auto* candidate = find(id)) {
        // Preserve the furthest right-context requirement found under the
        // outgoing wrap.  Shape cursors are wrap-local, but their aggregate
        // extent is bank work: if no ready wrap validates, the bank restarts
        // after that context arrives rather than forgetting a possible frame.
        candidate->required_right_context = std::max(
            candidate->required_right_context,
            candidate->incomplete_shape_required_right_context);
        candidate->cfo_trial_cursor = cfo_cursor;
        candidate->shape_trial_cursor = 0;
        candidate->next_untried_shape_cursor = 0;
        candidate->earliest_incomplete_shape_cursor =
            std::numeric_limits<std::size_t>::max();
        candidate->incomplete_shape_required_right_context = 0;
        candidate->incomplete_shape_work.clear();
        candidate->status = OfdmAcquisitionStatus::Ready;
    }
}

void OfdmAcquisitionState::remember_incomplete_shape(
    std::uint64_t id, std::size_t shape_cursor, std::uint64_t absolute_end)
{
    if (auto* candidate = find(id)) {
        auto work = std::find_if(
            candidate->incomplete_shape_work.begin(),
            candidate->incomplete_shape_work.end(),
            [&](const OfdmIncompleteShapeWork& item) {
                return item.cursor == shape_cursor;
            });
        if (work == candidate->incomplete_shape_work.end()) {
            candidate->incomplete_shape_work.push_back(
                {shape_cursor, absolute_end});
        } else {
            work->required_right_context = std::max(
                work->required_right_context, absolute_end);
        }
        candidate->earliest_incomplete_shape_cursor = std::min(
            candidate->earliest_incomplete_shape_cursor, shape_cursor);
        candidate->shape_trial_cursor = std::min(
            candidate->shape_trial_cursor, shape_cursor);
        candidate->incomplete_shape_required_right_context = std::max(
            candidate->incomplete_shape_required_right_context, absolute_end);
    }
}

void OfdmAcquisitionState::resolve_shape_trial(
    std::uint64_t id, std::size_t shape_cursor)
{
    if (auto* candidate = find(id)) {
        auto& work = candidate->incomplete_shape_work;
        work.erase(std::remove_if(work.begin(), work.end(),
                                  [&](const OfdmIncompleteShapeWork& item) {
                                      return item.cursor == shape_cursor;
                                  }),
                   work.end());
        candidate->earliest_incomplete_shape_cursor =
            std::numeric_limits<std::size_t>::max();
        candidate->incomplete_shape_required_right_context = 0;
        for (const auto& item : work) {
            candidate->earliest_incomplete_shape_cursor = std::min(
                candidate->earliest_incomplete_shape_cursor, item.cursor);
            candidate->incomplete_shape_required_right_context = std::max(
                candidate->incomplete_shape_required_right_context,
                item.required_right_context);
        }
        candidate->shape_trial_cursor = std::min(
            candidate->next_untried_shape_cursor,
            candidate->earliest_incomplete_shape_cursor);
    }
}

std::uint64_t OfdmAcquisitionState::safe_retirement_watermark(
    std::uint64_t frontend_overlap) const
{
    std::uint64_t safe = searched_watermark_;
    for (const auto& candidate : candidates_) {
        if (candidate.status != OfdmAcquisitionStatus::NeedContext &&
            candidate.status != OfdmAcquisitionStatus::Ready)
            continue;
        const std::uint64_t retain = candidate.support_begin > frontend_overlap
            ? candidate.support_begin - frontend_overlap : 0;
        safe = std::min(safe, retain);
    }
    return std::max(buffer_origin_, safe);
}

void OfdmAcquisitionState::retire_prefix(std::uint64_t absolute_end)
{
    if (absolute_end <= buffer_origin_) return;
    buffer_origin_ = absolute_end;
    while (!candidates_.empty()) {
        const auto& front = candidates_.front();
        const bool resolved = front.status == OfdmAcquisitionStatus::Rejected ||
                              front.status == OfdmAcquisitionStatus::Validated;
        if (!resolved || front.support_end > absolute_end) break;
        candidates_.pop_front();
    }
}

static std::uint64_t sync_waveform_fingerprint(const OfdmConfig& config)
{
    // FNV-1a over all parameters consumed while constructing or indexing the
    // training reference. Float values are hashed by representation so a cache
    // hit means the receiver is using the identical waveform definition.
    std::uint64_t h = 1469598103934665603ULL;
    auto mix = [&](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= static_cast<unsigned char>(v & 0xffU);
            h *= 1099511628211ULL;
            v >>= 8;
        }
    };
    auto mix_float = [&](float v) {
        std::uint32_t bits = 0;
        static_assert(sizeof(bits) == sizeof(v), "float fingerprint width");
        std::memcpy(&bits, &v, sizeof(bits));
        mix(bits);
    };
    mix(static_cast<std::uint64_t>(config.nfft));
    mix(static_cast<std::uint64_t>(config.cp_samples));
    mix(static_cast<std::uint64_t>(config.sample_rate));
    mix(static_cast<std::uint64_t>(config.n_used_carriers));
    mix(static_cast<std::uint64_t>(config.n_data_carriers));
    mix_float(config.fm_preemph_corner_hz);
    mix_float(config.fm_preemph_gain_cap);
    for (int bin : config.used_carrier_bins)
        mix(static_cast<std::uint64_t>(static_cast<std::uint32_t>(bin)));
    return h;
}

OfdmSyncResult ofdm_detect_frame(const std::complex<float>* iq, int n_samples,
                                  const OfdmConfig& config,
                                  OfdmSyncWorkspace* supplied_workspace)
{
    OfdmSyncResult result;
    OfdmSyncWorkspace local_workspace;
    OfdmSyncWorkspace& workspace = supplied_workspace
        ? *supplied_workspace : local_workspace;
    const int nfft = config.nfft;
    const int cp = config.cp_samples;
    const int symbol_len = nfft + cp;

    // Do not freeze timing from a fragmented preamble.  The receiver needs the
    // repeated pair plus the following sync symbol before a timing alternative
    // becomes committable.  With less context, callers retain the samples and
    // reconsider on the next append.
    const int min_samples = 3 * symbol_len;
    if (n_samples < min_samples) {
        IRIS_LOG("[OFDM-SYNC] insufficient samples for detection: %d < %d",
                 n_samples, min_samples);
        return result;
    }

    // -----------------------------------------------------------------------
    // Cache ZC training symbol for timing refinement
    // -----------------------------------------------------------------------
    const std::uint64_t fingerprint = sync_waveform_fingerprint(config);
    if (workspace.waveform_fingerprint != fingerprint) {
        workspace.reset();
        workspace.zc_td = generate_zc_training_symbol(config);
        workspace.zc_energy = 0.0f;
        for (int n = 0; n < nfft; ++n) {
            workspace.zc_energy += std::norm(workspace.zc_td[n]);
        }
        workspace.fft_train1.resize(nfft);
        workspace.fft_train2.resize(nfft);
        workspace.waveform_fingerprint = fingerprint;
        IRIS_LOG("[OFDM-SYNC] receiver ZC reference cached: nfft=%d, n_used=%d, energy=%.1f",
                 nfft, config.n_used_carriers, workspace.zc_energy);
    }
    const auto& zc_td = workspace.zc_td;
    const float zc_energy = workspace.zc_energy;

    // d is the CP start of train1.  Reserve a full sync symbol as right context
    // so a settling/train boundary fragment cannot become a permanent guess.
    const int search_len = n_samples - 3 * symbol_len + 1;
    if (search_len <= 0) {
        IRIS_LOG("[OFDM-SYNC] search range empty");
        return result;
    }

    // -----------------------------------------------------------------------
    // Phase 1: chronological rolling Schmidl-Cox region discovery
    // -----------------------------------------------------------------------
    // Correlate train1 body [d+cp, d+cp+nfft) with train2 body [d+cp+sym_len, ...)
    // M(d) = |P(d)|² / (A(d) · R(d))   [Cauchy-Schwarz normalized, 0..1]
    // Energy weighting: W(d) = M(d) · (A(d) + R(d))  [rejects silence]

    float sc_best_metric = 0.0f;
    float sc_best_weighted = 0.0f;
    int sc_best_d = -1;
    workspace.sc_profile.assign(search_len, 0.0f);
    std::complex<float> P(0.0f, 0.0f);
    float A = 0.0f;
    float R = 0.0f;
    const int first_body1 = cp;
    const int first_body2 = symbol_len + cp;
    for (int n = 0; n < nfft; ++n) {
        P += std::conj(iq[first_body1 + n]) * iq[first_body2 + n];
        A += std::norm(iq[first_body1 + n]);
        R += std::norm(iq[first_body2 + n]);
    }

    bool in_first_region = false;
    bool first_region_closed = false;
    int region_begin = -1;
    int region_last = -1;
    int region_support_begin = -1;
    int region_timing_limit = -1;
    float global_best_metric = 0.0f;
    for (int d = 0; d < search_len; ++d) {
        // One emitted preamble is [settling][train1][train2].  Front-end
        // ringing can move a rising fragment anywhere inside that three-symbol
        // support, so it is the total emitted support -- not the most recent
        // threshold crossing -- that bounds this candidate.  Once exhausted,
        // a later qualifying sample belongs to another candidate.
        if (in_first_region && d > region_timing_limit) {
            first_region_closed = true;
            break;
        }

        const float denom = A * R;
        const float M = (denom > 1e-20f) ? (std::norm(P) / denom) : 0.0f;
        const float W = M * (A + R);
        workspace.sc_profile[d] = M;
        result.search_examined_through = d;
        global_best_metric = std::max(global_best_metric, M);

        if (M >= SC_DETECTION_THRESHOLD) {
            if (!in_first_region) {
                region_begin = d;
                // A threshold crossing can be the rising fragment produced as
                // the leading settling symbol enters the two-window SC span.
                // Back up by one symbol, then bound the entire emitted
                // [settling][train1][train2] support to exactly three symbols.
                region_support_begin = std::max(0, d - symbol_len);
                region_timing_limit =
                    region_support_begin + 3 * symbol_len - 1;
            }
            in_first_region = true;
            region_last = d;
            // Energy ranks timing alternatives only inside this earliest
            // correlation region.  It never compares independent frames.
            if (W > sc_best_weighted) {
                sc_best_weighted = W;
                sc_best_metric = M;
                sc_best_d = d;
            }
        }

        if (d + 1 < search_len) {
            const int body1 = d + cp;
            const int body2 = d + symbol_len + cp;
            P += std::conj(iq[body1 + nfft]) * iq[body2 + nfft]
               - std::conj(iq[body1]) * iq[body2];
            A += std::norm(iq[body1 + nfft]) - std::norm(iq[body1]);
            R += std::norm(iq[body2 + nfft]) - std::norm(iq[body2]);
            A = std::max(A, 0.0f);
            R = std::max(R, 0.0f);
        }
    }

    // -----------------------------------------------------------------------
    // Threshold check
    // -----------------------------------------------------------------------
    float peak_metric = std::sqrt(sc_best_metric);  // sqrt for [0,1] range comparable to old ZC metric
    int peak_d = sc_best_d;

    if (region_begin >= 0 && region_last >= region_begin) {
        result.candidate_present = true;
        result.frame_start = sc_best_d;
        result.timing_interval_begin = region_begin;
        result.timing_interval_end = region_last;
        result.candidate_support_begin = region_support_begin;
        result.candidate_support_end = region_timing_limit + 1;
        // search_len reserves three symbols at every timing alternative.  One
        // sample beyond the fixed timing support proves the first candidate is
        // closed; without it, a boundary hit is provisional and must be
        // reconsidered after append.
        result.required_right_context = region_timing_limit + 3 * symbol_len + 1;
        if (!first_region_closed) {
            result.status = OfdmSyncStatus::NeedContext;
            IRIS_LOG("[OFDM-SYNC] first region open at search boundary: "
                     "timing=[%d,%d] need=%d samples",
                     region_begin, region_last, result.required_right_context);
            return result;
        }

        // Preserve the established CP-grid/fine timing estimator, but confine
        // it to this chronological candidate's fixed waveform support.  The
        // rolling profile above discovers and bounds candidates; changing the
        // timing seed to a global per-sample energy maximum shifts the FFT
        // window on the live band-limited frontend and invalidates the
        // calibrated SNR meter.  This two-stage form keeps the old timing
        // behavior without allowing a later independent preamble to win.
        auto measure_sc = [&](int d, float* weighted) {
            const int body1 = d + cp;
            const int body2 = d + symbol_len + cp;
            std::complex<float> corr(0.0f, 0.0f);
            float e1 = 0.0f;
            float e2 = 0.0f;
            for (int n = 0; n < nfft; ++n) {
                corr += std::conj(iq[body1 + n]) * iq[body2 + n];
                e1 += std::norm(iq[body1 + n]);
                e2 += std::norm(iq[body2 + n]);
            }
            const float denom = e1 * e2;
            const float metric = denom > 1e-20f
                ? std::norm(corr) / denom : 0.0f;
            if (weighted) *weighted = metric * (e1 + e2);
            return metric;
        };
        const int timing_lo = std::max(0, result.candidate_support_begin);
        const int timing_hi = std::min(search_len - 1, region_timing_limit);
        const int coarse_stride = std::max(1, cp);
        int coarse_d = ((timing_lo + coarse_stride - 1) / coarse_stride) *
                       coarse_stride;
        float timing_best_weighted = -1.0f;
        for (int d = coarse_d; d <= timing_hi; d += coarse_stride) {
            float weighted = 0.0f;
            const float metric = measure_sc(d, &weighted);
            if (weighted > timing_best_weighted) {
                timing_best_weighted = weighted;
                sc_best_metric = metric;
                sc_best_d = d;
            }
        }
        if (sc_best_d >= 0) {
            const int fine_lo = std::max(timing_lo, sc_best_d - coarse_stride);
            const int fine_hi = std::min(timing_hi, sc_best_d + coarse_stride);
            for (int d = fine_lo; d <= fine_hi; ++d) {
                float weighted = 0.0f;
                const float metric = measure_sc(d, &weighted);
                if (weighted > timing_best_weighted) {
                    timing_best_weighted = weighted;
                    sc_best_metric = metric;
                    sc_best_d = d;
                }
            }
            peak_d = sc_best_d;
            peak_metric = std::sqrt(std::max(0.0f, sc_best_metric));
            IRIS_LOG("[OFDM-SYNC] candidate support=[%d,%d) crossings=[%d,%d] timing-seed=%d",
                     result.candidate_support_begin, result.candidate_support_end,
                     region_begin, region_last, peak_d);
        }
    }

    // Always expose the raw squared SC metric for diagnostics (set even on a
    // no-detect so a fail-before/pass-after test can read the deflated peak).
    result.sc_peak_raw = global_best_metric;

    // Diagnostic (env IRIS_SYNC_DIAG): decompose M at the best d over
    // leading-trimmed windows. If M climbs as the leading samples are trimmed,
    // the first training symbol's body is corrupted by inter-symbol leakage
    // from a channel (narrow-FM audio bandpass) whose memory exceeds the CP.
    if (peak_d >= 0 && std::getenv("IRIS_SYNC_DIAG")) {
        const int b1 = peak_d + cp;
        const int b2 = peak_d + symbol_len + cp;
        auto Mtrim = [&](int g) -> float {
            std::complex<float> P(0.0f, 0.0f); float A = 0.0f, R = 0.0f;
            for (int n = g; n < nfft; ++n) {
                P += std::conj(iq[b1 + n]) * iq[b2 + n];
                A += std::norm(iq[b1 + n]); R += std::norm(iq[b2 + n]);
            }
            float d = A * R; return (d > 1e-20f) ? std::norm(P) / d : 0.0f;
        };
        IRIS_LOG("[SYNC-DIAG] n_used=%d M_full=%.4f M_trim[nfft/4]=%.4f "
                 "M_trim[nfft/2]=%.4f M_trim[3nfft/4]=%.4f",
                 config.n_used_carriers, Mtrim(0), Mtrim(nfft / 4),
                 Mtrim(nfft / 2), Mtrim(3 * nfft / 4));
    }

    if (peak_d < 0 || sc_best_metric < SC_DETECTION_THRESHOLD) {
        ++workspace.no_detection_count;
        if (workspace.no_detection_count % 200 == 1) {
            IRIS_LOG("[OFDM-SYNC] no detection: peak_sc=%.3f (sqrt=%.3f) at d=%d/%d (threshold=%.2f, %d samples)",
                     global_best_metric, peak_metric, peak_d, search_len,
                     SC_DETECTION_THRESHOLD, n_samples);
        }
        return result;
    }

    // -----------------------------------------------------------------------
    // Phase 3: ZC cross-correlation and settling/training identity
    // -----------------------------------------------------------------------
    // SC has a plateau of width cp (due to cyclic prefix periodicity).
    // ZC xcorr has a sharp peak at the exact body start.
    // Time-domain ZC is used ONLY for timing refinement, not detection gating.
    float best_zc_td = 0.0f;
    {
        auto zc_metric_at = [&](int d) {
            if (d < 0 || d + cp + nfft > n_samples)
                return 0.0f;
            int body = d + cp;
            std::complex<float> xc(0.0f, 0.0f);
            float se = 0.0f;
            for (int n = 0; n < nfft; ++n) {
                xc += iq[body + n] * std::conj(zc_td[n]);
                se += std::norm(iq[body + n]);
            }
            float denom = se * zc_energy;
            return (denom > 1e-20f)
                ? std::sqrt(std::norm(xc) / denom) : 0.0f;
        };

        // Preserve the established local timing behavior when the selected
        // alternative identifies both repeated root-7 training symbols.  This
        // matters on the live band-limited frontend: its correlation lobe can
        // lead the mathematical peak while still being the best ISI-free FFT
        // placement.  A global root maximum would move that window and perturb
        // the calibrated channel/noise measurements.
        int refine_lo = std::max(0, peak_d - cp);
        int refine_hi = std::min(search_len - 1, peak_d + cp);
        int best_zc_d = peak_d;
        for (int d = refine_lo; d <= refine_hi; ++d) {
            const float m = zc_metric_at(d);
            if (m > best_zc_td) {
                best_zc_td = m;
                best_zc_d = d;
            }
        }

        // The settling symbol uses a different ZC root.  Require a balanced
        // root-7 match one symbol apart before treating a local lobe as the
        // train1/train2 identity.  If energy selected settling/train or train2/
        // sync instead, search only this candidate's fixed three-symbol support
        // for the strongest balanced repeated-root pair.  Thus energy may rank
        // alternatives within one emitted preamble, but cannot merge or jump to
        // an independent preamble beyond the support bound.
        const float paired_zc_td = zc_metric_at(best_zc_d + symbol_len);
        const float pair_max = std::max(best_zc_td, paired_zc_td);
        const float pair_balance = pair_max > 1e-20f
            ? std::min(best_zc_td, paired_zc_td) / pair_max : 0.0f;
        IRIS_LOG("[OFDM-SYNC] local training identity: d=%d root1=%.3f root2=%.3f balance=%.3f",
                 best_zc_d, best_zc_td, paired_zc_td, pair_balance);
        bool identity_relocated = false;
        if (best_zc_td <= 0.1f || paired_zc_td <= 0.1f ||
            pair_balance < 0.70f) {
            const int identity_lo = std::max(0, region_begin);
            const int identity_hi = std::min(
                region_timing_limit, search_len - 1);
            float best_pair_score = 0.0f;
            float best_pair_first = 0.0f;
            int best_pair_d = -1;
            for (int d = identity_lo; d <= identity_hi; ++d) {
                const float first = zc_metric_at(d);
                const float second = zc_metric_at(d + symbol_len);
                const float high = std::max(first, second);
                const float balance = high > 1e-20f
                    ? std::min(first, second) / high : 0.0f;
                const float score = std::min(first, second);
                if (first > 0.1f && second > 0.1f && balance >= 0.70f &&
                    score > best_pair_score) {
                    best_pair_score = score;
                    best_pair_first = first;
                    best_pair_d = d;
                }
            }
            if (best_pair_d >= 0) {
                best_zc_d = best_pair_d;
                best_zc_td = best_pair_first;
                identity_relocated = true;
            }
        }

        // Keep the local metric profile around the identified first training
        // symbol: first-path selection needs candidates before its argmax.
        // When the original timing seed already has the repeated-root identity,
        // keep its original fine-search bounds exactly; a maximum on that
        // window's edge is an intentional live-frontend timing choice, not an
        // invitation to slide the window another CP.  Only a genuine identity
        // relocation recenters the fine search.
        const int profile_center = identity_relocated ? best_zc_d : peak_d;
        refine_lo = std::max(0, profile_center - cp);
        refine_hi = std::min(search_len - 1, profile_center + cp);
        auto& m_prof = workspace.timing_profile;
        m_prof.assign(refine_hi - refine_lo + 1, 0.0f);
        best_zc_td = 0.0f;
        int identified_zc_d = best_zc_d;
        for (int d = refine_lo; d <= refine_hi; ++d) {
            const float m = zc_metric_at(d);
            m_prof[d - refine_lo] = m;
            if (m > best_zc_td) {
                best_zc_td = m;
                identified_zc_d = d;
            }
        }
        best_zc_d = identified_zc_d;

        // Use ZC-refined timing if it found a reasonable peak
        if (best_zc_td > 0.1f) {
            // -----------------------------------------------------------------
            // FIRST-PATH selection + CP back-off (multipath ISI-free placement).
            //
            // The plain argmax locks the FFT window to the STRONGEST multipath
            // tap. On a 2-tap channel (Watterson MPG: 2nd tap 0.5 ms = 24
            // samples late, P(|g1|>|g0|) = 0.5) that puts the window dtau
            // samples PAST the ISI-free region on half the fade realizations:
            // every symbol then leaks dtau samples of the next symbol through
            // the earlier tap — structural ISI at ANY SNR. Measured on the
            // frozen-tap offline harness: late-tap locks run at EsNo 3.3-9.0 dB vs 15-19.4 dB for
            // first-path locks at the same 60 dB channel SNR.
            //
            // Standard receiver practice (Speth et al. 1999, IEEE Trans.Comm.,
            // "Optimum Receiver Design for Wireless Broad-Band Systems Using
            // OFDM, Part I"; FreeDV 700D places its window against the LEADING
            // pilot correlation; LTE/DVB first-significant-path detection):
            //   1. find the EARLIEST candidate within FP_WINDOW samples BEFORE
            //      the argmax whose metric clears FP_ALPHA * max — the first
            //      significant path, not the strongest;
            //   2. back the window off FP_ADVANCE samples INTO the CP so both
            //      early jitter and the detected path's own tail stay ISI-free
            //      (safe for delay spreads <= cp - FP_ADVANCE).
            // The per-carrier LS channel estimate (ofdm_estimate_channel)
            // absorbs the resulting linear phase; every downstream window
            // (training, sync word, data, pilot rows) shifts coherently with
            // frame_start.
            // FP_ALPHA/FP_ADVANCE chosen by measured sweep on the frozen-tap
            // offline harness (alpha in 0.5-0.9 x advance in 0-14): alpha 0.5
            // degenerates on the FM de-emphasis-tilted spectrum (the tilt
            // makes low carriers dominate the ZC correlation -> a wide lobe
            // whose 0.5-amplitude skirt extends ~40 samples early; the window
            // then slides to the search edge and the DELAYED tap exits the CP
            // -> EsNo collapse). Alpha 0.75 stays on the true leading tap on
            // both failure modes; advance 8 rescues small late-bias while
            // keeping the delayed tap inside the CP (worst-case early shift
            // 40 + 8 + dtau 24 < ... bounded well under cp + preamble CP
            // margin; measured EsNo 18-20 dB across both tap orderings).
            static const float FP_ALPHA = [] {
                const char* e = std::getenv("IRIS_FP_ALPHA");
                return e ? (float)atof(e) : 0.75f;
            }();
            static const int FP_ADVANCE = [] {
                const char* e = std::getenv("IRIS_FP_ADVANCE");
                return e ? std::max(0, atoi(e)) : 8;
            }();
            static const bool FP_OFF = [] {
                const char* e = std::getenv("IRIS_FP_TIMING");
                return e && atoi(e) == 0;   // IRIS_FP_TIMING=0 -> legacy argmax
            }();
            const int FP_WINDOW = 40;       // cp - dtau_max(in-CP) = 64 - 24

            int d_fp = best_zc_d;
            if (!FP_OFF) {
                int fp_lo = std::max(refine_lo, best_zc_d - FP_WINDOW);
                for (int d = fp_lo; d < best_zc_d; ++d) {
                    if (m_prof[d - refine_lo] >= FP_ALPHA * best_zc_td) {
                        d_fp = d;
                        break;
                    }
                }
                peak_d = std::max(0, d_fp - FP_ADVANCE);
            } else {
                peak_d = best_zc_d;
            }
            IRIS_LOG("[OFDM-SYNC] ZC timing refine: d=%d (argmax=%d first-path=%d adv=%d, ZC_td=%.3f, SC=%.3f)",
                     peak_d, best_zc_d, d_fp, FP_OFF ? 0 : FP_ADVANCE,
                     best_zc_td, peak_metric);
        }
    }

    // -----------------------------------------------------------------------
    // Phase 4: Frequency-domain ZC verification + CFO estimation
    // -----------------------------------------------------------------------
    // Differential FD-ZC: D[k]=Y2[k]·conj(Y1[k]) eliminates channel phase
    // (group delay, timing offset, frequency response) because both training
    // symbols pass through the identical channel.  Phase coherence metric
    // |ΣD[k]|/Σ|D[k]| = 1.0 for any real preamble, ~1/√n for noise.
    // This replaces the old Y1-vs-reference Cauchy-Schwarz metric which
    // failed on channels with >0.3ms group delay variation across the band
    // (the reverse path RSP→CMD consistently gave FD-ZC 0.05-0.24).
    float best_zc_metric = 0.0f;

    result.frame_start = peak_d;
    result.schmidl_metric = peak_metric;

    {
        int train1_body = peak_d + cp;
        int train2_body = peak_d + symbol_len + cp;

        if (train2_body + nfft <= n_samples) {
            // FFT both training symbols (reused for CFO estimation below)
            auto& Y1_buf = workspace.fft_train1;
            auto& Y2_buf = workspace.fft_train2;
            std::copy(iq + train1_body, iq + train1_body + nfft, Y1_buf.data());
            std::copy(iq + train2_body, iq + train2_body + nfft, Y2_buf.data());
            fft_complex(Y1_buf.data(), nfft);
            fft_complex(Y2_buf.data(), nfft);

            // --- Frequency-domain differential ZC quality metric ---
            // D[k] = Y2[k]·conj(Y1[k]) = |H[k]|² · exp(j·φ_cfo)
            // For a real preamble, all D[k] share the same phase (CFO).
            //
            // Power-weighted coherence: weight each carrier by |D[k]| so
            // that strong carriers (high |H[k]|²) dominate and noise-floor
            // carriers (SNR < 0 dB) are naturally suppressed.
            //
            //   metric = |Σ |D[k]|·D[k]| / Σ |D[k]|²
            //          = |Σ |D[k]|²·exp(j·arg(D[k]))| / Σ |D[k]|²
            //
            // Coherent: 1.0.  Noise: ≈ 1/√n.  Same separation as unweighted,
            // but robust on channels with large dynamic range (-22 to +17 dB).
            const int n_used = config.n_used_carriers;
            std::complex<float> diff_sum(0.0f, 0.0f);
            float diff_mag_sum = 0.0f;
            for (int i = 0; i < n_used; ++i) {
                int bin = config.used_carrier_bins[i];
                std::complex<float> d = Y2_buf[bin] * std::conj(Y1_buf[bin]);
                float w = std::abs(d);   // |D[k]| = |H[k]|²
                diff_sum += w * d;       // |D[k]|² · exp(j·arg(D[k]))
                diff_mag_sum += w * w;   // |D[k]|²
            }
            best_zc_metric = (diff_mag_sum > 1e-20f)
                ? std::abs(diff_sum) / diff_mag_sum : 0.0f;

            // --- ZC quality gate ---
            // Power-weighted metric: real frames FD-ZC ≈ 0.99, false detections
            // 0.30-0.54.  Threshold 0.70 eliminates false detections while
            // leaving 0.29 margin for degraded real frames.
            // fd_zc_threshold remains a useful tight-quality reference for
            // measurement, but it must not become a hard pre-LDPC veto.  Cap
            // normal admission at 0.70 so the seeded 0.899 recoverable class
            // is examined; callers may explicitly request an even looser
            // threshold (for measurement/tune acquisition).
            const float fd_zc_threshold =
                std::min(config.fd_zc_threshold, 0.70f);
            if (best_zc_metric < fd_zc_threshold) {
                IRIS_LOG("[OFDM-SYNC] SC passed (%.3f) but FD-ZC too low (%.3f < %.2f, td=%.3f, n_used=%d) — rejected",
                         peak_metric, best_zc_metric, fd_zc_threshold, best_zc_td, n_used);
                result.status = OfdmSyncStatus::Rejected;
                return result;
            }

            // --- CFO estimation (reuses diff_sum from above) ---
            // arg(ΣD[k]) = φ_cfo = phase rotation over one symbol period.
            float cfo_phase = std::arg(diff_sum);
            result.cfo_principal_hz = cfo_phase / (2.0f * (float)M_PI)
                                    * ((float)config.sample_rate / symbol_len);
            result.cfo_hz = result.cfo_principal_hz;
            result.cfo_alias_spacing_hz =
                static_cast<float>(config.sample_rate) /
                static_cast<float>(symbol_len);

            IRIS_LOG("[OFDM-SYNC] FD-ZC=%.3f (td=%.3f) modulo-CFO: phase=%.4f rad -> %.2f Hz, alias spacing %.3f Hz (|diff|=%.1f)",
                     best_zc_metric, best_zc_td, cfo_phase,
                     result.cfo_principal_hz, result.cfo_alias_spacing_hz,
                     std::abs(diff_sum));
        } else {
            IRIS_LOG("[OFDM-SYNC] cannot estimate CFO: train2 out of bounds");
            result.cfo_principal_hz = 0.0f;
            result.cfo_hz = 0.0f;
            result.cfo_alias_spacing_hz =
                static_cast<float>(config.sample_rate) /
                static_cast<float>(symbol_len);
            result.detected = false;
            result.status = OfdmSyncStatus::NeedContext;
            result.required_right_context =
                std::max(result.required_right_context, train2_body + nfft);
            result.estimator_validity = OfdmEstimatorValidity::Unavailable;
            return result;
        }
    }

    // -----------------------------------------------------------------------
    // SNR estimate from SC metric.
    // SC metric M_sq = |P|²/(A·R) = (SNR/(SNR+1))².
    // peak_metric = sqrt(M_sq) = SNR/(SNR+1).
    // Inversion: SNR = peak_metric / (1 - peak_metric).
    // -----------------------------------------------------------------------
    {
        float snr_linear = (peak_metric < 0.999f) ? (peak_metric / (1.0f - peak_metric)) : 1000.0f;
        result.snr_est = 10.0f * std::log10(std::max(snr_linear, 1e-10f));
    }

    IRIS_LOG("[OFDM-SYNC] detect: SC=%.3f FD-ZC=%.3f at sample %d, modulo-CFO=%.1f Hz unresolved, SNR~%.1f dB",
             peak_metric, best_zc_metric, peak_d, result.cfo_principal_hz,
             result.snr_est);

    // CFO sanity check: reject false triggers with implausible CFO.
    // Three regimes we need to accommodate:
    //  - FM radio audio paths: 17-25 Hz typical, up to ~35 Hz
    //  - OTA direct cable: 30-40 Hz from sample-clock drift between Pi ADCs
    //  - IONOS simulator: near zero
    // Moose fine-CFO correction in ofdm_demod covers up to ±n_IS · subcarrier
    // spacing (~±187 Hz NFFT=512, ±93 Hz NFFT=1024). 35 Hz was too tight for
    // direct-cable OTA and was rejecting real sync triggers as "false".
    // Raised to 100 Hz — still inside Moose range, but leaves headroom to
    // distinguish a genuine lock from a noise false-positive (which tends
    // to produce wild CFO estimates hundreds of Hz off).
    constexpr float CFO_MAX_HZ = 100.0f;
    if (std::abs(result.cfo_principal_hz) > CFO_MAX_HZ) {
        IRIS_LOG("[OFDM-SYNC] CFO sanity check FAILED: |%.1f Hz| > %.0f Hz — false trigger rejected",
                 result.cfo_principal_hz, CFO_MAX_HZ);
        result.detected = false;
        result.status = OfdmSyncStatus::Rejected;
        return result;
    }

    result.detected = true;
    result.status = OfdmSyncStatus::Ready;
    result.sc_metric = peak_metric;
    result.zc_metric = best_zc_metric;
    result.pair_coherence = best_zc_metric;
    result.estimator_validity =
        OfdmEstimatorValidity::CoherentPairUnresolved;

    return result;
}

std::vector<OfdmSyncResult> ofdm_cfo_hypotheses(
    const std::complex<float>* iq, int n_samples, const OfdmConfig& config,
    const OfdmSyncResult& principal_sync)
{
    std::vector<OfdmSyncResult> hypotheses;
    const int lag = config.nfft + config.cp_samples;
    if (!iq || n_samples <= 0 || lag <= 0 || config.sample_rate <= 0)
        return hypotheses;

    // The legal impairment envelope is +/-150 Hz.  Five hertz covers detector
    // and sample-clock uncertainty at an alias boundary without making the bank
    // open-ended.  The integer limits are derived from this configuration's
    // actual repeated-training lag, never from a nominal FFT spacing.
    constexpr float kMaxCfoHz = 150.0f;
    constexpr float kEstimatorMarginHz = 5.0f;
    const float spacing = static_cast<float>(config.sample_rate) /
                          static_cast<float>(lag);
    const float principal = principal_sync.cfo_principal_hz;
    const int m_min = static_cast<int>(std::ceil(
        (-kMaxCfoHz - kEstimatorMarginHz - principal) / spacing));
    const int m_max = static_cast<int>(std::floor(
        ( kMaxCfoHz + kEstimatorMarginHz - principal) / spacing));
    if (m_min > m_max) return hypotheses;

    std::vector<int> ambiguity_indices;
    ambiguity_indices.reserve(static_cast<std::size_t>(m_max - m_min + 1));
    for (int m = m_min; m <= m_max; ++m)
        ambiguity_indices.push_back(m);
    // Preserve the established low-offset fast path, then expand fairly in
    // distance from the principal interval.  No score removes an entry.
    std::stable_sort(ambiguity_indices.begin(), ambiguity_indices.end(),
        [](int a, int b) {
            const int aa = std::abs(a), ab = std::abs(b);
            if (aa != ab) return aa < ab;
            return a < b;
        });

    for (int m : ambiguity_indices) {
        OfdmSyncResult trial = principal_sync;
        trial.cfo_principal_hz = principal;
        trial.cfo_alias_spacing_hz = spacing;
        trial.cfo_ambiguity_index = m;
        trial.cfo_hz = principal + static_cast<float>(m) * spacing;
        trial.cfo_hypothesis_selected = true;
        trial.cfo_resolved = false;
        hypotheses.push_back(trial);
    }

    IRIS_LOG("[OFDM-CFO] bank: principal=%.3f Hz spacing=%.6f Hz lag=%d m=[%d,%d] count=%zu margin=%.1f Hz",
             principal, spacing, lag, m_min, m_max, hypotheses.size(),
             kEstimatorMarginHz);
    return hypotheses;
}

OfdmSyncResult ofdm_refine_cfo_hypothesis(
    const std::complex<float>* iq, int n_samples, const OfdmConfig& config,
    const OfdmSyncResult& hypothesis)
{
    OfdmSyncResult trial = hypothesis;
    const int lag = config.nfft + config.cp_samples;
    if (!iq || n_samples <= 0 || config.nfft <= 0 || lag <= 0 ||
        config.sample_rate <= 0)
        return trial;

    const auto zc = generate_zc_training_symbol(config);
    float zc_energy = 0.0f;
    for (const auto& sample : zc) zc_energy += std::norm(sample);

    int timing_lo = trial.timing_interval_begin;
    int timing_hi = trial.timing_interval_end;
    if (timing_lo < 0 || timing_hi < timing_lo || timing_lo >= n_samples) {
        timing_lo = trial.frame_start;
        timing_hi = trial.frame_start;
    }
    timing_lo = std::max(0, timing_lo);
    timing_hi = std::min(timing_hi, n_samples - 1);

    float best_metric = -1.0f;
    int best_timing = trial.frame_start;
    const float phase_step = -2.0f * static_cast<float>(M_PI) * trial.cfo_hz /
                             static_cast<float>(config.sample_rate);
    const std::complex<float> rotation_step(std::cos(phase_step),
                                            std::sin(phase_step));
    for (int d = timing_lo; d <= timing_hi; ++d) {
        const int body1 = d + config.cp_samples;
        const int body2 = body1 + lag;
        if (body1 < 0 || body2 + config.nfft > n_samples) continue;

        std::complex<float> corr1(0.0f, 0.0f);
        std::complex<float> corr2(0.0f, 0.0f);
        std::complex<float> rotation(1.0f, 0.0f);
        float energy1 = 0.0f, energy2 = 0.0f;
        for (int n = 0; n < config.nfft; ++n) {
            const auto corrected1 = iq[body1 + n] * rotation;
            const auto corrected2 = iq[body2 + n] * rotation;
            corr1 += corrected1 * std::conj(zc[n]);
            corr2 += corrected2 * std::conj(zc[n]);
            energy1 += std::norm(corrected1);
            energy2 += std::norm(corrected2);
            rotation *= rotation_step;
        }
        const float metric1 = energy1 * zc_energy > 1e-20f
            ? std::sqrt(std::norm(corr1) / (energy1 * zc_energy)) : 0.0f;
        const float metric2 = energy2 * zc_energy > 1e-20f
            ? std::sqrt(std::norm(corr2) / (energy2 * zc_energy)) : 0.0f;
        const float metric = std::min(metric1, metric2);
        if (metric > best_metric) {
            best_metric = metric;
            best_timing = d;
        }
    }

    trial.cfo_training_metric = std::max(0.0f, best_metric);
    // Preserve the established calibrated principal timing.  Non-zero wraps
    // are precisely where uncorrected reference correlation can move hundreds
    // of samples; back their corrected-reference peak eight samples into CP.
    if (trial.cfo_ambiguity_index != 0 && best_metric >= 0.0f)
        trial.frame_start = std::max(timing_lo, best_timing - 8);
    return trial;
}

// ---------------------------------------------------------------------------
// CFO correction (in-place)
// ---------------------------------------------------------------------------
// Complex derotation exp(-jθ) correctly shifts the positive-frequency carriers
// (bins 4-33) that the demodulator extracts.
//
// The input is now an analytic signal (via Hilbert transform in modem.cc),
// so the CFO estimator works correctly: Y2·conj(Y1) is a complex quantity
// whose arg() gives the true CFO phase. Before the Hilbert transform, the
// real-valued input produced Hermitian symmetry that made the correlation
// real-valued and biased the estimator toward zero.
void ofdm_correct_cfo(std::complex<float>* iq, int n_samples,
                       float cfo_hz, int sample_rate)
{
    if (std::abs(cfo_hz) < 1e-6f) return;

    float phase_inc = -2.0f * (float)M_PI * cfo_hz / sample_rate;
    std::complex<float> rot(std::cos(phase_inc), std::sin(phase_inc));
    std::complex<float> phasor(1.0f, 0.0f);
    for (int n = 0; n < n_samples; ++n) {
        iq[n] *= phasor;
        phasor *= rot;
        // Renormalize every 512 samples to prevent magnitude drift
        if ((n & 0x1FF) == 0x1FF) {
            float mag = std::abs(phasor);
            if (mag > 0.0f) phasor /= mag;
        }
    }
}

// ---------------------------------------------------------------------------
// Channel estimation from training symbol (ZC-based)
// ---------------------------------------------------------------------------
OfdmChannelEst ofdm_estimate_channel(const std::complex<float>* iq_symbol,
                                      const OfdmConfig& config)
{
    OfdmChannelEst est;
    const int N = config.nfft;
    const int n_used = config.n_used_carriers;

    if (n_used <= 0) {
        IRIS_LOG("[OFDM-CE] no used carriers in config");
        return est;
    }

    // This reference is small; construct it locally so independent receivers
    // never share mutable cache state.  The detector's larger reference and
    // FFT scratch are instance-owned in OfdmSyncWorkspace.
    const auto zc_freq = generate_zc_sequence(7, n_used);

    // FFT the received training symbol
    std::vector<std::complex<float>> Y(N);
    std::copy(iq_symbol, iq_symbol + N, Y.begin());
    fft_complex(Y.data(), N);

    // Extract H at used carrier bins: H[k] = Y[k] / X[k]
    // where X[k] is the known ZC frequency-domain value.
    // TX applies de-emphasis g[k] to training symbols, pilots, and data
    // carriers identically.  The raw ZC reference (|ZC[i]|=1) gives
    // H[i] = H_ch * g[i] — the de-emphasis gain is part of the effective
    // channel.  The equalizer divides data by H, which cancels g[i].
    est.H.resize(n_used);
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        // Division by ZC value: since |ZC[i]| = 1 (unit magnitude),
        // H[i] = Y[bin] * conj(ZC[i]) / |ZC[i]|^2 = Y[bin] * conj(ZC[i])
        est.H[i] = Y[bin] * std::conj(zc_freq[i]);
    }

    // Diagnostic: show |Y| and |H| at first, middle, and last carrier
    // to distinguish signal quality from channel estimation issues.
    if (n_used >= 3) {
        int i0 = 0, im = n_used / 2, il = n_used - 1;
        IRIS_LOG("[OFDM-CE] Y_diag: |Y[%d]|=%.4f |Y[%d]|=%.4f |Y[%d]|=%.4f  "
                 "|H[0]|=%.4f |H[mid]|=%.4f |H[last]|=%.4f  preemph=%.0f Hz",
                 config.used_carrier_bins[i0], std::abs(Y[config.used_carrier_bins[i0]]),
                 config.used_carrier_bins[im], std::abs(Y[config.used_carrier_bins[im]]),
                 config.used_carrier_bins[il], std::abs(Y[config.used_carrier_bins[il]]),
                 std::abs(est.H[i0]), std::abs(est.H[im]), std::abs(est.H[il]),
                 config.fm_preemph_corner_hz);
    }

    // Guard-bin noise floor — DIAGNOSTIC ONLY. It averages |Y|^2 over the FFT
    // bins OUTSIDE the used-carrier range, but the live RX band-limit
    // (modem.cc, default-ON) zeroes exactly those bins, so on the live path
    // this scalar can only ever see the spectral leakage of the used carriers
    // (~0.08 floor) — it is PINNED regardless of the channel (measured
    // injection-calibrated residuals +11.5..+26.4 dB, fact doc
    // data-flow-noise-var.md §8). And on any FM path the out-of-band noise
    // PSD genuinely differs from in-band (discriminator noise rises ~f^2),
    // so even un-zeroed guard bins measure the wrong quantity. The honest
    // in-band noise is measured from the training-symbol PAIR
    // (ofdm_noise_from_training_pair below); the demodulator installs it and
    // derives the real SNR view from it. Nothing may steer rate adaptation
    // from this guard-bin value.
    //
    // WHY NOT residual-after-smoothing: previously we fit a 7-tap quadratic
    // to H(f) and set noise_var = |H - H_smooth|². On any channel where
    // H(f) has non-quadratic shape (FM de-emphasis, BPF tails, ZC ripple,
    // band-edge rolloff), the fit residual is dominated by genuine signal
    // shape rather than noise. Fact doc §5.3/§6.5a measured 23 dB of
    // spurious "noise" from this mechanism, pinning effective_snr_db at
    // ~7 dB even on a 30 dB channel.
    float nv_frame = 1e-6f;
    {
        // Mark bins that carry signal (plus 2-bin guard to avoid spectral
        // leakage contamination from used-carrier tails).
        std::vector<bool> is_signal(N / 2 + 1, false);
        for (int i = 0; i < n_used; ++i) {
            int bin = config.used_carrier_bins[i];
            for (int off = -2; off <= 2; ++off) {
                int b = bin + off;
                if (b >= 0 && b <= N / 2) is_signal[b] = true;
            }
        }
        // DC and Nyquist may have residual clock leakage — treat as signal.
        is_signal[0] = true;
        is_signal[N / 2] = true;

        double noise_sum = 0.0;
        int noise_count = 0;
        for (int k = 1; k < N / 2; ++k) {
            if (is_signal[k]) continue;
            noise_sum += (double)std::norm(Y[k]);
            noise_count++;
        }
        if (noise_count > 0)
            nv_frame = (float)(noise_sum / noise_count);
        if (nv_frame < 1e-6f) nv_frame = 1e-6f;  // numerical floor
        IRIS_LOG("[OFDM-CE] noise floor from %d guard/out-of-band bins: nv=%.6f",
                 noise_count, nv_frame);
    }

    // Per-carrier noise variance via quadratic-fit H-residual. Retained for
    // MMSE equalizer (it needs per-carrier granularity to handle deep fades
    // on strongly frequency-selective channels like FM sim). For the
    // post-EQ gearshift metric (effective_snr_db), we use nv_frame above
    // instead — that one is bias-free on AWGN-dominant channels.
    std::vector<std::complex<float>> H_smooth(n_used);
    {
        // Local quadratic regression over a window of +/-W carriers.
        // For each carrier i, fit H(x) = a + b*x + c*x^2 to neighbors,
        // evaluate at x=0 to get H_smooth[i]. x is centered at i.
        // Using W=3 (7-tap window) balances noise averaging with slope tracking.
        constexpr int W = 3;
        for (int i = 0; i < n_used; ++i) {
            int lo = std::max(0, i - W);
            int hi = std::min(n_used - 1, i + W);
            int count = hi - lo + 1;

            if (count < 3) {
                // Not enough points for quadratic -- fall back to local mean
                std::complex<float> sum(0.0f, 0.0f);
                for (int j = lo; j <= hi; ++j) sum += est.H[j];
                H_smooth[i] = sum / (float)count;
            } else {
                // Solve normal equations for quadratic fit (real and imag independently)
                // Basis: 1, x, x^2 where x = j - i
                float S0 = 0, S1 = 0, S2 = 0, S3 = 0, S4 = 0;
                float Yr0 = 0, Yr1 = 0, Yr2 = 0;
                float Yi0 = 0, Yi1 = 0, Yi2 = 0;
                for (int j = lo; j <= hi; ++j) {
                    float x = (float)(j - i);
                    float x2 = x * x;
                    S0 += 1.0f;
                    S1 += x;
                    S2 += x2;
                    S3 += x * x2;
                    S4 += x2 * x2;
                    Yr0 += est.H[j].real();
                    Yr1 += est.H[j].real() * x;
                    Yr2 += est.H[j].real() * x2;
                    Yi0 += est.H[j].imag();
                    Yi1 += est.H[j].imag() * x;
                    Yi2 += est.H[j].imag() * x2;
                }
                float D = S0*(S2*S4 - S3*S3) - S1*(S1*S4 - S3*S2) + S2*(S1*S3 - S2*S2);
                if (std::abs(D) > 1e-12f) {
                    float Dr_a = Yr0*(S2*S4-S3*S3) - S1*(Yr1*S4-Yr2*S3) + S2*(Yr1*S3-Yr2*S2);
                    float Di_a = Yi0*(S2*S4-S3*S3) - S1*(Yi1*S4-Yi2*S3) + S2*(Yi1*S3-Yi2*S2);
                    H_smooth[i] = std::complex<float>(Dr_a / D, Di_a / D);
                } else {
                    H_smooth[i] = est.H[i];
                }
            }
        }
    }

    // Per-carrier noise variance from H-smoothness residual (MMSE
    // equalizer uses this). Store the per-frame AWGN floor separately in
    // est.noise_var_frame for the gearshift metric consumer.
    est.noise_var.resize(n_used);
    est.snr_per_carrier.resize(n_used);
    est.noise_var_frame = nv_frame;

    float snr_sum_db = 0.0f;
    float min_snr_db = 100.0f;
    float max_snr_db = -100.0f;
    float mean_H_mag = 0.0f;

    // Physically-correct noise model (DEFAULT): post-FFT AWGN is WHITE across
    // bins — the correct per-carrier noise is the scalar guard-bin floor
    // nv_frame for EVERY carrier; per-carrier SNR variation lives entirely in
    // |H[k]|². The legacy quadratic-fit H-residual instead reports channel
    // SHAPE as noise: on a frequency-selective (multipath) H — or on ANY
    // timing back-off, which is a linear phase ramp across carriers the
    // 7-tap quadratic cannot follow — the misfit residual inflates noise_var
    // by 10-30 dB, which (a) caps snr_per_carrier at ~1/eps² INDEPENDENT of
    // channel SNR (the phantom "min-carrier 3-14 dB at a 60 dB channel",
    // fact doc §9.4), (b) ripples the MMSE composite W·H (self-ISI through
    // the DFT despread), and (c) crushes the SC-FDMA LLR scalar. Measured on
    // the offline MPG harness: the misfit nv turns the first-path timing
    // back-off (Speth-style, above) into a 6-8 dB EsNo REGRESSION on
    // first-tap-dominant seeds; with the white-noise model the back-off is
    // free. Guard-bin scalar per Nino Carrillo passband_demod.py / Rhizo
    // Mercury / FreeDV practice.  IRIS_NV_QFIT=1 restores the legacy
    // quadratic-misfit model (A/B escape hatch only).
    static const bool NV_QFIT = [] {
        const char* e = std::getenv("IRIS_NV_QFIT");
        return e && atoi(e) == 1;
    }();
    for (int i = 0; i < n_used; ++i) {
        if (!NV_QFIT || config.clean_channel) {
            est.noise_var[i] = nv_frame;
        } else {
            // Legacy: per-carrier noise from H-smoothness residual.
            std::complex<float> residual = est.H[i] - H_smooth[i];
            est.noise_var[i] = std::norm(residual);
            if (est.noise_var[i] < 1e-6f) est.noise_var[i] = 1e-6f;
        }

        float signal_power = std::norm(est.H[i]);
        est.snr_per_carrier[i] = signal_power / est.noise_var[i];

        float snr_db = 10.0f * std::log10(std::max(est.snr_per_carrier[i], 1e-10f));
        snr_sum_db += snr_db;
        min_snr_db = std::min(min_snr_db, snr_db);
        max_snr_db = std::max(max_snr_db, snr_db);
        mean_H_mag += std::abs(est.H[i]);
    }

    est.mean_snr_db = snr_sum_db / n_used;
    mean_H_mag /= n_used;

    // PROVISIONAL single-symbol view (guard-bin nv, H still carrying the
    // preamble boost). The calibrated "[OFDM-CE] channel:" meter line is
    // emitted by the demodulator AFTER the training-pair in-band noise
    // estimate replaces noise_var (ofdm_demod.cc step 4c) — that one is the
    // gearshift feed and the value the instrumentation gate calibrates.
    IRIS_LOG("[OFDM-CE] guardbin-view: mean|H|=%.3f, SNR range %.1f-%.1f dB, mean=%.1f dB, %d carriers",
             mean_H_mag, min_snr_db, max_snr_db, est.mean_snr_db, n_used);

    return est;
}

// ---------------------------------------------------------------------------
// Per-carrier noise variance from the training-symbol pair
// ---------------------------------------------------------------------------
// The two preamble training symbols are IDENTICAL ZC references through the
// same channel, so at each used bin
//   Y1[k] = H[k]X[k] + N1[k],   Y2[k] = H[k]X[k]e^{j phi} + N2[k]
// where phi is the common inter-symbol rotation (residual CFO over one symbol
// period). After removing the measured phi, the difference
//   D[k] = Y2[k]e^{-j phi} - Y1[k] = N2[k]e^{-j phi} - N1[k]
// is pure noise with E|D|^2 = 2 sigma^2(k). This measures the REAL in-band
// noise color — triangular FM discriminator noise (+6 dB/oct), de-emphasis
// shaping, IF tilt — which the guard-bin scalar cannot see (guard bins are
// OUT of band, where the noise PSD differs from in-band on any FM path), and
// which the abandoned quadratic-misfit residual confused with channel shape.
// The per-bin |D|^2/2 is chi^2(2)-distributed (100% relative sd), so it is
// smoothed across frequency (boxcar, +-NV_SMOOTH_W carriers): the noise PSD
// mechanisms above are all smooth in f on scales far wider than the window.
// Scale note: this is the TRUE sigma^2 on the received-audio scale. It must
// NOT be divided by the preamble-boost^2 — the boost scales the SIGNAL bins
// of the training symbols, not the channel noise.
std::vector<float> ofdm_noise_from_training_pair(
    const std::complex<float>* Y1, const std::complex<float>* Y2,
    const OfdmConfig& config)
{
    const int n_used = config.n_used_carriers;
    std::vector<float> nv(n_used, 1e-9f);
    if (n_used <= 0) return nv;

    // Common inter-symbol rotation.  Normally use the established power-
    // weighted estimator.  On wide grids the legal low-side CFO extension can
    // also admit a strong narrow LF interferer; cap each carrier's influence
    // at four times the median pair-correlation magnitude so one edge tone
    // cannot rotate the residual on every payload carrier.  The wanted
    // repeated training contributes coherently across the whole grid.
    std::complex<float> acc(0.0f, 0.0f);
    if (n_used > 96) {
        std::vector<std::complex<float>> pair_corr(n_used);
        std::vector<float> pair_magnitude(n_used);
        for (int i = 0; i < n_used; ++i) {
            const int bin = config.used_carrier_bins[i];
            pair_corr[i] = Y2[bin] * std::conj(Y1[bin]);
            pair_magnitude[i] = std::abs(pair_corr[i]);
        }
        auto median_work = pair_magnitude;
        const auto median_at = median_work.begin() + n_used / 2;
        std::nth_element(median_work.begin(), median_at, median_work.end());
        const float weight_cap = std::max(1e-20f, 4.0f * *median_at);
        for (int i = 0; i < n_used; ++i) {
            const float magnitude = pair_magnitude[i];
            const float scale = magnitude > weight_cap
                ? weight_cap / magnitude : 1.0f;
            acc += pair_corr[i] * scale;
        }
    } else {
        for (int i = 0; i < n_used; ++i) {
            const int bin = config.used_carrier_bins[i];
            acc += Y2[bin] * std::conj(Y1[bin]);
        }
    }
    float phi = (std::abs(acc) > 1e-20f) ? std::arg(acc) : 0.0f;
    std::complex<float> derot(std::cos(-phi), std::sin(-phi));

    std::vector<float> raw(n_used);
    for (int i = 0; i < n_used; ++i) {
        int bin = config.used_carrier_bins[i];
        raw[i] = 0.5f * std::norm(Y2[bin] * derot - Y1[bin]);
    }
    if (n_used > 96) {
        // The wide-grid union passband must retain the legal -150 Hz edge,
        // which also admits LF energy below an unshifted frame.  Suppression is
        // authorized only by the UNUSED bins below the first used carrier.
        // In-band residual color is payload reliability, never evidence of an
        // out-of-band artifact and therefore never part of this predicate.
        const int half = n_used / 2;
        std::vector<float> upper(raw.begin() + half, raw.end());
        const auto median_at = upper.begin() + upper.size() / 2;
        std::nth_element(upper.begin(), median_at, upper.end());
        const float upper_median = *median_at;
        const auto lower_quartile_at = upper.begin() + upper.size() / 4;
        std::nth_element(upper.begin(), lower_quartile_at, upper.end());
        const float upper_lower_quartile = *lower_quartile_at;

        const int first_used_bin = config.used_carrier_bins.empty()
            ? 0 : config.used_carrier_bins.front();
        bool guard_lf_artifact = false;
        if (first_used_bin > 1) {
            double guard_resid = 0.0, guard_total = 0.0;
            for (int bin = 1; bin < first_used_bin; ++bin) {
                guard_resid += 0.5 * std::norm(Y2[bin] * derot - Y1[bin]);
                guard_total += 0.5 * (std::norm(Y1[bin]) + std::norm(Y2[bin]));
            }
            const float n_guard = static_cast<float>(first_used_bin - 1);
            const float r_g = static_cast<float>(guard_resid / n_guard);
            const float t_g = static_cast<float>(guard_total / n_guard);
            double in_total = 0.0;
            for (int i = 0; i < n_used; ++i) {
                const int bin = config.used_carrier_bins[i];
                in_total += 0.5 * (std::norm(Y1[bin]) + std::norm(Y2[bin]));
            }
            const float s_bar = static_cast<float>(in_total / n_used);
            // Signal-derived guard content repeats with the symbol period and
            // cancels in the pair difference, so compare residual with residual.
            // Require 12 dB dominance and a -15 dB in-band significance floor.
            guard_lf_artifact =
                (r_g > 16.0f * std::max(upper_median, 1e-9f)) &&
                (t_g > s_bar / 32.0f);
        }
        if (guard_lf_artifact) {
            // A simultaneous high-side skirt can contaminate the top edge of
            // the comparison half.  Its lower quartile is the robust interior
            // floor; the guard predicate above is the sole authorization for
            // using it as a clamp.
            const float outlier_cap = std::max(
                1e-9f, upper_lower_quartile);
            for (int i = 0; i < half; ++i)
                raw[i] = std::min(raw[i], outlier_cap);
        }
    }

    // Frequency smoothing (boxcar, window shrinks at band edges).
    constexpr int NV_SMOOTH_W = 5;
    for (int i = 0; i < n_used; ++i) {
        int lo = std::max(0, i - NV_SMOOTH_W);
        int hi = std::min(n_used - 1, i + NV_SMOOTH_W);
        double sum = 0.0;
        for (int j = lo; j <= hi; ++j) sum += (double)raw[j];
        nv[i] = (float)(sum / (hi - lo + 1));
        if (nv[i] < 1e-9f) nv[i] = 1e-9f;
    }
    return nv;
}

// (The old ofdm_interpolate_pilots helper was deleted along with
// ofdm_update_channel: both assumed an all-ones pilot observation and had no
// callers — pilot rows are ZC-modulated (ofdm_pilot_zc_root) and the live
// per-carrier refresh is s2_update_H in ofdm_demod.cc. ofdm_interpolate_pilots
// additionally overwrote noise_var with single-symbol comb residuals, the
// exact channel-shape-as-noise trap this file's noise model replaced.)

// ---------------------------------------------------------------------------
// Fine CFO estimation from training symbol
// ---------------------------------------------------------------------------
float ofdm_estimate_fine_cfo(const OfdmChannelEst& est, const OfdmConfig& config)
{
    const int n_used = config.n_used_carriers;
    if (n_used < 4 || (int)est.H.size() != n_used) return 0.0f;

    // CFO manifests as a common phase rotation on all subcarriers:
    // phase(H[k]) = 2*pi * delta_f * nfft / Fs
    // Weighted mean phase (by |H|^2) gives robust estimate.
    float sum_wp = 0.0f;
    float sum_w2 = 0.0f;
    for (int i = 0; i < n_used; i++) {
        float w = std::norm(est.H[i]);
        if (w < 1e-12f) continue;
        sum_wp += w * std::arg(est.H[i]);
        sum_w2 += w;
    }

    if (sum_w2 < 1e-12f) return 0.0f;

    float mean_phase = sum_wp / sum_w2;
    float fine_cfo = mean_phase * config.sample_rate / (2.0f * (float)M_PI * config.nfft);

    IRIS_LOG("[OFDM-SYNC] fine CFO: mean_phase=%.4f rad -> %.2f Hz", mean_phase, fine_cfo);

    return fine_cfo;
}


// ---------------------------------------------------------------------------
// RX front-end: analytic signal band-limited to the negotiated OFDM band
// ---------------------------------------------------------------------------
// Real audio -> analytic signal (FFT Hilbert: zero the negative frequencies,
// double the positives) with out-of-band positive frequencies suppressed so
// out-of-band energy (sub-band LF junk, high-side skirts, hum) cannot reach
// acquisition, channel estimation or the LLR path (the reason the band-limit
// exists: bisection confirmed guard-bin nv poisoning on live audio).
//
// EDGE DESIGN (TAPERED, the shipping mode). A hard 0/1 bin mask is a
// brick-wall frequency response; its impulse response is a sinc whose tails
// decay only as 1/t, so every spectral discontinuity in the record RINGS
// across the capture buffer (Gibbs):
//   (a) the record's own head/tail truncation edges splatter energy out of
//       band; the mask cuts it, and the cut re-appears as broadband error
//       concentrated near the buffer edges — frames demodulated near the
//       buffer head read ~31 dB on a clean channel;
//   (b) the OFDM signal's own rectangular-pulse spectral skirts extend past
//       the band edge; cutting them at a zero-width edge re-injects a
//       self-noise floor on every symbol — the training-pair noise estimate
//       floors at ~42 dB with the 1-carrier margin (data-flow-noise-var.md
//       §8.4), which caps the top end of the in-band SNR meter and falsely
//       refuses the 256QAM gears.
// Two standard remedies, both applied here:
//   1. TRANSITION BAND — the mask rolls off with a raised cosine over a few
//      carrier spacings instead of a step. Smoothing the sampled frequency
//      response between passband and stopband is the classical fix for the
//      Gibbs tails of a discontinuous ideal response (window/frequency-
//      sampling FIR design: Proakis & Manolakis, DSP 4e §10.2; Rabiner &
//      Gold 1975 — transition samples collapse the 1/t sinc tails). The flat
//      passband is UNCHANGED (used band ± 1 carrier, same as the brick-wall),
//      so nothing the old mask passed is attenuated; the roll-off lives in
//      previously-ZEROED bins, and beyond the short transition the rejection
//      is still exactly zero — the out-of-band poisoning stays killed. The
//      low-side transition is kept narrower than the high side because the
//      G3RUH-style LF junk on wide (6 kHz) data-port grids sits just below
//      the first carrier.
//   2. GUARD EXTENSION — the record is extended at both ends by an odd
//      (anti-symmetric about the end point) reflection of itself, tapered to
//      zero with a raised-cosine over the outer half, and the extensions are
//      DISCARDED after filtering: the filter's edge transient lands in the
//      discarded guard instead of on a frame parked at the buffer head. This
//      is the same edge treatment forward-backward filtering uses on finite
//      records (scipy/MATLAB filtfilt padtype 'odd'; F. Gustafsson, IEEE
//      Trans. Signal Processing 44(4), 1996).
std::vector<std::complex<float>> ofdm_analytic_bandlimit(
    const float* audio, int n, const OfdmConfig& cfg, RxBandlimitEdge edge,
    float trans_lo_carriers, float trans_hi_carriers, int guard_samples,
    float trans_profile_pow, float lo_flat_ext_carriers)
{
    if (n <= 0) return {};

    const bool limited = (edge != RxBandlimitEdge::OFF)
        && !cfg.used_carrier_bins.empty()
        && cfg.nfft > 0 && cfg.sample_rate > 0;
    const bool tapered = limited && (edge == RxBandlimitEdge::TAPERED);

    // Guard length: must cover the tapered-mask kernel tail. The kernel scale
    // is fs / transition_width; with the default 8-carrier high-side
    // transition (~375 Hz at 48 kHz / nfft 1024) that is ~128 samples, so
    // 1024 gives generous margin. Clamped so the reflection source exists.
    const int G = tapered ? std::max(0, std::min(guard_samples, n - 1)) : 0;

    int nfft_h = 1;
    while (nfft_h < n + 2 * G) nfft_h <<= 1;

    std::vector<std::complex<float>> hbuf(nfft_h, {0.0f, 0.0f});
    for (int i = 0; i < n; i++)
        hbuf[G + i] = std::complex<float>(audio[i], 0.0f);
    if (G > 0) {
        // Odd reflection about each end point (filtfilt-style).
        for (int j = 0; j < G; j++) {
            hbuf[j] = std::complex<float>(2.0f * audio[0] - audio[G - j], 0.0f);
            hbuf[G + n + j] =
                std::complex<float>(2.0f * audio[n - 1] - audio[n - 2 - j], 0.0f);
        }
        // Raised-cosine taper to zero over the outer half of each guard, so
        // the extended record meets the zero padding with no discontinuity.
        const int T = std::max(1, G / 2);
        for (int j = 0; j < T; j++) {
            float w = 0.5f - 0.5f * std::cos((float)M_PI * (j + 0.5f) / T);
            hbuf[j] *= w;                       // head guard: ramps 0 -> 1
            hbuf[G + n + G - 1 - j] *= w;       // tail guard: ramps 1 -> 0
        }
    }

    fft_complex(hbuf.data(), nfft_h);

    if (!limited) {
        // Plain Hilbert (byte-identical to the historical unlimited path):
        // DC and Nyquist left intact, positives doubled, negatives zeroed.
        for (int k = 1; k < nfft_h / 2; k++)
            hbuf[k] *= 2.0f;
    } else {
        // Used-carrier band edges (at nfft resolution) -> whole-buffer FFT
        // bins, with a 1-carrier margin so the band edges are never clipped.
        // Identical flat-band geometry to the legacy brick-wall.
        const int lo_cbin = cfg.used_carrier_bins.front();
        const int hi_cbin = cfg.used_carrier_bins.back();
        const double hz_lo = (double)(lo_cbin - 1) * cfg.sample_rate / cfg.nfft;
        const double hz_hi = (double)(hi_cbin + 1) * cfg.sample_rate / cfg.nfft;
        const int band_lo_k = std::max(1, (int)std::floor(hz_lo * nfft_h / cfg.sample_rate));
        const int band_hi_k = std::min(nfft_h / 2 - 1, (int)std::ceil(hz_hi * nfft_h / cfg.sample_rate));
        hbuf[0] = {0.0f, 0.0f};   // drop DC (out of band)

        // Transition widths in whole-buffer bins (carrier spacings scale by
        // nfft_h / cfg.nfft).
        const double bins_per_carrier = (double)nfft_h / cfg.nfft;
        const double w_lo = tapered ? trans_lo_carriers * bins_per_carrier : 0.0;
        const double w_hi = tapered ? trans_hi_carriers * bins_per_carrier : 0.0;
        // The low-side flat band extends a few carriers BELOW the
        // legacy edge before the roll-off starts: the DC-capped low side has
        // no room for a wide taper, and the meter self-noise from cutting the
        // signal skirt is dominated by the bins nearest the edge (measured:
        // opening the low side recovers 0.8-1.6 dB of top-end meter floor at
        // ~41 dB; the flat extension recovers most of that while the taper
        // still reaches zero by ~0-50 Hz, where the AC-coupling LF junk
        // lives). The default 3.25-carrier extension includes the minimum
        // quarter-carrier shoulder needed to keep the training-pair meter
        // calibrated after the authoritative carrier/pilot geometry change.
        // Apply the extension on every supported grid.  At the legal -150 Hz
        // endpoint the wide6k/1024 first carrier moves from 328.125 Hz to
        // 178.125 Hz; disabling this extension and capping the transition at
        // two carriers put that carrier below the 187.5 Hz hard stop before
        // CFO correction.  The finite transition still rejects LF stopband
        // energy while retaining the full shifted signal support.
        const float lo_ext_c = lo_flat_ext_carriers;
        const double w_lo_eff = w_lo;
        const int lo_ext_k = tapered
            ? std::max(1, band_lo_k - (int)std::lround(lo_ext_c
                                                       * bins_per_carrier))
            : band_lo_k;

        for (int k = 1; k < nfft_h / 2; k++) {
            float m;
            if (k >= lo_ext_k && k <= band_hi_k) {
                m = 1.0f;                         // flat passband (never narrower
                                                  // than the legacy brick-wall)
            } else if (k < lo_ext_k && lo_ext_k - k <= w_lo_eff) {
                double d = (double)(lo_ext_k - k);
                m = std::pow(0.5f * (1.0f + (float)std::cos(M_PI * d / (w_lo_eff + 1.0))),
                             trans_profile_pow);
            } else if (k > band_hi_k && k - band_hi_k <= w_hi) {
                double d = (double)(k - band_hi_k);
                m = std::pow(0.5f * (1.0f + (float)std::cos(M_PI * d / (w_hi + 1.0))),
                             trans_profile_pow);
            } else {
                m = 0.0f;                         // stopband: still exactly zero
            }
            hbuf[k] *= 2.0f * m;
        }
        hbuf[nfft_h / 2] = {0.0f, 0.0f};          // Nyquist: out of band
    }
    for (int k = nfft_h / 2 + 1; k < nfft_h; k++)
        hbuf[k] = {0.0f, 0.0f};

    ifft_complex(hbuf.data(), nfft_h);

    std::vector<std::complex<float>> out(n);
    for (int i = 0; i < n; i++)
        out[i] = hbuf[G + i];
    return out;
}

} // namespace iris
