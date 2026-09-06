#include "mfsk/mfsk_ack.h"
#include "engine/speed_level.h"   // NUM_OFDM_SPEED_LEVELS (level-field range check)
#include "common/logging.h"
#include <cmath>
#include <algorithm>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace iris {

void MfskAck::init(int first_bin, int sample_rate) {
    initialized_ = false;
    if (sample_rate <= 0 || first_bin < 0 || first_bin > NFFT - M)
        return;

    first_bin_ = first_bin;
    sample_rate_ = sample_rate;

    // Precompute tone frequencies
    float bin_hz = (float)sample_rate / NFFT;
    for (int t = 0; t < M; t++)
        tone_freq_[t] = (first_bin + t) * bin_hz;

    initialized_ = true;
    IRIS_LOG("[MFSK-ACK] init: first_bin=%d, tones %.0f-%.0f Hz, burst=%.0fms",
             first_bin, tone_freq_[0], tone_freq_[M - 1], 1000.0f * TOTAL_SAMPLES / sample_rate_);
}

// Goertzel algorithm: compute |X[k]|^2 for a single DFT bin k
// Much more efficient than full FFT when only checking M bins out of N.
float MfskAck::goertzel_mag2(const float* x, int N, int k) {
    float w = 2.0f * (float)M_PI * k / N;
    float coeff = 2.0f * cosf(w);
    float s0 = 0, s1 = 0, s2 = 0;
    for (int i = 0; i < N; i++) {
        s0 = x[i] + coeff * s1 - s2;
        s2 = s1;
        s1 = s0;
    }
    // |X[k]|^2 = s1^2 + s2^2 - coeff*s1*s2
    return s1 * s1 + s2 * s2 - coeff * s1 * s2;
}

std::vector<float> MfskAck::generate(int n_r, int pf, int proposed_level,
                                     int burst_epoch, float amplitude) const {
    if (!initialized_) return {};

    std::vector<float> audio(TOTAL_SAMPLES, 0.0f);

    // Encode a 7-bit N(R) + P/F into TWO M=16 tones (modulo-128 wide window needs an
    // unambiguous cumulative ACK point up to K=127):
    //   tone-lo = N(R) & 0x0F                     (low nibble)
    //   tone-hi = ((N(R)>>4) & 0x07) | (P/F<<3)   (high 3 bits + P/F)
    // In mod-8 mode N(R) is 0-7 so tone-hi's N(R) bits are 0 (P/F still carried).
    int nr_lo_tone = n_r & 0x0F;
    int nr_hi_tone = ((n_r >> 4) & 0x07) | ((pf & 0x01) << 3);

    // Encode the absolute forward-level proposal into a single M=16 tone.
    // A valid in-range level maps 1:1 to its tone index; anything else (no
    // proposal, or out of range) uses the reserved top tone (M-1), which the
    // receiver's range check rejects → treated as ABSENT (transmitter holds).
    int level_tone = (proposed_level >= 0 && proposed_level < NUM_OFDM_SPEED_LEVELS)
                         ? proposed_level
                         : (M - 1);

    // #2 burst-epoch echo: 3-bit epoch + even-parity bit -> one M=16 tone.  An
    // absent/invalid epoch maps to a parity-violating tone so the RX rejects it
    // (epoch = -1, ABSENT).  The base 8 ACK symbols are BYTE-FOR-BYTE unchanged
    // (D0 untouched); N(R)/level/epoch are appended suffixes.
    int epoch_tone = epoch_to_tone(burst_epoch);

    // Suffix symbol offsets (see mfsk_ack.h layout).
    const int off_nr_hi = ACK_LEN + NR_REPS;
    const int off_level = ACK_LEN + NR_SYMS;
    const int off_epoch = ACK_LEN + NR_SYMS + LEVEL_REPS;

    for (int s = 0; s < TOTAL_SYMS; s++) {
        int tone;
        if (s < ACK_LEN) {
            tone = ack_tone(s);
        } else if (s < off_nr_hi) {
            tone = nr_lo_tone;   // N(R) low nibble repeated NR_REPS times
        } else if (s < off_level) {
            tone = nr_hi_tone;   // N(R) high bits + P/F repeated NR_REPS times
        } else if (s < off_epoch) {
            tone = level_tone;   // forward-level proposal repeated LEVEL_REPS times
        } else {
            tone = epoch_tone;   // burst-epoch echo repeated EPOCH_REPS times
        }

        float freq = tone_freq_[tone];
        int base = s * SYM_SAMPLES;

        // Generate sinusoid with raised-cosine ramp (8 samples each end)
        // to reduce spectral splatter between symbols.
        constexpr int RAMP = 8;
        for (int i = 0; i < SYM_SAMPLES; i++) {
            float t = (float)i / sample_rate_;
            float env = 1.0f;
            if (i < RAMP)
                env = 0.5f * (1.0f - cosf((float)M_PI * i / RAMP));
            else if (i >= SYM_SAMPLES - RAMP)
                env = 0.5f * (1.0f - cosf((float)M_PI * (SYM_SAMPLES - 1 - i) / RAMP));
            audio[base + i] = amplitude * env * sinf(2.0f * (float)M_PI * freq * t);
        }
    }

    return audio;
}

MfskAckResult MfskAck::detect(const float* audio, int n_samples) const {
    if (!initialized_ || n_samples < TOTAL_SAMPLES)
        return {};

    MfskAckResult best;
    constexpr int DETECT_STEP = SYM_SAMPLES / 2;
    int n_positions = (n_samples - TOTAL_SAMPLES) / DETECT_STEP + 1;

    for (int pos = 0; pos < n_positions; pos++) {
        int offset = pos * DETECT_STEP;
        int matched = 0;
        float metric = 0;

        // Check ACK base pattern (8 symbols)
        for (int s = 0; s < ACK_LEN; s++) {
            const float* sym = audio + offset + s * SYM_SAMPLES;
            int expected = ack_tone(s);
            int expected_bin = tone_bin(expected);

            // Compute energy for all M tones via Goertzel
            float peak_e = -1;
            int peak_tone = -1;
            float expected_e = 0;
            float total_e = 0;

            for (int t = 0; t < M; t++) {
                float e = goertzel_mag2(sym, NFFT, tone_bin(t));
                total_e += e;
                if (e > peak_e) {
                    peak_e = e;
                    peak_tone = t;
                }
                if (t == expected)
                    expected_e = e;
            }

            // Also check mirror bin (real signal has symmetric spectrum)
            int mirror_bin = (NFFT - expected_bin) % NFFT;
            bool mirror_in_range = false;
            for (int t = 0; t < M; t++) {
                if (tone_bin(t) == mirror_bin) {
                    mirror_in_range = true;
                    break;
                }
            }

            // Peak must be the expected tone (or mirror if in range)
            if (peak_e > 0 && (peak_tone == expected ||
                (mirror_in_range && tone_bin(peak_tone) == mirror_bin))) {
                matched++;
                if (total_e > 0)
                    metric += expected_e / total_e;
            }
        }

        if (matched > best.matched ||
            (matched == best.matched && metric > best.metric)) {
            best.matched = matched;
            best.metric = metric;
            best.offset = offset;
        }
    }

    if (best.matched >= MATCH_THRESHOLD) {
        best.detected = true;

        // 2/3 majority-vote the M-ary tone in a run of `reps` symbols starting at
        // suffix symbol `sym_off`.  Returns the winning tone, or -1 on no majority.
        auto vote_tone = [&](int sym_off, int reps) -> int {
            int v[M] = {};
            for (int r = 0; r < reps; r++) {
                const float* sym = audio + best.offset + (sym_off + r) * SYM_SAMPLES;
                float peak_e = -1;
                int peak_tone = -1;
                for (int t = 0; t < M; t++) {
                    float e = goertzel_mag2(sym, NFFT, tone_bin(t));
                    if (e > peak_e) { peak_e = e; peak_tone = t; }
                }
                if (peak_e > 0 && peak_tone >= 0 && peak_tone < M) v[peak_tone]++;
            }
            int bt = 0;
            for (int t = 1; t < M; t++) if (v[t] > v[bt]) bt = t;
            return (v[bt] >= 2) ? bt : -1;   // require >= 2/3 agreement
        };

        // Decode the 7-bit N(R) + P/F from the two suffix tones (low nibble, then
        // high 3 bits + P/F).  If EITHER tone fails its 2/3 vote, N(R) stays -1
        // (ABSENT) so the sender holds V(A) (the cheap miss direction).
        const int off_nr_hi = ACK_LEN + NR_REPS;
        const int off_level = ACK_LEN + NR_SYMS;
        const int off_epoch = ACK_LEN + NR_SYMS + LEVEL_REPS;
        int nr_lo = vote_tone(ACK_LEN, NR_REPS);
        int nr_hi = vote_tone(off_nr_hi, NR_REPS);
        if (nr_lo >= 0 && nr_hi >= 0) {
            best.n_r = ((nr_hi & 0x07) << 4) | (nr_lo & 0x0F);
            best.pf  = (nr_hi >> 3) & 0x01;
        }

        // Forward-level proposal (2/3 vote + range check; out-of-range/no-majority
        // -> ABSENT so the transmitter holds its current level).
        int lbest = vote_tone(off_level, LEVEL_REPS);
        if (lbest >= 0 && lbest < NUM_OFDM_SPEED_LEVELS)
            best.proposed_level = lbest;  // both guards passed → valid proposal

        // #2 burst-epoch (2/3 vote + parity inside tone_to_epoch; -1 on any doubt so
        // the sender's guard falls back to advisory rather than mis-binding).
        int ebest = vote_tone(off_epoch, EPOCH_REPS);
        if (ebest >= 0)
            best.epoch = tone_to_epoch(ebest);

        IRIS_LOG("[MFSK-ACK] detected: %d/%d matched, metric=%.2f, N(R)=%d, PF=%d, L_prop=%d, epoch=%d, offset=%d",
                 best.matched, ACK_LEN, best.metric, best.n_r, best.pf,
                 best.proposed_level, best.epoch, best.offset);
    }

    return best;
}

} // namespace iris
