#ifndef IRIS_MFSK_ACK_H
#define IRIS_MFSK_ACK_H

// MFSK Tone ACK for OFDM-KISS half-duplex MAC
//
// Replaces full OFDM S-frames (544ms at O1) with a short tone burst (~249ms)
// for ACK/RR delivery. Non-coherent detection via FFT peak finding — no
// preamble sync, channel estimation, or LDPC decode needed. Works at much
// lower SNR than OFDM and is immune to the timing/self-hear issues that
// cause 89% OFDM ACK loss on the reverse path.
//
// Design: M=16 tones in OFDM passband (Welch-Costas, p=17, g=5)
//   8 ACK base symbols + 3 N(R) suffix symbols + 3 level-proposal suffix = 14
//   Symbol period = 1024 samples (21.33ms, no CP for MFSK)
//   Total burst: 14 * 21.33ms = 299.7ms
//   Detection: FFT peak in M bins, threshold 5/8 matched symbols
//   False alarm: ~0.2% per second of listening at threshold 5/8
//
// Receiver-drives-rate (SUPER-ACK): the level-proposal suffix carries B's
// ABSOLUTE proposed forward O-level (the level B's decode-margin climb has
// validated it can receive) so the DATA transmitter A adopts the rate the
// RECEIVER can sustain (VARA/ALE/STANAG model). Absolute (not relative/skip)
// so a dropped ACK is idempotent — A just holds and the next ACK re-asserts.
// The base 8 ACK symbols and the N(R) suffix are BYTE-FOR-BYTE unchanged, so
// a legacy peer still detects on the 8 base symbols and parses N(R) at the
// same offset; it simply ignores the 3 trailing level symbols. The level
// field is self-rejecting: 2/3 majority vote + range check; on any doubt it
// is ABSENT (proposed_level = -1) and A holds its current level.

#include <vector>
#include <complex>
#include <cstdint>

namespace iris {

struct MfskAckResult {
    bool detected = false;
    int n_r = -1;           // N(R) from suffix (0-127; modulo-128 wide window), -1 if not decoded
    int pf = 0;             // P/F bit from suffix
    int matched = 0;        // Number of matched ACK symbols (out of 8)
    float metric = 0;       // Energy ratio metric (higher = better)
    int offset = -1;        // Sample offset of detection in buffer
    // Receiver-driven forward level proposal (absolute O-level 0..12).
    // -1 = ABSENT/rejected (no proposal, out of range, or failed majority
    // vote). A dropped/corrupt field is ALWAYS -1 → the transmitter holds
    // its current level (conservative), never mis-applies a bad rate.
    int proposed_level = -1;
    // #2 burst-epoch echo (0-7), -1 if ABSENT/rejected (no echo, failed
    // 2/3 majority, or parity mismatch).  The RECEIVER echoes back the 3-bit
    // epoch of the forward burst it just decoded so the SENDER can bind a
    // reverse ACK to the burst it acknowledges (kills buffered-tone
    // re-attribution).  A dropped/corrupt field is ALWAYS -1 → the sender's
    // guard falls back to advisory (fail-open toward the #1 live-N(R) fix).
    int epoch = -1;
};

class MfskAck {
public:
    // Configuration — matches Iris OFDM NFFT=1024
    static constexpr int M = 16;                // Tones per symbol
    static constexpr int NFFT = 1024;           // FFT size
    static constexpr int SYM_SAMPLES = NFFT;    // No CP for MFSK (tone detection only)
    static constexpr int ACK_LEN = 8;           // Welch-Costas base pattern length
    static constexpr int NR_REPS = 3;           // repetitions PER N(R) tone (majority vote)
    // N(R) is carried in TWO M=16 tones (7-bit N(R) + P/F) so the modulo-128 wide
    // window's cumulative ACK point is unambiguous.  tone-lo = N(R)&0x0F; tone-hi =
    // ((N(R)>>4)&0x07) | (P/F<<3).  Fixed 2-tone format in ALL modes (mod-8 N(R) 0-7
    // just leaves tone-hi's N(R) bits zero) — both Iris ends rebuild, no version bit.
    static constexpr int NR_TONES = 2;          // low-nibble + high-nibble N(R) tones
    static constexpr int NR_SYMS = NR_TONES * NR_REPS;  // 6 N(R) symbols
    static constexpr int LEVEL_REPS = 3;        // Forward-level proposal suffix reps
    static constexpr int EPOCH_REPS = 3;        // #2 burst-epoch (3-bit+parity) suffix reps
    static constexpr int TOTAL_SYMS = ACK_LEN + NR_SYMS + LEVEL_REPS + EPOCH_REPS;  // 20 symbols
    static constexpr int TOTAL_SAMPLES = TOTAL_SYMS * SYM_SAMPLES;  // 20480 (~427 ms)
    static constexpr int MATCH_THRESHOLD = 5;   // Min matched ACK symbols (of 8)
    static constexpr int TONE_HOP = 7;          // Coprime with 16

    // Welch-Costas ACK pattern (p=17, g=5) — same as Mercury M=16
    static constexpr int ACK_TONES[ACK_LEN] = {4, 7, 5, 12, 13, 1, 9, 15};

    MfskAck() = default;

    // Initialize with the first FFT bin for tones.
    // Tones occupy bins [first_bin, first_bin + M).
    // Call after probe negotiates passband.
    void init(int first_bin, int sample_rate = 48000);

    // TX: Generate MFSK ACK burst as real audio samples.
    // n_r: AX.25 N(R) value (0-127), pf: P/F bit (0 or 1)
    // proposed_level: absolute forward O-level the RECEIVER endorses for the
    //   data sender (0..NUM_OFDM_SPEED_LEVELS-1). -1 (default) = no proposal;
    //   an out-of-range value is also emitted as the reserved "no proposal"
    //   tone so the peer's range check drops it.
    // amplitude: peak amplitude (0.0-1.0)
    // burst_epoch: #2 3-bit epoch (0-7) of the forward burst being acked, or -1
    //   (default) = no echo -> a parity-violating tone the RX rejects (epoch=-1).
    std::vector<float> generate(int n_r, int pf = 0, int proposed_level = -1,
                                int burst_epoch = -1, float amplitude = 0.5f) const;

    // #2 epoch codec (3-bit epoch + even-parity bit -> one M=16 tone).  Exposed
    // for the unit test.  epoch_to_tone(-1 or out-of-range) -> a parity-VIOLATING
    // tone so tone_to_epoch() returns -1 (ABSENT).  tone_to_epoch checks parity.
    static int epoch_to_tone(int epoch) {
        if (epoch < 0 || epoch > 7) return 0x01;   // parity-violating -> ABSENT
        int p = ((epoch) ^ (epoch >> 1) ^ (epoch >> 2)) & 1;
        return ((epoch & 0x07) << 1) | p;
    }
    static int tone_to_epoch(int tone) {
        int e = (tone >> 1) & 0x07;
        int p = tone & 1;
        return (((e ^ (e >> 1) ^ (e >> 2)) & 1) == p) ? e : -1;
    }

    // RX: Detect MFSK ACK in audio buffer.
    // Slides a window across the buffer at SYM_SAMPLES stride.
    // Returns best detection if above threshold.
    MfskAckResult detect(const float* audio, int n_samples) const;

    bool is_initialized() const { return initialized_; }

    // Minimum audio samples needed for detection
    int min_detect_samples() const { return TOTAL_SAMPLES; }

    // Burst duration in seconds
    constexpr float burst_duration_s() const {
        return (float)TOTAL_SAMPLES / sample_rate_;
    }

private:
    int sample_rate_ = 48000;
    int first_bin_ = 0;
    bool initialized_ = false;

    // Precomputed tone frequencies (Hz)
    float tone_freq_[M] = {};

    // Expected tone index for ACK symbol s (with hopping)
    int ack_tone(int s) const {
        return (ACK_TONES[s % ACK_LEN] + s * TONE_HOP) % M;
    }

    // FFT bin for a given tone index
    int tone_bin(int tone) const { return first_bin_ + tone; }

    // Goertzel magnitude squared for a specific bin
    static float goertzel_mag2(const float* x, int N, int k);
};

} // namespace iris

#endif // IRIS_MFSK_ACK_H
