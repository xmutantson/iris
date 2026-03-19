#include "engine/speed_level.h"

namespace iris {

// Net bits per baud = bits_per_symbol * fec_rate
// E.g., QPSK 3/4 = 2 * 3/4 = 1.5 bits per baud
// SNR thresholds calibrated for LDPC-coded performance (not uncoded AWGN).
// Gearshift adds 2 dB margin on top of these values.
const SpeedLevel SPEED_LEVELS[NUM_SPEED_LEVELS] = {
    // name       mod              num den  min_snr  net_bits/baud
    {"A0",  Modulation::BPSK,    1, 2,   0.0f,  0},   // BPSK 1/2
    {"A1",  Modulation::QPSK,    1, 2,   3.0f,  0},   // QPSK 1/2
    {"A2",  Modulation::QPSK,    3, 4,   5.0f,  0},   // QPSK 3/4 (lowered: Kalman smoother handles FM phase)
    {"A3",  Modulation::QAM16,   1, 2,  12.0f,  0},   // 16QAM 1/2 (raised from 9: needs real margin on FM)
    {"A4",  Modulation::QAM16,   3, 4,  12.0f,  0},   // 16QAM 3/4
    {"A5",  Modulation::QAM64,   3, 4,  17.0f,  0},   // 64QAM 3/4
    {"A6",  Modulation::QAM64,   7, 8,  20.0f,  0},   // 64QAM 7/8
    {"A7",  Modulation::QAM256,  7, 8,  25.0f,  0},   // 256QAM 7/8
};

// OFDM O-levels: maps 1:1 to uniform tone map presets (preset_id = level + 1).
// SNR thresholds raised for FM (deviation limiter distortion ≠ AWGN).
// Gearshift adds 1 dB margin on top of these values.
const SpeedLevel OFDM_SPEED_LEVELS[NUM_OFDM_SPEED_LEVELS] = {
    // name   mod                num den  min_snr  net_bits/baud
    {"O0",  Modulation::BPSK,    1, 2,   0.0f,  0},   // BPSK r1/2 — most robust
    {"O1",  Modulation::QPSK,    1, 2,   4.0f,  0},   // QPSK r1/2
    {"O2",  Modulation::QPSK,    3, 4,   8.0f,  0},   // QPSK r3/4
    {"O3",  Modulation::QAM16,   1, 2,  12.0f,  0},   // 16QAM r1/2
    {"O4",  Modulation::QAM16,   3, 4,  16.0f,  0},   // 16QAM r3/4
    {"O5",  Modulation::QAM64,   3, 4,  20.0f,  0},   // 64QAM r3/4
    {"O6",  Modulation::QAM64,   7, 8,  24.0f,  0},   // 64QAM r7/8
    {"O7",  Modulation::QAM256,  7, 8,  28.0f,  0},   // 256QAM r7/8 — data port only
};

int ofdm_snr_to_speed_level(float snr_db) {
    int level = 0;
    for (int i = NUM_OFDM_SPEED_LEVELS - 1; i >= 0; i--) {
        if (snr_db >= OFDM_SPEED_LEVELS[i].min_snr_db + 1.0f) {
            level = i;
            break;
        }
    }
    return level;
}

int snr_to_speed_level(float snr_db) {
    int level = 0;
    for (int i = NUM_SPEED_LEVELS - 1; i >= 0; i--) {
        // Require 1 dB margin above minimum (Kalman smoother provides phase stability)
        if (snr_db >= SPEED_LEVELS[i].min_snr_db + 1.0f) {
            level = i;
            break;
        }
    }
    return level;
}

int net_throughput(int level, int baud_rate) {
    if (level < 0 || level >= NUM_SPEED_LEVELS) return 0;
    const auto& sl = SPEED_LEVELS[level];
    int bps = bits_per_symbol(sl.modulation);
    // net = baud * bits_per_sym * fec_rate
    return baud_rate * bps * sl.fec_rate_num / sl.fec_rate_den;
}

int mode_baud_rate(char mode) {
    switch (mode) {
        case 'A': case 'a': return 2400;
        case 'B': case 'b': return 4800;
        case 'C': case 'c': return 9600;
        default: return 2400;
    }
}

} // namespace iris
