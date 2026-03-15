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
    {"A2",  Modulation::QPSK,    3, 4,   7.0f,  0},   // QPSK 3/4 (raised from 6.0: FM de-emphasis margin)
    {"A3",  Modulation::QAM16,   1, 2,  12.0f,  0},   // 16QAM 1/2 (raised from 9: needs real margin on FM)
    {"A4",  Modulation::QAM16,   3, 4,  12.0f,  0},   // 16QAM 3/4
    {"A5",  Modulation::QAM64,   3, 4,  17.0f,  0},   // 64QAM 3/4
    {"A6",  Modulation::QAM64,   7, 8,  20.0f,  0},   // 64QAM 7/8
    {"A7",  Modulation::QAM256,  7, 8,  25.0f,  0},   // 256QAM 7/8
};

int snr_to_speed_level(float snr_db) {
    int level = 0;
    for (int i = NUM_SPEED_LEVELS - 1; i >= 0; i--) {
        // Require 2 dB margin above minimum
        if (snr_db >= SPEED_LEVELS[i].min_snr_db + 2.0f) {
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
