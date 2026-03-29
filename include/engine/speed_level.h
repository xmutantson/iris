#ifndef IRIS_SPEED_LEVEL_H
#define IRIS_SPEED_LEVEL_H

#include "native/constellation.h"
#include <cstdint>

namespace iris {

struct SpeedLevel {
    const char* name;
    Modulation modulation;
    int fec_rate_num;   // Numerator (e.g., 1 for 1/2)
    int fec_rate_den;   // Denominator (e.g., 2 for 1/2)
    float min_snr_db;   // Minimum SNR for reliable operation
    int net_bps_per_baud;  // Net bits per second per baud of symbol rate
};

// Mode A speed levels (2400 baud base)
constexpr int NUM_SPEED_LEVELS = 8;
extern const SpeedLevel SPEED_LEVELS[NUM_SPEED_LEVELS];

// OFDM O-level speed levels (uniform preset modulation + FEC rate).
// Maps 1:1 to kUniformPresets[] in ofdm_frame.cc (preset_id = level + 1).
constexpr int NUM_OFDM_SPEED_LEVELS = 13;
extern const SpeedLevel OFDM_SPEED_LEVELS[NUM_OFDM_SPEED_LEVELS];

// Get the speed level index appropriate for a given SNR
int snr_to_speed_level(float snr_db);

// Get the OFDM O-level index appropriate for a given SNR
// Picks highest O-level where SNR exceeds threshold + 1 dB margin
int ofdm_snr_to_speed_level(float snr_db);

// Calculate net throughput for a speed level at a given baud rate
int net_throughput(int level, int baud_rate);

// Mode multiplier: baud rate for each mode
int mode_baud_rate(char mode);  // 'A', 'B', 'C'

} // namespace iris

#endif
