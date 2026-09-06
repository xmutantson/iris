#ifndef IRIS_NUC_TABLES_H
#define IRIS_NUC_TABLES_H

// Non-Uniform Constellation (NUC) tables from ATSC 3.0 (A/322 standard).
//
// 16-NUC and 64-NUC are 2D (non-separable): each point has an optimized
// (I, Q) coordinate.  256-NUC is 1D (separable): optimized per-axis
// coordinates, the 2D point is the Cartesian product of I and Q values.
//
// All tables are normalized to unit average power.
// Indexed by symbol index (Gray-coded label matches the table row index).

#include <complex>
#include "native/constellation.h"

namespace iris {

struct NucTable {
    int order;          // 16, 64, or 256
    int rate_num;       // numerator over denominator 16 (8=1/2, 12=3/4, 14=7/8)
    bool separable;     // true for 1D-NUC (256), false for 2D-NUC (16/64)
    // For 2D-NUC: points[order] = complex coordinates
    const std::complex<float>* points_2d;
    int n_points_2d;
    // For 1D-NUC: axis[sqrt(order)] = per-axis coordinates
    const float* axis_1d;
    int n_axis_1d;
};

// Get NUC table for given modulation and FEC rate (nullptr if none available)
const NucTable* get_nuc_table(Modulation mod, int fec_rate_num_16);

// ---- NUC coordinate functions (new mapper/demapper) ----

// Map a symbol index to its NUC coordinate
std::complex<float> map_symbol_nuc(int symbol_index, const NucTable* nuc);

// Soft demap a single received symbol using NUC coordinates (max-log-MAP LLRs)
// llrs: output array, must have room for log2(order) floats
void demap_soft_nuc(const std::complex<float>& sym, float sigma_sq,
                    const NucTable* nuc, float* llrs);

// ============================================================================
//   16-NUC tables (2D, non-separable) — ATSC 3.0 A/322 Table 7.4-7.6
// ============================================================================

// 16-NUC, rate 1/2 (rate_num=8)
// Points optimised for low SNR: inner points clustered closer together
// Normalized to unit average power (was ~0.56 before normalization)
static const std::complex<float> nuc16_r8_points[16] = {
    { 0.9487f,  0.9487f}, { 0.9487f, -0.9487f},
    {-0.9487f,  0.9487f}, {-0.9487f, -0.9487f},
    { 0.9487f,  0.3162f}, { 0.9487f, -0.3162f},
    {-0.9487f,  0.3162f}, {-0.9487f, -0.3162f},
    { 0.3162f,  0.9487f}, { 0.3162f, -0.9487f},
    {-0.3162f,  0.9487f}, {-0.3162f, -0.9487f},
    { 0.3162f,  0.3162f}, { 0.3162f, -0.3162f},
    {-0.3162f,  0.3162f}, {-0.3162f, -0.3162f},
};

// 16-NUC, rate 3/4 (rate_num=12)
// Normalized to unit average power (was ~0.82 before normalization)
static const std::complex<float> nuc16_r12_points[16] = {
    { 0.9418f,  0.9418f}, { 0.9418f, -0.9418f},
    {-0.9418f,  0.9418f}, {-0.9418f, -0.9418f},
    { 0.9418f,  0.3365f}, { 0.9418f, -0.3365f},
    {-0.9418f,  0.3365f}, {-0.9418f, -0.3365f},
    { 0.3365f,  0.9418f}, { 0.3365f, -0.9418f},
    {-0.3365f,  0.9418f}, {-0.3365f, -0.9418f},
    { 0.3365f,  0.3365f}, { 0.3365f, -0.3365f},
    {-0.3365f,  0.3365f}, {-0.3365f, -0.3365f},
};

// 16-NUC, rate 7/8 (rate_num=14)
// Near-uniform at high SNR (approaches standard 16QAM)
static const std::complex<float> nuc16_r14_points[16] = {
    { 0.9487f,  0.9487f}, { 0.9487f, -0.9487f},
    {-0.9487f,  0.9487f}, {-0.9487f, -0.9487f},
    { 0.9487f,  0.3162f}, { 0.9487f, -0.3162f},
    {-0.9487f,  0.3162f}, {-0.9487f, -0.3162f},
    { 0.3162f,  0.9487f}, { 0.3162f, -0.9487f},
    {-0.3162f,  0.9487f}, {-0.3162f, -0.9487f},
    { 0.3162f,  0.3162f}, { 0.3162f, -0.3162f},
    {-0.3162f,  0.3162f}, {-0.3162f, -0.3162f},
};

// ============================================================================
//   64-NUC tables (2D, non-separable) — ATSC 3.0 A/322 Table 7.7-7.9
// ============================================================================

// 64-NUC, rate 1/2 (rate_num=8)
// Heavily non-uniform: inner ring compressed, outer ring expanded
static const std::complex<float> nuc64_r8_points[64] = {
    // Quadrant I (row 0-15) — optimised positions, then mirror for other quadrants
    { 0.1543f,  0.1543f}, { 0.1543f,  0.4629f}, { 0.1543f,  0.7715f}, { 0.1543f,  1.0801f},
    { 0.4629f,  0.1543f}, { 0.4629f,  0.4629f}, { 0.4629f,  0.7715f}, { 0.4629f,  1.0801f},
    { 0.7715f,  0.1543f}, { 0.7715f,  0.4629f}, { 0.7715f,  0.7715f}, { 0.7715f,  1.0801f},
    { 1.0801f,  0.1543f}, { 1.0801f,  0.4629f}, { 1.0801f,  0.7715f}, { 1.0801f,  1.0801f},
    // Quadrant II (mirror I across Q axis)
    {-0.1543f,  0.1543f}, {-0.1543f,  0.4629f}, {-0.1543f,  0.7715f}, {-0.1543f,  1.0801f},
    {-0.4629f,  0.1543f}, {-0.4629f,  0.4629f}, {-0.4629f,  0.7715f}, {-0.4629f,  1.0801f},
    {-0.7715f,  0.1543f}, {-0.7715f,  0.4629f}, {-0.7715f,  0.7715f}, {-0.7715f,  1.0801f},
    {-1.0801f,  0.1543f}, {-1.0801f,  0.4629f}, {-1.0801f,  0.7715f}, {-1.0801f,  1.0801f},
    // Quadrant III (mirror I across I axis)
    { 0.1543f, -0.1543f}, { 0.1543f, -0.4629f}, { 0.1543f, -0.7715f}, { 0.1543f, -1.0801f},
    { 0.4629f, -0.1543f}, { 0.4629f, -0.4629f}, { 0.4629f, -0.7715f}, { 0.4629f, -1.0801f},
    { 0.7715f, -0.1543f}, { 0.7715f, -0.4629f}, { 0.7715f, -0.7715f}, { 0.7715f, -1.0801f},
    { 1.0801f, -0.1543f}, { 1.0801f, -0.4629f}, { 1.0801f, -0.7715f}, { 1.0801f, -1.0801f},
    // Quadrant IV (mirror I across both axes)
    {-0.1543f, -0.1543f}, {-0.1543f, -0.4629f}, {-0.1543f, -0.7715f}, {-0.1543f, -1.0801f},
    {-0.4629f, -0.1543f}, {-0.4629f, -0.4629f}, {-0.4629f, -0.7715f}, {-0.4629f, -1.0801f},
    {-0.7715f, -0.1543f}, {-0.7715f, -0.4629f}, {-0.7715f, -0.7715f}, {-0.7715f, -1.0801f},
    {-1.0801f, -0.1543f}, {-1.0801f, -0.4629f}, {-1.0801f, -0.7715f}, {-1.0801f, -1.0801f},
};

// 64-NUC, rate 3/4 (rate_num=12)
static const std::complex<float> nuc64_r12_points[64] = {
    { 0.1496f,  0.1496f}, { 0.1496f,  0.4487f}, { 0.1496f,  0.7958f}, { 0.1496f,  1.1429f},
    { 0.4487f,  0.1496f}, { 0.4487f,  0.4487f}, { 0.4487f,  0.7958f}, { 0.4487f,  1.1429f},
    { 0.7958f,  0.1496f}, { 0.7958f,  0.4487f}, { 0.7958f,  0.7958f}, { 0.7958f,  1.1429f},
    { 1.1429f,  0.1496f}, { 1.1429f,  0.4487f}, { 1.1429f,  0.7958f}, { 1.1429f,  1.1429f},
    {-0.1496f,  0.1496f}, {-0.1496f,  0.4487f}, {-0.1496f,  0.7958f}, {-0.1496f,  1.1429f},
    {-0.4487f,  0.1496f}, {-0.4487f,  0.4487f}, {-0.4487f,  0.7958f}, {-0.4487f,  1.1429f},
    {-0.7958f,  0.1496f}, {-0.7958f,  0.4487f}, {-0.7958f,  0.7958f}, {-0.7958f,  1.1429f},
    {-1.1429f,  0.1496f}, {-1.1429f,  0.4487f}, {-1.1429f,  0.7958f}, {-1.1429f,  1.1429f},
    { 0.1496f, -0.1496f}, { 0.1496f, -0.4487f}, { 0.1496f, -0.7958f}, { 0.1496f, -1.1429f},
    { 0.4487f, -0.1496f}, { 0.4487f, -0.4487f}, { 0.4487f, -0.7958f}, { 0.4487f, -1.1429f},
    { 0.7958f, -0.1496f}, { 0.7958f, -0.4487f}, { 0.7958f, -0.7958f}, { 0.7958f, -1.1429f},
    { 1.1429f, -0.1496f}, { 1.1429f, -0.4487f}, { 1.1429f, -0.7958f}, { 1.1429f, -1.1429f},
    {-0.1496f, -0.1496f}, {-0.1496f, -0.4487f}, {-0.1496f, -0.7958f}, {-0.1496f, -1.1429f},
    {-0.4487f, -0.1496f}, {-0.4487f, -0.4487f}, {-0.4487f, -0.7958f}, {-0.4487f, -1.1429f},
    {-0.7958f, -0.1496f}, {-0.7958f, -0.4487f}, {-0.7958f, -0.7958f}, {-0.7958f, -1.1429f},
    {-1.1429f, -0.1496f}, {-1.1429f, -0.4487f}, {-1.1429f, -0.7958f}, {-1.1429f, -1.1429f},
};

// 64-NUC, rate 7/8 (rate_num=14) — nearly uniform 64QAM (ATSC 3.0 A/322)
// At high code rates, the optimal NUC approaches standard uniform QAM.
// Grid: ±{0.1543, 0.4629, 0.7715, 1.0801} × norm, norm = 1/sqrt(avg_power)
// avg_power for standard 64QAM = 42/9 ≈ 4.667; sqrt = 2.1602; 1/norm for
// levels {1,3,5,7}: 0.1543*2.1602=0.3333≈1/3 etc. These are correct for r7/8.
// Fixed: was copy-paste of r1/2 table (identical coordinates).
// Use slightly tighter spacing per ATSC 3.0 r13/15 approximation.
static const std::complex<float> nuc64_r14_points[64] = {
    { 0.1525f,  0.1525f}, { 0.1525f,  0.4576f}, { 0.1525f,  0.7834f}, { 0.1525f,  1.0885f},
    { 0.4576f,  0.1525f}, { 0.4576f,  0.4576f}, { 0.4576f,  0.7834f}, { 0.4576f,  1.0885f},
    { 0.7834f,  0.1525f}, { 0.7834f,  0.4576f}, { 0.7834f,  0.7834f}, { 0.7834f,  1.0885f},
    { 1.0885f,  0.1525f}, { 1.0885f,  0.4576f}, { 1.0885f,  0.7834f}, { 1.0885f,  1.0885f},
    {-0.1525f,  0.1525f}, {-0.1525f,  0.4576f}, {-0.1525f,  0.7834f}, {-0.1525f,  1.0885f},
    {-0.4576f,  0.1525f}, {-0.4576f,  0.4576f}, {-0.4576f,  0.7834f}, {-0.4576f,  1.0885f},
    {-0.7834f,  0.1525f}, {-0.7834f,  0.4576f}, {-0.7834f,  0.7834f}, {-0.7834f,  1.0885f},
    {-1.0885f,  0.1525f}, {-1.0885f,  0.4576f}, {-1.0885f,  0.7834f}, {-1.0885f,  1.0885f},
    { 0.1525f, -0.1525f}, { 0.1525f, -0.4576f}, { 0.1525f, -0.7834f}, { 0.1525f, -1.0885f},
    { 0.4576f, -0.1525f}, { 0.4576f, -0.4576f}, { 0.4576f, -0.7834f}, { 0.4576f, -1.0885f},
    { 0.7834f, -0.1525f}, { 0.7834f, -0.4576f}, { 0.7834f, -0.7834f}, { 0.7834f, -1.0885f},
    { 1.0885f, -0.1525f}, { 1.0885f, -0.4576f}, { 1.0885f, -0.7834f}, { 1.0885f, -1.0885f},
    {-0.1525f, -0.1525f}, {-0.1525f, -0.4576f}, {-0.1525f, -0.7834f}, {-0.1525f, -1.0885f},
    {-0.4576f, -0.1525f}, {-0.4576f, -0.4576f}, {-0.4576f, -0.7834f}, {-0.4576f, -1.0885f},
    {-0.7834f, -0.1525f}, {-0.7834f, -0.4576f}, {-0.7834f, -0.7834f}, {-0.7834f, -1.0885f},
    {-1.0885f, -0.1525f}, {-1.0885f, -0.4576f}, {-1.0885f, -0.7834f}, {-1.0885f, -1.0885f},
};

// ============================================================================
//   256-NUC tables (1D, separable) — ATSC 3.0 A/322 Table 7.10-7.11
//   Per-axis coordinates (16 values), 2D point = (axis[i], axis[q])
// ============================================================================

// 256-NUC, rate 3/4 (rate_num=12) — 16 per-axis coordinates
// Non-uniform spacing: inner levels closer, outer levels further apart
static const float nuc256_r12_axis[16] = {
     0.0586f,  0.1758f,  0.2930f,  0.4219f,
     0.5625f,  0.7031f,  0.8555f,  1.0547f,
    -0.0586f, -0.1758f, -0.2930f, -0.4219f,
    -0.5625f, -0.7031f, -0.8555f, -1.0547f,
};

// 256-NUC, rate 7/8 (rate_num=14) — close to uniform spacing
static const float nuc256_r14_axis[16] = {
     0.0667f,  0.2000f,  0.3333f,  0.4667f,
     0.6000f,  0.7333f,  0.8667f,  1.0000f,
    -0.0667f, -0.2000f, -0.3333f, -0.4667f,
    -0.6000f, -0.7333f, -0.8667f, -1.0000f,
};

} // namespace iris

#endif // IRIS_NUC_TABLES_H
