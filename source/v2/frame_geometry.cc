#include "v2/frame_geometry.h"

// Author: xmutantson

#include <algorithm>
#include <limits>
#include <stdexcept>

namespace iris::v2 {
namespace {

bool checked_add(std::uint64_t a, std::uint64_t b,
                 std::uint64_t& out) noexcept {
    if (b > std::numeric_limits<std::uint64_t>::max() - a) return false;
    out = a + b;
    return true;
}

bool checked_mul(std::uint64_t a, std::uint64_t b,
                 std::uint64_t& out) noexcept {
    if (a != 0 && b > std::numeric_limits<std::uint64_t>::max() / a) return false;
    out = a * b;
    return true;
}

bool supported_fec(LdpcRate fec) noexcept {
    switch (fec) {
    case LdpcRate::RATE_4_16:
    case LdpcRate::RATE_6_16:
    case LdpcRate::RATE_1_2:
    case LdpcRate::RATE_5_8:
    case LdpcRate::RATE_3_4:
    case LdpcRate::RATE_7_8:
        return LdpcCodec::codeword_size(fec) ==
                   static_cast<int>(FrameGeometry::kCodedBitsPerCodeword) &&
               LdpcCodec::block_size(fec) >= 48;
    default:
        return false;
    }
}

bool supported_bpc(std::uint8_t bpc) noexcept {
    return bpc == 0 || bpc == 1 || bpc == 2 || bpc == 4 || bpc == 5 ||
           bpc == 6 || bpc == 8 || bpc == 10;
}

bool same_tone_map(const FrameToneMap& a, const FrameToneMap& b) noexcept {
    return a.used_carrier_bins == b.used_carrier_bins &&
           a.pilot_carrier_bins == b.pilot_carrier_bins &&
           a.data_carrier_bins == b.data_carrier_bins &&
           a.coded_bits_per_data_carrier == b.coded_bits_per_data_carrier &&
           a.tone_map_id == b.tone_map_id &&
           a.use_non_uniform_constellations == b.use_non_uniform_constellations;
}

FrameGeometryValidation derive(const FrameToneMap& map, LdpcRate fec,
                               std::uint8_t codewords, std::uint32_t nfft,
                               std::uint32_t cp, std::uint32_t spacing,
                               std::uint64_t& bits_per_symbol,
                               std::uint64_t& data_symbols,
                               std::uint64_t& pilot_rows,
                               std::uint64_t& symbol_samples,
                               std::uint64_t& complete_samples) noexcept {
    auto fail = [](FrameGeometryError e) { return FrameGeometryValidation{e}; };
    if (!supported_fec(fec)) return fail(FrameGeometryError::UnsupportedFec);
    if (codewords < FrameGeometry::kMinCodewordCount ||
        codewords > FrameGeometry::kMaxCodewordCount)
        return fail(FrameGeometryError::InvalidCodewordCount);
    if ((nfft != 256 && nfft != 512 && nfft != 1024) || cp >= nfft)
        return fail(FrameGeometryError::InvalidFftOrCyclicPrefix);
    if (map.used_carrier_bins.empty() || map.pilot_carrier_bins.empty() ||
        map.data_carrier_bins.empty())
        return fail(FrameGeometryError::InvalidCarrierPlacement);
    if (map.coded_bits_per_data_carrier.size() != map.data_carrier_bins.size())
        return fail(FrameGeometryError::ToneMapLengthMismatch);

    auto valid_ordered_bins = [nfft](const std::vector<std::int32_t>& bins) {
        for (std::size_t i = 0; i < bins.size(); ++i) {
            if (bins[i] == 0 || bins[i] < -static_cast<std::int32_t>(nfft / 2) ||
                bins[i] >= static_cast<std::int32_t>(nfft / 2)) return false;
            if (i != 0 && bins[i - 1] >= bins[i]) return false;
        }
        return true;
    };
    if (!valid_ordered_bins(map.used_carrier_bins) ||
        !valid_ordered_bins(map.pilot_carrier_bins) ||
        !valid_ordered_bins(map.data_carrier_bins))
        return fail(FrameGeometryError::CarrierOrderViolation);

    std::vector<std::int32_t> joined = map.pilot_carrier_bins;
    joined.insert(joined.end(), map.data_carrier_bins.begin(),
                  map.data_carrier_bins.end());
    std::sort(joined.begin(), joined.end());
    if (std::adjacent_find(joined.begin(), joined.end()) != joined.end())
        return fail(FrameGeometryError::DuplicateCarrier);
    if (joined != map.used_carrier_bins)
        return fail(FrameGeometryError::CarrierSetRelationshipViolation);

    bits_per_symbol = 0;
    for (std::uint8_t bpc : map.coded_bits_per_data_carrier) {
        if (!supported_bpc(bpc))
            return fail(FrameGeometryError::UnsupportedConstellation);
        if (!checked_add(bits_per_symbol, bpc, bits_per_symbol))
            return fail(FrameGeometryError::ArithmeticOverflow);
    }
    if (bits_per_symbol == 0)
        return fail(FrameGeometryError::ZeroCodedBitsPerDataSymbol);

    std::uint64_t coded_bits = 0;
    if (!checked_mul(FrameGeometry::kCodedBitsPerCodeword, codewords, coded_bits))
        return fail(FrameGeometryError::ArithmeticOverflow);
    data_symbols = coded_bits / bits_per_symbol +
                   static_cast<std::uint64_t>(coded_bits % bits_per_symbol != 0);
    pilot_rows = spacing == 0 ? 0 : (data_symbols - 1) / spacing;
    if (!checked_add(nfft, cp, symbol_samples))
        return fail(FrameGeometryError::ArithmeticOverflow);
    std::uint64_t total_symbols = 0;
    if (!checked_add(FrameGeometry::kFixedNonDataSymbols, data_symbols,
                     total_symbols) ||
        !checked_add(total_symbols, pilot_rows, total_symbols) ||
        !checked_mul(total_symbols, symbol_samples, complete_samples))
        return fail(FrameGeometryError::ArithmeticOverflow);
    return {};
}

} // namespace

FrameGeometryValidation validate_admitted_frame_profiles(
    const AdmittedFrameProfileSet& admitted) noexcept {
    auto fail = [](FrameGeometryError e) { return FrameGeometryValidation{e}; };
    if (admitted.authoritative_sample_rate_hz == 0)
        return fail(FrameGeometryError::InvalidAuthoritativeSampleRate);
    if (admitted.profiles.empty()) return fail(FrameGeometryError::InvalidProfileSet);

    std::uint64_t max_data = 0, max_complete = 0, max_remaining = 0;
    std::uint32_t max_margin = 0;
    std::uint16_t previous_id = 0;
    for (std::size_t i = 0; i < admitted.profiles.size(); ++i) {
        const auto& p = admitted.profiles[i];
        if (p.profile_id.value == 0 || (i != 0 && p.profile_id.value <= previous_id))
            return fail(FrameGeometryError::InvalidProfileSet);
        previous_id = p.profile_id.value;
        if (p.min_codeword_count < FrameGeometry::kMinCodewordCount ||
            p.max_codeword_count > FrameGeometry::kMaxCodewordCount ||
            p.min_codeword_count > p.max_codeword_count)
            return fail(FrameGeometryError::InvalidCodewordCount);
        for (std::size_t j = 0; j < i; ++j) {
            if (p.tone_map.tone_map_id == admitted.profiles[j].tone_map.tone_map_id &&
                !same_tone_map(p.tone_map, admitted.profiles[j].tone_map))
                return fail(FrameGeometryError::ToneMapIdMismatch);
        }
        std::uint64_t b = 0, d = 0, rows = 0, l = 0, samples = 0;
        auto v = derive(p.tone_map, p.fec, p.max_codeword_count, p.nfft,
                        p.cyclic_prefix_samples, p.pilot_row_spacing,
                        b, d, rows, l, samples);
        if (v.error != FrameGeometryError::None) return v;
        max_data = std::max(max_data, d);
        max_complete = std::max(max_complete, samples);
        std::uint64_t remaining = samples - l;
        if (!checked_add(remaining, p.timing_margin_samples, remaining))
            return fail(FrameGeometryError::ArithmeticOverflow);
        max_remaining = std::max(max_remaining, remaining);
        max_margin = std::max(max_margin, p.timing_margin_samples);
    }
    if (admitted.maximum_data_symbol_count != max_data ||
        admitted.maximum_complete_sample_count != max_complete ||
        admitted.maximum_first_training_remaining_samples != max_remaining ||
        admitted.maximum_timing_margin_samples != max_margin)
        return fail(FrameGeometryError::SharedMaximumMismatch);
    return {};
}

FrameGeometryValidation validate_selected_frame_profiles(
    const AdmittedFrameProfileSet& admitted,
    const SelectedSessionConfig& selected) noexcept {
    auto v = validate_admitted_frame_profiles(admitted);
    if (v.error != FrameGeometryError::None) return v;
    if (selected.authoritative_sample_rate_hz !=
        admitted.authoritative_sample_rate_hz)
        return {FrameGeometryError::InvalidAuthoritativeSampleRate};
    std::uint8_t max_c = 0;
    for (const auto& p : admitted.profiles)
        max_c = std::max(max_c, p.max_codeword_count);
    if (selected.max_frame_codeword_count != max_c)
        return {FrameGeometryError::InvalidCodewordCount};
    return {};
}

FrameGeometryValidation FrameGeometry::validate(
    const FrameGeometryInput& input,
    const AdmittedFrameProfileSet& admitted) noexcept {
    auto set_validation = validate_admitted_frame_profiles(admitted);
    if (set_validation.error != FrameGeometryError::None) return set_validation;
    const AdmittedFrameProfile* match = nullptr;
    for (const auto& p : admitted.profiles) {
        if (p.profile_id.value == input.profile_id.value) {
            match = &p;
            break;
        }
    }
    if (!match) return {FrameGeometryError::UnsupportedProfile};
    if (!same_tone_map(input.tone_map, match->tone_map))
        return {input.tone_map.tone_map_id == match->tone_map.tone_map_id
                    ? FrameGeometryError::ProfileDefinitionMismatch
                    : FrameGeometryError::ToneMapIdMismatch};
    if (input.fec != match->fec || input.nfft != match->nfft ||
        input.cyclic_prefix_samples != match->cyclic_prefix_samples ||
        input.pilot_row_spacing != match->pilot_row_spacing)
        return {FrameGeometryError::ProfileDefinitionMismatch};
    if (input.codeword_count < match->min_codeword_count ||
        input.codeword_count > match->max_codeword_count)
        return {FrameGeometryError::InvalidCodewordCount};
    std::uint64_t b = 0, d = 0, p = 0, l = 0, samples = 0;
    return derive(input.tone_map, input.fec, input.codeword_count, input.nfft,
                  input.cyclic_prefix_samples, input.pilot_row_spacing,
                  b, d, p, l, samples);
}

FrameGeometry::FrameGeometry(const FrameGeometryInput& input,
                             const AdmittedFrameProfileSet& admitted) {
    auto validation = validate(input, admitted);
    if (validation.error != FrameGeometryError::None)
        throw std::invalid_argument("invalid OFDM frame geometry");
    profile_id_ = input.profile_id;
    tone_map_ = input.tone_map;
    fec_ = input.fec;
    codeword_count_ = input.codeword_count;
    nfft_ = input.nfft;
    cyclic_prefix_samples_ = input.cyclic_prefix_samples;
    pilot_row_spacing_ = input.pilot_row_spacing;
    authoritative_sample_rate_hz_ = admitted.authoritative_sample_rate_hz;
    for (const auto& profile : admitted.profiles) {
        if (profile.profile_id.value == profile_id_.value) {
            timing_margin_samples_ = profile.timing_margin_samples;
            break;
        }
    }
    derive(tone_map_, fec_, codeword_count_, nfft_, cyclic_prefix_samples_,
           pilot_row_spacing_, coded_bits_per_data_symbol_, data_symbol_count_,
           pilot_rows_, symbol_length_samples_, complete_sample_count_);
}

} // namespace iris::v2
