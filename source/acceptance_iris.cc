#include "acceptance_manifest.h"
#include "arq/arq.h"
#include "common/fft.h"
#include "fec/ldpc.h"
#include "ofdm/ofdm_config.h"
#include "ofdm/ofdm_demod.h"
#include "ofdm/ofdm_frame.h"
#include "ofdm/ofdm_mod.h"
#include "ofdm/ofdm_sync.h"

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <random>
#include <utility>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace iris;

namespace {

void acc_check(const char* name, bool ok) {
    acceptance_manifest_record(name, ok);
}

std::vector<uint8_t> acceptance_payload(size_t size, uint32_t salt) {
    std::vector<uint8_t> payload(size);
    uint32_t x = 0x9E3779B9u ^ salt;
    for (auto& byte : payload) {
        x ^= x << 13;
        x ^= x >> 17;
        x ^= x << 5;
        byte = static_cast<uint8_t>(x);
    }
    return payload;
}

OfdmConfig acceptance_config(int nfft = 1024, int cp = 64) {
    return ofdm_config_from_probe(narrow_passband(), nfft, cp, 4, 24);
}

std::vector<std::complex<float>> padded_capture(
    const std::vector<std::complex<float>>& frame, int pad_samples) {
    std::vector<std::complex<float>> capture(
        static_cast<size_t>(pad_samples) + frame.size() +
        static_cast<size_t>(pad_samples), {0.0f, 0.0f});
    std::copy(frame.begin(), frame.end(), capture.begin() + pad_samples);
    return capture;
}

void add_carrier_offset(std::vector<std::complex<float>>& samples,
                        float offset_hz, int sample_rate) {
    const float phase_step = 2.0f * static_cast<float>(M_PI) *
                             offset_hz / static_cast<float>(sample_rate);
    for (size_t n = 0; n < samples.size(); ++n) {
        const float phase = phase_step * static_cast<float>(n);
        samples[n] *= std::complex<float>(std::cos(phase), std::sin(phase));
    }
}

void impair_second_training_symbol(std::vector<std::complex<float>>& frame,
                                   const OfdmConfig& cfg) {
    // Apply a short, two-tone interferer during the second repeated training
    // symbol. Its equal-power opposite rotations leave the broad SC repetition
    // usable while pulling the frequency-domain coherence below a tight gate.
    const int sym_len = cfg.symbol_samples();
    const int body_start = 2 * sym_len + cfg.cp_samples;
    if (cfg.used_carrier_bins.size() < 2 ||
        body_start + cfg.nfft > static_cast<int>(frame.size()))
        return;

    std::vector<std::complex<float>> train1(cfg.nfft);
    std::vector<std::complex<float>> train2(cfg.nfft);
    std::copy(frame.begin() + sym_len + cfg.cp_samples,
              frame.begin() + sym_len + cfg.cp_samples + cfg.nfft,
              train1.begin());
    std::copy(frame.begin() + body_start,
              frame.begin() + body_start + cfg.nfft,
              train2.begin());
    fft_complex(train1.data(), cfg.nfft);
    fft_complex(train2.data(), cfg.nfft);

    const int a = cfg.used_carrier_bins[cfg.used_carrier_bins.size() / 3];
    const int b = cfg.used_carrier_bins[cfg.used_carrier_bins.size() / 3 + 1];
    const float mean_power = 0.5f * (std::norm(train1[a]) + std::norm(train1[b]));
    const float dmag = 4.0f * mean_power;
    const std::complex<float> d_a(0.0f, dmag);
    const std::complex<float> d_b(0.0f, -dmag);
    train2[a] = d_a / std::conj(train1[a]);
    train2[b] = d_b / std::conj(train1[b]);
    train2[cfg.nfft - a] = std::conj(train2[a]);
    train2[cfg.nfft - b] = std::conj(train2[b]);

    ifft_complex(train2.data(), cfg.nfft);
    std::copy(train2.begin(), train2.end(), frame.begin() + body_start);
    std::copy(train2.end() - cfg.cp_samples, train2.end(),
              frame.begin() + 2 * sym_len);
}

bool decode_exact(const std::vector<std::complex<float>>& capture,
                  const OfdmConfig& cfg, const ToneMap& tone_map,
                  const std::vector<uint8_t>& expected,
                  OfdmDemodResult* observed = nullptr) {
    const auto sync = ofdm_detect_frame(capture.data(),
                                        static_cast<int>(capture.size()), cfg);
    if (!sync.detected)
        return false;

    OfdmDemodulator demod(cfg);
    auto result = demod.demodulate(capture.data(),
                                   static_cast<int>(capture.size()),
                                   tone_map, &sync);
    const bool exact = result.success && result.payload == expected;
    if (observed)
        *observed = std::move(result);
    return exact;
}

void acc_cfo_150_hz() {
    const OfdmConfig cfg = acceptance_config();
    ToneMap tone_map = get_uniform_tone_map(2, cfg);  // QPSK, rate 1/2
    const auto payload = acceptance_payload(64, 1);

    OfdmModulator mod(cfg);
    const auto frame = mod.build_ofdm_frame(payload.data(), payload.size(),
                                             tone_map, LdpcRate::RATE_1_2, 1);
    bool both_exact = !frame.empty();
    bool positive_exact = false;
    bool negative_exact = false;
    for (float offset_hz : {150.0f, -150.0f}) {
        auto capture = padded_capture(frame, cfg.symbol_samples() * 4);
        add_carrier_offset(capture, offset_hz, cfg.sample_rate);
        const bool exact = decode_exact(capture, cfg, tone_map, payload);
        if (offset_hz > 0.0f)
            positive_exact = exact;
        else
            negative_exact = exact;
        both_exact = both_exact && exact;
    }

    std::printf("ACCEPTANCE_OBS name=acc_cfo_150_hz frame=%d plus150_exact=%d minus150_exact=%d\n",
                !frame.empty(), positive_exact, negative_exact);
    acc_check("acc_cfo_150_hz", both_exact);
}

// Bit classification stays fail-closed in the release -ffast-math build.
bool widegrid_finite(float value) {
    uint32_t bits;
    static_assert(sizeof(bits) == sizeof(value), "32-bit float required");
    std::memcpy(&bits, &value, sizeof(bits));
    return (bits & 0x7f800000u) != 0x7f800000u;
}

void acc_widegrid_nv_guard() {
    // xmutantson: pair-noise honesty and junk rejection must hold together.
    // Mirror the WIDE-6K arm of test_ofdm_bandlimit_junk_rejection.
    NegotiatedPassband pb;
    pb.low_hz = 300.0f;
    pb.high_hz = 6300.0f;
    pb.center_hz = 3300.0f;
    pb.bandwidth_hz = 6000.0f;
    pb.valid = true;
    OfdmConfig cfg = ofdm_config_from_probe(pb, 1024, 64, 4, 24);
    cfg.clean_channel = true;
    cfg.skip_papr_clip = true;
    const bool grid_ok = cfg.n_used_carriers == 127 &&
                         cfg.used_carrier_bins.size() == 127 &&
                         cfg.used_carrier_bins.front() == 7 &&
                         cfg.used_carrier_bins.back() == 133;
    ToneMap tm = get_uniform_tone_map(4, cfg);  // 16QAM r1/2
    std::vector<uint8_t> payload(64);
    for (size_t i = 0; i < payload.size(); ++i)
        payload[i] = static_cast<uint8_t>(i * 91 + 7);
    OfdmModulator mod(cfg);
    const auto iq = mod.build_ofdm_frame(payload.data(), payload.size(),
                                          tm, LdpcRate::RATE_1_2);
    if (!grid_ok || iq.size() <= 4u * cfg.symbol_samples()) {
        std::printf("ACCEPTANCE_OBS name=acc_widegrid_nv_guard setup=FAIL carriers=%d samples=%zu\n",
                    cfg.n_used_carriers, iq.size());
        acc_check("acc_widegrid_nv_guard", false);
        return;
    }

    std::vector<float> clean(iq.size());
    for (size_t i = 0; i < iq.size(); ++i) clean[i] = iq[i].real();
    double ss = 0.0;
    const size_t data_start = 4u * cfg.symbol_samples();
    for (size_t i = data_start; i < clean.size(); ++i)
        ss += static_cast<double>(clean[i]) * clean[i];
    const float sig_rms = std::sqrt(ss / (clean.size() - data_start));
    const float used_bw = cfg.n_used_carriers * cfg.subcarrier_spacing_hz;
    const float band_gain_db = 10.0f * std::log10(24000.0f / used_bw);
    const int pre = 240, post = 4800;
    const float hz_hi_edge =
        (cfg.used_carrier_bins.back() + 1) * static_cast<float>(cfg.sample_rate) / cfg.nfft;

    double ratio = -1.0, lower_mean = -1.0, upper_mean = -1.0;
    float shifts[2] = {0.0f, 0.0f};
    bool meter_ok[2] = {false, false};
    bool honesty_ok = false;
    for (int condition = 0; condition < 2; ++condition) {
        const float snr_db = condition == 0 ? 30.0f : 40.0f;
        const float limit_db = condition == 0 ? 2.0f : 3.0f;
        // Local RNGs isolate this case from the rest of the suite. Reusing
        // the seed at both SNRs also keeps the junk waveform identical.
        std::mt19937 rng(77125);
        const float noise_rms = sig_rms /
            std::pow(10.0f, (snr_db - band_gain_db) / 20.0f);
        std::normal_distribution<float> dist(0.0f, noise_rms);
        float meters[2] = {0.0f, 0.0f};
        bool exact[2] = {false, false};
        for (int arm = 0; arm < 2; ++arm) {
            std::vector<float> buf(pre + clean.size() + post);
            for (auto& v : buf) v = dist(rng);
            for (size_t i = 0; i < clean.size(); ++i) buf[pre + i] += clean[i];
            if (arm == 1) {
                // Same comb, amplitudes and hum phases as the existing junk test.
                std::uniform_real_distribution<float> ph(0.0f, 6.2831853f);
                const int NT_LO = 24, NT_HI = 60;
                std::vector<float> fr, am, p0;
                float amp_lo = sig_rms * 0.7f / std::sqrt((float)NT_LO / 2);
                float amp_hi = sig_rms * 0.7f / std::sqrt((float)NT_HI / 2);
                for (int t = 0; t < NT_LO; t++) {
                    fr.push_back(10.0f + 240.0f * t / NT_LO);
                    am.push_back(amp_lo); p0.push_back(ph(rng));
                }
                for (int t = 0; t < NT_HI; t++) {
                    fr.push_back(hz_hi_edge + 250.0f + 3000.0f * t / NT_HI);
                    am.push_back(amp_hi); p0.push_back(ph(rng));
                }
                fr.push_back(60.0f);  am.push_back(sig_rms * 0.25f); p0.push_back(0.3f);
                fr.push_back(120.0f); am.push_back(sig_rms * 0.25f); p0.push_back(1.1f);
                fr.push_back(180.0f); am.push_back(sig_rms * 0.25f); p0.push_back(2.0f);
                fr.push_back(hz_hi_edge + 300.0f);
                am.push_back(sig_rms * 0.5f);  p0.push_back(0.7f);
                for (size_t i = 0; i < buf.size(); i++) {
                    float t = (float)i / cfg.sample_rate, j = 0.0f;
                    for (size_t q = 0; q < fr.size(); q++)
                        j += am[q] * std::sin(6.2831853f * fr[q] * t + p0[q]);
                    buf[i] += j;
                }
            }
            const auto rx = ofdm_analytic_bandlimit(buf.data(), buf.size(), cfg,
                                                     RxBandlimitEdge::TAPERED);
            OfdmDemodResult res;
            exact[arm] = decode_exact(rx, cfg, tm, payload, &res);
            meters[arm] = res.mean_channel_snr_db;
            if (condition == 1 && arm == 0) {
                const auto& nv = res.channel_estimate.noise_var;
                bool valid_nv = nv.size() == 127;
                for (float value : nv)
                    valid_nv = valid_nv && widegrid_finite(value) && value > 0.0f;
                if (valid_nv) {
                    const size_t half = nv.size() / 2;
                    double lower = 0.0, upper = 0.0;
                    for (size_t i = 0; i < nv.size(); ++i) {
                        if (i < half) lower += nv[i];
                        else upper += nv[i];
                    }
                    lower_mean = lower / half;
                    upper_mean = upper / (nv.size() - half);
                    ratio = lower_mean / upper_mean;
                }
                // White noise gives equal half-band means. A false clamp
                // slashes only the lower half; the upper half is the reference.
                honesty_ok = exact[arm] && valid_nv && ratio >= 0.5;
            }
        }
        shifts[condition] = meters[1] - meters[0];
        meter_ok[condition] = exact[0] && exact[1] &&
            widegrid_finite(meters[0]) && widegrid_finite(meters[1]) &&
            widegrid_finite(shifts[condition]) && std::fabs(shifts[condition]) <= limit_db;
        std::printf("ACCEPTANCE_OBS name=acc_widegrid_nv_guard snr=%.0f ref_exact=%d junk_exact=%d ref_meter=%.3f junk_meter=%.3f shift=%+.3f limit=%.1f meter_ok=%d\n",
                    snr_db, exact[0], exact[1], meters[0], meters[1],
                    shifts[condition], limit_db, meter_ok[condition]);
    }
    std::printf("ACCEPTANCE_OBS name=acc_widegrid_nv_guard ratio=%.3f shift30=%.2f shift40=%.2f lower_nv=%.6g upper_nv=%.6g honesty_ok=%d junk30_ok=%d junk40_ok=%d\n",
                ratio, shifts[0], shifts[1], lower_mean, upper_mean,
                honesty_ok, meter_ok[0], meter_ok[1]);
    acc_check("acc_widegrid_nv_guard", honesty_ok && meter_ok[0] && meter_ok[1]);
}

void acc_accept_all_frames() {
    // A 256-point FFT leaves eight QPSK data carriers in the narrow passband.
    // Eight 1600-bit codewords therefore occupy 800 data symbols: wire-valid,
    // but beyond both the historical 200-symbol guard and the rejected 320+
    // class this acceptance decision explicitly admits.
    const OfdmConfig cfg = acceptance_config(256, 32);
    ToneMap tone_map = get_uniform_tone_map(2, cfg);  // QPSK, rate 1/2
    tone_map.n_codewords = 8;
    const auto payload = acceptance_payload(700, 2);

    OfdmModulator mod(cfg);
    const auto frame = mod.build_ofdm_frame(payload.data(), payload.size(),
                                             tone_map, LdpcRate::RATE_1_2, 8);
    OfdmDemodResult result;
    const bool exact = !frame.empty() &&
        decode_exact(padded_capture(frame, cfg.symbol_samples() * 4),
                     cfg, tone_map, payload, &result);
    const bool reached_ldpc_crc = result.n_data_symbols > 320 &&
                                  result.n_ldpc_blocks == 8 &&
                                  result.block_results.size() == 8;

    std::printf("ACCEPTANCE_OBS name=acc_accept_all_frames exact=%d data_symbols=%d ldpc_blocks=%d block_results=%zu reached_ldpc_crc=%d\n",
                exact, result.n_data_symbols, result.n_ldpc_blocks,
                result.block_results.size(), reached_ldpc_crc);
    acc_check("acc_accept_all_frames", exact && reached_ldpc_crc);
}

void acc_earliest_preamble() {
    const OfdmConfig cfg = acceptance_config();
    ToneMap tone_map = get_uniform_tone_map(2, cfg);
    const auto early_payload = acceptance_payload(48, 3);
    const auto late_payload = acceptance_payload(48, 4);

    OfdmModulator mod(cfg);
    auto early = mod.build_ofdm_frame(early_payload.data(), early_payload.size(),
                                      tone_map, LdpcRate::RATE_1_2, 1);
    auto late = mod.build_ofdm_frame(late_payload.data(), late_payload.size(),
                                     tone_map, LdpcRate::RATE_1_2, 1);

    const int lead = cfg.symbol_samples() * 3;
    const int gap = cfg.symbol_samples() * 3;
    const int late_begin = lead + static_cast<int>(early.size()) + gap;
    std::vector<std::complex<float>> capture(
        static_cast<size_t>(late_begin) + late.size() +
        static_cast<size_t>(lead), {0.0f, 0.0f});
    for (size_t i = 0; i < early.size(); ++i)
        capture[static_cast<size_t>(lead) + i] += 0.30f * early[i];
    std::copy(late.begin(), late.end(), capture.begin() + late_begin);

    const auto sync = ofdm_detect_frame(capture.data(),
                                        static_cast<int>(capture.size()), cfg);
    bool selected_early = sync.detected && sync.frame_start < late_begin;
    bool decoded_early = false;
    if (selected_early) {
        OfdmDemodulator demod(cfg);
        const auto result = demod.demodulate(capture.data(),
                                             static_cast<int>(capture.size()),
                                             tone_map, &sync);
        decoded_early = result.success && result.payload == early_payload;
    }

    std::printf("ACCEPTANCE_OBS name=acc_earliest_preamble detected=%d frame_start=%d late_begin=%d selected_early=%d decoded_early=%d\n",
                sync.detected, sync.frame_start, late_begin, selected_early,
                decoded_early);
    acc_check("acc_earliest_preamble", selected_early && decoded_early);
}

void acc_loose_pre_ldpc_gate() {
    const OfdmConfig cfg = acceptance_config();
    ToneMap tone_map = get_uniform_tone_map(2, cfg);
    const auto payload = acceptance_payload(64, 5);

    OfdmModulator mod(cfg);
    auto frame = mod.build_ofdm_frame(payload.data(), payload.size(),
                                      tone_map, LdpcRate::RATE_1_2, 1);
    impair_second_training_symbol(frame, cfg);
    auto capture = padded_capture(frame, cfg.symbol_samples() * 4);

    double signal_energy = 0.0;
    for (const auto& sample : frame)
        signal_energy += std::norm(sample);
    const float signal_rms = frame.empty()
        ? 0.0f
        : static_cast<float>(std::sqrt(signal_energy / frame.size()));
    const float target_snr_db = 5.0f;
    const float noise_sigma = signal_rms /
        std::pow(10.0f, target_snr_db / 20.0f) / std::sqrt(2.0f);
    std::mt19937 rng(0x1A15C0DEu);
    std::normal_distribution<float> noise(0.0f, noise_sigma);
    for (auto& sample : capture)
        sample += std::complex<float>(noise(rng), noise(rng));

    // Observe the frame's metric with the public detector in measurement mode,
    // then present the identical samples to the shipping gate. The acceptance
    // condition requires a frame below today's tight FD gate to reach the public
    // LDPC/CRC result surface; CRC remains the authority on success or failure.
    OfdmConfig measure_cfg = cfg;
    measure_cfg.fd_zc_threshold = 0.0f;
    const auto measured = ofdm_detect_frame(capture.data(),
                                            static_cast<int>(capture.size()),
                                            measure_cfg);
    const bool below_tight_gate = measured.detected &&
                                  measured.zc_metric < cfg.fd_zc_threshold;

    const auto sync = ofdm_detect_frame(capture.data(),
                                        static_cast<int>(capture.size()), cfg);
    bool reached_ldpc_crc = false;
    if (sync.detected) {
        OfdmDemodulator demod(cfg);
        const auto result = demod.demodulate(capture.data(),
                                             static_cast<int>(capture.size()),
                                             tone_map, &sync);
        reached_ldpc_crc = !result.block_results.empty();
    }

    std::printf("ACCEPTANCE_OBS name=acc_loose_pre_ldpc_gate measured_detected=%d measured_metric=%.3f threshold=%.3f below_tight=%d shipping_detected=%d reached_ldpc_crc=%d\n",
                measured.detected, measured.zc_metric, cfg.fd_zc_threshold,
                below_tight_gate, sync.detected, reached_ldpc_crc);
    acc_check("acc_loose_pre_ldpc_gate", below_tight_gate && reached_ldpc_crc);
}

void pump_arq(std::vector<std::vector<uint8_t>>& a_to_b,
              ArqSession& b,
              std::vector<std::vector<uint8_t>>& b_to_a,
              ArqSession& a) {
    auto outbound_a = std::move(a_to_b);
    a_to_b.clear();
    for (const auto& frame : outbound_a)
        b.on_frame_received(frame.data(), frame.size());

    auto outbound_b = std::move(b_to_a);
    b_to_a.clear();
    for (const auto& frame : outbound_b)
        a.on_frame_received(frame.data(), frame.size());
}

void acc_fail_closed_discontinuity() {
    ArqSession sender;
    ArqSession receiver;
    sender.set_callsign("ACC01");
    receiver.set_callsign("ACC02");

    std::vector<std::vector<uint8_t>> sender_to_receiver;
    std::vector<std::vector<uint8_t>> receiver_to_sender;
    std::vector<uint8_t> delivered;
    bool completion_reported = false;
    bool completion_success = false;

    ArqCallbacks sender_callbacks;
    sender_callbacks.send_frame = [&](const uint8_t* data, size_t len) {
        sender_to_receiver.emplace_back(data, data + len);
    };
    sender_callbacks.on_transfer_complete = [&](bool success) {
        completion_reported = true;
        completion_success = success;
    };
    sender.set_callbacks(sender_callbacks);

    ArqCallbacks receiver_callbacks;
    receiver_callbacks.send_frame = [&](const uint8_t* data, size_t len) {
        receiver_to_sender.emplace_back(data, data + len);
    };
    receiver_callbacks.on_data_received = [&](const uint8_t* data, size_t len) {
        delivered.insert(delivered.end(), data, data + len);
    };
    receiver.set_callbacks(receiver_callbacks);

    receiver.listen();
    sender.connect("ACC02");
    for (int i = 0; i < 20 &&
         sender.state() != ArqState::CONNECTED &&
         sender.state() != ArqState::TURBOSHIFT; ++i) {
        pump_arq(sender_to_receiver, receiver,
                 receiver_to_sender, sender);
    }
    const bool connected = sender.state() == ArqState::CONNECTED ||
                           sender.state() == ArqState::TURBOSHIFT;

    const auto payload = acceptance_payload(1200, 6);
    sender.send_data(payload.data(), payload.size());

    // Deliver exactly one DATA frame, then lose the rest at the capture/playback
    // boundary. This leaves both an observable delivered prefix and sender-side
    // custody that has not been acknowledged end-to-end.
    bool delivered_one_data_frame = false;
    auto outbound = std::move(sender_to_receiver);
    sender_to_receiver.clear();
    for (const auto& bytes : outbound) {
        ArqFrame frame;
        if (!delivered_one_data_frame &&
            ArqFrame::deserialize(bytes.data(), bytes.size(), frame) &&
            frame.type == ArqType::DATA) {
            receiver.on_frame_received(bytes.data(), bytes.size());
            delivered_one_data_frame = true;
        }
    }
    auto acknowledgements = std::move(receiver_to_sender);
    receiver_to_sender.clear();
    for (const auto& bytes : acknowledgements)
        sender.on_frame_received(bytes.data(), bytes.size());

    // TODO_ACCEPTANCE(discontinuity API): AudioStream/Modem currently exposes no
    // capture/playback-discontinuity notification. reset() is the closest public
    // session-boundary operation available to represent that event. Once the real API
    // exists, drive it here and retain the same observable failure/custody verdict.
    sender.reset();

    const bool partial_delivery = connected && delivered_one_data_frame && !delivered.empty() &&
                                  delivered.size() < payload.size();
    const bool failed_closed = sender.state() == ArqState::IDLE &&
                               completion_reported && !completion_success;
    std::printf("ACCEPTANCE_OBS name=acc_fail_closed_discontinuity connected=%d delivered_data_frame=%d delivered=%zu payload=%zu sender_idle=%d completion_reported=%d completion_success=%d\n",
                connected, delivered_one_data_frame, delivered.size(),
                payload.size(), sender.state() == ArqState::IDLE,
                completion_reported, completion_success);
    acc_check("acc_fail_closed_discontinuity", partial_delivery && failed_closed);
}

}  // namespace

void run_acceptance_iris() {
    std::printf("\n--- Iris Acceptance Decisions ---\n");
    acc_cfo_150_hz();
    acc_widegrid_nv_guard();
    acc_accept_all_frames();
    acc_earliest_preamble();
    acc_loose_pre_ldpc_gate();
    acc_fail_closed_discontinuity();
}
