/*
 * B2F protocol handler implementation.
 * Ported from Mercury's cl_b2f_handler.
 */

#include "b2f/b2f_handler.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <algorithm>

namespace iris {

// C1 HARD GATE predicate (see b2f_handler.h). Shippable iff the LZHUF encode
// succeeded AND the rerolled length reproduces the sender's declared comp_size
// exactly. Anything else (encode failure, size mismatch, or an absent/zero declared
// size) is REFUSED so a non-bit-identical reroll can never ship a fabricated ratio.
bool b2f_reroll_shippable(int encode_rc, size_t rerolled_len, int expected_comp_size) {
    return encode_rc == 0 && rerolled_len > 0 &&
           expected_comp_size > 0 && (int)rerolled_len == expected_comp_size;
}

B2fHandler::B2fHandler()
    : unroll_enabled(true), state_(B2F_IDLE), current_proposer_(PROPOSER_NONE),
      b2f_detected_(false), num_proposals_(0), current_payload_idx_(-1),
      payload_bytes_remaining_(0), tx_line_pos_(0), rx_line_pos_(0),
      payload_buf_(nullptr), plain_buf_(nullptr), initialized_(false) {
    reset();
}

B2fHandler::~B2fHandler() { deinit(); }

void B2fHandler::init() {
    if (initialized_) return;
    payload_buf_ = (uint8_t*)malloc(B2F_PAYLOAD_BUF_SIZE);
    plain_buf_ = (uint8_t*)malloc(B2F_PAYLOAD_BUF_SIZE);
    if (!payload_buf_ || !plain_buf_) { deinit(); return; }
    reset();
    initialized_ = true;
    printf("[B2F] Handler initialized\n");
    fflush(stdout);
}

void B2fHandler::deinit() {
    if (payload_buf_) { free(payload_buf_); payload_buf_ = nullptr; }
    if (plain_buf_) { free(plain_buf_); plain_buf_ = nullptr; }
    reset();
    initialized_ = false;
}

void B2fHandler::reset() {
    if (state_ != B2F_IDLE)
        printf("[B2F] Reset (was state %d)\n", state_);
    state_ = B2F_IDLE;
    current_proposer_ = PROPOSER_NONE;
    b2f_detected_ = false;
    num_proposals_ = 0;
    current_payload_idx_ = -1;
    payload_bytes_remaining_ = 0;
    tx_line_pos_ = 0;
    rx_line_pos_ = 0;
    payload_buf_pos_ = 0;
    pending_tx_output_.clear();
    pending_rx_output_.clear();
    pending_tx_offset_ = pending_rx_offset_ = 0;
    pending_tx_advance_ = pending_rx_advance_ = false;
    filter_failed_ = false;
    last_tx_input_consumed_ = last_rx_input_consumed_ = 0;
}

void B2fHandler::advance_payload(bool tx_direction) {
    payload_buf_pos_ = 0;
    current_payload_idx_ = find_next_accepted(current_payload_idx_ + 1);
    if (current_payload_idx_ < 0) {
        payload_bytes_remaining_ = 0;
        state_ = B2F_CHECKSUM;
        return;
    }
    const auto& next = proposals_[current_payload_idx_];
    if (tx_direction || next.resume_offset > 0)
        payload_bytes_remaining_ = next.comp_size - next.resume_offset;
    else
        payload_bytes_remaining_ = next.uncomp_size;
}

int B2fHandler::drain_pending(bool tx_direction, char* out, int out_cap) {
    auto& pending = tx_direction ? pending_tx_output_ : pending_rx_output_;
    auto& offset = tx_direction ? pending_tx_offset_ : pending_rx_offset_;
    auto& advance = tx_direction ? pending_tx_advance_ : pending_rx_advance_;
    if (out_cap < 0 || (!out && out_cap > 0)) return -1;
    const size_t remaining = offset <= pending.size() ? pending.size() - offset : 0;
    const size_t count = std::min(remaining, static_cast<size_t>(out_cap));
    if (count > 0) memcpy(out, pending.data() + offset, count);
    offset += count;
    if (offset == pending.size() && !pending.empty()) {
        pending.clear();
        offset = 0;
        if (advance) {
            advance = false;
            advance_payload(tx_direction);
        }
    }
    return static_cast<int>(count);
}

bool B2fHandler::has_retained_input_suffix(bool tx_direction) const {
    if (!initialized_ || !b2f_detected_) return false;

    const Proposer direction = tx_direction ? PROPOSER_LOCAL : PROPOSER_REMOTE;
    if (state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == direction) {
        if (current_payload_idx_ < 0 ||
            proposals_[current_payload_idx_].resume_offset > 0 ||
            !unroll_enabled)
            return false;
        return payload_buf_pos_ > 0;
    }

    return tx_direction ? tx_line_pos_ > 0 : rx_line_pos_ > 0;
}

// ---- Line parsers ----

bool B2fHandler::parse_sid_line(const char* line, int len) {
    if (len < 5 || line[0] != '[' || line[len-1] != ']') return false;
    for (int i = 0; i < len - 2; i++)
        if (line[i] == 'B' && line[i+1] == '2' && line[i+2] == 'F') return true;
    return false;
}

bool B2fHandler::parse_fc_line(const char* line, int len, B2fProposal* prop) {
    if (len < 10 || line[0] != 'F' || line[1] != 'C' || line[2] != ' ') return false;
    if (line[3] == 'E' && line[4] == 'M') prop->type = 'E';
    else if (line[3] == 'C' && line[4] == 'M') prop->type = 'C';
    else return false;

    int pos = 6;
    int mid_start = pos;
    while (pos < len && line[pos] != ' ') pos++;
    int mid_len = pos - mid_start;
    if (mid_len <= 0 || mid_len > 12) return false;
    memcpy(prop->mid, line + mid_start, mid_len);
    prop->mid[mid_len] = '\0';
    if (pos >= len || line[pos++] != ' ') return false;

    auto parse_bounded_size = [&](uint32_t& value) {
        if (pos >= len || line[pos] < '0' || line[pos] > '9') return false;
        uint64_t parsed = 0;
        while (pos < len && line[pos] >= '0' && line[pos] <= '9') {
            parsed = parsed * 10 + static_cast<unsigned>(line[pos++] - '0');
            if (parsed > static_cast<uint64_t>(B2F_PAYLOAD_BUF_SIZE)) return false;
        }
        value = static_cast<uint32_t>(parsed);
        return true;
    };
    if (!parse_bounded_size(prop->uncomp_size) ||
        pos >= len || line[pos++] != ' ' ||
        !parse_bounded_size(prop->comp_size)) return false;

    // These peer declarations are allocation and custody limits, not hints.
    // Reject before the proposal can enter an accepted payload state.
    if (prop->uncomp_size == 0 || prop->comp_size == 0 ||
        prop->uncomp_size > B2F_PAYLOAD_BUF_SIZE ||
        prop->comp_size > B2F_PAYLOAD_BUF_SIZE)
        return false;

    prop->accepted = -1;
    prop->resume_offset = 0;
    return true;
}

bool B2fHandler::parse_fs_line(const char* line, int len) {
    if (len < 3 || line[0] != 'F' || line[1] != 'S' || line[2] != ' ') return false;
    int pos = 3;
    for (int i = 0; i < num_proposals_ && pos < len; i++, pos++) {
        // FBB protocol FS response codes:
        //   +/Y = accepted, -/N/R/E = rejected, =/L = deferred, H = hold (accepted)
        //   !offset = accepted with resume from byte offset
        switch (line[pos]) {
            case '+': case 'Y':
                proposals_[i].accepted = 1; break;
            case '-': case 'N': case 'R': case 'E':
                proposals_[i].accepted = 0; break;
            case '=': case 'L':
                proposals_[i].accepted = -1; break;
            case 'H':
                proposals_[i].accepted = 1; break;
            case '!': {
                proposals_[i].accepted = 1;
                // Parse trailing offset digits
                uint64_t parsed_offset = 0;
                pos++;
                while (pos < len && line[pos] >= '0' && line[pos] <= '9') {
                    parsed_offset = parsed_offset * 10 +
                                    static_cast<unsigned>(line[pos++] - '0');
                    if (parsed_offset > B2F_PAYLOAD_BUF_SIZE) break;
                }
                const uint32_t offset = parsed_offset <= UINT32_MAX
                    ? static_cast<uint32_t>(parsed_offset) : UINT32_MAX;
                if (offset > proposals_[i].comp_size) {
                    proposals_[i].accepted = 0;
                    proposals_[i].resume_offset = 0;
                } else {
                    proposals_[i].resume_offset = offset;
                }
                pos--;  // loop will increment
                printf("[B2F] FS: proposal %d accepted with resume offset %u\n",
                       i, offset);
                fflush(stdout);
                break;
            }
            default: break;
        }
    }
    return true;
}

int B2fHandler::find_next_accepted(int from) {
    for (int i = from; i < num_proposals_; i++)
        if (proposals_[i].accepted == 1) return i;
    return -1;
}

// ---- Line processing ----

int B2fHandler::process_tx_line(const char* line, int len, char* out, int out_cap) {
    if (len == 0) goto passthrough;

    if (!b2f_detected_ || state_ == B2F_SID_EXCHANGE) {
        if (parse_sid_line(line, len)) {
            b2f_detected_ = true;
            state_ = B2F_SID_EXCHANGE;
            goto passthrough;
        }
    }
    if (!b2f_detected_) goto passthrough;
    if (state_ == B2F_SID_EXCHANGE) state_ = B2F_WAIT_PROPOSALS;

    {
        B2fProposal prop;
        if (parse_fc_line(line, len, &prop)) {
            if (current_proposer_ != PROPOSER_LOCAL) {
                num_proposals_ = 0;
                current_proposer_ = PROPOSER_LOCAL;
            }
            state_ = B2F_PARSING_FC;
            if (num_proposals_ < B2F_MAX_PROPOSALS)
                proposals_[num_proposals_++] = prop;
            goto passthrough;
        }
    }

    if (len >= 2 && line[0] == 'F' && line[1] == '>') {
        state_ = B2F_WAIT_FS;
        goto passthrough;
    }

    if (line[0] == 'F' && line[1] == 'S' && len >= 3) {
        if (parse_fs_line(line, len)) {
            if (current_proposer_ == PROPOSER_REMOTE) {
                current_payload_idx_ = find_next_accepted(0);
                if (current_payload_idx_ >= 0) {
                    auto& prop = proposals_[current_payload_idx_];
                    // Resume transfers send partial LZHUF — can't unroll
                    bool can_unroll = unroll_enabled && prop.resume_offset == 0;
                    uint32_t data_size = can_unroll ?
                        prop.uncomp_size :
                        (prop.comp_size - prop.resume_offset);
                    payload_bytes_remaining_ = data_size;
                    state_ = B2F_PAYLOAD_TRANSFER;
                    payload_buf_pos_ = 0;
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
            goto passthrough;
        }
    }

    if (len >= 2 && line[0] == 'F' && (line[1] == 'F' || line[1] == 'Q')) {
        state_ = B2F_WAIT_PROPOSALS;
        current_proposer_ = PROPOSER_NONE;
        goto passthrough;
    }

passthrough:
    if (len + 1 > out_cap) {
        pending_tx_output_.assign(line, line + len);
        pending_tx_output_.push_back('\r');
        pending_tx_offset_ = 0;
        return drain_pending(true, out, out_cap);
    }
    memcpy(out, line, len);
    out[len] = '\r';
    return len + 1;
}

int B2fHandler::process_rx_line(const char* line, int len, char* out, int out_cap) {
    if (len == 0) goto passthrough;

    if (!b2f_detected_ || state_ == B2F_SID_EXCHANGE) {
        if (parse_sid_line(line, len)) {
            b2f_detected_ = true;
            state_ = B2F_SID_EXCHANGE;
            goto passthrough;
        }
    }
    if (!b2f_detected_) goto passthrough;
    if (state_ == B2F_SID_EXCHANGE) state_ = B2F_WAIT_PROPOSALS;

    {
        B2fProposal prop;
        if (parse_fc_line(line, len, &prop)) {
            if (current_proposer_ != PROPOSER_REMOTE) {
                num_proposals_ = 0;
                current_proposer_ = PROPOSER_REMOTE;
            }
            state_ = B2F_PARSING_FC;
            if (num_proposals_ < B2F_MAX_PROPOSALS)
                proposals_[num_proposals_++] = prop;
            goto passthrough;
        }
    }

    if (len >= 2 && line[0] == 'F' && line[1] == '>') {
        state_ = B2F_WAIT_FS;
        goto passthrough;
    }

    if (line[0] == 'F' && line[1] == 'S' && len >= 3) {
        if (parse_fs_line(line, len)) {
            if (current_proposer_ == PROPOSER_LOCAL) {
                current_payload_idx_ = find_next_accepted(0);
                if (current_payload_idx_ >= 0) {
                    auto& prop = proposals_[current_payload_idx_];
                    payload_bytes_remaining_ = prop.comp_size - prop.resume_offset;
                    state_ = B2F_PAYLOAD_TRANSFER;
                    payload_buf_pos_ = 0;
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
            goto passthrough;
        }
    }

    if (len >= 2 && line[0] == 'F' && (line[1] == 'F' || line[1] == 'Q')) {
        state_ = B2F_WAIT_PROPOSALS;
        current_proposer_ = PROPOSER_NONE;
        goto passthrough;
    }

passthrough:
    if (len + 1 > out_cap) {
        pending_rx_output_.assign(line, line + len);
        pending_rx_output_.push_back('\r');
        pending_rx_offset_ = 0;
        return drain_pending(false, out, out_cap);
    }
    memcpy(out, line, len);
    out[len] = '\r';
    return len + 1;
}

// ---- Payload handling ----

int B2fHandler::process_tx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed) {
    *in_consumed = 0;
    if (current_proposer_ != PROPOSER_LOCAL || current_payload_idx_ < 0) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        *in_consumed = copy;
        return copy;
    }

    int out_pos = 0, in_pos = 0;
    while (in_pos < in_len && payload_bytes_remaining_ > 0) {
        int chunk = in_len - in_pos;
        if (chunk > payload_bytes_remaining_) chunk = payload_bytes_remaining_;

        // Can only unroll full transfers — resume sends partial LZHUF
        bool can_unroll = unroll_enabled && initialized_ &&
                          proposals_[current_payload_idx_].resume_offset == 0;
        if (can_unroll) {
            if (chunk > B2F_PAYLOAD_BUF_SIZE - payload_buf_pos_) {
                filter_failed_ = true;
                *in_consumed = in_pos;
                return -1;
            }
            memcpy(payload_buf_ + payload_buf_pos_, in + in_pos, chunk);
            payload_buf_pos_ += chunk;
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;

            if (payload_bytes_remaining_ == 0) {
                size_t plain_len = 0;
                int rc = lzhuf_decode_buffer(payload_buf_, payload_buf_pos_,
                    plain_buf_, proposals_[current_payload_idx_].uncomp_size, &plain_len);

                if (rc == 0 && plain_len == proposals_[current_payload_idx_].uncomp_size) {
                    // Retaining the semantic unroll is safe only when this exact
                    // endpoint codec can reconstruct the sender's original LZHUF
                    // bytes, not merely their length. The peer runs the same
                    // deterministic registered codec before handing bytes to B2F.
                    std::vector<uint8_t> reroll_proof(payload_buf_pos_);
                    size_t reroll_proof_len = 0;
                    const int proof_rc = lzhuf_encode_buffer(
                        plain_buf_, plain_len, reroll_proof.data(),
                        reroll_proof.size(), &reroll_proof_len);
                    if (proof_rc != 0 || reroll_proof_len !=
                            static_cast<size_t>(payload_buf_pos_) ||
                        !std::equal(reroll_proof.begin(), reroll_proof.end(),
                                    payload_buf_)) {
                        printf("[B2F-TX] Exact reroll proof failed for %s\n",
                               proposals_[current_payload_idx_].mid);
                        fflush(stdout);
                        filter_failed_ = true;
                        *in_consumed = in_pos;
                        return -1;
                    }
                    // Validate unrolled size against proposal's uncomp_size
                    int expected = proposals_[current_payload_idx_].uncomp_size;
                    if ((int)plain_len != expected) {
                        printf("[B2F-TX] WARNING: unroll size mismatch for %s: "
                               "got %zu, expected %d (delta %+d)\n",
                               proposals_[current_payload_idx_].mid,
                               plain_len, expected, (int)plain_len - expected);
                        fflush(stdout);
                    }
                    printf("[B2F-TX] Unrolled %s: %d LZHUF -> %zu plaintext\n",
                        proposals_[current_payload_idx_].mid,
                        payload_buf_pos_, plain_len);
                    fflush(stdout);
                    pending_tx_output_.assign(plain_buf_, plain_buf_ + plain_len);
                    pending_tx_offset_ = 0;
                    pending_tx_advance_ = true;
                    const int drained = drain_pending(true, out + out_pos, out_cap - out_pos);
                    if (drained < 0) {
                        *in_consumed = in_pos;
                        return -1;
                    }
                    out_pos += drained;
                } else {
                    printf("[B2F-TX] Unroll FAILED for %s (rc=%d, len=%zu)\n",
                           proposals_[current_payload_idx_].mid,
                           rc, plain_len);
                    fflush(stdout);
                    filter_failed_ = true;
                    *in_consumed = in_pos;
                    return -1;
                }
            }
        } else {
            const int copy = std::min(chunk, out_cap - out_pos);
            if (copy <= 0) break;
            memcpy(out + out_pos, in + in_pos, copy);
            out_pos += copy;
            in_pos += copy;
            payload_bytes_remaining_ -= copy;
            if (payload_bytes_remaining_ == 0) {
                current_payload_idx_ = find_next_accepted(current_payload_idx_ + 1);
                if (current_payload_idx_ >= 0) {
                    auto& np = proposals_[current_payload_idx_];
                    payload_bytes_remaining_ = np.comp_size - np.resume_offset;
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
        }
    }
    *in_consumed = in_pos;
    return out_pos;
}

int B2fHandler::process_rx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed) {
    *in_consumed = 0;
    if (current_proposer_ != PROPOSER_REMOTE || current_payload_idx_ < 0) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        *in_consumed = copy;
        return copy;
    }

    int out_pos = 0, in_pos = 0;
    while (in_pos < in_len && payload_bytes_remaining_ > 0) {
        int chunk = in_len - in_pos;
        if (chunk > payload_bytes_remaining_) chunk = payload_bytes_remaining_;

        // Can only reroll full transfers — resume sends partial LZHUF
        bool can_unroll = unroll_enabled && initialized_ &&
                          proposals_[current_payload_idx_].resume_offset == 0;
        if (can_unroll) {
            if (chunk > B2F_PAYLOAD_BUF_SIZE - payload_buf_pos_) {
                filter_failed_ = true;
                *in_consumed = in_pos;
                return -1;
            }
            memcpy(payload_buf_ + payload_buf_pos_, in + in_pos, chunk);
            payload_buf_pos_ += chunk;
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;

            if (payload_bytes_remaining_ == 0) {
                size_t lzhuf_len = 0;
                int rc = lzhuf_encode_buffer(payload_buf_, payload_buf_pos_,
                    plain_buf_, proposals_[current_payload_idx_].comp_size, &lzhuf_len);

                int expected = proposals_[current_payload_idx_].comp_size;
                // C1 HARD GATE: ship the reroll ONLY when it is bit-identical to the
                // original wire blob (necessary condition: LZHUF length == the sender's
                // declared comp_size). A size mismatch is no longer a warn-and-ship — it
                // is a REFUSAL, so a non-bit-identical reroll can never ship a fabricated
                // ratio (nor a blob the local Winlink client's B2F checksum would reject).
                if (b2f_reroll_shippable(rc, lzhuf_len, expected)) {
                    printf("[B2F-RX] Rerolled %s: %d plaintext -> %zu LZHUF (== comp_size %d)\n",
                           proposals_[current_payload_idx_].mid,
                           payload_buf_pos_, lzhuf_len, expected);
                    fflush(stdout);
                    pending_rx_output_.assign(plain_buf_, plain_buf_ + lzhuf_len);
                    pending_rx_offset_ = 0;
                    pending_rx_advance_ = true;
                    const int drained = drain_pending(false, out + out_pos, out_cap - out_pos);
                    if (drained < 0) {
                        *in_consumed = in_pos;
                        return -1;
                    }
                    out_pos += drained;
                } else {
                    printf("[B2F-RX] REROLL REJECTED (hard gate) for %s: rc=%d got=%zu "
                           "expected=%d (delta %+d)\n",
                           proposals_[current_payload_idx_].mid,
                           rc, lzhuf_len, expected, (int)lzhuf_len - expected);
                    fflush(stdout);
                    filter_failed_ = true;
                    *in_consumed = in_pos;
                    return -1;
                }
            }
        } else {
            const int copy = std::min(chunk, out_cap - out_pos);
            if (copy <= 0) break;
            memcpy(out + out_pos, in + in_pos, copy);
            out_pos += copy;
            in_pos += copy;
            payload_bytes_remaining_ -= copy;
            if (payload_bytes_remaining_ == 0) {
                current_payload_idx_ = find_next_accepted(current_payload_idx_ + 1);
                if (current_payload_idx_ >= 0) {
                    auto& np = proposals_[current_payload_idx_];
                    payload_bytes_remaining_ = np.comp_size - np.resume_offset;
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
        }
    }
    *in_consumed = in_pos;
    return out_pos;
}

// ---- Top-level filters ----

int B2fHandler::filter_tx(const char* in, int in_len, char* out, int out_cap) {
    last_tx_input_consumed_ = 0;
    if (in_len < 0 || out_cap < 0 || (!in && in_len > 0) || (!out && out_cap > 0)) return -1;
    int pending = drain_pending(true, out, out_cap);
    if (pending < 0) return -1;
    if (!pending_tx_output_.empty() || pending == out_cap) return pending;
    out += pending;
    out_cap -= pending;
    if (!initialized_) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        last_tx_input_consumed_ = static_cast<size_t>(copy);
        return pending + copy;
    }

    int out_pos = 0, in_pos = 0;

    if (!b2f_detected_) {
        for (; in_pos < in_len && !b2f_detected_ && out_pos < out_cap; in_pos++) {
            char c = in[in_pos];
            if (out_pos < out_cap) out[out_pos++] = c;
            if (c == '\r') {
                if (tx_line_pos_ > 0) {
                    tx_line_buf_[tx_line_pos_] = '\0';
                    if (parse_sid_line(tx_line_buf_, tx_line_pos_)) {
                        b2f_detected_ = true;
                        state_ = B2F_SID_EXCHANGE;
                    }
                    tx_line_pos_ = 0;
                }
            } else if (c != '\n') {
                if (tx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    tx_line_buf_[tx_line_pos_++] = c;
                else
                    tx_line_pos_ = 0;
            }
        }
        if (!b2f_detected_) {
            last_tx_input_consumed_ = static_cast<size_t>(in_pos);
            return pending + out_pos;
        }
    }

    while (in_pos < in_len) {
        if (state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_LOCAL) {
            int consumed = 0;
            int written = process_tx_payload(in + in_pos, in_len - in_pos,
                                             out + out_pos, out_cap - out_pos, &consumed);
            if (written < 0) {
                last_tx_input_consumed_ = static_cast<size_t>(in_pos + consumed);
                return -1;
            }
            out_pos += written;
            in_pos += consumed;
            if (written == 0 && consumed == 0) break;
        } else {
            char c = in[in_pos++];
            if (c == '\r') {
                tx_line_buf_[tx_line_pos_] = '\0';
                int written = process_tx_line(tx_line_buf_, tx_line_pos_,
                                              out + out_pos, out_cap - out_pos);
                if (written < 0) {
                    last_tx_input_consumed_ = static_cast<size_t>(in_pos);
                    return -1;
                }
                out_pos += written;
                tx_line_pos_ = 0;
                if (!pending_tx_output_.empty()) break;
            } else if (c != '\n') {
                if (tx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    tx_line_buf_[tx_line_pos_++] = c;
            }
        }
    }
    last_tx_input_consumed_ = static_cast<size_t>(in_pos);
    return pending + out_pos;
}

int B2fHandler::filter_rx(const char* in, int in_len, char* out, int out_cap) {
    last_rx_input_consumed_ = 0;
    if (in_len < 0 || out_cap < 0 || (!in && in_len > 0) || (!out && out_cap > 0)) return -1;
    int pending = drain_pending(false, out, out_cap);
    if (pending < 0) return -1;
    if (!pending_rx_output_.empty() || pending == out_cap) return pending;
    out += pending;
    out_cap -= pending;
    if (!initialized_) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        last_rx_input_consumed_ = static_cast<size_t>(copy);
        return pending + copy;
    }

    int out_pos = 0, in_pos = 0;

    if (!b2f_detected_) {
        for (; in_pos < in_len && !b2f_detected_ && out_pos < out_cap; in_pos++) {
            char c = in[in_pos];
            if (out_pos < out_cap) out[out_pos++] = c;
            if (c == '\r') {
                if (rx_line_pos_ > 0) {
                    rx_line_buf_[rx_line_pos_] = '\0';
                    if (parse_sid_line(rx_line_buf_, rx_line_pos_)) {
                        b2f_detected_ = true;
                        state_ = B2F_SID_EXCHANGE;
                    }
                    rx_line_pos_ = 0;
                }
            } else if (c != '\n') {
                if (rx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    rx_line_buf_[rx_line_pos_++] = c;
                else
                    rx_line_pos_ = 0;
            }
        }
        if (!b2f_detected_) {
            last_rx_input_consumed_ = static_cast<size_t>(in_pos);
            return pending + out_pos;
        }
    }

    while (in_pos < in_len) {
        if (state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_REMOTE) {
            int consumed = 0;
            int written = process_rx_payload(in + in_pos, in_len - in_pos,
                                             out + out_pos, out_cap - out_pos, &consumed);
            if (written < 0) {
                last_rx_input_consumed_ = static_cast<size_t>(in_pos + consumed);
                return -1;
            }
            out_pos += written;
            in_pos += consumed;
            if (written == 0 && consumed == 0) break;
        } else {
            char c = in[in_pos++];
            if (c == '\r') {
                rx_line_buf_[rx_line_pos_] = '\0';
                int written = process_rx_line(rx_line_buf_, rx_line_pos_,
                                              out + out_pos, out_cap - out_pos);
                if (written < 0) {
                    last_rx_input_consumed_ = static_cast<size_t>(in_pos);
                    return -1;
                }
                out_pos += written;
                rx_line_pos_ = 0;
                if (!pending_rx_output_.empty()) break;
            } else if (c != '\n') {
                if (rx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    rx_line_buf_[rx_line_pos_++] = c;
            }
        }
    }
    last_rx_input_consumed_ = static_cast<size_t>(in_pos);
    return pending + out_pos;
}

v2::RecordTransformResult B2fHandler::filter_tx_record(const uint8_t* in,
                                                       size_t in_len) {
    v2::RecordTransformResult result;
    if (in_len > B2F_PAYLOAD_BUF_SIZE || (in_len > 0 && !in)) {
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::OriginalSizeLimitExceeded;
        return result;
    }
    filter_failed_ = false;
    size_t offset = 0;
    std::vector<uint8_t> chunk(64 * 1024);
    for (;;) {
        const size_t remaining = in_len - offset;
        const char* input = remaining
            ? reinterpret_cast<const char*>(in + offset) : "";
        const int count = filter_tx(input, static_cast<int>(remaining),
            reinterpret_cast<char*>(chunk.data()), static_cast<int>(chunk.size()));
        if (count < 0 || filter_failed_) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::TransformFailure;
            result.input_bytes_consumed = offset + last_tx_input_consumed_;
            result.produced_bytes.clear();
            return result;
        }
        offset += last_tx_input_consumed_;
        if (static_cast<size_t>(count) > B2F_PAYLOAD_BUF_SIZE -
                std::min(result.produced_bytes.size(),
                         static_cast<size_t>(B2F_PAYLOAD_BUF_SIZE))) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::AggregateLimitExceeded;
            result.input_bytes_consumed = offset;
            result.produced_bytes.clear();
            return result;
        }
        result.produced_bytes.insert(result.produced_bytes.end(), chunk.begin(),
                                     chunk.begin() + count);
        const bool pending = pending_tx_offset_ < pending_tx_output_.size();
        if (offset == in_len && !pending) break;
        if (count == 0 && last_tx_input_consumed_ == 0) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::TransformFailure;
            result.input_bytes_consumed = offset;
            result.produced_bytes.clear();
            return result;
        }
    }
    result.input_bytes_consumed = offset;
    result.retained_output_bytes = 0;
    // Completed output may precede an unfinished line or payload from the same
    // submitted record.  Report that partial coverage as Buffered even though
    // the completed prefix is returned: callers must not use output presence as
    // proof that the record's retained suffix has entered transport custody.
    result.status = has_retained_input_suffix(true)
        ? v2::TransformStatus::Buffered
        : (result.produced_bytes.empty() ? v2::TransformStatus::Buffered
                                         : v2::TransformStatus::Produced);
    return result;
}

v2::RecordTransformResult B2fHandler::filter_rx_record(const uint8_t* in,
                                                       size_t in_len) {
    v2::RecordTransformResult result;
    if (in_len > B2F_PAYLOAD_BUF_SIZE || (in_len > 0 && !in)) {
        result.status = v2::TransformStatus::Failed;
        result.error = v2::EnvelopeValidationError::OriginalSizeLimitExceeded;
        return result;
    }
    filter_failed_ = false;
    size_t offset = 0;
    std::vector<uint8_t> chunk(64 * 1024);
    for (;;) {
        const size_t remaining = in_len - offset;
        const char* input = remaining
            ? reinterpret_cast<const char*>(in + offset) : "";
        const int count = filter_rx(input, static_cast<int>(remaining),
            reinterpret_cast<char*>(chunk.data()), static_cast<int>(chunk.size()));
        if (count < 0 || filter_failed_) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::TransformFailure;
            result.input_bytes_consumed = offset + last_rx_input_consumed_;
            result.produced_bytes.clear();
            return result;
        }
        offset += last_rx_input_consumed_;
        if (static_cast<size_t>(count) > B2F_PAYLOAD_BUF_SIZE -
                std::min(result.produced_bytes.size(),
                         static_cast<size_t>(B2F_PAYLOAD_BUF_SIZE))) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::AggregateLimitExceeded;
            result.input_bytes_consumed = offset;
            result.produced_bytes.clear();
            return result;
        }
        result.produced_bytes.insert(result.produced_bytes.end(), chunk.begin(),
                                     chunk.begin() + count);
        const bool pending = pending_rx_offset_ < pending_rx_output_.size();
        if (offset == in_len && !pending) break;
        if (count == 0 && last_rx_input_consumed_ == 0) {
            result.status = v2::TransformStatus::Failed;
            result.error = v2::EnvelopeValidationError::TransformFailure;
            result.input_bytes_consumed = offset;
            result.produced_bytes.clear();
            return result;
        }
    }
    result.input_bytes_consumed = offset;
    result.retained_output_bytes = 0;
    result.status = has_retained_input_suffix(false)
        ? v2::TransformStatus::Buffered
        : (result.produced_bytes.empty() ? v2::TransformStatus::Buffered
                                         : v2::TransformStatus::Produced);
    return result;
}

} // namespace iris
