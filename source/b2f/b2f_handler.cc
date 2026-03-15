/*
 * B2F protocol handler implementation.
 * Ported from Mercury's cl_b2f_handler.
 */

#include "b2f/b2f_handler.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace iris {

B2fHandler::B2fHandler()
    : unroll_enabled(true),
      payload_buf_(nullptr), plain_buf_(nullptr), initialized_(false) {
    reset();
}

B2fHandler::~B2fHandler() { deinit(); }

void B2fHandler::init() {
    if (initialized_) return;
    payload_buf_ = (uint8_t*)malloc(B2F_PAYLOAD_BUF_SIZE);
    plain_buf_ = (uint8_t*)malloc(B2F_PLAIN_BUF_SIZE);
    if (!payload_buf_ || !plain_buf_) { deinit(); return; }
    initialized_ = true;
    printf("[B2F] Handler initialized\n");
    fflush(stdout);
}

void B2fHandler::deinit() {
    if (payload_buf_) { free(payload_buf_); payload_buf_ = nullptr; }
    if (plain_buf_) { free(plain_buf_); plain_buf_ = nullptr; }
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
    pos++;

    prop->uncomp_size = 0;
    while (pos < len && line[pos] >= '0' && line[pos] <= '9')
        prop->uncomp_size = prop->uncomp_size * 10 + (line[pos++] - '0');
    pos++;

    prop->comp_size = 0;
    while (pos < len && line[pos] >= '0' && line[pos] <= '9')
        prop->comp_size = prop->comp_size * 10 + (line[pos++] - '0');

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
                uint32_t offset = 0;
                pos++;
                while (pos < len && line[pos] >= '0' && line[pos] <= '9')
                    offset = offset * 10 + (line[pos++] - '0');
                proposals_[i].resume_offset = offset;
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
    if (len + 1 > out_cap) return -1;
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
    if (len + 1 > out_cap) return -1;
    memcpy(out, line, len);
    out[len] = '\r';
    return len + 1;
}

// ---- Payload handling ----

int B2fHandler::process_tx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed) {
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
            if (payload_buf_pos_ + chunk <= B2F_PAYLOAD_BUF_SIZE) {
                memcpy(payload_buf_ + payload_buf_pos_, in + in_pos, chunk);
                payload_buf_pos_ += chunk;
            }
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;

            if (payload_bytes_remaining_ == 0) {
                size_t plain_len = 0;
                int rc = lzhuf_decode_buffer(payload_buf_, payload_buf_pos_,
                    plain_buf_, B2F_PLAIN_BUF_SIZE, &plain_len);

                if (rc == 0 && plain_len > 0) {
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
                    if (out_pos + (int)plain_len <= out_cap) {
                        memcpy(out + out_pos, plain_buf_, plain_len);
                        out_pos += (int)plain_len;
                    }
                } else {
                    printf("[B2F-TX] Unroll FAILED for %s (rc=%d, len=%zu), "
                           "passing through %d raw bytes\n",
                           proposals_[current_payload_idx_].mid,
                           rc, plain_len, payload_buf_pos_);
                    fflush(stdout);
                    if (out_pos + payload_buf_pos_ <= out_cap) {
                        memcpy(out + out_pos, payload_buf_, payload_buf_pos_);
                        out_pos += payload_buf_pos_;
                    }
                }

                payload_buf_pos_ = 0;
                current_payload_idx_ = find_next_accepted(current_payload_idx_ + 1);
                if (current_payload_idx_ >= 0) {
                    auto& np = proposals_[current_payload_idx_];
                    payload_bytes_remaining_ = np.comp_size - np.resume_offset;
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
        } else {
            if (out_pos + chunk <= out_cap) {
                memcpy(out + out_pos, in + in_pos, chunk);
                out_pos += chunk;
            }
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;
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
            if (payload_buf_pos_ + chunk <= B2F_PAYLOAD_BUF_SIZE) {
                memcpy(payload_buf_ + payload_buf_pos_, in + in_pos, chunk);
                payload_buf_pos_ += chunk;
            }
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;

            if (payload_bytes_remaining_ == 0) {
                size_t lzhuf_len = 0;
                int rc = lzhuf_encode_buffer(payload_buf_, payload_buf_pos_,
                    plain_buf_, B2F_PLAIN_BUF_SIZE, &lzhuf_len);

                if (rc == 0 && lzhuf_len > 0) {
                    // Validate rerolled size against proposal's comp_size
                    int expected = proposals_[current_payload_idx_].comp_size;
                    if ((int)lzhuf_len != expected) {
                        printf("[B2F-RX] WARNING: reroll size mismatch for %s: "
                               "got %zu, expected %d (delta %+d)\n",
                               proposals_[current_payload_idx_].mid,
                               lzhuf_len, expected, (int)lzhuf_len - expected);
                        fflush(stdout);
                    }
                    printf("[B2F-RX] Rerolled %s: %d plaintext -> %zu LZHUF\n",
                           proposals_[current_payload_idx_].mid,
                           payload_buf_pos_, lzhuf_len);
                    fflush(stdout);
                    if (out_pos + (int)lzhuf_len <= out_cap) {
                        memcpy(out + out_pos, plain_buf_, lzhuf_len);
                        out_pos += (int)lzhuf_len;
                    }
                } else {
                    printf("[B2F-RX] Reroll FAILED for %s (rc=%d, len=%zu), "
                           "passing through %d raw bytes\n",
                           proposals_[current_payload_idx_].mid,
                           rc, lzhuf_len, payload_buf_pos_);
                    fflush(stdout);
                    if (out_pos + payload_buf_pos_ <= out_cap) {
                        memcpy(out + out_pos, payload_buf_, payload_buf_pos_);
                        out_pos += payload_buf_pos_;
                    }
                }

                payload_buf_pos_ = 0;
                current_payload_idx_ = find_next_accepted(current_payload_idx_ + 1);
                if (current_payload_idx_ >= 0) {
                    auto& np = proposals_[current_payload_idx_];
                    bool next_can_unroll = np.resume_offset == 0;
                    payload_bytes_remaining_ = next_can_unroll ?
                        np.uncomp_size :
                        (np.comp_size - np.resume_offset);
                } else {
                    state_ = B2F_CHECKSUM;
                }
            }
        } else {
            if (out_pos + chunk <= out_cap) {
                memcpy(out + out_pos, in + in_pos, chunk);
                out_pos += chunk;
            }
            in_pos += chunk;
            payload_bytes_remaining_ -= chunk;
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
    if (!initialized_) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        return copy;
    }

    int out_pos = 0, in_pos = 0;

    if (!b2f_detected_) {
        for (; in_pos < in_len && !b2f_detected_; in_pos++) {
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
        if (!b2f_detected_) return out_pos;
    }

    while (in_pos < in_len) {
        if (state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_LOCAL) {
            int consumed = 0;
            int written = process_tx_payload(in + in_pos, in_len - in_pos,
                                             out + out_pos, out_cap - out_pos, &consumed);
            if (written < 0) return -1;
            out_pos += written;
            in_pos += consumed;
        } else {
            char c = in[in_pos++];
            if (c == '\r') {
                tx_line_buf_[tx_line_pos_] = '\0';
                int written = process_tx_line(tx_line_buf_, tx_line_pos_,
                                              out + out_pos, out_cap - out_pos);
                if (written < 0) return -1;
                out_pos += written;
                tx_line_pos_ = 0;
            } else if (c != '\n') {
                if (tx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    tx_line_buf_[tx_line_pos_++] = c;
            }
        }
    }
    return out_pos;
}

int B2fHandler::filter_rx(const char* in, int in_len, char* out, int out_cap) {
    if (!initialized_) {
        int copy = in_len < out_cap ? in_len : out_cap;
        memcpy(out, in, copy);
        return copy;
    }

    int out_pos = 0, in_pos = 0;

    if (!b2f_detected_) {
        for (; in_pos < in_len && !b2f_detected_; in_pos++) {
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
        if (!b2f_detected_) return out_pos;
    }

    while (in_pos < in_len) {
        if (state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_REMOTE) {
            int consumed = 0;
            int written = process_rx_payload(in + in_pos, in_len - in_pos,
                                             out + out_pos, out_cap - out_pos, &consumed);
            if (written < 0) return -1;
            out_pos += written;
            in_pos += consumed;
        } else {
            char c = in[in_pos++];
            if (c == '\r') {
                rx_line_buf_[rx_line_pos_] = '\0';
                int written = process_rx_line(rx_line_buf_, rx_line_pos_,
                                              out + out_pos, out_cap - out_pos);
                if (written < 0) return -1;
                out_pos += written;
                rx_line_pos_ = 0;
            } else if (c != '\n') {
                if (rx_line_pos_ < B2F_LINE_BUF_SIZE - 1)
                    rx_line_buf_[rx_line_pos_++] = c;
            }
        }
    }
    return out_pos;
}

} // namespace iris
