#ifndef IRIS_B2F_HANDLER_H
#define IRIS_B2F_HANDLER_H

/*
 * B2F (Bin2Forwarding) protocol handler for Iris.
 * Ported from Mercury's cl_b2f_handler.
 *
 * Detects Winlink B2F sessions on the data port and intercepts LZHUF
 * compressed payloads. On TX: decompresses LZHUF to plaintext so Iris's
 * compressor (PPMd/zstd) can achieve better ratios. On RX: recompresses
 * plaintext back to LZHUF so the local Winlink client receives valid B2F.
 */

#include <cstdint>
#include <cstddef>

namespace iris {

static constexpr int B2F_MAX_PROPOSALS    = 5;
static constexpr int B2F_PAYLOAD_BUF_SIZE = 10 * 1024 * 1024;  // 10 MB
static constexpr int B2F_PLAIN_BUF_SIZE   = 256 * 1024;         // 256 KB
static constexpr int B2F_LINE_BUF_SIZE    = 512;

struct B2fProposal {
    char type;              // 'E' = EM, 'C' = CM
    char mid[13];
    uint32_t uncomp_size;
    uint32_t comp_size;
    int accepted;           // 1=accepted (+/Y/!/H), 0=rejected (-/N/R/E), -1=deferred (=/L)
    uint32_t resume_offset; // byte offset for '!' resume (0 = full transfer)
};

// LZHUF buffer wrappers
int lzhuf_decode_buffer(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len);
int lzhuf_encode_buffer(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len);

class B2fHandler {
public:
    B2fHandler();
    ~B2fHandler();

    void init();
    void deinit();
    void reset();

    // TX: Winlink client -> FIFO (decompresses LZHUF to plaintext)
    int filter_tx(const char* in, int in_len, char* out, int out_cap);

    // RX: FIFO -> Winlink client (recompresses plaintext to LZHUF)
    int filter_rx(const char* in, int in_len, char* out, int out_cap);

    bool is_b2f_session() const { return b2f_detected_; }
    bool is_initialized() const { return initialized_; }

    // State accessors for OFDM-KISS B2F proxy
    bool is_payload_transfer() const { return state_ == B2F_PAYLOAD_TRANSFER; }
    bool is_tx_payload_active() const {
        return state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_LOCAL;
    }
    bool is_rx_payload_active() const {
        return state_ == B2F_PAYLOAD_TRANSFER && current_proposer_ == PROPOSER_REMOTE;
    }
    // True if payload bytes have actually been consumed (not just FS parsed).
    // Used to distinguish "FS arrived but no data yet" from "data in flight".
    bool has_payload_data_in_flight() const {
        return state_ == B2F_PAYLOAD_TRANSFER && payload_buf_pos_ > 0;
    }
    // True if current proposal is a resume transfer (partial LZHUF, can't unroll).
    bool is_resume_transfer() const {
        return state_ == B2F_PAYLOAD_TRANSFER &&
               current_payload_idx_ >= 0 &&
               proposals_[current_payload_idx_].resume_offset > 0;
    }

    bool unroll_enabled;

private:
    enum State {
        B2F_IDLE,
        B2F_SID_EXCHANGE,
        B2F_WAIT_PROPOSALS,
        B2F_PARSING_FC,
        B2F_WAIT_FS,
        B2F_PAYLOAD_TRANSFER,
        B2F_CHECKSUM,
    };

    enum Proposer {
        PROPOSER_NONE,
        PROPOSER_LOCAL,
        PROPOSER_REMOTE,
    };

    State state_;
    Proposer current_proposer_;
    bool b2f_detected_;

    B2fProposal proposals_[B2F_MAX_PROPOSALS];
    int num_proposals_;
    int current_payload_idx_;
    int payload_bytes_remaining_;

    char tx_line_buf_[B2F_LINE_BUF_SIZE];
    int tx_line_pos_;
    char rx_line_buf_[B2F_LINE_BUF_SIZE];
    int rx_line_pos_;

    uint8_t* payload_buf_;
    int payload_buf_pos_;
    uint8_t* plain_buf_;
    bool initialized_;

    bool parse_sid_line(const char* line, int len);
    bool parse_fc_line(const char* line, int len, B2fProposal* prop);
    bool parse_fs_line(const char* line, int len);
    int find_next_accepted(int from);

    int process_tx_line(const char* line, int len, char* out, int out_cap);
    int process_rx_line(const char* line, int len, char* out, int out_cap);
    int process_tx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed);
    int process_rx_payload(const char* in, int in_len, char* out, int out_cap, int* in_consumed);
};

} // namespace iris

#endif
