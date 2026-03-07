#ifndef IRIS_CRYPTO_H
#define IRIS_CRYPTO_H

/*
 * Iris post-quantum hybrid encryption suite.
 * Ported from Mercury's cl_cipher_suite.
 *
 * Hybrid key exchange: X25519 (classical EC-DH) + ML-KEM-768 (post-quantum KEM).
 * Symmetric encryption: ChaCha20-Poly1305 (IETF, 256-bit key, 96-bit nonce).
 * Key derivation: HKDF-Blake2b (monocypher).
 * Counter-based nonces: direction(4) + batch_counter(8) = 12 bytes, zero overhead.
 */

#include <cstdint>
#include <cstddef>
#include <vector>
#include <array>

namespace iris {

// Key material sizes
static constexpr int MLKEM_PK_SIZE    = 1184;
static constexpr int MLKEM_SK_SIZE    = 2400;
static constexpr int MLKEM_CT_SIZE    = 1088;
static constexpr int MLKEM_SS_SIZE    = 32;
static constexpr int X25519_KEY_SIZE  = 32;
static constexpr int SESSION_KEY_SIZE = 32;
static constexpr int AUTH_TAG_SIZE    = 16;
static constexpr int AUTH_TAG_COMPACT = 4;
static constexpr int CRYPTO_NONCE_SIZE = 12;

// Direction tags for nonce derivation
static constexpr uint32_t DIR_CMD_TO_RSP = 0x00000000;
static constexpr uint32_t DIR_RSP_TO_CMD = 0x00000001;

using CryptoKey = std::array<uint8_t, 32>;

// Key exchange phases
enum class KxPhase {
    IDLE,
    X25519_SENT,
    X25519_DONE,
    MLKEM_PK_SENT,
    MLKEM_CT_SENT,
    HYBRID_DONE,
    ACTIVE,
};

class CipherSuite {
public:
    CipherSuite();
    ~CipherSuite();

    // Phase 1: X25519
    int generate_x25519_keypair(uint8_t pubkey_out[X25519_KEY_SIZE]);
    int compute_x25519_shared(const uint8_t peer_pubkey[X25519_KEY_SIZE]);

    // Phase 2: ML-KEM-768
    int generate_mlkem_keypair(uint8_t encaps_key_out[MLKEM_PK_SIZE]);
    int encapsulate_mlkem(const uint8_t encaps_key[MLKEM_PK_SIZE],
                          uint8_t ciphertext_out[MLKEM_CT_SIZE]);
    int decapsulate_mlkem(const uint8_t ciphertext[MLKEM_CT_SIZE]);

    // Key derivation
    void derive_session_key(const char* local_call, const char* remote_call,
                            const uint8_t* psk, int psk_len, bool mlkem_done);

    // Per-batch encrypt/decrypt with counter-based nonces
    int encrypt(const uint8_t* in, int in_len, uint8_t* out, int out_capacity,
                uint64_t batch_counter, uint32_t direction, int tag_size);
    int decrypt(const uint8_t* in, int in_len, uint8_t* out, int out_capacity,
                uint64_t batch_counter, uint32_t direction, int tag_size);

    // PSK mismatch detection
    void compute_key_confirmation(uint8_t tag_out[8]);

    // Activation / state
    void activate();
    void wipe();
    bool is_active() const { return encryption_active_; }
    bool is_pq_upgraded() const { return pq_active_; }
    KxPhase get_kx_phase() const { return kx_phase_; }
    int get_tag_size(bool compact) const { return compact ? AUTH_TAG_COMPACT : AUTH_TAG_SIZE; }

private:
    uint8_t x25519_sk_[X25519_KEY_SIZE];
    uint8_t x25519_pk_[X25519_KEY_SIZE];
    uint8_t x25519_shared_[X25519_KEY_SIZE];
    bool x25519_ready_;

    uint8_t* mlkem_sk_;
    uint8_t mlkem_shared_[MLKEM_SS_SIZE];
    bool mlkem_ready_;

    uint8_t session_key_[SESSION_KEY_SIZE];
    bool encryption_active_;
    bool pq_active_;
    KxPhase kx_phase_;
};

// Legacy free-function API (backward compat with existing code)
CryptoKey crypto_random_key();

struct KeyPair {
    CryptoKey secret;
    CryptoKey public_key;
};
KeyPair crypto_generate_keypair();
CryptoKey crypto_key_exchange(const CryptoKey& my_secret, const CryptoKey& their_public);

std::vector<uint8_t> crypto_encrypt(const uint8_t* plaintext, size_t len, const CryptoKey& key);
std::vector<uint8_t> crypto_decrypt(const uint8_t* data, size_t len, const CryptoKey& key);
std::vector<uint8_t> encrypt_frame(const uint8_t* data, size_t len, const CryptoKey& key);
std::vector<uint8_t> decrypt_frame(const uint8_t* data, size_t len, const CryptoKey& key);

} // namespace iris

#endif
