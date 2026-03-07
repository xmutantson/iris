#ifndef IRIS_CRYPTO_H
#define IRIS_CRYPTO_H

#include <vector>
#include <cstdint>
#include <cstddef>
#include <array>

namespace iris {

static constexpr size_t KEY_SIZE = 32;
static constexpr size_t NONCE_SIZE = 24;
static constexpr size_t MAC_SIZE = 16;

using CryptoKey = std::array<uint8_t, KEY_SIZE>;

// Generate a random key (from OS entropy)
CryptoKey crypto_random_key();

// X25519 key exchange
struct KeyPair {
    CryptoKey secret;
    CryptoKey public_key;
};

KeyPair crypto_generate_keypair();
CryptoKey crypto_key_exchange(const CryptoKey& my_secret, const CryptoKey& their_public);

// AEAD encrypt: plaintext -> nonce(24) + mac(16) + ciphertext
// Uses XChaCha20-Poly1305
std::vector<uint8_t> crypto_encrypt(const uint8_t* plaintext, size_t len,
                                     const CryptoKey& key);

// AEAD decrypt: nonce(24) + mac(16) + ciphertext -> plaintext
// Returns empty on authentication failure
std::vector<uint8_t> crypto_decrypt(const uint8_t* data, size_t len,
                                     const CryptoKey& key);

// Encrypt a frame with header byte (0x00=plain, 0x01=encrypted)
std::vector<uint8_t> encrypt_frame(const uint8_t* data, size_t len,
                                    const CryptoKey& key);
std::vector<uint8_t> decrypt_frame(const uint8_t* data, size_t len,
                                    const CryptoKey& key);

} // namespace iris

#endif
