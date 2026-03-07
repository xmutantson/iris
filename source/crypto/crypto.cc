#include "crypto/crypto.h"

extern "C" {
#include "monocypher.h"
}

#include <cstring>

#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif

namespace iris {

static void fill_random(uint8_t* buf, size_t len) {
#ifdef _WIN32
    BCryptGenRandom(nullptr, buf, (ULONG)len, BCRYPT_USE_SYSTEM_PREFERRED_RNG);
#else
    int fd = open("/dev/urandom", O_RDONLY);
    if (fd >= 0) {
        ssize_t n = read(fd, buf, len);
        (void)n;
        close(fd);
    }
#endif
}

CryptoKey crypto_random_key() {
    CryptoKey key;
    fill_random(key.data(), KEY_SIZE);
    return key;
}

KeyPair crypto_generate_keypair() {
    KeyPair kp;
    fill_random(kp.secret.data(), KEY_SIZE);
    crypto_x25519_public_key(kp.public_key.data(), kp.secret.data());
    return kp;
}

CryptoKey crypto_key_exchange(const CryptoKey& my_secret, const CryptoKey& their_public) {
    CryptoKey shared;
    crypto_x25519(shared.data(), my_secret.data(), their_public.data());
    // Hash the raw shared secret for use as a symmetric key
    uint8_t hash[64];
    crypto_blake2b(hash, 32, shared.data(), KEY_SIZE);
    std::memcpy(shared.data(), hash, KEY_SIZE);
    crypto_wipe(hash, sizeof(hash));
    return shared;
}

std::vector<uint8_t> crypto_encrypt(const uint8_t* plaintext, size_t len,
                                     const CryptoKey& key) {
    // Output: nonce(24) + mac(16) + ciphertext(len)
    std::vector<uint8_t> out(NONCE_SIZE + MAC_SIZE + len);

    // Generate random nonce
    fill_random(out.data(), NONCE_SIZE);

    // Encrypt
    crypto_aead_lock(out.data() + NONCE_SIZE + MAC_SIZE,  // ciphertext
                     out.data() + NONCE_SIZE,              // mac
                     key.data(),                           // key
                     out.data(),                           // nonce
                     nullptr, 0,                           // no AD
                     plaintext, len);                      // plaintext

    return out;
}

std::vector<uint8_t> crypto_decrypt(const uint8_t* data, size_t len,
                                     const CryptoKey& key) {
    if (len < NONCE_SIZE + MAC_SIZE) return {};

    size_t text_size = len - NONCE_SIZE - MAC_SIZE;
    std::vector<uint8_t> plaintext(text_size);

    int ret = crypto_aead_unlock(plaintext.data(),
                                  data + NONCE_SIZE,      // mac
                                  key.data(),             // key
                                  data,                   // nonce
                                  nullptr, 0,             // no AD
                                  data + NONCE_SIZE + MAC_SIZE,
                                  text_size);

    if (ret != 0) return {};  // authentication failed
    return plaintext;
}

std::vector<uint8_t> encrypt_frame(const uint8_t* data, size_t len,
                                    const CryptoKey& key) {
    auto encrypted = crypto_encrypt(data, len, key);
    std::vector<uint8_t> out;
    out.reserve(1 + encrypted.size());
    out.push_back(0x01);  // encrypted
    out.insert(out.end(), encrypted.begin(), encrypted.end());
    return out;
}

std::vector<uint8_t> decrypt_frame(const uint8_t* data, size_t len,
                                    const CryptoKey& key) {
    if (len < 1) return {};
    if (data[0] == 0x00) {
        // Unencrypted
        return std::vector<uint8_t>(data + 1, data + len);
    } else if (data[0] == 0x01) {
        // Encrypted
        return crypto_decrypt(data + 1, len - 1, key);
    }
    return {};
}

} // namespace iris
