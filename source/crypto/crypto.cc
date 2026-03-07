/*
 * Iris post-quantum hybrid encryption suite — implementation.
 * Ported from Mercury's cl_cipher_suite.
 *
 * X25519 + ML-KEM-768 hybrid key exchange with HKDF-Blake2b key derivation.
 * ChaCha20-Poly1305 (IETF) symmetric encryption with counter-based nonces.
 */

#include "crypto/crypto.h"

extern "C" {
#include "monocypher.h"
}

#include <cstdio>
#include <cstdlib>
#include <cstring>

// ML-KEM-768 API
extern "C" {
#include "mlkem_native.h"
}

namespace iris {

// ---------------------------------------------------------------------------
// OS CSPRNG
// ---------------------------------------------------------------------------
#ifdef _WIN32
#include <windows.h>
#include <bcrypt.h>
static int os_random(uint8_t* out, size_t len) {
    NTSTATUS status = BCryptGenRandom(NULL, out, (ULONG)len,
                                       BCRYPT_USE_SYSTEM_PREFERRED_RNG);
    return (status == 0) ? 0 : -1;
}
#else
#include <sys/random.h>
static int os_random(uint8_t* out, size_t len) {
    ssize_t got = getrandom(out, len, 0);
    return (got == (ssize_t)len) ? 0 : -1;
}
#endif

// Provide randombytes() for mlkem-native
extern "C" int randombytes(uint8_t* out, size_t outlen) {
    return os_random(out, outlen);
}

// ---------------------------------------------------------------------------
// HKDF-Blake2b
// ---------------------------------------------------------------------------

static void hkdf_extract(const uint8_t* salt, size_t salt_len,
                          const uint8_t* ikm, size_t ikm_len,
                          uint8_t prk[32]) {
    crypto_blake2b_keyed(prk, 32, salt, salt_len, ikm, ikm_len);
}

static void hkdf_expand(const uint8_t prk[32],
                         const uint8_t* info, size_t info_len,
                         uint8_t okm[32]) {
    crypto_blake2b_ctx ctx;
    crypto_blake2b_keyed_init(&ctx, 32, prk, 32);
    crypto_blake2b_update(&ctx, info, info_len);
    uint8_t one = 0x01;
    crypto_blake2b_update(&ctx, &one, 1);
    crypto_blake2b_final(&ctx, okm);
    crypto_wipe(&ctx, sizeof(ctx));
}

// ---------------------------------------------------------------------------
// CipherSuite
// ---------------------------------------------------------------------------

CipherSuite::CipherSuite() {
    memset(x25519_sk_, 0, sizeof(x25519_sk_));
    memset(x25519_pk_, 0, sizeof(x25519_pk_));
    memset(x25519_shared_, 0, sizeof(x25519_shared_));
    x25519_ready_ = false;
    mlkem_sk_ = nullptr;
    memset(mlkem_shared_, 0, sizeof(mlkem_shared_));
    mlkem_ready_ = false;
    memset(session_key_, 0, sizeof(session_key_));
    encryption_active_ = false;
    pq_active_ = false;
    kx_phase_ = KxPhase::IDLE;
}

CipherSuite::~CipherSuite() { wipe(); }

void CipherSuite::wipe() {
    crypto_wipe(x25519_sk_, sizeof(x25519_sk_));
    crypto_wipe(x25519_pk_, sizeof(x25519_pk_));
    crypto_wipe(x25519_shared_, sizeof(x25519_shared_));
    x25519_ready_ = false;
    if (mlkem_sk_) {
        crypto_wipe(mlkem_sk_, MLKEM_SK_SIZE);
        free(mlkem_sk_);
        mlkem_sk_ = nullptr;
    }
    crypto_wipe(mlkem_shared_, sizeof(mlkem_shared_));
    mlkem_ready_ = false;
    crypto_wipe(session_key_, sizeof(session_key_));
    encryption_active_ = false;
    pq_active_ = false;
    kx_phase_ = KxPhase::IDLE;
}

// ---------------------------------------------------------------------------
// X25519
// ---------------------------------------------------------------------------

int CipherSuite::generate_x25519_keypair(uint8_t pubkey_out[X25519_KEY_SIZE]) {
    if (os_random(x25519_sk_, 32) != 0) return -1;
    crypto_x25519_public_key(x25519_pk_, x25519_sk_);
    memcpy(pubkey_out, x25519_pk_, 32);
    return 0;
}

int CipherSuite::compute_x25519_shared(const uint8_t peer_pubkey[X25519_KEY_SIZE]) {
    crypto_x25519(x25519_shared_, x25519_sk_, peer_pubkey);
    uint8_t zero[32] = {0};
    if (memcmp(x25519_shared_, zero, 32) == 0) {
        crypto_wipe(x25519_shared_, 32);
        return -1;
    }
    x25519_ready_ = true;
    return 0;
}

// ---------------------------------------------------------------------------
// ML-KEM-768
// ---------------------------------------------------------------------------

int CipherSuite::generate_mlkem_keypair(uint8_t encaps_key_out[MLKEM_PK_SIZE]) {
    if (!mlkem_sk_) {
        mlkem_sk_ = (uint8_t*)malloc(MLKEM_SK_SIZE);
        if (!mlkem_sk_) return -1;
    }
    uint8_t pk[MLKEM_PK_SIZE];
    int rc = crypto_kem_keypair(pk, mlkem_sk_);
    if (rc != 0) return -1;
    memcpy(encaps_key_out, pk, MLKEM_PK_SIZE);
    return 0;
}

int CipherSuite::encapsulate_mlkem(const uint8_t encaps_key[MLKEM_PK_SIZE],
                                    uint8_t ciphertext_out[MLKEM_CT_SIZE]) {
    int rc = crypto_kem_enc(ciphertext_out, mlkem_shared_, encaps_key);
    if (rc != 0) return -1;
    mlkem_ready_ = true;
    return 0;
}

int CipherSuite::decapsulate_mlkem(const uint8_t ciphertext[MLKEM_CT_SIZE]) {
    if (!mlkem_sk_) return -1;
    int rc = crypto_kem_dec(mlkem_shared_, ciphertext, mlkem_sk_);
    if (rc != 0) return -1;
    mlkem_ready_ = true;
    return 0;
}

// ---------------------------------------------------------------------------
// Key Derivation
// ---------------------------------------------------------------------------

void CipherSuite::derive_session_key(const char* local_call, const char* remote_call,
                                      const uint8_t* psk, int psk_len, bool mlkem_done) {
    uint8_t ikm[64];
    int ikm_len;
    if (mlkem_done && mlkem_ready_) {
        memcpy(ikm, x25519_shared_, 32);
        memcpy(ikm + 32, mlkem_shared_, 32);
        ikm_len = 64;
        pq_active_ = true;
    } else {
        memcpy(ikm, x25519_shared_, 32);
        ikm_len = 32;
        pq_active_ = false;
    }

    uint8_t salt[256];
    int salt_len = 0;
    const char* prefix = "iris-v1";
    int prefix_len = (int)strlen(prefix);
    memcpy(salt, prefix, prefix_len);
    salt_len += prefix_len;

    if (psk && psk_len > 0) {
        uint8_t psk_hash[32];
        crypto_blake2b(psk_hash, 32, psk, psk_len);
        memcpy(salt + salt_len, psk_hash, 32);
        salt_len += 32;
        crypto_wipe(psk_hash, 32);
    }

    if (local_call) {
        int cl = (int)strlen(local_call);
        memcpy(salt + salt_len, local_call, cl);
        salt_len += cl;
    }
    if (remote_call) {
        int cl = (int)strlen(remote_call);
        memcpy(salt + salt_len, remote_call, cl);
        salt_len += cl;
    }

    uint8_t prk[32];
    hkdf_extract(salt, salt_len, ikm, ikm_len, prk);

    const char* info = "data-encryption";
    hkdf_expand(prk, (const uint8_t*)info, strlen(info), session_key_);

    crypto_wipe(ikm, sizeof(ikm));
    crypto_wipe(prk, sizeof(prk));
    crypto_wipe(salt, sizeof(salt));
}

// ---------------------------------------------------------------------------
// Key Confirmation
// ---------------------------------------------------------------------------

void CipherSuite::compute_key_confirmation(uint8_t tag_out[8]) {
    const char* msg = "iris-key-confirm";
    uint8_t full_hash[32];
    crypto_blake2b_keyed(full_hash, 32, session_key_, SESSION_KEY_SIZE,
                         (const uint8_t*)msg, strlen(msg));
    memcpy(tag_out, full_hash, 8);
    crypto_wipe(full_hash, sizeof(full_hash));
}

// ---------------------------------------------------------------------------
// Counter-based nonce
// ---------------------------------------------------------------------------

static void build_nonce(uint8_t nonce[CRYPTO_NONCE_SIZE],
                         uint32_t direction, uint64_t batch_counter) {
    nonce[0] = (uint8_t)((direction >> 24) & 0xFF);
    nonce[1] = (uint8_t)((direction >> 16) & 0xFF);
    nonce[2] = (uint8_t)((direction >>  8) & 0xFF);
    nonce[3] = (uint8_t)((direction      ) & 0xFF);
    nonce[4]  = (uint8_t)((batch_counter >> 56) & 0xFF);
    nonce[5]  = (uint8_t)((batch_counter >> 48) & 0xFF);
    nonce[6]  = (uint8_t)((batch_counter >> 40) & 0xFF);
    nonce[7]  = (uint8_t)((batch_counter >> 32) & 0xFF);
    nonce[8]  = (uint8_t)((batch_counter >> 24) & 0xFF);
    nonce[9]  = (uint8_t)((batch_counter >> 16) & 0xFF);
    nonce[10] = (uint8_t)((batch_counter >>  8) & 0xFF);
    nonce[11] = (uint8_t)((batch_counter      ) & 0xFF);
}

// ---------------------------------------------------------------------------
// Encrypt / Decrypt
// ---------------------------------------------------------------------------

int CipherSuite::encrypt(const uint8_t* in, int in_len, uint8_t* out, int out_capacity,
                          uint64_t batch_counter, uint32_t direction, int tag_size) {
    if (!encryption_active_ || in_len <= 0) return -1;
    if (in_len + tag_size > out_capacity) return -1;

    uint8_t nonce[CRYPTO_NONCE_SIZE];
    build_nonce(nonce, direction, batch_counter);

    uint8_t full_mac[16];
    crypto_aead_ctx ctx;
    crypto_aead_init_ietf(&ctx, session_key_, nonce);
    crypto_aead_write(&ctx, out, full_mac, NULL, 0, in, in_len);
    crypto_wipe(&ctx, sizeof(ctx));

    memcpy(out + in_len, full_mac, tag_size);
    crypto_wipe(full_mac, sizeof(full_mac));
    return in_len + tag_size;
}

int CipherSuite::decrypt(const uint8_t* in, int in_len, uint8_t* out, int out_capacity,
                          uint64_t batch_counter, uint32_t direction, int tag_size) {
    if (!encryption_active_ || in_len <= tag_size) return -1;
    int plain_len = in_len - tag_size;
    if (plain_len > out_capacity) return -1;

    uint8_t nonce[CRYPTO_NONCE_SIZE];
    build_nonce(nonce, direction, batch_counter);

    const uint8_t* ciphertext = in;
    const uint8_t* received_tag = in + plain_len;

    if (tag_size == AUTH_TAG_SIZE) {
        crypto_aead_ctx ctx;
        crypto_aead_init_ietf(&ctx, session_key_, nonce);
        int rc = crypto_aead_read(&ctx, out, received_tag, NULL, 0, ciphertext, plain_len);
        crypto_wipe(&ctx, sizeof(ctx));
        if (rc != 0) { crypto_wipe(out, plain_len); return -1; }
    } else {
        // Truncated tag: manual ChaCha20 + Poly1305
        uint8_t block0[64] = {0};
        crypto_chacha20_ietf(block0, block0, 64, session_key_, nonce, 0);
        uint8_t poly_key[32];
        memcpy(poly_key, block0, 32);
        crypto_wipe(block0, sizeof(block0));

        // Compute expected MAC
        int padded_ct = ((plain_len + 15) / 16) * 16;
        int msg_len = padded_ct + 16;
        uint8_t* msg = (uint8_t*)calloc(1, msg_len);
        if (!msg) return -1;
        memcpy(msg, ciphertext, plain_len);
        uint64_t ct_len64 = (uint64_t)plain_len;
        msg[padded_ct + 8]  = (uint8_t)(ct_len64 & 0xFF);
        msg[padded_ct + 9]  = (uint8_t)((ct_len64 >> 8) & 0xFF);
        msg[padded_ct + 10] = (uint8_t)((ct_len64 >> 16) & 0xFF);
        msg[padded_ct + 11] = (uint8_t)((ct_len64 >> 24) & 0xFF);

        uint8_t expected_mac[16];
        crypto_poly1305(expected_mac, msg, msg_len, poly_key);
        crypto_wipe(poly_key, sizeof(poly_key));
        crypto_wipe(msg, msg_len);
        free(msg);

        uint8_t diff = 0;
        for (int i = 0; i < tag_size; i++)
            diff |= expected_mac[i] ^ received_tag[i];
        crypto_wipe(expected_mac, sizeof(expected_mac));

        if (diff != 0) { crypto_wipe(out, plain_len); return -1; }
        crypto_chacha20_ietf(out, ciphertext, plain_len, session_key_, nonce, 1);
    }

    return plain_len;
}

void CipherSuite::activate() {
    encryption_active_ = true;
    kx_phase_ = KxPhase::ACTIVE;
}

// ---------------------------------------------------------------------------
// Legacy free-function API
// ---------------------------------------------------------------------------

static void fill_random(uint8_t* buf, size_t len) {
    os_random(buf, len);
}

CryptoKey crypto_random_key() {
    CryptoKey key;
    fill_random(key.data(), 32);
    return key;
}

KeyPair crypto_generate_keypair() {
    KeyPair kp;
    fill_random(kp.secret.data(), 32);
    crypto_x25519_public_key(kp.public_key.data(), kp.secret.data());
    return kp;
}

CryptoKey crypto_key_exchange(const CryptoKey& my_secret, const CryptoKey& their_public) {
    CryptoKey shared;
    crypto_x25519(shared.data(), my_secret.data(), their_public.data());
    uint8_t hash[64];
    crypto_blake2b(hash, 32, shared.data(), 32);
    std::memcpy(shared.data(), hash, 32);
    crypto_wipe(hash, sizeof(hash));
    return shared;
}

std::vector<uint8_t> crypto_encrypt(const uint8_t* plaintext, size_t len, const CryptoKey& key) {
    // nonce(24) + mac(16) + ciphertext
    std::vector<uint8_t> out(24 + 16 + len);
    fill_random(out.data(), 24);
    crypto_aead_lock(out.data() + 24 + 16, out.data() + 24,
                     key.data(), out.data(), nullptr, 0, plaintext, len);
    return out;
}

std::vector<uint8_t> crypto_decrypt(const uint8_t* data, size_t len, const CryptoKey& key) {
    if (len < 24 + 16) return {};
    size_t text_size = len - 24 - 16;
    std::vector<uint8_t> plaintext(text_size);
    int ret = crypto_aead_unlock(plaintext.data(), data + 24, key.data(),
                                  data, nullptr, 0, data + 24 + 16, text_size);
    if (ret != 0) return {};
    return plaintext;
}

std::vector<uint8_t> encrypt_frame(const uint8_t* data, size_t len, const CryptoKey& key) {
    auto encrypted = crypto_encrypt(data, len, key);
    std::vector<uint8_t> out;
    out.reserve(1 + encrypted.size());
    out.push_back(0x01);
    out.insert(out.end(), encrypted.begin(), encrypted.end());
    return out;
}

std::vector<uint8_t> decrypt_frame(const uint8_t* data, size_t len, const CryptoKey& key) {
    if (len < 1) return {};
    if (data[0] == 0x00) return std::vector<uint8_t>(data + 1, data + len);
    if (data[0] == 0x01) return crypto_decrypt(data + 1, len - 1, key);
    return {};
}

} // namespace iris
