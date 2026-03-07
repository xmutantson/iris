/*
 * DirectSound backend for Iris (Windows fallback).
 * Used when WASAPI is unavailable (older Windows versions).
 * Compile with IRIS_USE_DSOUND defined.
 */
#include "audio/audio.h"

#if defined(_WIN32) && defined(IRIS_USE_DSOUND)

#include <windows.h>
#include <dsound.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>
#include <cmath>

#pragma comment(lib, "dsound.lib")
#pragma comment(lib, "dxguid.lib")

namespace iris {

class DSoundCapture : public AudioCapture {
public:
    DSoundCapture() = default;
    ~DSoundCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        sample_rate_ = sample_rate;

        HRESULT hr = DirectSoundCaptureCreate(nullptr, &capture_, nullptr);
        if (FAILED(hr)) return false;

        WAVEFORMATEX wfx = {};
        wfx.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
        wfx.nChannels = (WORD)channels;
        wfx.nSamplesPerSec = (DWORD)sample_rate;
        wfx.wBitsPerSample = 32;
        wfx.nBlockAlign = wfx.nChannels * wfx.wBitsPerSample / 8;
        wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

        buffer_bytes_ = frames_per_buffer * wfx.nBlockAlign * 4;

        DSCBUFFERDESC desc = {};
        desc.dwSize = sizeof(desc);
        desc.dwBufferBytes = buffer_bytes_;
        desc.lpwfxFormat = &wfx;

        hr = capture_->CreateCaptureBuffer(&desc, &buffer_, nullptr);
        if (FAILED(hr)) { capture_->Release(); capture_ = nullptr; return false; }

        return true;
    }

    bool start() override {
        if (!buffer_ || running_) return false;
        buffer_->Start(DSCBSTART_LOOPING);
        running_ = true;
        thread_ = std::thread(&DSoundCapture::capture_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (buffer_) buffer_->Stop();
        return true;
    }

    void close() override {
        stop();
        if (buffer_) { buffer_->Release(); buffer_ = nullptr; }
        if (capture_) { capture_->Release(); capture_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }
    int read(float*, int) override { return 0; }

private:
    void capture_thread() {
        DWORD read_pos = 0;
        int chunk_bytes = buffer_frames_ * channels_ * (int)sizeof(float);
        std::vector<float> buf(buffer_frames_ * channels_);

        while (running_) {
            DWORD cap_pos = 0;
            buffer_->GetCurrentPosition(&cap_pos, nullptr);

            DWORD avail = (cap_pos >= read_pos) ?
                cap_pos - read_pos :
                buffer_bytes_ - read_pos + cap_pos;

            if ((int)avail >= chunk_bytes) {
                void* p1 = nullptr; void* p2 = nullptr;
                DWORD s1 = 0, s2 = 0;
                buffer_->Lock(read_pos, chunk_bytes, &p1, &s1, &p2, &s2, 0);
                if (p1) memcpy(buf.data(), p1, s1);
                if (p2) memcpy((uint8_t*)buf.data() + s1, p2, s2);
                buffer_->Unlock(p1, s1, p2, s2);

                read_pos = (read_pos + chunk_bytes) % buffer_bytes_;

                if (volume_ != 1.0f) {
                    for (size_t i = 0; i < buf.size(); i++)
                        buf[i] *= volume_;
                }
                if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            } else {
                Sleep(5);
            }
        }
    }

    IDirectSoundCapture* capture_ = nullptr;
    IDirectSoundCaptureBuffer* buffer_ = nullptr;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    DWORD buffer_bytes_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

class DSoundPlayback : public AudioPlayback {
public:
    DSoundPlayback() = default;
    ~DSoundPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        sample_rate_ = sample_rate;

        HRESULT hr = DirectSoundCreate(nullptr, &dsound_, nullptr);
        if (FAILED(hr)) return false;

        dsound_->SetCooperativeLevel(GetDesktopWindow(), DSSCL_PRIORITY);

        WAVEFORMATEX wfx = {};
        wfx.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
        wfx.nChannels = (WORD)channels;
        wfx.nSamplesPerSec = (DWORD)sample_rate;
        wfx.wBitsPerSample = 32;
        wfx.nBlockAlign = wfx.nChannels * wfx.wBitsPerSample / 8;
        wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

        buffer_bytes_ = frames_per_buffer * wfx.nBlockAlign * 4;

        DSBUFFERDESC desc = {};
        desc.dwSize = sizeof(desc);
        desc.dwFlags = DSBCAPS_GLOBALFOCUS;
        desc.dwBufferBytes = buffer_bytes_;
        desc.lpwfxFormat = &wfx;

        hr = dsound_->CreateSoundBuffer(&desc, &buffer_, nullptr);
        if (FAILED(hr)) { dsound_->Release(); dsound_ = nullptr; return false; }

        return true;
    }

    bool start() override {
        if (!buffer_ || running_) return false;
        buffer_->Play(0, 0, DSBPLAY_LOOPING);
        running_ = true;
        thread_ = std::thread(&DSoundPlayback::playback_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
        if (buffer_) buffer_->Stop();
        return true;
    }

    void close() override {
        stop();
        if (buffer_) { buffer_->Release(); buffer_ = nullptr; }
        if (dsound_) { dsound_->Release(); dsound_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }
    int write(const float*, int) override { return 0; }

private:
    void playback_thread() {
        DWORD write_pos = 0;
        int chunk_bytes = buffer_frames_ * channels_ * (int)sizeof(float);
        std::vector<float> buf(buffer_frames_ * channels_, 0.0f);

        while (running_) {
            if (callback_) callback_(buf.data(), buffer_frames_, channels_);
            if (volume_ != 1.0f) {
                for (size_t i = 0; i < buf.size(); i++)
                    buf[i] *= volume_;
            }

            void* p1 = nullptr; void* p2 = nullptr;
            DWORD s1 = 0, s2 = 0;
            HRESULT hr = buffer_->Lock(write_pos, chunk_bytes, &p1, &s1, &p2, &s2, 0);
            if (SUCCEEDED(hr)) {
                if (p1) memcpy(p1, buf.data(), s1);
                if (p2) memcpy(p2, (uint8_t*)buf.data() + s1, s2);
                buffer_->Unlock(p1, s1, p2, s2);
                write_pos = (write_pos + chunk_bytes) % buffer_bytes_;
            }
            Sleep(buffer_frames_ * 1000 / sample_rate_);
        }
    }

    IDirectSound* dsound_ = nullptr;
    IDirectSoundBuffer* buffer_ = nullptr;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    DWORD buffer_bytes_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

std::vector<AudioDevice> enumerate_audio_devices() {
    std::vector<AudioDevice> devices;
    AudioDevice dev;
    dev.id = 0;
    dev.name = "DirectSound Default";
    dev.max_input_channels = 1;
    dev.max_output_channels = 1;
    dev.default_sample_rate = 48000;
    devices.push_back(dev);
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<DSoundCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<DSoundPlayback>();
}

} // namespace iris

#endif // _WIN32 && IRIS_USE_DSOUND
