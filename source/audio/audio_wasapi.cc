#include "audio/audio.h"

#ifdef _WIN32

#include <initguid.h>
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <thread>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <cmath>
#include <cstring>

#pragma comment(lib, "ole32.lib")

namespace iris {

// COM initialization helper
struct ComInit {
    ComInit()  { CoInitializeEx(nullptr, COINIT_MULTITHREADED); }
    ~ComInit() { CoUninitialize(); }
};

// WASAPI Capture implementation
// Audio capture and decode are on SEPARATE threads to prevent decode latency
// from causing WASAPI buffer overflows and sample drops.
class WasapiCapture : public AudioCapture {
public:
    WasapiCapture() = default;
    ~WasapiCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        device_id_ = device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        return true;
    }

    bool start() override {
        if (running_) return false;
        running_ = true;
        capture_thread_ = std::thread(&WasapiCapture::capture_thread, this);
        delivery_thread_ = std::thread(&WasapiCapture::delivery_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        fifo_cv_.notify_all();
        if (capture_thread_.joinable()) capture_thread_.join();
        if (delivery_thread_.joinable()) delivery_thread_.join();
        return true;
    }

    void close() override { stop(); }
    bool is_running() const override { return running_; }

    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }

    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float* buffer, int frame_count) override {
        (void)buffer; (void)frame_count;
        return 0;  // Use callback mode
    }

private:
    // Push audio into FIFO (called from capture thread, must be fast)
    void fifo_push(const float* data, size_t count, int channels) {
        std::lock_guard<std::mutex> lock(fifo_mutex_);
        // Limit FIFO to ~10 seconds of audio to prevent unbounded growth
        size_t max_fifo = (size_t)sample_rate_ * channels * 10;
        if (fifo_.size() + count > max_fifo) {
            printf("[Audio] Capture: FIFO overflow, dropping %zu samples\n", count);
            return;
        }
        fifo_.insert(fifo_.end(), data, data + count);
        fifo_channels_ = channels;
        fifo_cv_.notify_one();
    }

    // Delivery thread: pulls from FIFO, calls callback (may block for decode)
    void delivery_thread() {
        std::vector<float> chunk;
        while (running_) {
            {
                std::unique_lock<std::mutex> lock(fifo_mutex_);
                fifo_cv_.wait(lock, [this] { return !fifo_.empty() || !running_; });
                if (!running_ && fifo_.empty()) break;
                chunk.swap(fifo_);
                fifo_.clear();
            }
            if (chunk.empty()) continue;

            int channels = fifo_channels_;
            int frame_count = (int)chunk.size() / std::max(channels, 1);

            // Apply volume
            if (volume_ != 1.0f) {
                for (auto& s : chunk) s *= volume_;
            }

            if (callback_)
                callback_(chunk.data(), frame_count, channels);

            chunk.clear();
        }
    }

    void capture_thread() {
        ComInit com;

        IMMDeviceEnumerator* enumerator = nullptr;
        IMMDevice* device = nullptr;
        IAudioClient* client = nullptr;
        IAudioCaptureClient* capture = nullptr;

        HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                                      CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                      (void**)&enumerator);
        if (FAILED(hr)) { printf("[Audio] Capture: CoCreateInstance failed 0x%08lx\n", hr); running_ = false; return; }

        if (device_id_ >= 0) {
            IMMDeviceCollection* col = nullptr;
            hr = enumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &col);
            if (SUCCEEDED(hr)) {
                hr = col->Item((UINT)device_id_, &device);
                col->Release();
            }
        } else {
            hr = enumerator->GetDefaultAudioEndpoint(eCapture, eConsole, &device);
        }
        if (FAILED(hr)) { printf("[Audio] Capture: device %d not found 0x%08lx\n", device_id_, hr); enumerator->Release(); running_ = false; return; }

        // Log device friendly name
        {
            IPropertyStore* props = nullptr;
            if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
                PROPVARIANT name;
                PropVariantInit(&name);
                if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &name)) && name.vt == VT_LPWSTR) {
                    char nbuf[256];
                    WideCharToMultiByte(CP_UTF8, 0, name.pwszVal, -1, nbuf, sizeof(nbuf), nullptr, nullptr);
                    printf("[Audio] Capture device name: %s\n", nbuf);
                }
                PropVariantClear(&name);
                props->Release();
            }
        }

        hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
                              (void**)&client);
        if (FAILED(hr)) { printf("[Audio] Capture: Activate failed 0x%08lx\n", hr); device->Release(); enumerator->Release(); running_ = false; return; }

        // Use device's mix format (WASAPI shared mode requires it)
        WAVEFORMATEX* mix_fmt = nullptr;
        hr = client->GetMixFormat(&mix_fmt);
        if (FAILED(hr) || !mix_fmt) {
            printf("[Audio] Capture: GetMixFormat failed 0x%08lx\n", hr);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        int actual_channels = mix_fmt->nChannels;
        int actual_rate = mix_fmt->nSamplesPerSec;
        printf("[Audio] Capture dev %d: %d ch, %d Hz, %d bit\n",
               device_id_, actual_channels, actual_rate, mix_fmt->wBitsPerSample);

        REFERENCE_TIME duration = (REFERENCE_TIME)(10000000.0 * buffer_frames_ / actual_rate);
        hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, duration, 0, mix_fmt, nullptr);
        if (FAILED(hr)) {
            printf("[Audio] Capture: Initialize failed 0x%08lx\n", hr);
            CoTaskMemFree(mix_fmt);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }
        CoTaskMemFree(mix_fmt);

        UINT32 actual_buffer_frames = 0;
        client->GetBufferSize(&actual_buffer_frames);
        printf("[Audio] Capture: requested %d frames (%dms), got %u frames (%ums)\n",
               buffer_frames_, buffer_frames_ * 1000 / actual_rate,
               actual_buffer_frames, actual_buffer_frames * 1000 / actual_rate);

        hr = client->GetService(__uuidof(IAudioCaptureClient), (void**)&capture);
        if (FAILED(hr)) {
            printf("[Audio] Capture: GetService failed 0x%08lx\n", hr);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        client->Start();

        while (running_) {
            Sleep(5);  // ~5ms polling

            UINT32 packet_length = 0;
            capture->GetNextPacketSize(&packet_length);

            while (packet_length > 0 && running_) {
                BYTE* data = nullptr;
                UINT32 frames_available = 0;
                DWORD flags = 0;

                hr = capture->GetBuffer(&data, &frames_available, &flags, nullptr, nullptr);
                if (SUCCEEDED(hr) && frames_available > 0) {
                    if (flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY) {
                        discont_count_++;
                        if (discont_count_ <= 10)
                            printf("[Audio] Capture: DATA_DISCONTINUITY #%d (%u frames)\n",
                                   discont_count_, frames_available);
                    }

                    float* fdata = (float*)data;
                    fifo_push(fdata, frames_available * actual_channels, actual_channels);

                    capture->ReleaseBuffer(frames_available);
                }

                capture->GetNextPacketSize(&packet_length);
            }
        }

        client->Stop();
        capture->Release();
        client->Release();
        device->Release();
        enumerator->Release();
    }

    int device_id_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    int discont_count_ = 0;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread capture_thread_;
    std::thread delivery_thread_;

    // FIFO between capture and delivery threads
    std::mutex fifo_mutex_;
    std::condition_variable fifo_cv_;
    std::vector<float> fifo_;
    int fifo_channels_ = 1;
};

// WASAPI Playback implementation
// Two-thread architecture: producer thread runs process_tx into a FIFO,
// render thread copies from FIFO into WASAPI buffer. This prevents
// crackling caused by process_tx latency starving the render buffer.
class WasapiPlayback : public AudioPlayback {
public:
    WasapiPlayback() = default;
    ~WasapiPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        device_id_ = device_id >= 1000 ? device_id - 1000 : device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        return true;
    }

    bool start() override {
        if (running_) return false;
        running_ = true;
        render_thread_ = std::thread(&WasapiPlayback::render_thread_func, this);
        producer_thread_ = std::thread(&WasapiPlayback::producer_thread_func, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        fifo_cv_.notify_all();
        producer_cv_.notify_all();
        if (producer_thread_.joinable()) producer_thread_.join();
        if (render_thread_.joinable()) render_thread_.join();
        return true;
    }

    void close() override { stop(); }
    bool is_running() const override { return running_; }

    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }

    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float* buffer, int frame_count) override {
        (void)buffer; (void)frame_count;
        return 0;  // Use callback mode
    }

    void mark_drain() override {
        drain_mark_.store(frames_produced_.load());
    }

    bool is_drained() const override {
        int64_t mark = drain_mark_.load();
        return mark < 0 || frames_rendered_.load() >= mark;
    }

private:
    // Producer thread: generates TX samples via callback into FIFO
    void producer_thread_func() {
        // Produce in fixed chunks to keep latency predictable
        const int chunk_frames = 512;
        int channels = 0;

        // Wait until render thread sets actual_channels_
        while (running_ && actual_channels_.load() == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        channels = actual_channels_.load();
        if (!channels) return;

        std::vector<float> buf(chunk_frames * channels, 0.0f);

        while (running_) {
            // Wait if FIFO is already well-stocked (>100ms worth)
            {
                std::unique_lock<std::mutex> lock(fifo_mutex_);
                size_t high_water = (size_t)actual_rate_.load() * channels * 100 / 1000;
                if (fifo_.size() >= high_water) {
                    producer_cv_.wait_for(lock, std::chrono::milliseconds(5),
                        [&] { return fifo_.size() < high_water || !running_; });
                    if (!running_) break;
                }
            }

            // Generate samples
            std::fill(buf.begin(), buf.end(), 0.0f);
            if (callback_) {
                callback_(buf.data(), chunk_frames, channels);
            }

            // Apply volume
            if (volume_ != 1.0f) {
                float v = volume_.load();
                for (auto& s : buf) s *= v;
            }

            // Push to FIFO
            {
                std::lock_guard<std::mutex> lock(fifo_mutex_);
                fifo_.insert(fifo_.end(), buf.begin(), buf.end());
            }
            frames_produced_ += chunk_frames;
            fifo_cv_.notify_one();
        }
    }

    // Render thread: drains FIFO into WASAPI buffer
    void render_thread_func() {
        ComInit com;

        IMMDeviceEnumerator* enumerator = nullptr;
        IMMDevice* device = nullptr;
        IAudioClient* client = nullptr;
        IAudioRenderClient* render = nullptr;

        HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                                      CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                      (void**)&enumerator);
        if (FAILED(hr)) { printf("[Audio] Playback: CoCreateInstance failed 0x%08lx\n", hr); running_ = false; return; }

        if (device_id_ >= 0) {
            IMMDeviceCollection* col = nullptr;
            hr = enumerator->EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE, &col);
            if (SUCCEEDED(hr)) {
                hr = col->Item((UINT)device_id_, &device);
                col->Release();
            }
        } else {
            hr = enumerator->GetDefaultAudioEndpoint(eRender, eConsole, &device);
        }
        if (FAILED(hr)) { printf("[Audio] Playback: device %d not found 0x%08lx\n", device_id_, hr); enumerator->Release(); running_ = false; return; }

        // Log device friendly name
        {
            IPropertyStore* props = nullptr;
            if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
                PROPVARIANT name;
                PropVariantInit(&name);
                if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &name)) && name.vt == VT_LPWSTR) {
                    char nbuf[256];
                    WideCharToMultiByte(CP_UTF8, 0, name.pwszVal, -1, nbuf, sizeof(nbuf), nullptr, nullptr);
                    printf("[Audio] Playback device name: %s\n", nbuf);
                }
                PropVariantClear(&name);
                props->Release();
            }
        }

        hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
                              (void**)&client);
        if (FAILED(hr)) { printf("[Audio] Playback: Activate failed 0x%08lx\n", hr); device->Release(); enumerator->Release(); running_ = false; return; }

        WAVEFORMATEX* mix_fmt = nullptr;
        hr = client->GetMixFormat(&mix_fmt);
        if (FAILED(hr) || !mix_fmt) {
            printf("[Audio] Playback: GetMixFormat failed 0x%08lx\n", hr);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        int channels = mix_fmt->nChannels;
        int rate = mix_fmt->nSamplesPerSec;
        printf("[Audio] Playback dev %d: %d ch, %d Hz, %d bit\n",
               device_id_, channels, rate, mix_fmt->wBitsPerSample);

        // Request 50ms buffer for comfortable margin
        REFERENCE_TIME duration = 500000;  // 50ms in 100ns units
        hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, duration, 0, mix_fmt, nullptr);
        if (FAILED(hr)) {
            printf("[Audio] Playback: Initialize failed 0x%08lx\n", hr);
            CoTaskMemFree(mix_fmt);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }
        CoTaskMemFree(mix_fmt);

        UINT32 buffer_size = 0;
        client->GetBufferSize(&buffer_size);
        printf("[Audio] Playback: buffer %u frames (%.1f ms)\n",
               buffer_size, 1000.0 * buffer_size / rate);

        hr = client->GetService(__uuidof(IAudioRenderClient), (void**)&render);
        if (FAILED(hr)) {
            printf("[Audio] Playback: GetService failed 0x%08lx\n", hr);
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        // Signal producer thread with actual device parameters
        actual_channels_.store(channels);
        actual_rate_.store(rate);

        client->Start();
        int underrun_count = 0;

        while (running_) {
            Sleep(2);  // ~2ms polling — finer than 5ms, still low CPU

            UINT32 padding = 0;
            client->GetCurrentPadding(&padding);
            UINT32 available = buffer_size - padding;

            if (available == 0) continue;

            BYTE* data = nullptr;
            hr = render->GetBuffer(available, &data);
            if (FAILED(hr)) continue;

            float* fdata = (float*)data;
            size_t samples_needed = (size_t)available * channels;

            // Drain from FIFO
            size_t copied = 0;
            {
                std::lock_guard<std::mutex> lock(fifo_mutex_);
                copied = std::min(fifo_.size(), samples_needed);
                if (copied > 0) {
                    std::memcpy(fdata, fifo_.data(), copied * sizeof(float));
                    fifo_.erase(fifo_.begin(), fifo_.begin() + copied);
                }
            }
            if (copied > 0)
                frames_rendered_ += (int64_t)(copied / channels);
            producer_cv_.notify_one();

            // Zero-fill any remainder (underrun)
            if (copied < samples_needed) {
                std::memset(fdata + copied, 0, (samples_needed - copied) * sizeof(float));
                if (copied > 0 && underrun_count++ < 3) {
                    printf("[Audio] Playback: underrun, had %zu/%zu samples\n",
                           copied, samples_needed);
                }
            }

            render->ReleaseBuffer(available, 0);
        }

        client->Stop();
        render->Release();
        client->Release();
        device->Release();
        enumerator->Release();
    }

    int device_id_ = -1;
    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    std::atomic<int> actual_channels_{0};
    std::atomic<int> actual_rate_{48000};
    AudioCallback callback_;
    std::thread render_thread_;
    std::thread producer_thread_;
    mutable std::mutex fifo_mutex_;
    std::condition_variable fifo_cv_;
    std::condition_variable producer_cv_;
    std::vector<float> fifo_;
    std::atomic<int64_t> frames_produced_{0};
    std::atomic<int64_t> frames_rendered_{0};
    std::atomic<int64_t> drain_mark_{-1};  // -1 = no drain pending
};

// Device enumeration
std::vector<AudioDevice> enumerate_audio_devices() {
    std::vector<AudioDevice> devices;
    ComInit com;

    IMMDeviceEnumerator* enumerator = nullptr;
    HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                                  CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                  (void**)&enumerator);
    if (FAILED(hr)) return devices;

    // Enumerate capture devices
    IMMDeviceCollection* collection = nullptr;
    hr = enumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &collection);
    if (SUCCEEDED(hr)) {
        UINT count = 0;
        collection->GetCount(&count);
        for (UINT i = 0; i < count; i++) {
            IMMDevice* dev = nullptr;
            collection->Item(i, &dev);
            if (dev) {
                IPropertyStore* props = nullptr;
                dev->OpenPropertyStore(STGM_READ, &props);
                if (props) {
                    PROPVARIANT name;
                    PropVariantInit(&name);
                    props->GetValue(PKEY_Device_FriendlyName, &name);
                    if (name.vt == VT_LPWSTR) {
                        char buf[256];
                        WideCharToMultiByte(CP_UTF8, 0, name.pwszVal, -1,
                                           buf, sizeof(buf), nullptr, nullptr);
                        devices.push_back({(int)i, buf, 1, 0, 48000});
                    }
                    PropVariantClear(&name);
                    props->Release();
                }
                dev->Release();
            }
        }
        collection->Release();
    }

    // Enumerate render devices
    hr = enumerator->EnumAudioEndpoints(eRender, DEVICE_STATE_ACTIVE, &collection);
    if (SUCCEEDED(hr)) {
        UINT count = 0;
        collection->GetCount(&count);
        for (UINT i = 0; i < count; i++) {
            IMMDevice* dev = nullptr;
            collection->Item(i, &dev);
            if (dev) {
                IPropertyStore* props = nullptr;
                dev->OpenPropertyStore(STGM_READ, &props);
                if (props) {
                    PROPVARIANT name;
                    PropVariantInit(&name);
                    props->GetValue(PKEY_Device_FriendlyName, &name);
                    if (name.vt == VT_LPWSTR) {
                        char buf[256];
                        WideCharToMultiByte(CP_UTF8, 0, name.pwszVal, -1,
                                           buf, sizeof(buf), nullptr, nullptr);
                        devices.push_back({1000 + (int)i, buf, 0, 1, 48000});
                    }
                    PropVariantClear(&name);
                    props->Release();
                }
                dev->Release();
            }
        }
        collection->Release();
    }

    enumerator->Release();
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<WasapiCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<WasapiPlayback>();
}

float measure_rms(const float* samples, int count) {
    if (count <= 0) return 0.0f;
    float sum = 0;
    for (int i = 0; i < count; i++)
        sum += samples[i] * samples[i];
    return std::sqrt(sum / count);
}

float measure_peak(const float* samples, int count) {
    float peak = 0;
    for (int i = 0; i < count; i++) {
        float a = std::abs(samples[i]);
        if (a > peak) peak = a;
    }
    return peak;
}

} // namespace iris

#else
// Non-Windows: ALSA backend in audio_alsa.cc provides factory functions on Linux.
#ifndef __linux__
// Fallback stubs for platforms with no audio backend
namespace iris {
std::vector<AudioDevice> enumerate_audio_devices() { return {}; }
std::unique_ptr<AudioCapture> create_capture() { return nullptr; }
std::unique_ptr<AudioPlayback> create_playback() { return nullptr; }
} // namespace iris
#endif

namespace iris {

float measure_rms(const float* samples, int count) {
    if (count <= 0) return 0.0f;
    float sum = 0;
    for (int i = 0; i < count; i++) sum += samples[i] * samples[i];
    return std::sqrt(sum / count);
}

float measure_peak(const float* samples, int count) {
    float peak = 0;
    for (int i = 0; i < count; i++) {
        float a = std::abs(samples[i]);
        if (a > peak) peak = a;
    }
    return peak;
}

} // namespace iris

#endif
