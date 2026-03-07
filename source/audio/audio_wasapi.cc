#include "audio/audio.h"

#ifdef _WIN32

#include <initguid.h>
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <thread>
#include <atomic>
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
class WasapiCapture : public AudioCapture {
public:
    WasapiCapture() = default;
    ~WasapiCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;  // TODO: device selection
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        return true;
    }

    bool start() override {
        if (running_) return false;
        running_ = true;
        thread_ = std::thread(&WasapiCapture::capture_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
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
    void capture_thread() {
        ComInit com;

        IMMDeviceEnumerator* enumerator = nullptr;
        IMMDevice* device = nullptr;
        IAudioClient* client = nullptr;
        IAudioCaptureClient* capture = nullptr;

        HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                                      CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                      (void**)&enumerator);
        if (FAILED(hr)) { running_ = false; return; }

        hr = enumerator->GetDefaultAudioEndpoint(eCapture, eConsole, &device);
        if (FAILED(hr)) { enumerator->Release(); running_ = false; return; }

        hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
                              (void**)&client);
        if (FAILED(hr)) { device->Release(); enumerator->Release(); running_ = false; return; }

        WAVEFORMATEX fmt = {};
        fmt.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
        fmt.nChannels = (WORD)channels_;
        fmt.nSamplesPerSec = (DWORD)sample_rate_;
        fmt.wBitsPerSample = 32;
        fmt.nBlockAlign = fmt.nChannels * fmt.wBitsPerSample / 8;
        fmt.nAvgBytesPerSec = fmt.nSamplesPerSec * fmt.nBlockAlign;

        REFERENCE_TIME duration = (REFERENCE_TIME)(10000000.0 * buffer_frames_ / sample_rate_);
        hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, duration, 0, &fmt, nullptr);
        if (FAILED(hr)) {
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        hr = client->GetService(__uuidof(IAudioCaptureClient), (void**)&capture);
        if (FAILED(hr)) {
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
                    float* fdata = (float*)data;

                    // Apply volume
                    if (volume_ != 1.0f) {
                        // Work on a copy
                        std::vector<float> buf(fdata, fdata + frames_available * channels_);
                        for (auto& s : buf) s *= volume_;
                        if (callback_) callback_(buf.data(), frames_available, channels_);
                    } else {
                        if (callback_) callback_(fdata, frames_available, channels_);
                    }

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

    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
};

// WASAPI Playback implementation
class WasapiPlayback : public AudioPlayback {
public:
    WasapiPlayback() = default;
    ~WasapiPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        sample_rate_ = sample_rate;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;
        return true;
    }

    bool start() override {
        if (running_) return false;
        running_ = true;
        thread_ = std::thread(&WasapiPlayback::playback_thread, this);
        return true;
    }

    bool stop() override {
        running_ = false;
        if (thread_.joinable()) thread_.join();
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

private:
    void playback_thread() {
        ComInit com;

        IMMDeviceEnumerator* enumerator = nullptr;
        IMMDevice* device = nullptr;
        IAudioClient* client = nullptr;
        IAudioRenderClient* render = nullptr;

        HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                                      CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                                      (void**)&enumerator);
        if (FAILED(hr)) { running_ = false; return; }

        hr = enumerator->GetDefaultAudioEndpoint(eRender, eConsole, &device);
        if (FAILED(hr)) { enumerator->Release(); running_ = false; return; }

        hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr,
                              (void**)&client);
        if (FAILED(hr)) { device->Release(); enumerator->Release(); running_ = false; return; }

        WAVEFORMATEX fmt = {};
        fmt.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
        fmt.nChannels = (WORD)channels_;
        fmt.nSamplesPerSec = (DWORD)sample_rate_;
        fmt.wBitsPerSample = 32;
        fmt.nBlockAlign = fmt.nChannels * fmt.wBitsPerSample / 8;
        fmt.nAvgBytesPerSec = fmt.nSamplesPerSec * fmt.nBlockAlign;

        UINT32 buffer_size = 0;
        REFERENCE_TIME duration = (REFERENCE_TIME)(10000000.0 * buffer_frames_ / sample_rate_);
        hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, duration, 0, &fmt, nullptr);
        if (FAILED(hr)) {
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        client->GetBufferSize(&buffer_size);

        hr = client->GetService(__uuidof(IAudioRenderClient), (void**)&render);
        if (FAILED(hr)) {
            client->Release(); device->Release(); enumerator->Release();
            running_ = false; return;
        }

        client->Start();

        while (running_) {
            Sleep(5);

            UINT32 padding = 0;
            client->GetCurrentPadding(&padding);
            UINT32 available = buffer_size - padding;

            if (available > 0) {
                BYTE* data = nullptr;
                hr = render->GetBuffer(available, &data);
                if (SUCCEEDED(hr)) {
                    float* fdata = (float*)data;

                    if (callback_) {
                        callback_(fdata, available, channels_);
                        // Apply volume
                        if (volume_ != 1.0f) {
                            for (UINT32 i = 0; i < available * (UINT32)channels_; i++)
                                fdata[i] *= volume_;
                        }
                    } else {
                        std::memset(data, 0, available * channels_ * sizeof(float));
                    }

                    render->ReleaseBuffer(available, 0);
                }
            }
        }

        client->Stop();
        render->Release();
        client->Release();
        device->Release();
        enumerator->Release();
    }

    int sample_rate_ = 48000;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
    std::thread thread_;
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
// Stub for non-Windows — ALSA implementation would go here
namespace iris {

std::vector<AudioDevice> enumerate_audio_devices() { return {}; }
std::unique_ptr<AudioCapture> create_capture() { return nullptr; }
std::unique_ptr<AudioPlayback> create_playback() { return nullptr; }

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
