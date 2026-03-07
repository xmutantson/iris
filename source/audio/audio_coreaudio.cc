/*
 * CoreAudio backend for Iris (macOS).
 * Uses AudioUnit for low-latency capture and playback.
 */
#include "audio/audio.h"

#if defined(__APPLE__) && !defined(IRIS_USE_PULSE)

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <thread>
#include <atomic>
#include <cstring>
#include <vector>
#include <mutex>

namespace iris {

class CoreAudioCapture : public AudioCapture {
public:
    CoreAudioCapture() = default;
    ~CoreAudioCapture() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        AudioComponentDescription desc = {};
        desc.componentType = kAudioUnitType_Output;
        desc.componentSubType = kAudioUnitSubType_HALOutput;
        desc.componentManufacturer = kAudioUnitManufacturer_Apple;

        AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
        if (!comp) return false;
        if (AudioComponentInstanceNew(comp, &au_) != noErr) return false;

        // Enable input
        UInt32 enable = 1;
        AudioUnitSetProperty(au_, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Input, 1, &enable, sizeof(enable));
        // Disable output
        UInt32 disable = 0;
        AudioUnitSetProperty(au_, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Output, 0, &disable, sizeof(disable));

        AudioStreamBasicDescription fmt = {};
        fmt.mSampleRate = (Float64)sample_rate;
        fmt.mFormatID = kAudioFormatLinearPCM;
        fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        fmt.mBytesPerPacket = (UInt32)(channels * sizeof(float));
        fmt.mFramesPerPacket = 1;
        fmt.mBytesPerFrame = fmt.mBytesPerPacket;
        fmt.mChannelsPerFrame = (UInt32)channels;
        fmt.mBitsPerChannel = 32;

        AudioUnitSetProperty(au_, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output, 1, &fmt, sizeof(fmt));

        AURenderCallbackStruct cb;
        cb.inputProc = input_callback;
        cb.inputProcRefCon = this;
        AudioUnitSetProperty(au_, kAudioOutputUnitProperty_SetInputCallback,
                             kAudioUnitScope_Global, 0, &cb, sizeof(cb));

        if (AudioUnitInitialize(au_) != noErr) {
            AudioComponentInstanceDispose(au_);
            au_ = nullptr;
            return false;
        }
        return true;
    }

    bool start() override {
        if (!au_ || running_) return false;
        running_ = true;
        return AudioOutputUnitStart(au_) == noErr;
    }

    bool stop() override {
        running_ = false;
        if (au_) AudioOutputUnitStop(au_);
        return true;
    }

    void close() override {
        stop();
        if (au_) { AudioComponentInstanceDispose(au_); au_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int read(float*, int) override { return 0; } // callback-driven only

private:
    static OSStatus input_callback(void* ref, AudioUnitRenderActionFlags* flags,
                                    const AudioTimeStamp* ts, UInt32 bus,
                                    UInt32 frames, AudioBufferList*) {
        auto* self = (CoreAudioCapture*)ref;
        if (!self->running_) return noErr;

        AudioBufferList abl;
        abl.mNumberBuffers = 1;
        abl.mBuffers[0].mNumberChannels = (UInt32)self->channels_;
        std::vector<float> buf(frames * self->channels_);
        abl.mBuffers[0].mDataByteSize = (UInt32)(buf.size() * sizeof(float));
        abl.mBuffers[0].mData = buf.data();

        AudioUnitRender(self->au_, flags, ts, bus, frames, &abl);

        if (self->volume_ != 1.0f) {
            for (size_t i = 0; i < buf.size(); i++)
                buf[i] *= self->volume_;
        }
        if (self->callback_)
            self->callback_(buf.data(), (int)frames, self->channels_);
        return noErr;
    }

    AudioUnit au_ = nullptr;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
};

class CoreAudioPlayback : public AudioPlayback {
public:
    CoreAudioPlayback() = default;
    ~CoreAudioPlayback() override { close(); }

    bool open(int device_id, int sample_rate, int channels,
              int frames_per_buffer) override {
        (void)device_id;
        channels_ = channels;
        buffer_frames_ = frames_per_buffer;

        AudioComponentDescription desc = {};
        desc.componentType = kAudioUnitType_Output;
        desc.componentSubType = kAudioUnitSubType_DefaultOutput;
        desc.componentManufacturer = kAudioUnitManufacturer_Apple;

        AudioComponent comp = AudioComponentFindNext(nullptr, &desc);
        if (!comp) return false;
        if (AudioComponentInstanceNew(comp, &au_) != noErr) return false;

        AudioStreamBasicDescription fmt = {};
        fmt.mSampleRate = (Float64)sample_rate;
        fmt.mFormatID = kAudioFormatLinearPCM;
        fmt.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked;
        fmt.mBytesPerPacket = (UInt32)(channels * sizeof(float));
        fmt.mFramesPerPacket = 1;
        fmt.mBytesPerFrame = fmt.mBytesPerPacket;
        fmt.mChannelsPerFrame = (UInt32)channels;
        fmt.mBitsPerChannel = 32;

        AudioUnitSetProperty(au_, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input, 0, &fmt, sizeof(fmt));

        AURenderCallbackStruct cb;
        cb.inputProc = render_callback;
        cb.inputProcRefCon = this;
        AudioUnitSetProperty(au_, kAudioUnitProperty_SetRenderCallback,
                             kAudioUnitScope_Input, 0, &cb, sizeof(cb));

        if (AudioUnitInitialize(au_) != noErr) {
            AudioComponentInstanceDispose(au_);
            au_ = nullptr;
            return false;
        }
        return true;
    }

    bool start() override {
        if (!au_ || running_) return false;
        running_ = true;
        return AudioOutputUnitStart(au_) == noErr;
    }

    bool stop() override {
        running_ = false;
        if (au_) AudioOutputUnitStop(au_);
        return true;
    }

    void close() override {
        stop();
        if (au_) { AudioComponentInstanceDispose(au_); au_ = nullptr; }
    }

    bool is_running() const override { return running_; }
    float volume() const override { return volume_; }
    void set_volume(float v) override { volume_ = v; }
    void set_callback(AudioCallback cb) override { callback_ = cb; }

    int write(const float*, int) override { return 0; } // callback-driven only

private:
    static OSStatus render_callback(void* ref, AudioUnitRenderActionFlags*,
                                     const AudioTimeStamp*, UInt32,
                                     UInt32 frames, AudioBufferList* abl) {
        auto* self = (CoreAudioPlayback*)ref;
        float* out = (float*)abl->mBuffers[0].mData;
        int n = (int)frames;

        if (self->callback_)
            self->callback_(out, n, self->channels_);
        else
            memset(out, 0, frames * self->channels_ * sizeof(float));

        if (self->volume_ != 1.0f) {
            for (int i = 0; i < n * self->channels_; i++)
                out[i] *= self->volume_;
        }
        return noErr;
    }

    AudioUnit au_ = nullptr;
    int channels_ = 1;
    int buffer_frames_ = 1024;
    std::atomic<bool> running_{false};
    std::atomic<float> volume_{1.0f};
    AudioCallback callback_;
};

std::vector<AudioDevice> enumerate_audio_devices() {
    std::vector<AudioDevice> devices;

    AudioObjectPropertyAddress prop;
    prop.mSelector = kAudioHardwarePropertyDevices;
    prop.mScope = kAudioObjectPropertyScopeGlobal;
    prop.mElement = kAudioObjectPropertyElementMain;

    UInt32 size = 0;
    AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &prop, 0, nullptr, &size);
    int count = (int)(size / sizeof(AudioDeviceID));
    std::vector<AudioDeviceID> ids(count);
    AudioObjectGetPropertyData(kAudioObjectSystemObject, &prop, 0, nullptr, &size, ids.data());

    for (int i = 0; i < count; i++) {
        AudioDevice dev;
        dev.id = i;

        // Get name
        CFStringRef name = nullptr;
        prop.mSelector = kAudioDevicePropertyDeviceNameCFString;
        size = sizeof(name);
        AudioObjectGetPropertyData(ids[i], &prop, 0, nullptr, &size, &name);
        if (name) {
            char buf[256];
            CFStringGetCString(name, buf, sizeof(buf), kCFStringEncodingUTF8);
            dev.name = buf;
            CFRelease(name);
        }

        // Input channels
        prop.mSelector = kAudioDevicePropertyStreamConfiguration;
        prop.mScope = kAudioDevicePropertyScopeInput;
        size = 0;
        AudioObjectGetPropertyDataSize(ids[i], &prop, 0, nullptr, &size);
        dev.max_input_channels = size > sizeof(AudioBufferList) ? 1 : 0;

        // Output channels
        prop.mScope = kAudioDevicePropertyScopeOutput;
        size = 0;
        AudioObjectGetPropertyDataSize(ids[i], &prop, 0, nullptr, &size);
        dev.max_output_channels = size > sizeof(AudioBufferList) ? 1 : 0;

        dev.default_sample_rate = 48000;
        devices.push_back(dev);
    }
    return devices;
}

std::unique_ptr<AudioCapture> create_capture() {
    return std::make_unique<CoreAudioCapture>();
}

std::unique_ptr<AudioPlayback> create_playback() {
    return std::make_unique<CoreAudioPlayback>();
}

} // namespace iris

#endif // __APPLE__
