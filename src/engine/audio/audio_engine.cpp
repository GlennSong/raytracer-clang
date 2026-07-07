#include "audio_engine.h"

#include "../../log.h"

#ifdef RT_ENABLE_AUDIO

// The real implementation: miniaudio's high-level engine (mixing graph, format
// conversion, spatializer) over whichever device backend the platform offers
// (CoreAudio / ALSA / PulseAudio / WASAPI), the null device, or no device at
// all (Manual mode — the caller pumps the mix; deterministic under test).
// All ma_* usage stays inside this file (ADR-0069).

#include "../../slot_map.h"

#include <miniaudio/miniaudio.h>

#include <algorithm>
#include <cstring>
#include <vector>

namespace engine {

namespace {

struct ClipData {
    std::vector<float> pcm;   // interleaved f32
    ma_uint64 frames = 0;
    ma_uint32 channels = 0;
    ma_uint32 sampleRate = 0;
};

struct VoiceData {
    ma_audio_buffer buffer{};   // references the clip's pcm (no copy)
    ma_sound sound{};
    AudioClipHandle clip;
    bool loop = false;
    bool initialized = false;

    ~VoiceData() {
        if (initialized) {
            ma_sound_uninit(&sound);
            ma_audio_buffer_uninit(&buffer);
        }
    }
};

}  // namespace

struct AudioEngine::Impl {
    ma_context context{};
    ma_engine engine{};
    ma_sound_group groups[AUDIO_BUS_COUNT] = {};
    bool haveContext = false;
    bool haveEngine = false;
    bool haveGroups = false;
    AudioBackendMode mode = AudioBackendMode::Auto;
    float master = 1.0f;
    float busVolumes[AUDIO_BUS_COUNT] = {1.0f, 1.0f, 1.0f};

    // shared_ptr elements keep ma_* objects at stable heap addresses while
    // satisfying SlotMap's copyable-value requirement.
    SlotMap<std::shared_ptr<ClipData>, AudioClipTag> clips;
    SlotMap<std::shared_ptr<VoiceData>, AudioVoiceTag> voices;

    ma_sound_group* group(AudioBus bus) {
        return haveGroups ? &groups[static_cast<int>(bus)] : nullptr;
    }

    AudioVoiceHandle start(AudioClipHandle clipHandle,
                           const AudioPlayParams& params, bool spatial,
                           const Vec3& position, Real range) {
        auto* clipPtr = clips.get(clipHandle);
        if (!clipPtr) return {};
        ClipData& clip = **clipPtr;

        auto voice = std::make_shared<VoiceData>();
        voice->clip = clipHandle;
        voice->loop = params.loop;

        ma_audio_buffer_config bufferConfig = ma_audio_buffer_config_init(
            ma_format_f32, clip.channels, clip.frames, clip.pcm.data(), nullptr);
        bufferConfig.sampleRate = clip.sampleRate;
        if (ma_audio_buffer_init(&bufferConfig, &voice->buffer) != MA_SUCCESS)
            return {};

        ma_uint32 flags = spatial ? 0 : MA_SOUND_FLAG_NO_SPATIALIZATION;
        if (ma_sound_init_from_data_source(&engine, &voice->buffer, flags,
                                           group(params.bus),
                                           &voice->sound) != MA_SUCCESS) {
            ma_audio_buffer_uninit(&voice->buffer);
            return {};
        }
        voice->initialized = true;

        ma_sound_set_volume(&voice->sound, params.volume);
        ma_sound_set_pitch(&voice->sound, std::max(params.pitch, 0.01f));
        ma_sound_set_looping(&voice->sound, params.loop ? MA_TRUE : MA_FALSE);
        if (spatial) {
            ma_sound_set_position(&voice->sound, static_cast<float>(position.x),
                                  static_cast<float>(position.y),
                                  static_cast<float>(position.z));
            // Linear falloff to silence at `range`: predictable and testable.
            ma_sound_set_attenuation_model(&voice->sound,
                                           ma_attenuation_model_linear);
            ma_sound_set_min_distance(&voice->sound, 1.0f);
            ma_sound_set_max_distance(&voice->sound,
                                      std::max(static_cast<float>(range), 1.0f));
        } else {
            ma_sound_set_pan(&voice->sound,
                             std::max(-1.0f, std::min(params.pan, 1.0f)));
        }
        if (ma_sound_start(&voice->sound) != MA_SUCCESS) return {};
        return voices.insert(voice);
    }
};

AudioEngine::AudioEngine() : impl(std::make_unique<Impl>()) {}
AudioEngine::~AudioEngine() { shutdown(); }

bool AudioEngine::initialize(AudioBackendMode mode) {
    if (impl->haveEngine) return true;
    impl->mode = mode;

    ma_engine_config config = ma_engine_config_init();
    if (mode == AudioBackendMode::Manual) {
        config.noDevice = MA_TRUE;
        config.channels = 2;
        config.sampleRate = 48000;
    } else if (mode == AudioBackendMode::Null) {
        ma_backend backends[] = {ma_backend_null};
        if (ma_context_init(backends, 1, nullptr, &impl->context) != MA_SUCCESS)
            return false;
        impl->haveContext = true;
        config.pContext = &impl->context;
    }

    ma_result result = ma_engine_init(&config, &impl->engine);
    if (result != MA_SUCCESS && mode == AudioBackendMode::Auto) {
        // No real output device (headless/CI): fall back to the null device,
        // which advances in real time but writes nowhere. Sounds still "play".
        ma_backend backends[] = {ma_backend_null};
        if (ma_context_init(backends, 1, nullptr, &impl->context) == MA_SUCCESS) {
            impl->haveContext = true;
            config = ma_engine_config_init();
            config.pContext = &impl->context;
            result = ma_engine_init(&config, &impl->engine);
            if (result == MA_SUCCESS)
                LOG_WARN("AudioEngine: no output device, using the null backend");
        }
    }
    if (result != MA_SUCCESS) {
        LOG_ERROR("AudioEngine: engine init failed (%d)", static_cast<int>(result));
        if (impl->haveContext) {
            ma_context_uninit(&impl->context);
            impl->haveContext = false;
        }
        return false;
    }
    impl->haveEngine = true;

    for (int i = 0; i < AUDIO_BUS_COUNT; i++) {
        if (ma_sound_group_init(&impl->engine, 0, nullptr, &impl->groups[i])
            != MA_SUCCESS) {
            for (int j = 0; j < i; j++) ma_sound_group_uninit(&impl->groups[j]);
            shutdown();
            return false;
        }
    }
    impl->haveGroups = true;
    return true;
}

void AudioEngine::shutdown() {
    if (!impl) return;
    impl->voices.forEach([](AudioVoiceHandle, std::shared_ptr<VoiceData>& v) {
        v.reset();   // ~VoiceData uninits the ma_sound before the engine dies
    });
    impl->voices.clear();
    impl->clips.clear();
    if (impl->haveGroups) {
        for (auto& group : impl->groups) ma_sound_group_uninit(&group);
        impl->haveGroups = false;
    }
    if (impl->haveEngine) {
        ma_engine_uninit(&impl->engine);
        impl->haveEngine = false;
    }
    if (impl->haveContext) {
        ma_context_uninit(&impl->context);
        impl->haveContext = false;
    }
}

bool AudioEngine::ready() const { return impl->haveEngine; }
AudioBackendMode AudioEngine::backendMode() const { return impl->mode; }

AudioClipHandle AudioEngine::loadClip(const std::string& path) {
    if (!impl->haveEngine) return {};
    ma_decoder_config config = ma_decoder_config_init(ma_format_f32, 0, 0);
    ma_decoder decoder;
    if (ma_decoder_init_file(path.c_str(), &config, &decoder) != MA_SUCCESS) {
        LOG_ERROR("AudioEngine: failed to open '%s'", path.c_str());
        return {};
    }
    auto clip = std::make_shared<ClipData>();
    ma_format format;
    ma_decoder_get_data_format(&decoder, &format, &clip->channels,
                               &clip->sampleRate, nullptr, 0);

    // Chunked read: robust for formats with unknown up-front length.
    constexpr ma_uint64 CHUNK_FRAMES = 4096;
    std::vector<float> chunk(CHUNK_FRAMES * clip->channels);
    for (;;) {
        ma_uint64 framesRead = 0;
        ma_result result = ma_decoder_read_pcm_frames(
            &decoder, chunk.data(), CHUNK_FRAMES, &framesRead);
        if (framesRead > 0) {
            clip->pcm.insert(clip->pcm.end(), chunk.begin(),
                             chunk.begin() + framesRead * clip->channels);
            clip->frames += framesRead;
        }
        if (result != MA_SUCCESS || framesRead < CHUNK_FRAMES) break;
    }
    ma_decoder_uninit(&decoder);

    if (clip->frames == 0) {
        LOG_ERROR("AudioEngine: '%s' decoded to zero frames", path.c_str());
        return {};
    }
    return impl->clips.insert(clip);
}

AudioClipHandle AudioEngine::createClip(const float* frames,
                                        uint64_t frameCount, uint32_t channels,
                                        uint32_t sampleRate) {
    if (!impl->haveEngine || !frames || frameCount == 0 || channels == 0 ||
        sampleRate == 0)
        return {};
    auto clip = std::make_shared<ClipData>();
    clip->pcm.assign(frames, frames + frameCount * channels);
    clip->frames = frameCount;
    clip->channels = channels;
    clip->sampleRate = sampleRate;
    return impl->clips.insert(clip);
}

void AudioEngine::destroyClip(AudioClipHandle clip) {
    if (!impl->clips.contains(clip)) return;
    // Voices reference the clip's PCM directly — stop them before freeing it.
    std::vector<AudioVoiceHandle> playing;
    impl->voices.forEach([&](AudioVoiceHandle h, std::shared_ptr<VoiceData>& v) {
        if (v->clip == clip) playing.push_back(h);
    });
    for (AudioVoiceHandle h : playing) stop(h);
    impl->clips.get(clip)->reset();   // free the PCM now (see stop())
    impl->clips.erase(clip);
}

AudioVoiceHandle AudioEngine::play(AudioClipHandle clip,
                                   const AudioPlayParams& params) {
    if (!impl->haveEngine) return {};
    return impl->start(clip, params, false, Vec3(), 0);
}

AudioVoiceHandle AudioEngine::playAt(AudioClipHandle clip, const Vec3& position,
                                     Real range, const AudioPlayParams& params) {
    if (!impl->haveEngine) return {};
    return impl->start(clip, params, true, position, range);
}

void AudioEngine::stop(AudioVoiceHandle voice) {
    auto* v = impl->voices.get(voice);
    if (!v) return;
    ma_sound_stop(&(*v)->sound);
    // SlotMap::erase leaves the freed slot's value in place until reuse, so
    // release explicitly: ~VoiceData must uninit the ma_sound NOW, not when
    // the slot is recycled.
    v->reset();
    impl->voices.erase(voice);
}

void AudioEngine::stopAll() {
    std::vector<AudioVoiceHandle> all;
    impl->voices.forEach(
        [&](AudioVoiceHandle h, std::shared_ptr<VoiceData>&) { all.push_back(h); });
    for (AudioVoiceHandle h : all) stop(h);
}

bool AudioEngine::isPlaying(AudioVoiceHandle voice) const {
    auto* v = impl->voices.get(voice);
    if (!v) return false;
    return ma_sound_is_playing(&(*v)->sound) == MA_TRUE;
}

void AudioEngine::setVoicePosition(AudioVoiceHandle voice, const Vec3& position) {
    auto* v = impl->voices.get(voice);
    if (!v) return;
    ma_sound_set_position(&(*v)->sound, static_cast<float>(position.x),
                          static_cast<float>(position.y),
                          static_cast<float>(position.z));
}

void AudioEngine::setVoiceVolume(AudioVoiceHandle voice, float volume) {
    auto* v = impl->voices.get(voice);
    if (v) ma_sound_set_volume(&(*v)->sound, volume);
}

void AudioEngine::setVoicePitch(AudioVoiceHandle voice, float pitch) {
    auto* v = impl->voices.get(voice);
    if (v) ma_sound_set_pitch(&(*v)->sound, std::max(pitch, 0.01f));
}

void AudioEngine::setListener(const Vec3& position, const Vec3& forward,
                              const Vec3& up, const Vec3& velocity) {
    if (!impl->haveEngine) return;
    ma_engine_listener_set_position(&impl->engine, 0,
                                    static_cast<float>(position.x),
                                    static_cast<float>(position.y),
                                    static_cast<float>(position.z));
    ma_engine_listener_set_direction(&impl->engine, 0,
                                     static_cast<float>(forward.x),
                                     static_cast<float>(forward.y),
                                     static_cast<float>(forward.z));
    ma_engine_listener_set_world_up(&impl->engine, 0, static_cast<float>(up.x),
                                    static_cast<float>(up.y),
                                    static_cast<float>(up.z));
    ma_engine_listener_set_velocity(&impl->engine, 0,
                                    static_cast<float>(velocity.x),
                                    static_cast<float>(velocity.y),
                                    static_cast<float>(velocity.z));
}

void AudioEngine::setMasterVolume(float volume) {
    impl->master = std::max(volume, 0.0f);
    if (impl->haveEngine) ma_engine_set_volume(&impl->engine, impl->master);
}

float AudioEngine::masterVolume() const { return impl->master; }

void AudioEngine::setBusVolume(AudioBus bus, float volume) {
    int index = static_cast<int>(bus);
    impl->busVolumes[index] = std::max(volume, 0.0f);
    if (impl->haveGroups)
        ma_sound_group_set_volume(&impl->groups[index], impl->busVolumes[index]);
}

float AudioEngine::busVolume(AudioBus bus) const {
    return impl->busVolumes[static_cast<int>(bus)];
}

void AudioEngine::update() {
    if (!impl->haveEngine) return;
    std::vector<AudioVoiceHandle> finished;
    impl->voices.forEach([&](AudioVoiceHandle h, std::shared_ptr<VoiceData>& v) {
        if (!v->loop && ma_sound_at_end(&v->sound) == MA_TRUE)
            finished.push_back(h);
    });
    for (AudioVoiceHandle h : finished) stop(h);
}

uint64_t AudioEngine::readFrames(float* out, uint64_t frameCount) {
    if (!impl->haveEngine || impl->mode != AudioBackendMode::Manual || !out)
        return 0;
    ma_uint64 framesRead = 0;
    ma_engine_read_pcm_frames(&impl->engine, out, frameCount, &framesRead);
    return framesRead;
}

uint32_t AudioEngine::sampleRate() const {
    if (!impl->haveEngine) return 0;
    return ma_engine_get_sample_rate(&impl->engine);
}

std::size_t AudioEngine::clipCount() const { return impl->clips.size(); }
std::size_t AudioEngine::activeVoiceCount() const { return impl->voices.size(); }

}  // namespace engine

#else  // !RT_ENABLE_AUDIO — inert stub, same shape (like the ImGui hooks)

namespace engine {

struct AudioEngine::Impl {};

AudioEngine::AudioEngine() = default;
AudioEngine::~AudioEngine() = default;

bool AudioEngine::initialize(AudioBackendMode) {
    LOG_INFO("AudioEngine: built without RT_ENABLE_AUDIO; audio disabled");
    return false;
}
void AudioEngine::shutdown() {}
bool AudioEngine::ready() const { return false; }
AudioBackendMode AudioEngine::backendMode() const {
    return AudioBackendMode::Auto;
}
AudioClipHandle AudioEngine::loadClip(const std::string&) { return {}; }
AudioClipHandle AudioEngine::createClip(const float*, uint64_t, uint32_t,
                                        uint32_t) {
    return {};
}
void AudioEngine::destroyClip(AudioClipHandle) {}
AudioVoiceHandle AudioEngine::play(AudioClipHandle, const AudioPlayParams&) {
    return {};
}
AudioVoiceHandle AudioEngine::playAt(AudioClipHandle, const Vec3&, Real,
                                     const AudioPlayParams&) {
    return {};
}
void AudioEngine::stop(AudioVoiceHandle) {}
void AudioEngine::stopAll() {}
bool AudioEngine::isPlaying(AudioVoiceHandle) const { return false; }
void AudioEngine::setVoicePosition(AudioVoiceHandle, const Vec3&) {}
void AudioEngine::setVoiceVolume(AudioVoiceHandle, float) {}
void AudioEngine::setVoicePitch(AudioVoiceHandle, float) {}
void AudioEngine::setListener(const Vec3&, const Vec3&, const Vec3&,
                              const Vec3&) {}
void AudioEngine::setMasterVolume(float) {}
float AudioEngine::masterVolume() const { return 1.0f; }
void AudioEngine::setBusVolume(AudioBus, float) {}
float AudioEngine::busVolume(AudioBus) const { return 1.0f; }
void AudioEngine::update() {}
uint64_t AudioEngine::readFrames(float*, uint64_t) { return 0; }
uint32_t AudioEngine::sampleRate() const { return 0; }
std::size_t AudioEngine::clipCount() const { return 0; }
std::size_t AudioEngine::activeVoiceCount() const { return 0; }

}  // namespace engine

#endif  // RT_ENABLE_AUDIO
