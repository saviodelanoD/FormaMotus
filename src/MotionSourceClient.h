#pragma once
// Forma Motus R5.7 non-human motion-source resolver.
// Incessus is the optional authored locomotion provider for creature actors.
#include "IncessusSDK/IncessusAPI.h"

namespace MotionSourceClient
{
    enum class Source : std::uint8_t
    {
        kNone = 0,
        kIncessus
    };

    struct ResolvedSnapshot
    {
        Incessus::API::LocomotionSnapshotV1 snapshot{};
        Source source{ Source::kNone };
    };

    bool Initialize();
    bool HasIncessus();

    bool QueryLocomotion(RE::Actor* actor, ResolvedSnapshot& out) noexcept;
    bool QueryClipMetrics(Source source, const char* profile, const char* key,
        Incessus::API::ClipMetricsV1& out) noexcept;

    const char* SourceName(Source source) noexcept;
}

