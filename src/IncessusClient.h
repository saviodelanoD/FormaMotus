#pragma once
#include "IncessusSDK/IncessusAPI.h"

namespace IncessusClient
{
    bool Initialize();
    bool IsAvailable();
    bool QueryLocomotion(RE::Actor* actor, Incessus::API::LocomotionSnapshotV1& out) noexcept;
    bool QueryClipMetrics(const char* profile, const char* key, Incessus::API::ClipMetricsV1& out) noexcept;
}

