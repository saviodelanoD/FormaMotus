#pragma once

#include <UniversalKinematicsAPI.h>

namespace UKClient
{
    bool Initialize();
    bool IsAvailable();
    const UniversalKinematics::SkeletonProfile* GetProfileForActor(RE::Actor* actor) noexcept;
}

