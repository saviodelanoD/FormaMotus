#pragma once
#include "MotusContinuusSDK/MotusContinuusAPI.h"

namespace MotusClient
{
    bool Initialize();
    bool IsAvailable();
    bool QueryPose(RE::Actor* actor, MotusContinuus::API::PoseKinematicsSnapshot& out) noexcept;
    bool QueryBone(RE::Actor* actor, const char* boneName, MotusContinuus::API::BoneKinematicSample& out) noexcept;
}

