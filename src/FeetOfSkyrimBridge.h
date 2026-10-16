#pragma once

#include <array>
#include <cstdint>

namespace RE { class Actor; class BShkbAnimationGraph; }

namespace FeetOfSkyrimBridge
{
    inline constexpr int kMaxLegs = 8;
    inline constexpr int kBoneCap = 512;
    inline constexpr int kBoneNameCap = 64;

    struct LegSlot
    {
        std::int16_t hipIndex{ -1 };
        std::int16_t kneeIndex{ -1 };
        std::int16_t ankleIndex{ -1 };
        float kneeAxis[3]{ 0.0f, 0.0f, 0.0f };
        float plantedHeight{ 0.0f };
        float raisedHeight{ 0.0f };
        float minAnkleHeight{ 0.0f };
        float maxAnkleHeight{ 0.0f };
        char hipBone[kBoneNameCap]{};
        char kneeBone[kBoneNameCap]{};
        char ankleBone[kBoneNameCap]{};
    };

    struct PoseContext
    {
        std::uint64_t data{ 0 };
        std::uint64_t charData{ 0 };
        std::uint64_t skeleton{ 0 };
        std::uint64_t driverInfo{ 0 };
        float* pose{ nullptr };
        std::uint16_t poseOffset{ 0 };
        std::int16_t poseCapacity{ 0 };
        std::int16_t poseCount{ 0 };
        int boneCount{ 0 };
        int legCount{ 0 };
        bool lockFeetWhenPlanted{ false };
        float modelForward[3]{ 0.0f, 1.0f, 0.0f };
        float modelRight[3]{ 1.0f, 0.0f, 0.0f };
        std::array<std::int16_t, kBoneCap> parents{};
        std::array<LegSlot, kMaxLegs> legs{};
    };

    bool Initialize();
    bool IsAvailable();
    bool InstallPreHooks();
    void Reset();
    void ObserveGroundedActor(RE::Actor* actor);

    bool AcquirePoseContext(RE::BShkbAnimationGraph* graph, void* output, PoseContext& out, bool requireFootIK = true) noexcept;
    int FindBoneIndexByNameContains(const PoseContext& context, const char* token) noexcept;
    void MarkBoneValid(const PoseContext& context, int boneIndex) noexcept;
}

