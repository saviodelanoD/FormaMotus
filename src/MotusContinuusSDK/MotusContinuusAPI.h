#pragma once

#include <RE/Skyrim.h>

#include <cstdint>

#if defined(MOTUS_CONTINUUS_PROVIDER)
#  define MC_PROVIDER_API __declspec(dllexport)
#else
#  define MC_PROVIDER_API
#endif

namespace MotusContinuus::API
{
    inline constexpr std::uint32_t kABIVersion = 1;

    enum class PoseSampleBoundary : std::uint32_t
    {
        Unknown = 0,
        PreGraphPresented = 1
    };

    enum class BoneResolveMethod : std::uint32_t
    {
        None = 0,
        Exact = 1,
        ExactInsensitive = 2,
        CanonicalAlias = 3
    };

    enum PoseSnapshotFlags : std::uint32_t
    {
        kPoseCurrentValid = 1u << 0,
        kPosePreviousValid = 1u << 1
    };

    enum BoneSampleFlags : std::uint32_t
    {
        kBoneCurrentValid = 1u << 0,
        kBonePreviousValid = 1u << 1,
        kBoneVelocityValid = 1u << 2
    };

    struct Vec3
    {
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    struct Quat
    {
        float w{ 1.0f };
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
    };

    // Translation values are Skyrim world/model units. Quaternions are WXYZ.
    struct Transform
    {
        Quat rotation{};
        Vec3 translation{};
        float scale{ 1.0f };
    };

    struct PoseKinematicsSnapshot
    {
        std::uint32_t flags{ 0 };
        std::uint32_t boneCount{ 0 };
        std::uint64_t sampleSerial{ 0 };
        float deltaTime{ 0.0f };
        PoseSampleBoundary boundary{ PoseSampleBoundary::Unknown };
    };

    struct BoneKinematicSample
    {
        std::uint32_t flags{ 0 };
        std::uint32_t reserved{ 0 };
        std::uint64_t sampleSerial{ 0 };
        float deltaTime{ 0.0f };
        PoseSampleBoundary boundary{ PoseSampleBoundary::Unknown };
        BoneResolveMethod resolveMethod{ BoneResolveMethod::None };

        // The actual runtime node selected after exact/case-insensitive/canonical
        // resolution. Fixed storage keeps the ABI independent of std::string.
        char runtimeName[128]{};

        Transform currentLocal{};
        Transform currentComponent{};
        Transform currentWorld{};

        Transform previousLocal{};
        Transform previousComponent{};
        Transform previousWorld{};

        // Linear velocities are units/second. Angular velocities are radians/second.
        // "component" is root-relative model/component space; "world" includes
        // actor/root motion.
        Vec3 localLinearVelocity{};
        Vec3 componentLinearVelocity{};
        Vec3 worldLinearVelocity{};

        Vec3 localAngularVelocity{};
        Vec3 componentAngularVelocity{};
        Vec3 worldAngularVelocity{};
    };

    using GetAPIVersionFn = std::uint32_t (*)() noexcept;
    using QueryPoseKinematicsFn = bool (*)(RE::Actor*, PoseKinematicsSnapshot*) noexcept;
    using QueryBoneKinematicsFn = bool (*)(RE::Actor*, const char*, BoneKinematicSample*) noexcept;
}

extern "C"
{
    MC_PROVIDER_API std::uint32_t MC_GetAPIVersion() noexcept;
    MC_PROVIDER_API bool MC_QueryPoseKinematics(
        RE::Actor* actor,
        MotusContinuus::API::PoseKinematicsSnapshot* out) noexcept;
    MC_PROVIDER_API bool MC_QueryBoneKinematics(
        RE::Actor* actor,
        const char* boneName,
        MotusContinuus::API::BoneKinematicSample* out) noexcept;
}

#undef MC_PROVIDER_API

