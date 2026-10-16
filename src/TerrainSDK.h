#pragma once

#include <cstdint>

namespace UKPhysicsAPI
{
    inline constexpr std::uint32_t kTerrainInterfaceVersion = 1;
    inline constexpr const char* kTerrainRequestExport = "UKP_RequestTerrainInterface";

    enum class TerrainSource : std::uint32_t
    {
        None = 0,
        NativeSupportNormal = 1,
        MultiProbePlane = 2
    };

    enum TerrainFlags : std::uint32_t
    {
        kTerrainValid = 1u << 0,
        kTerrainGrounded = 1u << 1,
        kTerrainStaticSurface = 1u << 2,
        kTerrainPlaneCoherent = 1u << 3,
        kTerrainBiped = 1u << 4,
        kTerrainQuadruped = 1u << 5,
        kTerrainNativeSupport = 1u << 6,
        kTerrainMultiProbe = 1u << 7
    };

    struct TerrainSnapshotV1
    {
        std::uint32_t size{ sizeof(TerrainSnapshotV1) };
        std::uint32_t version{ kTerrainInterfaceVersion };
        std::uint32_t flags{ 0 };
        std::uint32_t actorFormID{ 0 };

        std::uint32_t source{ static_cast<std::uint32_t>(TerrainSource::None) };
        std::uint32_t sampleCount{ 0 };
        std::uint32_t supportOwnerFormID{ 0 };
        std::uint32_t reserved0{ 0 };

        std::uint64_t serial{ 0 };

        float normalX{ 0.0f };
        float normalY{ 0.0f };
        float normalZ{ 1.0f };

        float pitchRad{ 0.0f };
        float rollRad{ 0.0f };
        float slopeAngleRad{ 0.0f };

        float confidence{ 0.0f };
        float planeRms{ 0.0f };
        float planeMaxError{ 0.0f };
        float sampleAgeSeconds{ 0.0f };
    };

    using QueryTerrainFn = bool (*)(std::uint32_t actorFormID, TerrainSnapshotV1* outSnapshot);
    using GetBuildStringFn = const char* (*)();

    struct TerrainInterfaceV1
    {
        std::uint32_t version{ kTerrainInterfaceVersion };
        std::uint32_t size{ sizeof(TerrainInterfaceV1) };
        QueryTerrainFn QueryTerrain{ nullptr };
        GetBuildStringFn GetBuildString{ nullptr };
    };

    static_assert(sizeof(TerrainSnapshotV1) == 80);
    static_assert(sizeof(TerrainInterfaceV1) == 24);
}

