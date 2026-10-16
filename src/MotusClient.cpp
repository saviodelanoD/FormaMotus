#include "PCH.h"
#include "MotusClient.h"

#include <Windows.h>
#include <atomic>
#include <mutex>

namespace MotusClient
{
    namespace
    {
        std::once_flag g_resolveOnce;
        std::atomic_bool g_available{ false };
        MotusContinuus::API::GetAPIVersionFn g_getVersion = nullptr;
        MotusContinuus::API::QueryPoseKinematicsFn g_queryPose = nullptr;
        MotusContinuus::API::QueryBoneKinematicsFn g_queryBone = nullptr;

        void Resolve()
        {
            HMODULE module = ::GetModuleHandleW(L"MotusContinuus.dll");
            if (!module) {
                SKSE::log::warn("[FormaMotus][Motus] MotusContinuus.dll not loaded; R4 Stride/FootLock contract cannot be satisfied.");
                return;
            }

            g_getVersion = reinterpret_cast<MotusContinuus::API::GetAPIVersionFn>(::GetProcAddress(module, "MC_GetAPIVersion"));
            g_queryPose = reinterpret_cast<MotusContinuus::API::QueryPoseKinematicsFn>(::GetProcAddress(module, "MC_QueryPoseKinematics"));
            g_queryBone = reinterpret_cast<MotusContinuus::API::QueryBoneKinematicsFn>(::GetProcAddress(module, "MC_QueryBoneKinematics"));
            if (!g_getVersion || !g_queryPose || !g_queryBone) {
                g_getVersion = nullptr;
                g_queryPose = nullptr;
                g_queryBone = nullptr;
                SKSE::log::warn("[FormaMotus][Motus] Pose Kinematics SDK exports missing; R4 Stride/FootLock contract cannot be satisfied.");
                return;
            }

            const std::uint32_t abi = g_getVersion();
            if (abi != MotusContinuus::API::kABIVersion) {
                SKSE::log::warn("[FormaMotus][Motus] ABI mismatch required={} provider={}; R4 Stride/FootLock contract cannot be satisfied.", MotusContinuus::API::kABIVersion, abi);
                g_getVersion = nullptr;
                g_queryPose = nullptr;
                g_queryBone = nullptr;
                return;
            }

            g_available.store(true, std::memory_order_release);
            SKSE::log::info("[FormaMotus][Motus] Pose Kinematics SDK connected ABI={}", abi);
        }
    }

    bool Initialize()
    {
        std::call_once(g_resolveOnce, Resolve);
        return g_available.load(std::memory_order_acquire);
    }

    bool IsAvailable()
    {
        return Initialize();
    }

    bool QueryPose(RE::Actor* actor, MotusContinuus::API::PoseKinematicsSnapshot& out) noexcept
    {
        out = {};
        return actor && Initialize() && g_queryPose && g_queryPose(actor, &out);
    }

    bool QueryBone(RE::Actor* actor, const char* boneName, MotusContinuus::API::BoneKinematicSample& out) noexcept
    {
        out = {};
        return actor && boneName && boneName[0] && Initialize() && g_queryBone && g_queryBone(actor, boneName, &out);
    }
}

