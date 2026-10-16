#include "PCH.h"
#include "UKClient.h"

#include <Windows.h>

#include <atomic>
#include <mutex>

namespace UKClient
{
    namespace
    {
        std::once_flag g_resolveOnce;
        std::atomic_bool g_available{ false };
        UniversalKinematics::API::GetAPIVersionFn g_getAPIVersion = nullptr;
        UniversalKinematics::API::GetProfileForActorFn g_getProfileForActor = nullptr;

        void Resolve()
        {
            HMODULE module = ::GetModuleHandleW(L"UniversalKinematics.dll");
            if (!module) {
                SKSE::log::critical(
                    "[UKClient] UniversalKinematics.dll is not loaded; procedural animation hooks will remain disabled.");
                return;
            }

            g_getAPIVersion = reinterpret_cast<UniversalKinematics::API::GetAPIVersionFn>(
                ::GetProcAddress(module, "UK_GetAPIVersion"));
            g_getProfileForActor = reinterpret_cast<UniversalKinematics::API::GetProfileForActorFn>(
                ::GetProcAddress(module, "UK_GetProfileForActor"));

            if (!g_getAPIVersion || !g_getProfileForActor) {
                SKSE::log::critical(
                    "[UKClient] UniversalKinematics.dll does not expose the required ABI entry points.");
                g_getAPIVersion = nullptr;
                g_getProfileForActor = nullptr;
                return;
            }

            const std::uint32_t abi = g_getAPIVersion();
            if (abi != UniversalKinematics::API::kABIVersion) {
                SKSE::log::critical(
                    "[UKClient] ABI mismatch. required={} provider={}",
                    UniversalKinematics::API::kABIVersion,
                    abi);
                g_getAPIVersion = nullptr;
                g_getProfileForActor = nullptr;
                return;
            }

            g_available.store(true, std::memory_order_release);
            SKSE::log::info(
                "[UKClient] Connected to UniversalKinematics.dll ABI={}",
                abi);
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

    const UniversalKinematics::SkeletonProfile* GetProfileForActor(RE::Actor* actor) noexcept
    {
        if (!actor || !Initialize() || !g_getProfileForActor) {
            return nullptr;
        }
        return g_getProfileForActor(actor);
    }
}

