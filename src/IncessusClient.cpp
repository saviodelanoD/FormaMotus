#include "PCH.h"
#include "IncessusClient.h"

#include <Windows.h>
#include <atomic>
#include <mutex>

namespace IncessusClient
{
    namespace
    {
        std::once_flag g_resolveOnce;
        std::atomic_bool g_available{ false };
        const Incessus::API::InterfaceV1* g_api = nullptr;

        void Resolve()
        {
            HMODULE module = ::GetModuleHandleW(L"Incessus.dll");
            if (!module) {
                SKSE::log::warn("[FormaMotus][Incessus] Incessus.dll not loaded; creature locomotion-source coverage unavailable.");
                return;
            }

            const auto request = reinterpret_cast<Incessus::API::RequestInterfaceFn>(
                ::GetProcAddress(module, "ALR_RequestInterface"));
            if (!request) {
                SKSE::log::warn("[FormaMotus][Incessus] legacy SDK export ALR_RequestInterface not found in Incessus.dll; creature locomotion-source coverage unavailable.");
                return;
            }

            g_api = request(Incessus::API::kInterfaceVersion1);
            if (!Incessus::SDK::Compatible(g_api)) {
                g_api = nullptr;
                SKSE::log::warn("[FormaMotus][Incessus] SDK V1 unavailable or incompatible; creature locomotion-source coverage unavailable.");
                return;
            }

            g_available.store(true, std::memory_order_release);
            SKSE::log::info("[FormaMotus][Incessus] SDK U9 connected snapshotBytes={} build=\"{}\"", sizeof(Incessus::API::LocomotionSnapshotV1), g_api->buildId ? g_api->buildId : "<unknown>");
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

    bool QueryLocomotion(RE::Actor* actor, Incessus::API::LocomotionSnapshotV1& out) noexcept
    {
        out = {};
        if (!actor || !actor->As<RE::Actor>() || !Initialize()) return false;
        return Incessus::SDK::Query(g_api, actor->GetFormID(), out);
    }

    bool QueryClipMetrics(const char* profile, const char* key, Incessus::API::ClipMetricsV1& out) noexcept
    {
        out = {};
        if (!profile || !*profile || !key || !*key || !Initialize() || !g_api || !g_api->QueryClipMetrics) return false;
        out.structSize = sizeof(out);
        out.interfaceVersion = Incessus::API::kInterfaceVersion1;
        return g_api->QueryClipMetrics(profile, key, &out) != 0 && out.valid != 0;
    }
}

