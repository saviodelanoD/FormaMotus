#include "PCH.h"
#include "MotionSourceClient.h"
#include "ActorDomain.h"
#include "IncessusClient.h"

#include <mutex>
#include <unordered_map>

namespace MotionSourceClient
{
    namespace
    {
        std::mutex g_sourceTelemetryMutex;
        std::unordered_map<RE::FormID, Source> g_lastSource;

        void LogResolvedSource(RE::Actor* actor, Source source, const Incessus::API::LocomotionSnapshotV1& snapshot)
        {
            if (!actor || source == Source::kNone) return;
            bool changed = false;
            {
                std::scoped_lock lock(g_sourceTelemetryMutex);
                auto [it, inserted] = g_lastSource.emplace(actor->GetFormID(), source);
                if (!inserted && it->second != source) { it->second = source; changed = true; }
                else changed = inserted;
                if (g_lastSource.size() > 512) g_lastSource.clear();
            }
            if (changed) {
                SKSE::log::info(
                    "[FormaMotus][MOTION-SOURCE] actor={:08X} source={} profile=\"{}\" serial={} flags=0x{:08X} gait={} carrierMask=0x{:02X}",
                    actor->GetFormID(), SourceName(source), snapshot.profile, snapshot.snapshotSerial, snapshot.flags,
                    static_cast<unsigned>(snapshot.desiredGait), static_cast<unsigned>(snapshot.runtimeCarrierMask));
            }
        }
    }

    bool Initialize()
    {
        const bool incessus = IncessusClient::Initialize();
        SKSE::log::info(
            "[FormaMotus][MotionSource] R5.8 nonHumanOnly=true creatureProvider[Incessus={}]",
            incessus ? "READY" : "MISSING_SOFT");
        return incessus;
    }

    bool HasIncessus()
    {
        return IncessusClient::IsAvailable();
    }

    bool QueryLocomotion(RE::Actor* actor, ResolvedSnapshot& out) noexcept
    {
        out = {};
        if (!ActorDomain::IsFormaNonHuman(actor)) return false;

        Incessus::API::LocomotionSnapshotV1 snapshot{};
        if (IncessusClient::QueryLocomotion(actor, snapshot) &&
            (snapshot.flags & Incessus::API::kSupportedActor) != 0) {
            out.snapshot = snapshot;
            out.source = Source::kIncessus;
            LogResolvedSource(actor, out.source, out.snapshot);
            return true;
        }

        return false;
    }

    bool QueryClipMetrics(Source source, const char* profile, const char* key,
        Incessus::API::ClipMetricsV1& out) noexcept
    {
        switch (source) {
        case Source::kIncessus:
            return IncessusClient::QueryClipMetrics(profile, key, out);
        default:
            out = {};
            return false;
        }
    }

    const char* SourceName(Source source) noexcept
    {
        switch (source) {
        case Source::kIncessus: return "INCESSUS";
        default: return "NONE";
        }
    }
}

