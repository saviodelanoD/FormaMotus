#include "PCH.h"
#include "SlopeWarpModule.h"
#include "ActorDomain.h"

#include "FeetOfSkyrimBridge.h"
#include "TerrainSDK.h"
#include "UKClient.h"

#include <UniversalKinematics.h>

#include "RE/B/BShkbAnimationGraph.h"

#include <Windows.h>
#include <TlHelp32.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cctype>
#include <initializer_list>
#include <cstdint>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string_view>
#include <unordered_map>

namespace SlopeWarpModule
{
    namespace
    {
        using Clock = std::chrono::steady_clock;

        constexpr float kDeg2Rad = 0.01745329251994329577f;
        constexpr float kRad2Deg = 57.295779513082320876f;
        constexpr float kTerrainMaxAge = 0.25f;
        constexpr float kBipedPitchCap = 18.0f * kDeg2Rad;
        constexpr float kBipedRollCap = 14.0f * kDeg2Rad;
        constexpr float kQuadrupedPitchCap = 22.0f * kDeg2Rad;
        constexpr float kQuadrupedRollCap = 18.0f * kDeg2Rad;
        constexpr float kDeadzone = 1.0f * kDeg2Rad;
        constexpr float kBipedGain = 0.85f;
        constexpr float kQuadrupedGain = 0.95f;
        constexpr float kBipedResponse = 8.0f;
        constexpr float kQuadrupedResponse = 6.5f;
        constexpr float kReturnResponse = 10.0f;
        constexpr float kMaxAcceptedSlope = 42.0f * kDeg2Rad;
        constexpr float kMinPlaneConfidence = 0.45f;
        constexpr float kMaxPlaneRms = 6.0f;
        constexpr float kMaxPlaneError = 12.0f;
        constexpr float kTelemetryPeriod = 1.0f;

        struct Quat
        {
            float x{ 0.0f };
            float y{ 0.0f };
            float z{ 0.0f };
            float w{ 1.0f };
        };

        struct ActorState
        {
            std::mutex mutex;
            float pitch{ 0.0f };
            float roll{ 0.0f };
            Clock::time_point lastUpdate{};
            Clock::time_point lastLog{};
            std::uint64_t lastTerrainSerial{ 0 };
            bool wasActive{ false };
        };

        std::unordered_map<RE::FormID, std::shared_ptr<ActorState>> g_states;
        std::shared_mutex g_statesMutex;

        std::mutex g_providerMutex;
        std::atomic<const UKPhysicsAPI::TerrainInterfaceV1*> g_terrainInterface{ nullptr };
        std::atomic_bool g_providerAttemptLogged{ false };
        Clock::time_point g_lastProviderAttempt{};

        thread_local std::array<Quat, FeetOfSkyrimBridge::kBoneCap> t_modelRot{};

        using RequestTerrainInterfaceFn = const void* (*)(std::uint32_t);

        float Clamp01(float v)
        {
            return std::clamp(v, 0.0f, 1.0f);
        }

        float ExpAlpha(float response, float dt)
        {
            if (!std::isfinite(response) || response <= 0.0f || !std::isfinite(dt) || dt <= 0.0f) {
                return 1.0f;
            }
            return 1.0f - std::exp(-response * dt);
        }

        Quat NormalizeQ(Quat q)
        {
            const float len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(len2) || len2 <= 1.0e-12f) return {};
            const float inv = 1.0f / std::sqrt(len2);
            q.x *= inv; q.y *= inv; q.z *= inv; q.w *= inv;
            return q;
        }

        Quat MulQ(const Quat& a, const Quat& b)
        {
            return NormalizeQ({
                a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
                a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
                a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w,
                a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z
            });
        }

        Quat ConjQ(const Quat& q)
        {
            return { -q.x, -q.y, -q.z, q.w };
        }

        Quat AxisAngle(const float* axis, float angle)
        {
            const float len2 = axis[0]*axis[0] + axis[1]*axis[1] + axis[2]*axis[2];
            if (!std::isfinite(len2) || len2 <= 1.0e-8f || !std::isfinite(angle)) return {};
            const float invLen = 1.0f / std::sqrt(len2);
            const float half = angle * 0.5f;
            const float s = std::sin(half);
            return NormalizeQ({ axis[0]*invLen*s, axis[1]*invLen*s, axis[2]*invLen*s, std::cos(half) });
        }

        bool RebuildModelRotations(const FeetOfSkyrimBridge::PoseContext& c)
        {
            if (!c.pose || c.poseCount <= 0 || c.poseCount > FeetOfSkyrimBridge::kBoneCap) return false;
            const int count = std::min<int>(c.poseCount, c.boneCount);
            if (count <= 0) return false;

            for (int i = 0; i < count; ++i) {
                const float* q = c.pose + i * 12 + 4;
                const Quat local = NormalizeQ({ q[0], q[1], q[2], q[3] });
                const int parent = c.parents[i];
                if (parent >= i || parent >= count) return false;
                t_modelRot[i] = parent >= 0 ? MulQ(t_modelRot[parent], local) : local;
            }
            return true;
        }

        bool WriteModelDelta(
            FeetOfSkyrimBridge::PoseContext& c,
            int bone,
            float pitch,
            float roll)
        {
            const int count = std::min<int>(c.poseCount, c.boneCount);
            if (bone < 0 || bone >= count) return false;
            if (!RebuildModelRotations(c)) return false;

            const Quat pitchQ = AxisAngle(c.modelRight, pitch);
            const Quat rollQ = AxisAngle(c.modelForward, roll);
            const Quat deltaModel = MulQ(rollQ, pitchQ);
            const Quat wantedModel = MulQ(deltaModel, t_modelRot[bone]);

            const int parent = c.parents[bone];
            Quat local = wantedModel;
            if (parent >= 0) {
                if (parent >= bone || parent >= count) return false;
                local = MulQ(ConjQ(t_modelRot[parent]), wantedModel);
            }

            float* q = c.pose + bone * 12 + 4;
            q[0] = local.x; q[1] = local.y; q[2] = local.z; q[3] = local.w;
            FeetOfSkyrimBridge::MarkBoneValid(c, bone);
            return true;
        }

        bool EqualsCI(std::string_view a, std::string_view b)
        {
            if (a.size() != b.size()) return false;
            for (std::size_t i = 0; i < a.size(); ++i) {
                const unsigned char ca = static_cast<unsigned char>(a[i]);
                const unsigned char cb = static_cast<unsigned char>(b[i]);
                if (std::tolower(ca) != std::tolower(cb)) return false;
            }
            return true;
        }

        bool ContainsCI(std::string_view haystack, std::string_view needle)
        {
            if (needle.empty() || haystack.size() < needle.size()) return false;
            for (std::size_t i = 0; i + needle.size() <= haystack.size(); ++i) {
                bool same = true;
                for (std::size_t j = 0; j < needle.size(); ++j) {
                    const unsigned char a = static_cast<unsigned char>(haystack[i + j]);
                    const unsigned char b = static_cast<unsigned char>(needle[j]);
                    if (std::tolower(a) != std::tolower(b)) {
                        same = false;
                        break;
                    }
                }
                if (same) return true;
            }
            return false;
        }

        std::size_t AnimationBoneCount(const RE::BShkbAnimationGraph* graph)
        {
            if (!graph) return 0;
            const std::size_t nodeCount = graph->boneNodes.size();
            return graph->numAnimBones > 0 ? std::min<std::size_t>(nodeCount, graph->numAnimBones) : nodeCount;
        }

        int FindBoneIndexExact(RE::BShkbAnimationGraph* graph, std::string_view wanted)
        {
            if (!graph || wanted.empty()) return -1;
            const std::size_t count = AnimationBoneCount(graph);
            for (std::size_t i = 0; i < count; ++i) {
                auto* node = static_cast<RE::NiAVObject*>(graph->boneNodes[static_cast<RE::BSTArray<RE::BShkbAnimationGraph::BoneNodeEntry, RE::BSTArrayHeapAllocator>::size_type>(i)].node);
                if (!node) continue;
                const char* raw = node->name.c_str();
                if (raw && EqualsCI(raw, wanted)) return static_cast<int>(i);
            }
            return -1;
        }

        int FindBoneIndexHint(RE::BShkbAnimationGraph* graph, std::initializer_list<std::string_view> hints)
        {
            if (!graph) return -1;
            const std::size_t count = AnimationBoneCount(graph);
            for (std::size_t i = 0; i < count; ++i) {
                auto* node = static_cast<RE::NiAVObject*>(graph->boneNodes[static_cast<RE::BSTArray<RE::BShkbAnimationGraph::BoneNodeEntry, RE::BSTArrayHeapAllocator>::size_type>(i)].node);
                if (!node) continue;
                const char* raw = node->name.c_str();
                if (!raw) continue;
                const std::string_view name(raw);
                for (const auto hint : hints) {
                    if (ContainsCI(name, hint)) return static_cast<int>(i);
                }
            }
            return -1;
        }

        struct Anchors
        {
            int pelvis{ -1 };
            int chest{ -1 };
        };

        Anchors ResolveAnchors(
            RE::BShkbAnimationGraph* graph,
            const UniversalKinematics::SkeletonProfile& profile)
        {
            Anchors out{};
            out.pelvis = FindBoneIndexExact(graph, profile.pelvisNode);
            out.chest = FindBoneIndexExact(graph, profile.chestNode);

            if (out.pelvis < 0) {
                out.pelvis = FindBoneIndexHint(graph, { "pelvis", "hips", "hip" });
            }
            if (out.chest < 0) {
                out.chest = FindBoneIndexHint(graph, { "ribcage", "chest", "spine2", "spine3", "hub01" });
            }

            // Profile spine names are the safest fallback after explicit anchors.
            if (out.pelvis < 0) {
                for (const auto& name : profile.spineNodes) {
                    if (ContainsCI(name, "root") || ContainsCI(name, "skull") ||
                        ContainsCI(name, "scull") || ContainsCI(name, "head") || ContainsCI(name, "neck")) {
                        continue;
                    }
                    const int index = FindBoneIndexExact(graph, name);
                    if (index >= 0) {
                        out.pelvis = index;
                        break;
                    }
                }
            }
            if (out.chest < 0) {
                for (auto it = profile.spineNodes.rbegin(); it != profile.spineNodes.rend(); ++it) {
                    if (ContainsCI(*it, "root") || ContainsCI(*it, "skull") ||
                        ContainsCI(*it, "scull") || ContainsCI(*it, "head") || ContainsCI(*it, "neck")) {
                        continue;
                    }
                    const int index = FindBoneIndexExact(graph, *it);
                    if (index >= 0) {
                        out.chest = index;
                        break;
                    }
                }
            }

            if (out.chest == out.pelvis) out.chest = -1;
            return out;
        }

        std::shared_ptr<ActorState> GetState(RE::FormID id)
        {
            {
                std::shared_lock lock(g_statesMutex);
                const auto it = g_states.find(id);
                if (it != g_states.end()) return it->second;
            }
            auto state = std::make_shared<ActorState>();
            std::unique_lock lock(g_statesMutex);
            auto [it, inserted] = g_states.emplace(id, state);
            return inserted ? state : it->second;
        }

        bool IsSpecialInteraction(RE::Actor* actor)
        {
            if (!actor) return true;
            auto* actorState = actor->AsActorState();
            if (actorState && actorState->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) return true;
            if (actor->GetOccupiedFurniture().get() != nullptr) return true;
            bool synced = false;
            if (actor->GetGraphVariableBool("bIsSyncedAnim", synced) && synced) return true;
            if (actor->IsInKillMove() || actor->IsDead() || actor->IsInRagdollState()) return true;
            if (actorState && (actorState->IsBleedingOut() || actorState->IsSwimming())) return true;
            return actor->IsInMidair();
        }

        float ApplyDeadzoneAndCap(float value, float cap)
        {
            if (!std::isfinite(value)) return 0.0f;
            const float sign = value < 0.0f ? -1.0f : 1.0f;
            const float magnitude = std::abs(value);
            if (magnitude <= kDeadzone) return 0.0f;
            return sign * std::min(magnitude - kDeadzone, cap);
        }

        const char* SourceName(std::uint32_t source)
        {
            switch (static_cast<UKPhysicsAPI::TerrainSource>(source)) {
            case UKPhysicsAPI::TerrainSource::NativeSupportNormal: return "NATIVE_SUPPORT";
            case UKPhysicsAPI::TerrainSource::MultiProbePlane: return "MULTIPROBE_PLANE";
            default: return "NONE";
            }
        }

        bool ValidateTerrain(
            const UKPhysicsAPI::TerrainSnapshotV1& sample,
            bool quadruped,
            float& qualityGain)
        {
            qualityGain = 0.0f;
            const std::uint32_t required = UKPhysicsAPI::kTerrainValid | UKPhysicsAPI::kTerrainGrounded;
            if ((sample.flags & required) != required) return false;
            if (sample.actorFormID == 0 || sample.sampleAgeSeconds < 0.0f || sample.sampleAgeSeconds > kTerrainMaxAge) return false;
            if (!std::isfinite(sample.pitchRad) || !std::isfinite(sample.rollRad) ||
                !std::isfinite(sample.slopeAngleRad) || !std::isfinite(sample.confidence)) return false;
            if (sample.slopeAngleRad < 0.0f || sample.slopeAngleRad > kMaxAcceptedSlope) return false;

            if (quadruped) {
                if ((sample.flags & UKPhysicsAPI::kTerrainMultiProbe) == 0 ||
                    (sample.flags & UKPhysicsAPI::kTerrainPlaneCoherent) == 0 || sample.sampleCount < 4) {
                    return false;
                }
                if (!std::isfinite(sample.planeRms) || !std::isfinite(sample.planeMaxError) ||
                    sample.planeRms > kMaxPlaneRms || sample.planeMaxError > kMaxPlaneError ||
                    sample.confidence < kMinPlaneConfidence) {
                    return false;
                }
                qualityGain = Clamp01((sample.confidence - kMinPlaneConfidence) / (1.0f - kMinPlaneConfidence));
                qualityGain = 0.55f + 0.45f * qualityGain;
            } else {
                if ((sample.flags & UKPhysicsAPI::kTerrainNativeSupport) == 0) return false;
                qualityGain = Clamp01(sample.confidence);
            }
            return qualityGain > 0.0f;
        }

        const void* SafeRequestTerrainInterface(RequestTerrainInterfaceFn request, std::uint32_t version) noexcept
        {
            if (!request) return nullptr;
            __try {
                return request(version);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
        }

        const char* SafeGetBuildString(const UKPhysicsAPI::TerrainInterfaceV1* iface) noexcept
        {
            if (!iface || !iface->GetBuildString) return nullptr;
            __try {
                return iface->GetBuildString();
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return nullptr;
            }
        }

        bool QueryTerrain(RE::Actor* actor, UKPhysicsAPI::TerrainSnapshotV1& out)
        {
            const auto* iface = g_terrainInterface.load(std::memory_order_acquire);
            if (!actor || !iface || !iface->QueryTerrain) return false;

            out = {};
            out.size = sizeof(UKPhysicsAPI::TerrainSnapshotV1);
            out.version = UKPhysicsAPI::kTerrainInterfaceVersion;
            bool ok = false;
            __try {
                ok = iface->QueryTerrain(actor->GetFormID(), std::addressof(out));
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                ok = false;
            }
            return ok && out.version == UKPhysicsAPI::kTerrainInterfaceVersion &&
                out.size >= sizeof(UKPhysicsAPI::TerrainSnapshotV1) && out.actorFormID == actor->GetFormID();
        }

        bool ResolveProvider()
        {
            if (g_terrainInterface.load(std::memory_order_acquire)) return true;
            std::scoped_lock lock(g_providerMutex);
            if (g_terrainInterface.load(std::memory_order_relaxed)) return true;
            const auto now = Clock::now();
            if (g_lastProviderAttempt != Clock::time_point{} &&
                std::chrono::duration<float>(now - g_lastProviderAttempt).count() < 5.0f) {
                return false;
            }
            g_lastProviderAttempt = now;

            const HANDLE snap = ::CreateToolhelp32Snapshot(TH32CS_SNAPMODULE | TH32CS_SNAPMODULE32, ::GetCurrentProcessId());
            if (snap == INVALID_HANDLE_VALUE) return false;

            MODULEENTRY32W entry{};
            entry.dwSize = sizeof(entry);
            bool resolved = false;
            if (::Module32FirstW(snap, &entry)) {
                do {
                    const auto proc = ::GetProcAddress(entry.hModule, UKPhysicsAPI::kTerrainRequestExport);
                    if (!proc) continue;
                    const auto request = reinterpret_cast<RequestTerrainInterfaceFn>(proc);
                    const void* raw = SafeRequestTerrainInterface(request, UKPhysicsAPI::kTerrainInterfaceVersion);
                    if (!raw) continue;

                    const auto* iface = static_cast<const UKPhysicsAPI::TerrainInterfaceV1*>(raw);
                    if (iface->version != UKPhysicsAPI::kTerrainInterfaceVersion ||
                        iface->size < sizeof(UKPhysicsAPI::TerrainInterfaceV1) || !iface->QueryTerrain) {
                        continue;
                    }

                    g_terrainInterface.store(iface, std::memory_order_release);
                    const char* build = "UNKNOWN";
                    if (iface->GetBuildString) {
                        if (const char* value = SafeGetBuildString(iface); value && *value) {
                            build = value;
                        } else {
                            build = "FAULTED_BUILD_STRING";
                        }
                    }
                    SKSE::log::info(
                        "[FormaMotus][SLOPE][SDK] connected ABI={} providerBase=0x{:X} build=\"{}\"",
                        iface->version, reinterpret_cast<std::uintptr_t>(entry.hModule), build);
                    resolved = true;
                    break;
                } while (::Module32NextW(snap, &entry));
            }
            ::CloseHandle(snap);

            if (!resolved && !g_providerAttemptLogged.exchange(true)) {
                SKSE::log::warn(
                    "[FormaMotus][SLOPE][SDK] UKP_RequestTerrainInterface V1 not found; Slope Warp is disabled but the rest of Forma Motus remains active.");
            }
            return resolved;
        }
    }

    bool Initialize()
    {
        return ResolveProvider();
    }

    bool IsAvailable()
    {
        return g_terrainInterface.load(std::memory_order_acquire) != nullptr || ResolveProvider();
    }

    void ApplyPreFootIkPose(
        RE::Actor* actor,
        RE::BShkbAnimationGraph* graph,
        FeetOfSkyrimBridge::PoseContext& c)
    {
        static std::once_flag buildOnce;
        std::call_once(buildOnce, []() {
            SKSE::log::info(
                "[FormaMotus][SLOPE] BUILD R5.7-NONHUMAN-ONLY terrainSDK=V1 additivePose=true IK=false footLock=false order=AFTER_STRIDE_BEFORE_FOS");
        });

        if (!actor || !graph || !c.pose || c.poseCount <= 0 || !IsAvailable()) return;
        if (!ActorDomain::IsFormaNonHuman(actor)) return;
        auto state = GetState(actor->GetFormID());
        if (!state) return;
        std::scoped_lock stateLock(state->mutex);

        const auto now = Clock::now();
        float dt = 1.0f / 60.0f;
        if (state->lastUpdate != Clock::time_point{}) {
            dt = std::clamp(std::chrono::duration<float>(now - state->lastUpdate).count(), 1.0f / 240.0f, 0.05f);
        }
        state->lastUpdate = now;

        if (!actor->Is3DLoaded() || actor->IsDisabled() || IsSpecialInteraction(actor)) {
            state->pitch = 0.0f;
            state->roll = 0.0f;
            state->wasActive = false;
            return;
        }

        const auto* profile = UKClient::GetProfileForActor(actor);
        if (!profile) return;
        const bool biped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped;
        const bool quadruped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped;
        if (!biped && !quadruped) return;

        UKPhysicsAPI::TerrainSnapshotV1 sample{};
        float qualityGain = 0.0f;
        const bool terrainOk = QueryTerrain(actor, sample) && ValidateTerrain(sample, quadruped, qualityGain);

        float targetPitch = 0.0f;
        float targetRoll = 0.0f;
        if (terrainOk) {
            const float gain = quadruped ? kQuadrupedGain : kBipedGain;
            const float pitchCap = quadruped ? kQuadrupedPitchCap : kBipedPitchCap;
            const float rollCap = quadruped ? kQuadrupedRollCap : kBipedRollCap;
            targetPitch = ApplyDeadzoneAndCap(sample.pitchRad * gain * qualityGain, pitchCap);
            targetRoll = ApplyDeadzoneAndCap(sample.rollRad * gain * qualityGain, rollCap);
            state->lastTerrainSerial = sample.serial;
        }

        const float response = terrainOk ? (quadruped ? kQuadrupedResponse : kBipedResponse) : kReturnResponse;
        const float alpha = ExpAlpha(response, dt);
        state->pitch += (targetPitch - state->pitch) * alpha;
        state->roll += (targetRoll - state->roll) * alpha;

        if (std::abs(state->pitch) < 0.00025f) state->pitch = 0.0f;
        if (std::abs(state->roll) < 0.00025f) state->roll = 0.0f;
        const bool active = std::abs(state->pitch) > 0.00025f || std::abs(state->roll) > 0.00025f;
        state->wasActive = active;
        if (!active) return;

        const Anchors anchors = ResolveAnchors(graph, *profile);
        if (anchors.pelvis < 0 && anchors.chest < 0) return;

        float pelvisWeight = 1.0f;
        float chestWeight = 0.0f;
        if (anchors.pelvis >= 0 && anchors.chest >= 0) {
            if (quadruped) {
                pelvisWeight = 0.45f;
                chestWeight = 0.55f;
            } else {
                pelvisWeight = 0.65f;
                chestWeight = 0.35f;
            }
        } else if (anchors.pelvis < 0 && anchors.chest >= 0) {
            pelvisWeight = 0.0f;
            chestWeight = 1.0f;
        }

        int writes = 0;
        if (anchors.pelvis >= 0 && pelvisWeight > 0.0f &&
            WriteModelDelta(c, anchors.pelvis, state->pitch * pelvisWeight, state->roll * pelvisWeight)) {
            ++writes;
        }
        if (anchors.chest >= 0 && chestWeight > 0.0f &&
            WriteModelDelta(c, anchors.chest, state->pitch * chestWeight, state->roll * chestWeight)) {
            ++writes;
        }

        if (writes > 0 && (state->lastLog == Clock::time_point{} ||
            std::chrono::duration<float>(now - state->lastLog).count() >= kTelemetryPeriod)) {
            state->lastLog = now;
            SKSE::log::info(
                "[FormaMotus][SLOPE-WARP] actor={:08X} role={} source={} serial={} age={:.3f}s confidence={:.3f} plane[rms={:.2f} max={:.2f}] raw[pitchDeg={:+.2f} rollDeg={:+.2f}] smooth[pitchDeg={:+.2f} rollDeg={:+.2f}] anchors[pelvis={} chest={}] writes={} next=FEET_OF_SKYRIM",
                actor->GetFormID(), biped ? "BIPED_NONHUMAN" : "QUADRUPED",
                SourceName(sample.source), sample.serial, sample.sampleAgeSeconds, sample.confidence,
                sample.planeRms, sample.planeMaxError, sample.pitchRad * kRad2Deg, sample.rollRad * kRad2Deg,
                state->pitch * kRad2Deg, state->roll * kRad2Deg,
                anchors.pelvis, anchors.chest, writes);
        }
    }

    void Clear()
    {
        std::unique_lock lock(g_statesMutex);
        g_states.clear();
    }
}

