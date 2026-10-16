#include "PCH.h"
#include "StrideWarpModule.h"
#include "FeetOfSkyrimBridge.h"
#include "MotionSourceClient.h"
#include "MotusClient.h"
#include "UKClient.h"

#include <UniversalKinematics.h>

#include <array>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <unordered_map>

namespace StrideWarpModule
{
    namespace
    {
        using Clock = std::chrono::steady_clock;
        constexpr float PI = 3.14159265358979323846f;
        constexpr float DEG2RAD = PI / 180.0f;

        // R4.3 keeps the proven PRE_FOS_OUTPUT lifecycle but makes Foot Lock planar.
        // Motus remains the temporal stance witness; FoS driver-authored planted/raised
        // heights become a second contact witness when valid. X/Y anchoring belongs to
        // Forma Motus while Feet of Skyrim remains the final Z/terrain/pitch IK owner.
        constexpr float kMinStrideScale = 0.70f;
        constexpr float kMaxStrideScale = 1.40f;
        constexpr float kStrideScaleDeadzone = 0.025f;
        constexpr float kStrideScaleResponse = 8.0f;
        constexpr float kMaxHipWarpRad = 10.0f * DEG2RAD;
        constexpr float kMinSagittalUnits = 1.0f;
        constexpr float kMinDerivativeUnits = 2.0f;

        // Motus world-space ankle velocity is the authoritative temporal witness.
        // A planted foot approaches zero world velocity even while the capsule
        // moves; a swinging foot rapidly exceeds these thresholds.
        constexpr float kPlantPlanarSpeed = 28.0f;
        constexpr float kPlantVerticalSpeed = 22.0f;
        constexpr float kSwingPlanarSpeed = 58.0f;
        constexpr float kSwingVerticalSpeed = 40.0f;
        constexpr float kPhaseResponse = 12.0f;
        constexpr float kUnknownWarpFraction = 0.35f;
        constexpr float kStationarySpeed = 8.0f;
        constexpr float kMinAuthoredSpeed = 5.0f;
        constexpr float kTelemetryPeriod = 0.50f;
        constexpr float kSkipTelemetryPeriod = 1.00f;

        // R4.3 hybrid planar Foot Lock. Acquisition still comes from the same PRE_FOS
        // pose, but only world X/Y are preserved. FoS-authored ankle heights can veto
        // false contact while Feet of Skyrim remains free to solve world Z afterwards.
        constexpr float kHardStanceBegin = 0.60f;
        constexpr float kHardSwingBegin = 0.40f;
        constexpr float kLockAcquireWeight = 0.82f;
        constexpr float kLockReleaseWeight = 0.34f;
        constexpr float kAuthoredAcquireWeight = 0.72f;
        constexpr float kAuthoredReleaseWeight = 0.22f;
        constexpr float kAuthoredHeightMinRange = 1.0f;
        constexpr float kLockMinWorldError = 0.35f;
        constexpr float kLockMinBoneLength = 1.0f;
        constexpr float kLockReachMargin = 0.35f;
        constexpr float kLockBreakLegFraction = 0.60f;
        constexpr float kLockMaxCorrectionLegFraction = 0.12f;
        constexpr float kLockStrengthResponse = 24.0f;
        constexpr float kFootLockMinActorSpeed = 4.0f;
        constexpr float kFootLockTelemetryPeriod = 0.50f;
        // R003 transfers the single authoritative foot-lock owner to FullBodyIK.
        // Forma Motus still computes stance and stride, but it no longer writes an
        // independent per-leg planar correction before the global creature solve.
        constexpr bool kFullBodyIKOwnsFootLock = true;

        // C02: creature-only semantic speed-transition impulse. Incessus already
        // tells us the active gait and its authored goal speed, so we do not derive
        // starts/stops/gait changes from noisy instantaneous actor velocity. A new
        // speed regime must persist briefly before it is committed. The committed
        // speed delta generates one transient COM drive whose magnitude is directly
        // proportional to the gain/loss of speed. FullBodyIK remains foot-lock owner.
        constexpr float kBodyImpulseConfirmSeconds = 0.085f;
        constexpr float kBodyImpulseCandidateEpsilon = 2.0f;
        constexpr float kBodyImpulseCommitMinDelta = 5.0f;
        constexpr float kBodyImpulseDeltaSpeedToOffset = 0.012f;
        constexpr float kBodyImpulseAttackResponse = 18.0f;
        constexpr float kBodyImpulseDriveDecayResponse = 4.0f;
        constexpr float kBodyImpulseOffsetReturnResponse = 9.0f;
        constexpr float kBodyImpulseMaxBodyFraction = 0.075f;
        constexpr float kBodyImpulseMinMaxOffset = 2.0f;
        constexpr float kBodyImpulseMaxMaxOffset = 18.0f;
        constexpr float kBodyImpulseTravelSignSpeed = 8.0f;
        constexpr float kBodyImpulseTelemetryPeriod = 0.30f;

        struct Quat
        {
            float x{ 0.0f }, y{ 0.0f }, z{ 0.0f }, w{ 1.0f };
        };

        struct WorldFrame
        {
            Quat rotation{};
            float translation[3]{ 0.0f, 0.0f, 0.0f };
            float scale{ 1.0f };
            bool valid{ false };
        };

        struct LegState
        {
            float stanceWeight{ 0.35f };
            float lockStrength{ 0.0f };
            bool initialized{ false };
            bool locked{ false };
            float lockedWorldXY[2]{ 0.0f, 0.0f };
            float lockAge{ 0.0f };
            std::uint64_t lockSerial{ 0 };
            std::uint64_t lastPhaseSampleSerial{ 0 };
        };


        struct AuthoredPlantSample
        {
            float weight{ 0.0f };
            float ankleHeight{ 0.0f };
            float plantedHeight{ 0.0f };
            float raisedHeight{ 0.0f };
            bool valid{ false };
        };

        struct ActorState
        {
            float strideScale{ 1.0f };
            std::array<LegState, FeetOfSkyrimBridge::kMaxLegs> legs{};
            Clock::time_point lastLog{};
            Clock::time_point lastSkipLog{};
            Clock::time_point lastFootLockLog{};
            std::uint32_t lastSkipReason{ 0 };
            std::uint64_t lockAcquires{ 0 };
            std::uint64_t lockReleases{ 0 };
            std::uint64_t lastSnapshotSerial{ 0 };
            std::uint64_t lastMotusSerial{ 0 };
            bool bindLogged{ false };

            // C02 speed-regime state. Updated once per Incessus snapshot so the
            // three FoS callsites cannot integrate the same provider frame twice.
            float bodyImpulseCommittedSpeed{ 0.0f };
            float bodyImpulseCandidateSpeed{ 0.0f };
            float bodyImpulseCandidateAge{ 0.0f };
            float bodyImpulseDrive{ 0.0f };
            float bodyImpulseOffset{ 0.0f };
            float bodyImpulseTravelSign{ 1.0f };
            float bodyImpulseLastCommittedDelta{ 0.0f };
            std::uint64_t bodyImpulseLastSnapshotSerial{ 0 };
            int bodyImpulseBoneIndex{ -2 };  // -2 unresolved, -1 unavailable
            bool bodyImpulseInitialized{ false };
            bool bodyImpulseMissingBoneLogged{ false };
            Clock::time_point lastBodyImpulseLog{};

            std::mutex mutex;
        };

        struct ModelScratch
        {
            std::array<std::array<float, 3>, FeetOfSkyrimBridge::kBoneCap> pos{};
            std::array<Quat, FeetOfSkyrimBridge::kBoneCap> rot{};
        };

        thread_local ModelScratch t_model;
        std::unordered_map<RE::FormID, std::shared_ptr<ActorState>> g_states;
        std::shared_mutex g_statesMutex;

        float Clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

        float ExpAlpha(float response, float dt)
        {
            return 1.0f - std::exp(-std::max(0.0f, response) * std::clamp(dt, 0.0f, 0.050f));
        }

        bool Normalize3(float* v)
        {
            const float len2 = v[0]*v[0] + v[1]*v[1] + v[2]*v[2];
            if (!std::isfinite(len2) || len2 < 1.0e-8f) return false;
            const float inv = 1.0f / std::sqrt(len2);
            v[0] *= inv; v[1] *= inv; v[2] *= inv;
            return true;
        }

        float Dot3(const float* a, const float* b)
        {
            return a[0]*b[0] + a[1]*b[1] + a[2]*b[2];
        }

        void Cross3(const float* a, const float* b, float* out)
        {
            out[0] = a[1]*b[2] - a[2]*b[1];
            out[1] = a[2]*b[0] - a[0]*b[2];
            out[2] = a[0]*b[1] - a[1]*b[0];
        }

        float Length3(const float* v)
        {
            return std::sqrt(std::max(0.0f, Dot3(v, v)));
        }

        void Sub3(const float* a, const float* b, float* out)
        {
            out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
        }

        void AddScaled3(const float* a, const float* b, float scale, float* out)
        {
            out[0] = a[0] + b[0] * scale;
            out[1] = a[1] + b[1] * scale;
            out[2] = a[2] + b[2] * scale;
        }

        Quat NormalizeQ(Quat q)
        {
            const float len2 = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
            if (!std::isfinite(len2) || len2 < 1.0e-10f) return {};
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
            const float half = angle * 0.5f;
            const float s = std::sin(half);
            return NormalizeQ({ axis[0]*s, axis[1]*s, axis[2]*s, std::cos(half) });
        }

        Quat FromToQ(const float* fromRaw, const float* toRaw, const float* fallbackAxis)
        {
            float from[3]{ fromRaw[0], fromRaw[1], fromRaw[2] };
            float to[3]{ toRaw[0], toRaw[1], toRaw[2] };
            if (!Normalize3(from) || !Normalize3(to)) return {};
            const float d = std::clamp(Dot3(from, to), -1.0f, 1.0f);
            if (d > 0.99998f) return {};
            if (d < -0.9998f) {
                float axis[3];
                Cross3(from, fallbackAxis, axis);
                if (!Normalize3(axis)) {
                    const float fallback2[3]{ 0.0f, 0.0f, 1.0f };
                    Cross3(from, fallback2, axis);
                    if (!Normalize3(axis)) return {};
                }
                return AxisAngle(axis, PI);
            }
            float axis[3];
            Cross3(from, to, axis);
            return NormalizeQ({ axis[0], axis[1], axis[2], 1.0f + d });
        }

        void RotateQ(const Quat& q, const float* v, float* out)
        {
            // Optimized q*v*q^-1.
            const float qv[3] = { q.x, q.y, q.z };
            float uv[3], uuv[3];
            Cross3(qv, v, uv);
            Cross3(qv, uv, uuv);
            const float twoW = 2.0f * q.w;
            out[0] = v[0] + twoW * uv[0] + 2.0f * uuv[0];
            out[1] = v[1] + twoW * uv[1] + 2.0f * uuv[1];
            out[2] = v[2] + twoW * uv[2] + 2.0f * uuv[2];
        }

        bool BuildRootWorldFrame(
            const MotusContinuus::API::BoneKinematicSample& sample,
            WorldFrame& out)
        {
            out = {};
            if ((sample.flags & MotusContinuus::API::kBoneCurrentValid) == 0 ||
                sample.boundary != MotusContinuus::API::PoseSampleBoundary::PreGraphPresented) {
                return false;
            }

            Quat worldRot{
                sample.currentWorld.rotation.x, sample.currentWorld.rotation.y,
                sample.currentWorld.rotation.z, sample.currentWorld.rotation.w };
            Quat componentRot{
                sample.currentComponent.rotation.x, sample.currentComponent.rotation.y,
                sample.currentComponent.rotation.z, sample.currentComponent.rotation.w };
            worldRot = NormalizeQ(worldRot);
            componentRot = NormalizeQ(componentRot);
            out.rotation = MulQ(worldRot, ConjQ(componentRot));

            const float componentScale = sample.currentComponent.scale;
            const float worldScale = sample.currentWorld.scale;
            out.scale = (std::isfinite(componentScale) && std::isfinite(worldScale) &&
                std::abs(componentScale) > 1.0e-4f) ?
                (worldScale / componentScale) : 1.0f;
            if (!std::isfinite(out.scale) || std::abs(out.scale) < 1.0e-4f) out.scale = 1.0f;

            const float componentPos[3]{
                sample.currentComponent.translation.x,
                sample.currentComponent.translation.y,
                sample.currentComponent.translation.z };
            const float scaledComponent[3]{
                componentPos[0] * out.scale,
                componentPos[1] * out.scale,
                componentPos[2] * out.scale };
            float rotatedComponent[3];
            RotateQ(out.rotation, scaledComponent, rotatedComponent);
            out.translation[0] = sample.currentWorld.translation.x - rotatedComponent[0];
            out.translation[1] = sample.currentWorld.translation.y - rotatedComponent[1];
            out.translation[2] = sample.currentWorld.translation.z - rotatedComponent[2];
            out.valid = std::isfinite(out.translation[0]) &&
                std::isfinite(out.translation[1]) && std::isfinite(out.translation[2]);
            return out.valid;
        }

        bool ModelPointToWorld(const WorldFrame& frame, const float* model, float* world)
        {
            if (!frame.valid || !model || !world) return false;
            const float scaled[3]{
                model[0] * frame.scale, model[1] * frame.scale, model[2] * frame.scale };
            float rotated[3];
            RotateQ(frame.rotation, scaled, rotated);
            world[0] = frame.translation[0] + rotated[0];
            world[1] = frame.translation[1] + rotated[1];
            world[2] = frame.translation[2] + rotated[2];
            return std::isfinite(world[0]) && std::isfinite(world[1]) && std::isfinite(world[2]);
        }

        bool WorldPointToModel(const WorldFrame& frame, const float* world, float* model)
        {
            if (!frame.valid || !world || !model) return false;
            const float delta[3]{
                world[0] - frame.translation[0],
                world[1] - frame.translation[1],
                world[2] - frame.translation[2] };
            float unrotated[3];
            RotateQ(ConjQ(frame.rotation), delta, unrotated);
            const float invScale = 1.0f / frame.scale;
            model[0] = unrotated[0] * invScale;
            model[1] = unrotated[1] * invScale;
            model[2] = unrotated[2] * invScale;
            return std::isfinite(model[0]) && std::isfinite(model[1]) && std::isfinite(model[2]);
        }

        bool RebuildModelPose(const FeetOfSkyrimBridge::PoseContext& c)
        {
            if (!c.pose || c.boneCount < 1 || c.boneCount > FeetOfSkyrimBridge::kBoneCap) return false;
            for (int i = 0; i < c.boneCount; ++i) {
                const int p = c.parents[i];
                if (p >= i) return false;
                const float* lp = c.pose + i * 12;
                const float* lq = c.pose + i * 12 + 4;
                Quat local{ lq[0], lq[1], lq[2], lq[3] };
                local = NormalizeQ(local);
                if (p < 0) {
                    t_model.pos[i] = { lp[0], lp[1], lp[2] };
                    t_model.rot[i] = local;
                } else {
                    float rotated[3];
                    RotateQ(t_model.rot[p], lp, rotated);
                    t_model.pos[i] = {
                        t_model.pos[p][0] + rotated[0],
                        t_model.pos[p][1] + rotated[1],
                        t_model.pos[p][2] + rotated[2]
                    };
                    t_model.rot[i] = MulQ(t_model.rot[p], local);
                }
            }
            return true;
        }

        std::shared_ptr<ActorState> GetState(RE::FormID id)
        {
            {
                std::shared_lock lock(g_statesMutex);
                const auto it = g_states.find(id);
                if (it != g_states.end()) return it->second;
            }
            auto created = std::make_shared<ActorState>();
            std::unique_lock lock(g_statesMutex);
            auto [it, inserted] = g_states.emplace(id, created);
            return inserted ? created : it->second;
        }

        struct AuthoredReference
        {
            float motionDbSpeed{ 0.0f };
            float runtimePace{ 0.0f };
            float effectiveSpeed{ 0.0f };
            int clips{ 0 };
            char keys[96]{};
        };

        float ClipAuthoredSpeed(MotionSourceClient::Source source, const char* profile, const char* key)
        {
            Incessus::API::ClipMetricsV1 metrics{};
            if (!MotionSourceClient::QueryClipMetrics(source, profile, key, metrics)) return 0.0f;
            if (!std::isfinite(metrics.duration) || metrics.duration <= 1.0e-4f ||
                !std::isfinite(metrics.distanceXY) || metrics.distanceXY <= 0.0f) return 0.0f;
            const float speed = metrics.distanceXY / metrics.duration;
            return std::isfinite(speed) && speed >= kMinAuthoredSpeed ? speed : 0.0f;
        }

        void AppendKey(char* dst, std::size_t cap, const char* key)
        {
            if (!dst || cap == 0 || !key || !*key) return;
            const std::size_t used = std::strlen(dst);
            if (used + 1 >= cap) return;
            if (used > 0) std::strncat(dst, "+", cap - used - 1);
            const std::size_t now = std::strlen(dst);
            if (now + 1 < cap) std::strncat(dst, key, cap - now - 1);
        }

        float ResolveRuntimePace(const Incessus::API::LocomotionSnapshotV1& s)
        {
            float sum = 0.0f;
            int count = 0;
            if ((s.runtimeCarrierMask & Incessus::API::kCarrierWalk) && s.walkGoal >= kMinAuthoredSpeed) { sum += s.walkGoal; ++count; }
            if ((s.runtimeCarrierMask & Incessus::API::kCarrierTrot) && s.trotGoal >= kMinAuthoredSpeed) { sum += s.trotGoal; ++count; }
            if ((s.runtimeCarrierMask & Incessus::API::kCarrierRun) && s.runGoal >= kMinAuthoredSpeed) { sum += s.runGoal; ++count; }
            if (count > 0) return sum / static_cast<float>(count);

            // With no carrier witness, graphSpeed is a better transitional pace
            // witness than jumping immediately to the desired gait goal.
            if (std::isfinite(s.graphSpeed) && s.graphSpeed >= kMinAuthoredSpeed) return s.graphSpeed;
            switch (s.desiredGait) {
            case Incessus::API::Gait::kWalk: return s.walkGoal;
            case Incessus::API::Gait::kTrot: return s.trotGoal;
            case Incessus::API::Gait::kRun: return s.runGoal;
            default: return 0.0f;
            }
        }

        std::string AnimusDirectionalKey(const Incessus::API::LocomotionSnapshotV1& s, Incessus::API::Gait gait)
        {
            const bool backward = (s.flags & Incessus::API::kMovingBackward) != 0;
            const bool left = (s.flags & Incessus::API::kMovingLeft) != 0;
            const bool right = (s.flags & Incessus::API::kMovingRight) != 0;

            std::string key;
            switch (gait) {
            case Incessus::API::Gait::kWalk: key = "walk"; break;
            case Incessus::API::Gait::kTrot: key = "trot"; break;
            case Incessus::API::Gait::kRun: key = "run"; break;
            default: return {};
            }
            key += backward ? "backward" : "forward";
            if (left != right) key += left ? "left" : "right";
            return key;
        }

        AuthoredReference ResolveAuthoredReference(MotionSourceClient::Source source, const Incessus::API::LocomotionSnapshotV1& s)
        {
            AuthoredReference out{};
            if (!s.profile[0]) return out;

            const bool backward = (s.flags & Incessus::API::kMovingBackward) != 0;
            float sum = 0.0f;
            auto add = [&](const char* key) {
                const float speed = ClipAuthoredSpeed(source, s.profile, key);
                if (speed <= 0.0f) return;
                sum += speed;
                ++out.clips;
                AppendKey(out.keys, sizeof(out.keys), key);
            };

            auto addGait = [&](Incessus::API::Gait gait) {
                if (source != MotionSourceClient::Source::kIncessus) {
                    const auto directional = AnimusDirectionalKey(s, gait);
                    if (!directional.empty()) {
                        const int before = out.clips;
                        add(directional.c_str());
                        if (out.clips != before) return;
                    }
                }
                switch (gait) {
                case Incessus::API::Gait::kWalk: add(backward ? "walkbackward" : "walkforward"); break;
                case Incessus::API::Gait::kTrot: add("trotforward"); break;
                case Incessus::API::Gait::kRun: add(source != MotionSourceClient::Source::kIncessus && backward ? "runbackward" : "runforward"); break;
                default: break;
                }
            };

            if ((s.runtimeCarrierMask & Incessus::API::kCarrierWalk) != 0) addGait(Incessus::API::Gait::kWalk);
            if ((s.runtimeCarrierMask & Incessus::API::kCarrierTrot) != 0) addGait(Incessus::API::Gait::kTrot);
            if ((s.runtimeCarrierMask & Incessus::API::kCarrierRun) != 0) addGait(Incessus::API::Gait::kRun);
            if (out.clips == 0) {
                // R4.2: with no generated-carrier witness, direction is NOT allowed
                // to collapse the gait family to WALK. The previous shortcut chose
                // walkbackward for every backward witness, including RUN snapshots;
                // the R4.1 log showed this as gait=RUN + graphSpeed=833 +
                // keys=walkbackward, which severely under-estimated authored pace.
                // Choose the family whose configured graph goal is closest to the
                // current graphSpeed. Provider-specific directional keys are tried separately above.
                if (std::isfinite(s.graphSpeed) && s.graphSpeed >= kMinAuthoredSpeed) {
                    float best = 1.0e30f;
                    const char* bestKey = nullptr;
                    auto consider = [&](float goal, const char* key) {
                        if (!std::isfinite(goal) || goal < kMinAuthoredSpeed) return;
                        const float d = std::abs(s.graphSpeed - goal);
                        if (d < best) { best = d; bestKey = key; }
                    };
                    consider(s.walkGoal, backward ? "walkbackward" : "walkforward");
                    consider(s.trotGoal, "trotforward");
                    consider(s.runGoal, "runforward");
                    if (bestKey) add(bestKey);
                }
                if (out.clips == 0) {
                    addGait(s.desiredGait);
                }
            }

            if (out.clips > 0) out.motionDbSpeed = sum / static_cast<float>(out.clips);
            out.runtimePace = ResolveRuntimePace(s);

            // MotionDB distance/duration and graph-speed live in different runtime domains.
            // Keep the inherited conservative geometric-mean reference for either producer;
            // the producer remains sole owner of actual root transport. Forma only needs a
            // stable authored pace witness for stride correction.
            if (out.motionDbSpeed >= kMinAuthoredSpeed && out.runtimePace >= kMinAuthoredSpeed)
                out.effectiveSpeed = std::sqrt(out.motionDbSpeed * out.runtimePace);
            else if (out.motionDbSpeed >= kMinAuthoredSpeed)
                out.effectiveSpeed = out.motionDbSpeed;
            else if (out.runtimePace >= kMinAuthoredSpeed)
                out.effectiveSpeed = out.runtimePace;
            return out;
        }

        enum class SkipReason : std::uint32_t
        {
            kNone = 0,
            kUnsupportedLocomotionMode,
            kNoMotionSource,
            kNotLocomoting,
            kPivotOrAction,
            kNoAuthoredReference,
            kStationary
        };

        const char* SkipReasonName(SkipReason r)
        {
            switch (r) {
            case SkipReason::kUnsupportedLocomotionMode: return "UNSUPPORTED_LOCOMOTION_MODE";
            case SkipReason::kNoMotionSource: return "NO_MOTION_SOURCE";
            case SkipReason::kNotLocomoting: return "NO_LOCOMOTION";
            case SkipReason::kPivotOrAction: return "PIVOT_OR_ACTION";
            case SkipReason::kNoAuthoredReference: return "NO_AUTHORED_REFERENCE";
            case SkipReason::kStationary: return "STATIONARY";
            default: return "NONE";
            }
        }

        float PhysicalPlanarSpeed(RE::Actor* actor)
        {
            RE::NiPoint3 v{};
            actor->GetLinearVelocity(v);
            if (!std::isfinite(v.x) || !std::isfinite(v.y)) return 0.0f;
            return std::hypot(v.x, v.y);
        }

        float ResolveStanceWeight(
            RE::Actor* actor,
            const char* ankleName,
            LegState& state,
            float fallbackDt,
            std::uint64_t& motusSerial,
            MotusContinuus::API::BoneKinematicSample& sample,
            bool& motusValid,
            bool& currentValid,
            bool& sampleAdvanced)
        {
            sample = {};
            sampleAdvanced = false;
            motusValid = MotusClient::QueryBone(actor, ankleName, sample) &&
                (sample.flags & MotusContinuus::API::kBoneVelocityValid) != 0;
            currentValid = (sample.flags & MotusContinuus::API::kBoneCurrentValid) != 0 &&
                sample.boundary == MotusContinuus::API::PoseSampleBoundary::PreGraphPresented;
            if (!motusValid) {
                if (!state.initialized) {
                    state.stanceWeight = 1.0f - kUnknownWarpFraction;
                    state.initialized = true;
                }
                return state.stanceWeight;
            }

            motusSerial = std::max(motusSerial, sample.sampleSerial);
            if (sample.sampleSerial != 0 && state.lastPhaseSampleSerial == sample.sampleSerial) {
                return state.stanceWeight;
            }
            if (sample.sampleSerial != 0) state.lastPhaseSampleSerial = sample.sampleSerial;
            sampleAdvanced = true;

            const float planar = std::hypot(sample.worldLinearVelocity.x, sample.worldLinearVelocity.y);
            const float vertical = std::abs(sample.worldLinearVelocity.z);
            const float dt = std::clamp(
                std::isfinite(sample.deltaTime) && sample.deltaTime > 0.0f ? sample.deltaTime : fallbackDt,
                1.0f / 240.0f, 0.05f);

            float target = state.stanceWeight;
            if (!state.initialized) {
                target = (planar <= kPlantPlanarSpeed && vertical <= kPlantVerticalSpeed) ? 1.0f : 0.0f;
                state.initialized = true;
            } else if (planar <= kPlantPlanarSpeed && vertical <= kPlantVerticalSpeed) {
                target = 1.0f;
            } else if (planar >= kSwingPlanarSpeed || vertical >= kSwingVerticalSpeed) {
                target = 0.0f;
            }

            state.stanceWeight += (target - state.stanceWeight) * ExpAlpha(kPhaseResponse, dt);
            state.stanceWeight = Clamp01(state.stanceWeight);
            return state.stanceWeight;
        }

        float PhaseGainFromStance(float stance)
        {
            if (stance >= kHardStanceBegin) return 0.0f;
            if (stance <= kHardSwingBegin) return 1.0f;
            const float t = Clamp01((stance - kHardSwingBegin) / (kHardStanceBegin - kHardSwingBegin));
            const float smooth = t * t * (3.0f - 2.0f * t);
            return 1.0f - smooth;
        }

        AuthoredPlantSample ResolveAuthoredPlantSample(
            const FeetOfSkyrimBridge::PoseContext& c,
            const FeetOfSkyrimBridge::LegSlot& leg)
        {
            AuthoredPlantSample out{};
            const int ankle = leg.ankleIndex;
            if (ankle < 0 || ankle >= c.poseCount) return out;

            float planted = leg.plantedHeight;
            float raised = leg.raisedHeight;
            if (!std::isfinite(planted) || !std::isfinite(raised) || !(raised > planted)) return out;

            // Match Feet of Skyrim: native hkbFootIkDriverInfo can store authored
            // heights in Havok units. Convert those values to pose/game units before
            // comparing them with the generated PRE_FOS model-space ankle height.
            float legScale = 1.0f;
            if (std::abs(planted) < 2.0f && std::abs(raised) < 2.5f) {
                const float worldScale = RE::bhkWorld::GetWorldScale();
                if (std::isfinite(worldScale) && std::abs(worldScale) > 1.0e-6f) {
                    legScale = 1.0f / worldScale;
                }
            }
            planted *= legScale;
            raised *= legScale;
            const float range = raised - planted;
            if (!std::isfinite(range) || range < kAuthoredHeightMinRange) return out;

            const float ankleHeight = t_model.pos[ankle][2];
            if (!std::isfinite(ankleHeight)) return out;
            const float t = Clamp01((ankleHeight - planted) / range);
            const float smooth = t * t * (3.0f - 2.0f * t);
            out.weight = 1.0f - smooth;
            out.ankleHeight = ankleHeight;
            out.plantedHeight = planted;
            out.raisedHeight = raised;
            out.valid = true;
            return out;
        }

        bool UpdateFootLock(
            LegState& state,
            const MotusContinuus::API::BoneKinematicSample& sample,
            bool motusValid,
            bool poseWorldValid,
            const float* preFosWorld,
            float stance,
            const AuthoredPlantSample& authoredPlant,
            float dt,
            bool advanceTime,
            bool forceRelease,
            bool& acquired,
            bool& released)
        {
            acquired = false;
            released = false;
            const bool authoredRelease = authoredPlant.valid && authoredPlant.weight <= kAuthoredReleaseWeight;
            if (state.locked &&
                (forceRelease || !poseWorldValid || stance <= kLockReleaseWeight || authoredRelease)) {
                state.locked = false;
                state.lockStrength = 0.0f;
                state.lockAge = 0.0f;
                released = true;
            }

            const bool authoredAcquire = !authoredPlant.valid || authoredPlant.weight >= kAuthoredAcquireWeight;
            if (!state.locked && !forceRelease && advanceTime && motusValid && poseWorldValid &&
                preFosWorld && stance >= kLockAcquireWeight && authoredAcquire) {
                state.locked = true;
                state.lockedWorldXY[0] = preFosWorld[0];
                state.lockedWorldXY[1] = preFosWorld[1];
                state.lockStrength = 0.0f;
                state.lockAge = 0.0f;
                state.lockSerial = sample.sampleSerial;
                acquired = true;
            }

            if (state.locked && advanceTime) {
                state.lockAge += std::max(0.0f, dt);
                state.lockStrength += (1.0f - state.lockStrength) * ExpAlpha(kLockStrengthResponse, dt);
                state.lockStrength = Clamp01(state.lockStrength);
            }
            return state.locked;
        }

        bool WriteModelRotation(FeetOfSkyrimBridge::PoseContext& c, int bone, const Quat& wantedModel)
        {
            if (bone < 0 || bone >= c.poseCount) return false;
            const int parent = c.parents[bone];
            Quat local = wantedModel;
            if (parent >= 0) {
                if (parent >= bone) return false;
                local = MulQ(ConjQ(t_model.rot[parent]), wantedModel);
            }
            float* lq = c.pose + bone * 12 + 4;
            lq[0] = local.x; lq[1] = local.y; lq[2] = local.z; lq[3] = local.w;
            FeetOfSkyrimBridge::MarkBoneValid(c, bone);
            return true;
        }

        bool ApplyTwoBoneTarget(
            FeetOfSkyrimBridge::PoseContext& c,
            const FeetOfSkyrimBridge::LegSlot& leg,
            const float* targetModel,
            float& outResidual)
        {
            outResidual = 0.0f;
            const int hip = leg.hipIndex, knee = leg.kneeIndex, ankle = leg.ankleIndex;
            if (hip < 0 || knee < 0 || ankle < 0 || hip >= c.poseCount || knee >= c.poseCount || ankle >= c.poseCount) return false;

            float upper[3], lower[3], toTarget[3];
            Sub3(t_model.pos[knee].data(), t_model.pos[hip].data(), upper);
            Sub3(t_model.pos[ankle].data(), t_model.pos[knee].data(), lower);
            Sub3(targetModel, t_model.pos[hip].data(), toTarget);
            const float l1 = Length3(upper), l2 = Length3(lower);
            float distance = Length3(toTarget);
            if (l1 < kLockMinBoneLength || l2 < kLockMinBoneLength || distance < 1.0e-4f) return false;

            float dir[3]{ toTarget[0], toTarget[1], toTarget[2] };
            if (!Normalize3(dir)) return false;
            const float minReach = std::abs(l1 - l2) + kLockReachMargin;
            const float maxReach = std::max(minReach + 0.01f, l1 + l2 - kLockReachMargin);
            distance = std::clamp(distance, minReach, maxReach);
            float target[3];
            AddScaled3(t_model.pos[hip].data(), dir, distance, target);

            float hipToKnee[3];
            Sub3(t_model.pos[knee].data(), t_model.pos[hip].data(), hipToKnee);
            const float along = Dot3(hipToKnee, dir);
            float perp[3]{
                hipToKnee[0] - dir[0] * along,
                hipToKnee[1] - dir[1] * along,
                hipToKnee[2] - dir[2] * along
            };
            if (!Normalize3(perp)) {
                // FoS exposes the authored hkbFootIk knee hinge. Use its bend-positive
                // form as the first fallback when the animated limb is nearly straight
                // and therefore cannot define a stable bend plane by itself.
                float bendAxisLocal[3]{ -leg.kneeAxis[0], -leg.kneeAxis[1], -leg.kneeAxis[2] };
                float hingeModel[3]{};
                if (Normalize3(bendAxisLocal)) {
                    RotateQ(t_model.rot[knee], bendAxisLocal, hingeModel);
                    if (Normalize3(hingeModel)) {
                        Cross3(hingeModel, dir, perp);
                    }
                }
                if (!Normalize3(perp)) {
                    Cross3(c.modelRight, dir, perp);
                    if (!Normalize3(perp)) return false;
                }
            }

            const float x = std::clamp((l1*l1 - l2*l2 + distance*distance) / (2.0f * distance), -l1, l1);
            const float h = std::sqrt(std::max(0.0f, l1*l1 - x*x));
            float desiredKnee[3];
            AddScaled3(t_model.pos[hip].data(), dir, x, desiredKnee);
            desiredKnee[0] += perp[0] * h;
            desiredKnee[1] += perp[1] * h;
            desiredKnee[2] += perp[2] * h;

            float desiredUpper[3];
            Sub3(desiredKnee, t_model.pos[hip].data(), desiredUpper);
            const Quat hipDelta = FromToQ(upper, desiredUpper, c.modelRight);
            if (!WriteModelRotation(c, hip, MulQ(hipDelta, t_model.rot[hip]))) return false;
            if (!RebuildModelPose(c)) return false;

            float currentLower[3], desiredLower[3];
            Sub3(t_model.pos[ankle].data(), t_model.pos[knee].data(), currentLower);
            Sub3(target, t_model.pos[knee].data(), desiredLower);
            const Quat kneeDelta = FromToQ(currentLower, desiredLower, c.modelRight);
            if (!WriteModelRotation(c, knee, MulQ(kneeDelta, t_model.rot[knee]))) return false;
            if (!RebuildModelPose(c)) return false;

            float residual[3];
            Sub3(target, t_model.pos[ankle].data(), residual);
            outResidual = Length3(residual);
            return true;
        }

        bool ApplyWorldFootLock(
            FeetOfSkyrimBridge::PoseContext& c,
            const FeetOfSkyrimBridge::LegSlot& leg,
            LegState& state,
            const WorldFrame& frame,
            float& outWorldError,
            float& outWorldResidual,
            float& outBreakDistance,
            float& outMaxCorrection,
            float& outVerticalDrift)
        {
            outWorldError = 0.0f;
            outWorldResidual = 0.0f;
            outBreakDistance = 0.0f;
            outMaxCorrection = 0.0f;
            outVerticalDrift = 0.0f;
            if (!state.locked || !frame.valid) return false;

            const int hip = leg.hipIndex, knee = leg.kneeIndex, ankle = leg.ankleIndex;
            if (hip < 0 || knee < 0 || ankle < 0 ||
                hip >= c.poseCount || knee >= c.poseCount || ankle >= c.poseCount) return false;

            float upper[3], lower[3];
            Sub3(t_model.pos[knee].data(), t_model.pos[hip].data(), upper);
            Sub3(t_model.pos[ankle].data(), t_model.pos[knee].data(), lower);
            const float legLength = Length3(upper) + Length3(lower);
            if (!std::isfinite(legLength) || legLength < kLockMinBoneLength * 2.0f) return false;
            outBreakDistance = legLength * kLockBreakLegFraction * std::abs(frame.scale);
            outMaxCorrection = legLength * kLockMaxCorrectionLegFraction;

            float currentWorld[3];
            if (!ModelPointToWorld(frame, t_model.pos[ankle].data(), currentWorld)) return false;
            const float worldErrorXY[2]{
                state.lockedWorldXY[0] - currentWorld[0],
                state.lockedWorldXY[1] - currentWorld[1] };
            outWorldError = std::hypot(worldErrorXY[0], worldErrorXY[1]);
            if (!std::isfinite(outWorldError)) return false;
            if (outWorldError > outBreakDistance) {
                state.locked = false;
                state.lockStrength = 0.0f;
                state.lockAge = 0.0f;
                return false;
            }
            if (outWorldError < kLockMinWorldError) {
                outWorldResidual = outWorldError;
                return true;
            }

            // R4.3 owns only horizontal anchoring. Preserve the ankle's current PRE_FOS
            // world Z so Feet of Skyrim receives an unconstrained vertical starting point
            // for its terrain height/pitch solve.
            const float planarTargetWorld[3]{
                state.lockedWorldXY[0], state.lockedWorldXY[1], currentWorld[2] };
            float lockedTargetModel[3];
            if (!WorldPointToModel(frame, planarTargetWorld, lockedTargetModel)) return false;
            float modelCorrection[3];
            Sub3(lockedTargetModel, t_model.pos[ankle].data(), modelCorrection);
            modelCorrection[0] *= state.lockStrength;
            modelCorrection[1] *= state.lockStrength;
            modelCorrection[2] *= state.lockStrength;
            float correctionLen = Length3(modelCorrection);
            if (!std::isfinite(correctionLen)) return false;
            if (correctionLen > outMaxCorrection && outMaxCorrection > 0.0f) {
                const float scale = outMaxCorrection / correctionLen;
                modelCorrection[0] *= scale;
                modelCorrection[1] *= scale;
                modelCorrection[2] *= scale;
            }

            float targetModel[3]{
                t_model.pos[ankle][0] + modelCorrection[0],
                t_model.pos[ankle][1] + modelCorrection[1],
                t_model.pos[ankle][2] + modelCorrection[2] };
            float modelResidual = 0.0f;
            if (!ApplyTwoBoneTarget(c, leg, targetModel, modelResidual)) return false;

            float solvedWorld[3];
            if (!ModelPointToWorld(frame, t_model.pos[ankle].data(), solvedWorld)) return false;
            const float residualX = state.lockedWorldXY[0] - solvedWorld[0];
            const float residualY = state.lockedWorldXY[1] - solvedWorld[1];
            outWorldResidual = std::hypot(residualX, residualY);
            outVerticalDrift = std::abs(solvedWorld[2] - currentWorld[2]);
            return std::isfinite(outWorldResidual) && std::isfinite(outVerticalDrift);
        }

        bool ApplyHipSagittalWarp(
            FeetOfSkyrimBridge::PoseContext& c,
            const FeetOfSkyrimBridge::LegSlot& leg,
            float strideScale,
            float phaseGain,
            float anatomyGain,
            float& outAngle)
        {
            outAngle = 0.0f;
            const int hip = leg.hipIndex;
            const int knee = leg.kneeIndex;
            const int ankle = leg.ankleIndex;
            if (hip < 0 || knee < 0 || ankle < 0 || hip >= c.poseCount || knee >= c.poseCount || ankle >= c.poseCount) return false;
            const int parent = c.parents[hip];
            if (parent < 0 || parent >= hip) return false;

            const float v[3] = {
                t_model.pos[ankle][0] - t_model.pos[hip][0],
                t_model.pos[ankle][1] - t_model.pos[hip][1],
                t_model.pos[ankle][2] - t_model.pos[hip][2]
            };
            const float sagittal = Dot3(v, c.modelForward);
            if (std::abs(sagittal) < kMinSagittalUnits) return false;

            float derivativeVec[3];
            Cross3(c.modelRight, v, derivativeVec);
            const float derivative = Dot3(derivativeVec, c.modelForward);
            if (std::abs(derivative) < kMinDerivativeUnits) return false;

            const float deltaSagittal = sagittal * (strideScale - 1.0f) * phaseGain * anatomyGain;
            float angle = deltaSagittal / derivative;
            angle = std::clamp(angle, -kMaxHipWarpRad, kMaxHipWarpRad);
            if (!std::isfinite(angle) || std::abs(angle) < 0.00025f) return false;

            const Quat deltaModel = AxisAngle(c.modelRight, angle);
            const Quat wantedModel = MulQ(deltaModel, t_model.rot[hip]);
            const Quat local = MulQ(ConjQ(t_model.rot[parent]), wantedModel);
            float* lq = c.pose + hip * 12 + 4;
            lq[0] = local.x; lq[1] = local.y; lq[2] = local.z; lq[3] = local.w;
            FeetOfSkyrimBridge::MarkBoneValid(c, hip);
            outAngle = angle;
            return true;
        }

        bool IsAncestorOf(const FeetOfSkyrimBridge::PoseContext& c, int ancestor, int bone)
        {
            if (ancestor < 0 || bone < 0 || ancestor >= c.boneCount || bone >= c.boneCount) return false;
            for (int current = bone, guard = 0; current >= 0 && guard < c.boneCount; ++guard) {
                if (current == ancestor) return true;
                current = c.parents[current];
            }
            return false;
        }

        int ResolveBodyImpulseBone(FeetOfSkyrimBridge::PoseContext& c)
        {
            // Skyrim creature rigs normally expose NPC COM [COM ]. We require the
            // selected COM to be an ancestor of every FoS hip, preventing an
            // accidental partial-body translation on an unrelated bone name.
            const char* tokens[] = { "[COM", " COM", "COM" };
            for (const char* token : tokens) {
                const int index = FeetOfSkyrimBridge::FindBoneIndexByNameContains(c, token);
                if (index < 0 || index >= c.poseCount) continue;
                if (c.parents[index] < 0) continue;  // never translate the Havok root itself

                bool ownsAllLegs = true;
                for (int i = 0; i < c.legCount; ++i) {
                    if (!IsAncestorOf(c, index, c.legs[i].hipIndex)) {
                        ownsAllLegs = false;
                        break;
                    }
                }
                if (ownsAllLegs) return index;
            }
            return -1;
        }

        float ResolveBodyImpulseRegimeSpeed(const Incessus::API::LocomotionSnapshotV1& snapshot)
        {
            const bool locomotion = (snapshot.flags & Incessus::API::kGraphLocomotionActive) != 0;
            if (!locomotion) return 0.0f;

            float speed = 0.0f;
            switch (snapshot.desiredGait) {
            case Incessus::API::Gait::kWalk:
                speed = snapshot.walkGoal;
                break;
            case Incessus::API::Gait::kTrot:
                speed = snapshot.trotGoal;
                break;
            case Incessus::API::Gait::kRun:
                speed = snapshot.runGoal;
                break;
            default:
                speed = snapshot.graphSpeed;
                break;
            }

            // If a gait goal is temporarily unavailable while locomotion is active,
            // graphSpeed is still a valid Incessus-side fallback. We never fall back
            // to instantaneous actor velocity for the impulse signal itself.
            if (!std::isfinite(speed) || speed < kMinAuthoredSpeed) {
                speed = snapshot.graphSpeed;
            }
            return std::isfinite(speed) && speed > 0.0f ? speed : 0.0f;
        }

        bool ApplyCreatureSpeedImpulse(
            RE::Actor* actor,
            const UniversalKinematics::SkeletonProfile& profile,
            const MotionSourceClient::ResolvedSnapshot& resolved,
            float physicalSpeed,
            float dt,
            ActorState& state,
            FeetOfSkyrimBridge::PoseContext& c)
        {
            if (!actor || actor->IsPlayerRef() || resolved.source != MotionSourceClient::Source::kIncessus) return false;

            const auto& snapshot = resolved.snapshot;
            const bool supported = (snapshot.flags & Incessus::API::kSupportedActor) != 0;
            if (!supported) return false;

            if (state.bodyImpulseBoneIndex == -2 || state.bodyImpulseBoneIndex >= c.poseCount) {
                state.bodyImpulseBoneIndex = ResolveBodyImpulseBone(c);
            }
            if (state.bodyImpulseBoneIndex < 0) {
                if (!state.bodyImpulseMissingBoneLogged) {
                    state.bodyImpulseMissingBoneLogged = true;
                    SKSE::log::info(
                        "[FormaMotus][C02-SPEED-TRANSITION-SKIP] actor={:08X} profile=\"{}\" reason=NO_SHARED_COM_BONE legs={}",
                        actor->GetFormID(), snapshot.profile, c.legCount);
                }
                return false;
            }

            const bool locomotion = (snapshot.flags & Incessus::API::kGraphLocomotionActive) != 0;
            const bool pivot = (snapshot.flags & Incessus::API::kPivotBusy) != 0;
            const bool action = (snapshot.flags & Incessus::API::kActionBypassActive) != 0;
            const bool newSnapshot = snapshot.snapshotSerial == 0 ||
                snapshot.snapshotSerial != state.bodyImpulseLastSnapshotSerial;

            const float maxOffset = std::clamp(
                (std::isfinite(profile.bodyLength) && profile.bodyLength > 1.0f ? profile.bodyLength : 100.0f) *
                    kBodyImpulseMaxBodyFraction,
                kBodyImpulseMinMaxOffset, kBodyImpulseMaxMaxOffset);
            const float regimeSpeed = ResolveBodyImpulseRegimeSpeed(snapshot);

            bool committedNow = false;
            float committedDelta = 0.0f;

            if (!state.bodyImpulseInitialized) {
                // Loading/spawning into an already-active gait establishes the current
                // regime without inventing a fake Start impulse.
                state.bodyImpulseCommittedSpeed = regimeSpeed;
                state.bodyImpulseCandidateSpeed = regimeSpeed;
                state.bodyImpulseCandidateAge = 0.0f;
                state.bodyImpulseDrive = 0.0f;
                state.bodyImpulseOffset = 0.0f;
                state.bodyImpulseLastCommittedDelta = 0.0f;
                state.bodyImpulseLastSnapshotSerial = snapshot.snapshotSerial;
                state.bodyImpulseInitialized = true;
            } else if (newSnapshot) {
                if (regimeSpeed >= kBodyImpulseTravelSignSpeed || physicalSpeed >= kBodyImpulseTravelSignSpeed) {
                    const bool backward = (snapshot.flags & Incessus::API::kMovingBackward) != 0;
                    const bool forward = (snapshot.flags & Incessus::API::kMovingForward) != 0;
                    if (backward != forward) state.bodyImpulseTravelSign = backward ? -1.0f : 1.0f;
                }

                // Confirmation is on the semantic Incessus regime speed, not physical
                // speed. Step-cycle spikes therefore cannot repeatedly retrigger it.
                if (std::abs(regimeSpeed - state.bodyImpulseCandidateSpeed) > kBodyImpulseCandidateEpsilon) {
                    state.bodyImpulseCandidateSpeed = regimeSpeed;
                    state.bodyImpulseCandidateAge = dt;
                } else {
                    state.bodyImpulseCandidateAge += dt;
                }

                const float pendingDelta = state.bodyImpulseCandidateSpeed - state.bodyImpulseCommittedSpeed;
                if (state.bodyImpulseCandidateAge >= kBodyImpulseConfirmSeconds &&
                    std::abs(pendingDelta) >= kBodyImpulseCommitMinDelta) {
                    committedDelta = pendingDelta;
                    state.bodyImpulseCommittedSpeed = state.bodyImpulseCandidateSpeed;
                    state.bodyImpulseLastCommittedDelta = committedDelta;
                    committedNow = true;

                    if (!pivot && !action) {
                        // This is the proportional rule requested for C02:
                        // larger gain/loss in Incessus speed -> larger COM impulse.
                        state.bodyImpulseDrive = std::clamp(
                            committedDelta * kBodyImpulseDeltaSpeedToOffset * state.bodyImpulseTravelSign,
                            -maxOffset, maxOffset);
                    } else {
                        state.bodyImpulseDrive = 0.0f;
                    }
                }

                // The drive is a transient target. The COM attacks it rapidly, then
                // the drive decays and the offset returns smoothly to neutral.
                const float desiredOffset = (!pivot && !action) ? state.bodyImpulseDrive : 0.0f;
                const float response = std::abs(desiredOffset) > std::abs(state.bodyImpulseOffset) ?
                    kBodyImpulseAttackResponse : kBodyImpulseOffsetReturnResponse;
                state.bodyImpulseOffset +=
                    (desiredOffset - state.bodyImpulseOffset) * ExpAlpha(response, dt);
                state.bodyImpulseDrive +=
                    (0.0f - state.bodyImpulseDrive) * ExpAlpha(kBodyImpulseDriveDecayResponse, dt);

                if (std::abs(state.bodyImpulseDrive) < 0.005f) state.bodyImpulseDrive = 0.0f;
                if (std::abs(state.bodyImpulseOffset) < 0.005f && state.bodyImpulseDrive == 0.0f) {
                    state.bodyImpulseOffset = 0.0f;
                }
                state.bodyImpulseLastSnapshotSerial = snapshot.snapshotSerial;
            }

            // Telemetry is intentionally emitted even while the offset is zero. The
            // next test can therefore prove whether a transition was rejected,
            // confirmed, committed and scaled without relying on visual inspection.
            if (newSnapshot) {
                const auto now = Clock::now();
                const bool logNow = committedNow || state.lastBodyImpulseLog == Clock::time_point{} ||
                    std::chrono::duration<float>(now - state.lastBodyImpulseLog).count() >= kBodyImpulseTelemetryPeriod;
                if (logNow) {
                    state.lastBodyImpulseLog = now;
                    SKSE::log::info(
                        "[FormaMotus][C02-SPEED-TRANSITION] actor={:08X} profile=\"{}\" bone={} gait={} carrierMask=0x{:02X} locomotion={} physicalSpeed={:.2f} graphSpeed={:.2f} regimeSpeed={:.2f} committedSpeed={:.2f} candidateSpeed={:.2f} candidateMs={:.0f} commit={} delta={:+.2f} lastDelta={:+.2f} travelSign={:+.0f} drive={:+.2f} offset={:+.2f} max={:.2f} pivot={} action={} owner=COM_PRE_FOS footLock=FULLBODYIK",
                        actor->GetFormID(), snapshot.profile, state.bodyImpulseBoneIndex,
                        static_cast<unsigned>(snapshot.desiredGait), static_cast<unsigned>(snapshot.runtimeCarrierMask),
                        locomotion, physicalSpeed, snapshot.graphSpeed, regimeSpeed,
                        state.bodyImpulseCommittedSpeed, state.bodyImpulseCandidateSpeed,
                        state.bodyImpulseCandidateAge * 1000.0f, committedNow, committedDelta,
                        state.bodyImpulseLastCommittedDelta, state.bodyImpulseTravelSign,
                        state.bodyImpulseDrive, state.bodyImpulseOffset, maxOffset, pivot, action);
                }
            }

            const float offset = state.bodyImpulseOffset;
            if (!std::isfinite(offset) || std::abs(offset) < 0.01f) return false;

            const int bone = state.bodyImpulseBoneIndex;
            const int parent = c.parents[bone];
            if (parent < 0 || parent >= bone) return false;

            float modelDelta[3]{
                c.modelForward[0] * offset,
                c.modelForward[1] * offset,
                c.modelForward[2] * offset
            };
            float localDelta[3]{};
            RotateQ(ConjQ(t_model.rot[parent]), modelDelta, localDelta);

            float* translation = c.pose + bone * 12;
            translation[0] += localDelta[0];
            translation[1] += localDelta[1];
            translation[2] += localDelta[2];
            FeetOfSkyrimBridge::MarkBoneValid(c, bone);
            return true;
        }

    }

    void ApplyPreFootIkPose(RE::Actor* actor, RE::BShkbAnimationGraph*, FeetOfSkyrimBridge::PoseContext& c)
    {
        static std::once_flag s_buildLogOnce;
        std::call_once(s_buildLogOnce, []() {
            SKSE::log::info("[FormaMotus][STRIDE] BUILD R5.1-C02-SEMANTIC-SPEED-TRANSITION buildRev=501 bodyImpulse=INCESSUS_REGIME_DELTA_TO_COM footLock=DELEGATED_TO_FULLBODYIK");
        });
        if (!actor || !c.pose || c.legCount <= 0) return;

        auto state = GetState(actor->GetFormID());
        if (!state) return;
        std::scoped_lock stateLock(state->mutex);

        if (actor->IsPlayerRef()) {
            auto* camera = RE::PlayerCamera::GetSingleton();
            if (camera && camera->IsInFirstPerson()) {
                state->strideScale = 1.0f;
                for (auto& leg : state->legs) {
                    leg.locked = false;
                    leg.lockStrength = 0.0f;
                    leg.lockAge = 0.0f;
                }
                return;
            }
        }

        MotionSourceClient::ResolvedSnapshot resolved{};
        const bool haveSnapshot = MotionSourceClient::QueryLocomotion(actor, resolved);
        const auto& snapshot = resolved.snapshot;
        const auto source = resolved.source;
        const char* sourceName = MotionSourceClient::SourceName(source);
        const auto flags = haveSnapshot ? snapshot.flags : 0u;
        const bool supported = haveSnapshot && (flags & Incessus::API::kSupportedActor) != 0;
        const bool locomotion = supported && (flags & Incessus::API::kGraphLocomotionActive) != 0;
        const bool pivot = supported && (flags & Incessus::API::kPivotBusy) != 0;
        const bool action = supported && (flags & Incessus::API::kActionBypassActive) != 0;

        if (!state->bindLogged && c.legCount > 0) {
            state->bindLogged = true;
            SKSE::log::info(
                "[FormaMotus][STRIDE-BIND] actor={:08X} legs={} firstLeg[hip={} knee={} ankle={} ankleName=\"{}\" plantedRaw={:.4f} raisedRaw={:.4f} kneeAxis=({:+.3f},{:+.3f},{:+.3f})] source=FOS_DRIVER_INFO fosLockIntent={} phase=MOTUS_WORLD_VELOCITY footLock=DELEGATED_TO_FULLBODYIK_R003",
                actor->GetFormID(), c.legCount,
                c.legs[0].hipIndex, c.legs[0].kneeIndex, c.legs[0].ankleIndex, c.legs[0].ankleBone,
                c.legs[0].plantedHeight, c.legs[0].raisedHeight,
                c.legs[0].kneeAxis[0], c.legs[0].kneeAxis[1], c.legs[0].kneeAxis[2],
                c.lockFeetWhenPlanted);
        }

        auto logSkip = [&](SkipReason reason, const char* modeName, const char* profileName, float physicalSpeed, const char* extra) {
            const auto now = Clock::now();
            const auto code = static_cast<std::uint32_t>(reason);
            const bool changed = state->lastSkipReason != code;
            const bool elapsed = state->lastSkipLog == Clock::time_point{} ||
                std::chrono::duration<float>(now - state->lastSkipLog).count() >= kSkipTelemetryPeriod;
            if (!changed && !elapsed) return;
            state->lastSkipReason = code;
            state->lastSkipLog = now;
            SKSE::log::info(
                "[FormaMotus][STRIDE-SKIP] actor={:08X} source={} reason={} mode={} profile=\"{}\" legs={} physicalSpeed={:.2f} detail=\"{}\" footLock=CONTINUES_WHEN_POSSIBLE",
                actor->GetFormID(), sourceName, SkipReasonName(reason), modeName ? modeName : "UNKNOWN",
                profileName ? profileName : "", c.legCount, physicalSpeed, extra ? extra : "");
        };

        const auto* profile = UKClient::GetProfileForActor(actor);
        if (!profile) {
            logSkip(SkipReason::kUnsupportedLocomotionMode, "NO_UK_PROFILE", "", 0.0f, "UK profile unavailable");
            return;
        }

        const bool quadruped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped;
        const bool biped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped;
        if (!quadruped && !biped) {
            logSkip(SkipReason::kUnsupportedLocomotionMode, "OTHER", "", 0.0f, "R4.3 accepts grounded quadruped or biped FoS rigs");
            return;
        }
        const char* modeName = quadruped ? "QUADRUPED" : "BIPED";
        const float physicalSpeed = PhysicalPlanarSpeed(actor);

        // Foot Lock remains usable without a locomotion producer (important for generic FoS rigs),
        // but when a supported producer explicitly reports locomotion inactive that semantic
        // witness outranks potentially stale physical velocity. R4.1
        // could retain four locks while NO_LOCOMOTION + stale ~304 u/s was reported.
        const bool explicitNoLocomotion = haveSnapshot && supported && !locomotion;
        // R5: Player is no longer excluded. Animus now supplies the same completed-frame
        // locomotion semantics used to release locks safely; FoS remains final Z/terrain owner.
        const bool playerHasAnimusWitness = !actor->IsPlayerRef() ||
            (haveSnapshot && supported && source != MotionSourceClient::Source::kIncessus);
        const bool footLockAllowed = playerHasAnimusWitness &&
            physicalSpeed >= kFootLockMinActorSpeed && !explicitNoLocomotion;
        const bool forceLockRelease = pivot || action || !footLockAllowed;
        const float dt = std::clamp(
            haveSnapshot && std::isfinite(snapshot.steeringDeltaTime) && snapshot.steeringDeltaTime > 0.0f ?
                snapshot.steeringDeltaTime : 1.0f / 60.0f,
            1.0f / 240.0f, 0.05f);

        if (!RebuildModelPose(c)) return;

        // C02 remains before Stride/FoS. Only confirmed Incessus speed-regime
        // changes create COM drive; FullBodyIK then resolves the planted limbs.
        const bool bodyImpulseWrote = ApplyCreatureSpeedImpulse(
            actor, *profile, resolved, physicalSpeed, dt, *state, c);
        if (bodyImpulseWrote && !RebuildModelPose(c)) return;

        struct Observation {
            MotusContinuus::API::BoneKinematicSample sample{};
            WorldFrame worldFrame{};
            AuthoredPlantSample authoredPlant{};
            float preFosWorld[3]{ 0.0f, 0.0f, 0.0f };
            float boundaryDelta{ 0.0f };
            float stance{ 0.0f };
            bool motusValid{ false };
            bool currentValid{ false };
            bool poseWorldValid{ false };
            bool sampleAdvanced{ false };
        };
        std::array<Observation, FeetOfSkyrimBridge::kMaxLegs> observations{};
        std::uint64_t motusSerial = 0;
        int motusSamples = 0;
        int authoredPlantSamples = 0;
        int authoredPlantReady = 0;
        int stanceLegs = 0;
        int swingLegs = 0;
        int lockedLegs = 0;
        int lockAcquired = 0;
        int lockReleased = 0;

        for (int i = 0; i < c.legCount; ++i) {
            auto& obs = observations[i];
            auto& legState = state->legs[i];
            obs.stance = ResolveStanceWeight(
                actor, c.legs[i].ankleBone, legState, dt, motusSerial,
                obs.sample, obs.motusValid, obs.currentValid, obs.sampleAdvanced);
            if (obs.motusValid) ++motusSamples;
            obs.authoredPlant = ResolveAuthoredPlantSample(c, c.legs[i]);
            if (obs.authoredPlant.valid) {
                ++authoredPlantSamples;
                if (obs.authoredPlant.weight >= kAuthoredAcquireWeight) ++authoredPlantReady;
            }
            if (obs.stance >= kHardStanceBegin) ++stanceLegs;
            else if (obs.stance <= kHardSwingBegin) ++swingLegs;

            const int ankle = c.legs[i].ankleIndex;
            if (obs.currentValid && ankle >= 0 && ankle < c.poseCount &&
                BuildRootWorldFrame(obs.sample, obs.worldFrame) &&
                ModelPointToWorld(obs.worldFrame, t_model.pos[ankle].data(), obs.preFosWorld)) {
                obs.poseWorldValid = true;
                const float sampleWorld[3]{
                    obs.sample.currentWorld.translation.x,
                    obs.sample.currentWorld.translation.y,
                    obs.sample.currentWorld.translation.z };
                float boundary[3];
                Sub3(obs.preFosWorld, sampleWorld, boundary);
                obs.boundaryDelta = Length3(boundary);
                if (!std::isfinite(obs.boundaryDelta)) obs.boundaryDelta = 0.0f;
            }

            bool acquired = false, released = false;
            if (kFullBodyIKOwnsFootLock) {
                released = legState.locked;
                legState.locked = false;
                legState.lockStrength = 0.0f;
                legState.lockAge = 0.0f;
            } else {
                UpdateFootLock(legState, obs.sample, obs.motusValid, obs.poseWorldValid,
                    obs.preFosWorld, obs.stance, obs.authoredPlant, dt, obs.sampleAdvanced,
                    forceLockRelease, acquired, released);
            }
            if (acquired) { ++lockAcquired; ++state->lockAcquires; }
            if (released) { ++lockReleased; ++state->lockReleases; }
            if (legState.locked) ++lockedLegs;
        }

        bool strideEligible = true;
        SkipReason skipReason = SkipReason::kNone;
        const char* skipDetail = "";
        AuthoredReference authored{};
        float ratioRaw = 1.0f;
        float wantedScale = 1.0f;

        if (!haveSnapshot) {
            strideEligible = false; skipReason = SkipReason::kNoMotionSource;
            skipDetail = "FoS/Motus foot lock may still operate; Stride waits for Animus/Incessus locomotion snapshot";
        } else if (!supported || !locomotion) {
            strideEligible = false; skipReason = SkipReason::kNotLocomoting;
            skipDetail = supported ?
                "snapshot present but locomotion inactive; foot locks released regardless of stale physical velocity" :
                "snapshot does not mark actor supported; FoS/Motus foot lock remains physical-witness driven";
        } else if (pivot || action) {
            strideEligible = false; skipReason = SkipReason::kPivotOrAction;
            skipDetail = pivot ? "pivot busy; stride disabled and foot locks released" : "action bypass active; stride disabled and foot locks released";
        } else if (physicalSpeed < kStationarySpeed) {
            strideEligible = false; skipReason = SkipReason::kStationary;
            skipDetail = "stride neutralized; no stride writes while actor is effectively stationary";
        } else {
            authored = ResolveAuthoredReference(source, snapshot);
            if (!std::isfinite(authored.effectiveSpeed) || authored.effectiveSpeed < kMinAuthoredSpeed || authored.clips <= 0) {
                strideEligible = false; skipReason = SkipReason::kNoAuthoredReference;
                skipDetail = "MotionDB/runtime authored reference unavailable";
            }
        }

        if (!strideEligible) {
            state->strideScale += (1.0f - state->strideScale) * ExpAlpha(kStrideScaleResponse, dt);
            if (std::abs(state->strideScale - 1.0f) < 0.005f) state->strideScale = 1.0f;
            logSkip(skipReason, modeName, haveSnapshot ? snapshot.profile : "", physicalSpeed, skipDetail);
        } else {
            ratioRaw = physicalSpeed / authored.effectiveSpeed;
            wantedScale = std::clamp(ratioRaw, kMinStrideScale, kMaxStrideScale);
            if (!std::isfinite(wantedScale)) wantedScale = 1.0f;
            if (std::abs(wantedScale - 1.0f) < kStrideScaleDeadzone) wantedScale = 1.0f;
            state->lastSkipReason = 0;
            state->strideScale += (wantedScale - state->strideScale) * ExpAlpha(kStrideScaleResponse, dt);
            if (std::abs(state->strideScale - 1.0f) < 0.005f) state->strideScale = 1.0f;
        }

        const float anatomyGain = std::clamp(
            std::isfinite(profile->strideScale) && profile->strideScale > 0.0f ? profile->strideScale : 1.0f,
            0.60f, 1.40f);

        int strideWrites = 0;
        float maxAbsAngle = 0.0f;
        if (strideEligible) {
            for (int i = 0; i < c.legCount; ++i) {
                const float phaseGain = PhaseGainFromStance(observations[i].stance);
                if (phaseGain <= 0.001f) continue;  // hard stance = zero stride write
                float angle = 0.0f;
                if (ApplyHipSagittalWarp(c, c.legs[i], state->strideScale, phaseGain, anatomyGain, angle)) {
                    ++strideWrites;
                    maxAbsAngle = std::max(maxAbsAngle, std::abs(angle));
                    if (!RebuildModelPose(c)) break;
                }
            }
        }

        // R4.3: apply planar world lock after Stride. Swing arcs remain free while planted
        // limbs chase only their X/Y anchor; FoS receives the result and owns final Z/terrain.
        int lockWrites = 0;
        float maxBoundaryDelta = 0.0f;
        float maxLockWorldError = 0.0f;
        float maxLockResidual = 0.0f;
        float maxBreakDistance = 0.0f;
        float maxCorrection = 0.0f;
        float maxVerticalDrift = 0.0f;
        float maxLockStrength = 0.0f;
        for (int i = 0; !kFullBodyIKOwnsFootLock && i < c.legCount; ++i) {
            maxBoundaryDelta = std::max(maxBoundaryDelta, observations[i].boundaryDelta);
            auto& legState = state->legs[i];
            maxLockStrength = std::max(maxLockStrength, legState.lockStrength);
            if (!legState.locked) continue;
            float worldError = 0.0f, residual = 0.0f, breakDistance = 0.0f, maxLegCorrection = 0.0f, verticalDrift = 0.0f;
            const bool wasLocked = legState.locked;
            const bool ok = ApplyWorldFootLock(c, c.legs[i], legState, observations[i].worldFrame,
                worldError, residual, breakDistance, maxLegCorrection, verticalDrift);
            maxLockWorldError = std::max(maxLockWorldError, worldError);
            maxLockResidual = std::max(maxLockResidual, residual);
            maxBreakDistance = std::max(maxBreakDistance, breakDistance);
            maxCorrection = std::max(maxCorrection, maxLegCorrection);
            maxVerticalDrift = std::max(maxVerticalDrift, verticalDrift);
            if (ok && worldError >= kLockMinWorldError) ++lockWrites;
            if (wasLocked && !legState.locked) { ++lockReleased; ++state->lockReleases; --lockedLegs; }
        }

        state->lastSnapshotSerial = haveSnapshot ? snapshot.snapshotSerial : 0;
        state->lastMotusSerial = motusSerial;
        const auto now = Clock::now();
        const bool logNow = state->lastLog == Clock::time_point{} ||
            std::chrono::duration<float>(now - state->lastLog).count() >= kTelemetryPeriod;
        if (logNow && (strideWrites > 0 || lockWrites > 0 || std::abs(state->strideScale - 1.0f) >= kStrideScaleDeadzone)) {
            state->lastLog = now;
            SKSE::log::info(
                "[FormaMotus][LEG-WARP] actor={:08X} source={} profile=\"{}\" mode={} gait={} carrierMask=0x{:02X} physicalSpeed={:.2f} authored[motionDb={:.2f} runtimePace={:.2f} effective={:.2f} keys=\"{}\"] scale[ratioRaw={:.3f} target={:.3f} smooth={:.3f}] legs={} stance={} swing={} motusSamples={} fosPlant[samples={} ready={} lockIntent={}] strideWrites={} maxHipDeg={:.2f} footLock[mode=DELEGATED_TO_FULLBODYIK locked={} acquired={} released={} writes={} strength={:.3f} boundaryDelta={:.2f} maxPlanarError={:.2f} maxPlanarResidual={:.2f} breakAt={:.2f} maxCorrection={:.2f} maxZDrift={:.2f}] FoS=FINAL_Z_TERRAIN_IK",
                actor->GetFormID(), sourceName, haveSnapshot ? snapshot.profile : "", modeName,
                haveSnapshot ? static_cast<unsigned>(snapshot.desiredGait) : 0u,
                haveSnapshot ? static_cast<unsigned>(snapshot.runtimeCarrierMask) : 0u,
                physicalSpeed, authored.motionDbSpeed, authored.runtimePace, authored.effectiveSpeed, authored.keys,
                ratioRaw, wantedScale, state->strideScale, c.legCount, stanceLegs, swingLegs, motusSamples,
                authoredPlantSamples, authoredPlantReady, c.lockFeetWhenPlanted,
                strideWrites, maxAbsAngle / DEG2RAD, lockedLegs, lockAcquired, lockReleased, lockWrites,
                maxLockStrength, maxBoundaryDelta, maxLockWorldError, maxLockResidual, maxBreakDistance, maxCorrection, maxVerticalDrift);
        }

        const bool footLogNow = state->lastFootLockLog == Clock::time_point{} ||
            std::chrono::duration<float>(now - state->lastFootLockLog).count() >= kFootLockTelemetryPeriod;
        if (footLogNow && (lockedLegs > 0 || lockAcquired > 0 || lockReleased > 0)) {
            state->lastFootLockLog = now;
            SKSE::log::info(
                "[FormaMotus][FOOT-LOCK] actor={:08X} source={} mode={} legs={} locked={} acquired={} released={} writes={} motusSerial={} fosPlantSamples={} fosPlantReady={} fosLockIntent={} worldTarget=PRE_FOS_PLANAR_XY zAuthority=FEET_OF_SKYRIM phase=MOTUS_PRE_GRAPH_VELOCITY+FOS_AUTHORED_HEIGHT solve=HIP_KNEE_2BONE_PRE_FOS totals[acquire={} release={}]",
                actor->GetFormID(), sourceName, modeName, c.legCount, lockedLegs, lockAcquired, lockReleased,
                lockWrites, motusSerial, authoredPlantSamples, authoredPlantReady, c.lockFeetWhenPlanted,
                state->lockAcquires, state->lockReleases);
        }
    }

    void Clear()
    {
        std::unique_lock lock(g_statesMutex);
        g_states.clear();
    }
}

