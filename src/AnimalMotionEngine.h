#pragma once
#include <unordered_map>
#include <mutex>

namespace FeetOfSkyrimBridge { struct PoseContext; }

namespace AnimalMotionEngine {

    struct MotionData {
        float speed = 0.0f;
        float angularVelocity = 0.0f;
        float acceleration = 0.0f;
        float locomotionFactor = 0.0f;
        RE::NiPoint3 lastPosition;
        float lastYaw = 0.0f;
        bool initialized = false;
    };

    class MotionAnalyzer {
    public:
        void Update(RE::Actor* actor, MotionData& data);
    };

    class MovementIntentResolver {
    public:
        float ResolveBanking(const MotionData& data);
    };

    class SpineController {
    public:
        void Apply(RE::Actor* actor, float targetLean);
    };

    void Update(RE::Actor* actor);

    // R2 final axial owner for creatures: mutate the generated animation output
    // immediately before Feet of Skyrim consumes it. FoS remains final leg IK.
    void ApplyPreFootIkPose(
        RE::FormID actorID,
        RE::BShkbAnimationGraph* graph,
        FeetOfSkyrimBridge::PoseContext& context);

    // Retained for lineage/debug only. R2 Hooks no longer call this axial
    // HKB_UPDATE_POST writer, preventing double application after FoS.
    void ApplyHavokPose(RE::FormID actorID, RE::hkbCharacter* character);

    void Clear();
}

