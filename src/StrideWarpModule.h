#pragma once

namespace RE { class Actor; class BShkbAnimationGraph; }

namespace FeetOfSkyrimBridge { struct PoseContext; }

namespace StrideWarpModule
{
    void ApplyPreFootIkPose(
        RE::Actor* actor,
        RE::BShkbAnimationGraph* graph,
        FeetOfSkyrimBridge::PoseContext& context);
    void Clear();
}

