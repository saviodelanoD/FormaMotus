#pragma once

namespace RE { class Actor; class BShkbAnimationGraph; }
namespace FeetOfSkyrimBridge { struct PoseContext; }

namespace SlopeWarpModule
{
    // Terrain SDK is a soft dependency: Forma Motus remains functional when the
    // provider is absent; only Slope Warp becomes inert.
    bool Initialize();
    bool IsAvailable();

    void ApplyPreFootIkPose(
        RE::Actor* actor,
        RE::BShkbAnimationGraph* graph,
        FeetOfSkyrimBridge::PoseContext& context);

    void Clear();
}

