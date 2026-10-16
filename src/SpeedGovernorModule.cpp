#include "PCH.h"
#include "SpeedGovernorModule.h"

#include "RE/Skyrim.h"

#include <algorithm>
#include <cmath>

namespace SpeedGovernorModule
{
	namespace
	{
		constexpr float kReferenceSpeed = 310.0f;
	}

	void PreUpdate(RE::Actor*, float)
	{
	}

	void PostVanillaProbe(RE::Actor*, float)
	{
	}

	void Update(RE::Actor*)
	{
	}

	void ApplyPostGraphPose(RE::Actor*)
	{
	}

	float GetGovernedSpeed01(RE::Actor* actor)
	{
		if (!actor) {
			return 0.0f;
		}

		RE::NiPoint3 velocity{};
		actor->GetLinearVelocity(velocity);
		const float planarSpeed = std::sqrt(std::max(0.0f, velocity.x * velocity.x + velocity.y * velocity.y));
		return std::clamp(planarSpeed / kReferenceSpeed, 0.0f, 1.0f);
	}

	void Clear()
	{
	}
}
