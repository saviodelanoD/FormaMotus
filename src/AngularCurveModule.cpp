#include "PCH.h"
#include "AngularCurveModule.h"
#include "ActorDomain.h"
#include <UniversalKinematics.h>
#include "UKClient.h"


// R5.7 StableBank is non-human only, including Player-controlled mounts.

namespace SpeedGovernorModule
{
	float GetGovernedSpeed01(RE::Actor* actor);
}

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <initializer_list>
#include <memory>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace AngularCurveModule
{
	using Clock = std::chrono::steady_clock;

	enum class AnchorFamily
	{
		Biped,
		Quadruped,
		Hovering
	};

	struct AnchorPose
	{
		RE::NiAVObject* bone = nullptr;
		float factor = 1.0f;

		RE::NiMatrix3 lastBaseRotate{};
		RE::NiMatrix3 lastOutputRotate{};
		bool hasLastPose = false;
	};

	struct ActorMotionData
	{
		float lastYawDeg = 0.0f;
		float currentRoll = 0.0f;
		float smoothedYawRate = 0.0f;
		float smoothedPhysicalSpeed = 0.0f;
		float smoothedGovernedSpeed01 = 0.0f;
		int stableYawSign = 0;
		RE::NiPoint3 lastPosition{};
		bool initialized = false;
		bool playerMountRouteInitialized = false;
		bool lastPlayerControlledMount = false;

		Clock::time_point lastUpdateTime{};
		Clock::time_point lastTelemetryTime{};
		bool clockInitialized = false;

		RE::NiAVObject* lastRoot = nullptr;
		AnchorFamily lastFamily = AnchorFamily::Biped;

		AnchorPose rearAnchor;
		AnchorPose frontAnchor;
	};

	static std::unordered_map<RE::FormID, std::shared_ptr<ActorMotionData>> g_cache;
	static std::shared_mutex g_cacheMutex;

	static constexpr float SPEED_START = 18.0f;
	static constexpr float SPEED_FULL = 185.0f;

	static constexpr float MOVE_SPEED_GATE_START = 6.0f;
	static constexpr float MOVE_SPEED_GATE_FULL = 82.0f;

	// Base do calculo para velocidade
	static constexpr float MAX_BANK_WALK = 0.0029f;
	static constexpr float MAX_BANK_RUN = 0.0858f;

	static constexpr float YAW_RATE_FOR_FULL_BANK_WALK = 4.8f;
	static constexpr float YAW_RATE_FOR_FULL_BANK_RUN = 1.75f;

	static constexpr float YAW_SMOOTH_RESPONSE = 9.0f;
	static constexpr float SPEED_SMOOTH_RESPONSE = 10.0f;
	static constexpr float GOVERNED_SPEED_SMOOTH_RESPONSE = 8.0f;
	static constexpr float BANK_ACCEL_RESPONSE = 9.2f;
	static constexpr float BANK_DECEL_RESPONSE = 3.2f;

	// R5 anti-vibration filter. The bank amplitude is unchanged; only low-energy
	// yaw noise, one-frame sign flips and root-transport speed pulses are rejected.
	static constexpr float YAW_RATE_INPUT_DEADZONE = 0.035f;  // ~2.0 deg/s
	static constexpr float YAW_SIGN_ENTER = 0.060f;           // ~3.4 deg/s
	static constexpr float YAW_SIGN_RELEASE = 0.022f;         // ~1.3 deg/s
	static constexpr float MAX_OBSERVED_YAW_RATE = 8.0f;
	static constexpr float BANK_TELEMETRY_PERIOD = 1.0f;

	static constexpr float ROLL_EPSILON = 0.00012f;
	static constexpr float DEG2RAD = 3.14159265f / 180.0f;
	static constexpr float ROLL_SIGN = -1.0f;

	static inline float Saturate(float x)
	{
		return std::clamp(x, 0.0f, 1.0f);
	}

	static inline float Smoothstep(float edge0, float edge1, float x)
	{
		if (std::abs(edge1 - edge0) < 1e-6f) {
			return 0.0f;
		}
		float t = Saturate((x - edge0) / (edge1 - edge0));
		return t * t * (3.0f - 2.0f * t);
	}

	static inline float NormalizeAngleDeg(float angleDeg)
	{
		return std::remainder(angleDeg, 360.0f);
	}

	static inline char ToLowerASCII(char c)
	{
		return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}

	static std::string ToLowerCopy(std::string_view text)
	{
		std::string out;
		out.reserve(text.size());
		for (char c : text) {
			out.push_back(ToLowerASCII(c));
		}
		return out;
	}

	static bool ContainsCaseInsensitive(const char* haystack, std::string_view needleLower)
	{
		if (!haystack || needleLower.empty()) {
			return false;
		}

		const std::size_t n = needleLower.size();
		for (std::size_t i = 0; haystack[i] != '\0'; ++i) {
			std::size_t j = 0;
			while (j < n &&
			       haystack[i + j] != '\0' &&
			       ToLowerASCII(haystack[i + j]) == needleLower[j]) {
				++j;
			}
			if (j == n) {
				return true;
			}
		}
		return false;
	}

	static bool IsExcludedAnchorName(const std::string& name)
	{
		if (name.empty()) {
			return true;
		}
		const std::string lower = ToLowerCopy(name);
		return lower.find("root") != std::string::npos ||
		       lower.find("scull") != std::string::npos ||
		       lower.find("skull") != std::string::npos;
	}

	static RE::NiAVObject* FindBoneNodeFast(RE::NiAVObject* root, std::string_view needleLower)
	{
		if (!root) {
			return nullptr;
		}

		std::vector<RE::NiAVObject*> stack;
		stack.reserve(128);
		stack.push_back(root);

		while (!stack.empty()) {
			RE::NiAVObject* cur = stack.back();
			stack.pop_back();

			const char* nm = cur ? cur->name.c_str() : nullptr;
			if (nm && ContainsCaseInsensitive(nm, needleLower)) {
				return cur;
			}

			if (auto asNode = cur->AsNode(); asNode) {
				for (auto& child : asNode->GetChildren()) {
					if (child) {
						stack.push_back(child.get());
					}
				}
			}
		}

		return nullptr;
	}

	static RE::NiAVObject* FindBoneFromProfileName(RE::NiAVObject* root, const std::string& name)
	{
		if (!root || name.empty()) {
			return nullptr;
		}

		if (auto* exact = root->GetObjectByName(name)) {
			return exact;
		}

		return FindBoneNodeFast(root, ToLowerCopy(name));
	}

	static RE::NiAVObject* FindBoneByHints(RE::NiAVObject* root, std::initializer_list<std::string_view> hints)
	{
		if (!root) {
			return nullptr;
		}

		for (const auto& hint : hints) {
			if (hint.empty()) {
				continue;
			}
			if (auto* bone = FindBoneNodeFast(root, hint)) {
				return bone;
			}
		}
		return nullptr;
	}

	static bool MatrixNearlyEqual(const RE::NiMatrix3& a, const RE::NiMatrix3& b, float epsilon = 0.0001f)
	{
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (std::abs(a.entry[i][j] - b.entry[i][j]) > epsilon) {
					return false;
				}
			}
		}
		return true;
	}

	static float ComputeDeltaTime(ActorMotionData& data)
	{
		const auto now = Clock::now();

		if (!data.clockInitialized) {
			data.lastUpdateTime = now;
			data.clockInitialized = true;
			return 1.0f / 60.0f;
		}

		const float dt = std::chrono::duration<float>(now - data.lastUpdateTime).count();
		data.lastUpdateTime = now;
		return std::clamp(dt, 1.0f / 240.0f, 0.10f);
	}

	static std::shared_ptr<ActorMotionData> GetOrCreate(RE::FormID id)
	{
		{
			std::shared_lock lock(g_cacheMutex);
			auto it = g_cache.find(id);
			if (it != g_cache.end()) {
				return it->second;
			}
		}

		std::unique_lock lock(g_cacheMutex);
		auto& slot = g_cache[id];
		if (!slot) {
			slot = std::make_shared<ActorMotionData>();
		}
		return slot;
	}

	static void ResetAnchors(ActorMotionData& data)
	{
		data.lastRoot = nullptr;
		data.rearAnchor = {};
		data.frontAnchor = {};
	}

	static void ResetMotionState(ActorMotionData& data)
	{
		ResetAnchors(data);
		data.currentRoll = 0.0f;
		data.smoothedYawRate = 0.0f;
		data.smoothedPhysicalSpeed = 0.0f;
		data.smoothedGovernedSpeed01 = 0.0f;
		data.stableYawSign = 0;
		data.lastTelemetryTime = {};
		data.initialized = false;
		data.clockInitialized = false;
	}

	static void ResetCacheForActor(RE::Actor* actor)
	{
		if (!actor) {
			return;
		}
		std::shared_ptr<ActorMotionData> cache;
		{
			std::shared_lock lock(g_cacheMutex);
			auto it = g_cache.find(actor->GetFormID());
			if (it == g_cache.end()) {
				return;
			}
			cache = it->second;
		}
		if (cache) {
			ResetMotionState(*cache);
		}
	}

	static bool ReadGraphBool(RE::Actor* actor, const char* name)
	{
		bool value = false;
		if (actor && name && name[0]) {
			actor->GetGraphVariableBool(name, value);
		}
		return value;
	}

	static bool ReadGraphIntNonZero(RE::Actor* actor, const char* name)
	{
		int value = 0;
		if (actor && name && name[0]) {
			actor->GetGraphVariableInt(name, value);
		}
		return value != 0;
	}

	static bool AnyGraphBoolTrue(RE::Actor* actor, std::initializer_list<const char*> names)
	{
		for (const auto* name : names) {
			if (ReadGraphBool(actor, name)) {
				return true;
			}
		}
		return false;
	}

	static bool AnyGraphIntNonZero(RE::Actor* actor, std::initializer_list<const char*> names)
	{
		for (const auto* name : names) {
			if (ReadGraphIntNonZero(actor, name)) {
				return true;
			}
		}
		return false;
	}

	static bool IsPlayerControlledMount(RE::Actor* actor)
	{
		if (!actor) return false;
		RE::NiPointer<RE::Actor> rider;
		return actor->GetMountedBy(rider) && rider && rider->IsPlayerRef();
	}

	static bool IsInBlockedAnimationState(RE::Actor* actor, bool playerControlledMount)
	{
		if (!actor) {
			return true;
		}

		// Hard gates remain universal. Mount-state graph variables are ignored only
		// for the mount actor that is actually ridden by the Player; otherwise they
		// would incorrectly quarantine the very route we need to stabilize.
		if (AnyGraphBoolTrue(actor, {
				"bInRagdollState", "bIsRagdoll", "IsRagdoll", "bRagdoll",
				"bInKillMove", "bIsInKillMove", "IsInKillMove", "bKillMove",
				"bInFurniture", "bIsInFurniture", "IsInFurniture", "bFurniture",
				"bInWater", "bIsSwimming", "IsSwimming", "bSwimming",
				"bInJumpState", "bInAir", "bMidair", "IsInMidair"
			})) {
			return true;
		}

		if (!playerControlledMount && AnyGraphBoolTrue(actor, {
				"bIsRiding", "bOnMount", "bMounted", "IsOnMount"
			})) {
			return true;
		}

		if (AnyGraphIntNonZero(actor, {
				"iSitState", "iFurnitureState", "iRagdollState", "iKillMove"
			})) {
			return true;
		}

		if (!playerControlledMount && ReadGraphIntNonZero(actor, "iMountState")) {
			return true;
		}

		return false;
	}

	static bool IsActorEligible(RE::Actor* actor, bool playerControlledMount)
	{
		if (!actor || actor->IsDead() || !actor->Is3DLoaded()) {
			return false;
		}
		if (actor->Get3D() == nullptr) {
			return false;
		}
		return !IsInBlockedAnimationState(actor, playerControlledMount);
	}

	static AnchorFamily GetAnchorFamily(const UniversalKinematics::SkeletonProfile& profile)
	{
		switch (profile.locomotionMode) {
		case UniversalKinematics::LocomotionMode::Hovering:
		case UniversalKinematics::LocomotionMode::FlyingOnly:
		case UniversalKinematics::LocomotionMode::FlyingGroundCapable:
			return AnchorFamily::Hovering;
		case UniversalKinematics::LocomotionMode::GroundedQuadruped:
		case UniversalKinematics::LocomotionMode::GroundedCrawler:
		case UniversalKinematics::LocomotionMode::GroundedRadial:
			return AnchorFamily::Quadruped;
		default:
			return AnchorFamily::Biped;
		}
	}
 
	static bool UsesStableNonHumanBipedYawWitness(
		RE::Actor* actor,
		const UniversalKinematics::SkeletonProfile& profile)
	{
		return ActorDomain::IsFormaNonHuman(actor) &&
		       profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped &&
		       profile.isHumanoidLikeUpperBody;
	}

	static RE::NiAVObject* FindRearAnchorBiped(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.pelvisNode)) {
			if (auto* pelvis = FindBoneFromProfileName(root, profile.pelvisNode)) {
				return pelvis;
			}
		}
		return FindBoneByHints(root, { "pelvis", "hip", "npc pelvis", "bip01 pelvis" });
	}

	static RE::NiAVObject* FindFrontAnchorBiped(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.chestNode)) {
			if (auto* chest = FindBoneFromProfileName(root, profile.chestNode)) {
				return chest;
			}
		}

		for (auto it = profile.spineNodes.rbegin(); it != profile.spineNodes.rend(); ++it) {
			if (IsExcludedAnchorName(*it) || *it == profile.pelvisNode || *it == profile.headNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, *it)) {
				return spine;
			}
		}

		if (!IsExcludedAnchorName(profile.neckNode)) {
			if (auto* neck = FindBoneFromProfileName(root, profile.neckNode)) {
				return neck;
			}
		}

		return FindBoneByHints(root, { "spine2", "spine1", "chest", "ribcage", "torso", "neck" });
	}

	static RE::NiAVObject* FindRearAnchorQuadruped(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.pelvisNode)) {
			if (auto* pelvis = FindBoneFromProfileName(root, profile.pelvisNode)) {
				return pelvis;
			}
		}

		for (const auto& node : profile.spineNodes) {
			if (IsExcludedAnchorName(node) || node == profile.headNode || node == profile.neckNode || node == profile.chestNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, node)) {
				return spine;
			}
		}

		return FindBoneByHints(root, { "pelvis", "hip", "haunch", "spine1", "spine", "hind" });
	}

	static RE::NiAVObject* FindFrontAnchorQuadruped(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.chestNode)) {
			if (auto* chest = FindBoneFromProfileName(root, profile.chestNode)) {
				return chest;
			}
		}

		for (const auto& node : profile.spineNodes) {
			if (IsExcludedAnchorName(node) || node == profile.pelvisNode || node == profile.headNode || node == profile.neckNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, node)) {
				const char* nm = spine->name.c_str();
				if (nm && (ContainsCaseInsensitive(nm, "rib") ||
				           ContainsCaseInsensitive(nm, "chest") ||
				           ContainsCaseInsensitive(nm, "withers") ||
				           ContainsCaseInsensitive(nm, "spine4") ||
				           ContainsCaseInsensitive(nm, "spine3"))) {
					return spine;
				}
			}
		}

		if (auto* alt = FindBoneByHints(root, { "ribcage", "chest", "withers", "spine4", "spine3", "shoulder" })) {
			return alt;
		}

		for (const auto& node : profile.spineNodes) {
			if (IsExcludedAnchorName(node) || node == profile.pelvisNode || node == profile.headNode || node == profile.neckNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, node)) {
				return spine;
			}
		}

		return nullptr;
	}

	static RE::NiAVObject* FindRearAnchorHovering(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.pelvisNode)) {
			if (auto* pelvis = FindBoneFromProfileName(root, profile.pelvisNode)) {
				return pelvis;
			}
		}
		for (const auto& node : profile.spineNodes) {
			if (IsExcludedAnchorName(node) || node == profile.headNode || node == profile.neckNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, node)) {
				return spine;
			}
		}
		return FindBoneByHints(root, { "spine1", "spine", "core", "body", "center" });
	}

	static RE::NiAVObject* FindFrontAnchorHovering(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!IsExcludedAnchorName(profile.chestNode)) {
			if (auto* chest = FindBoneFromProfileName(root, profile.chestNode)) {
				return chest;
			}
		}
		if (!IsExcludedAnchorName(profile.neckNode)) {
			if (auto* neck = FindBoneFromProfileName(root, profile.neckNode)) {
				return neck;
			}
		}
		for (auto it = profile.spineNodes.rbegin(); it != profile.spineNodes.rend(); ++it) {
			if (IsExcludedAnchorName(*it) || *it == profile.pelvisNode || *it == profile.headNode) {
				continue;
			}
			if (auto* spine = FindBoneFromProfileName(root, *it)) {
				return spine;
			}
		}
		return FindBoneByHints(root, { "neck", "head", "front", "spine3", "spine2" });
	}

	static void RefreshAnchors(
		ActorMotionData& data,
		RE::NiAVObject* root,
		const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!root) {
			ResetAnchors(data);
			return;
		}

		const AnchorFamily family = GetAnchorFamily(profile);
		if (data.lastRoot == root &&
		    data.lastFamily == family &&
		    (data.rearAnchor.bone || data.frontAnchor.bone)) {
			return;
		}

		data.lastRoot = root;
		data.lastFamily = family;
		data.rearAnchor = {};
		data.frontAnchor = {};

		switch (family) {
		case AnchorFamily::Biped:
			// R5 ownership fix: AME already writes the non-human biped spine/chest Ni chain.
			// Banking the same chest bone gave two independent base-pose trackers
			// ownership of one transform and could produce visible vibration. Pelvis is
			// outside the AME biped chain and hierarchically carries the lean upstream.
			data.rearAnchor.bone = FindRearAnchorBiped(root, profile);
			data.frontAnchor.bone = nullptr;
			break;
		case AnchorFamily::Quadruped:
			data.rearAnchor.bone = FindRearAnchorQuadruped(root, profile);
			data.frontAnchor.bone = FindFrontAnchorQuadruped(root, profile);
			break;
		case AnchorFamily::Hovering:
			data.rearAnchor.bone = FindRearAnchorHovering(root, profile);
			data.frontAnchor.bone = FindFrontAnchorHovering(root, profile);
			break;
		}

		if (data.rearAnchor.bone && data.frontAnchor.bone && data.rearAnchor.bone != data.frontAnchor.bone) {
			switch (family) {
			case AnchorFamily::Biped:
				data.rearAnchor.factor = 0.46f;
				data.frontAnchor.factor = 0.52f;
				break;
			case AnchorFamily::Quadruped:
				data.rearAnchor.factor = 0.40f;
				data.frontAnchor.factor = 0.58f;
				break;
			case AnchorFamily::Hovering:
				data.rearAnchor.factor = 0.44f;
				data.frontAnchor.factor = 0.56f;
				break;
			}
		} else if (data.rearAnchor.bone) {
			// Preserve the previous two-anchor biped visual amplitude (~0.46+0.52)
			// while giving the roll one unambiguous Ni owner.
			data.rearAnchor.factor = family == AnchorFamily::Biped ? 0.98f : 0.82f;
		} else if (data.frontAnchor.bone) {
			data.frontAnchor.factor = 0.82f;
		}
	}

	static float ComputeBankScale(const UniversalKinematics::SkeletonProfile& profile, AnchorFamily family)
	{
		float bankFactor = profile.bankFactor;
		if (bankFactor <= 0.0f) {
			bankFactor = 1.0f;
		}

		float morphScale = 1.0f;
		if (profile.bodyWidth > 0.0f) {
			const float aspect = profile.bodyLength / std::max(profile.bodyWidth, 1.0f);
			morphScale = std::clamp(0.87f + (aspect - 1.0f) * 0.07f, 0.82f, 1.12f);
		}

		float familyScale = 1.0f;
		switch (family) {
		case AnchorFamily::Biped:
			familyScale = 1.35f; // ADJUSTMENT: Aumentado em 35% exclusivamente para B�pedes
			break;
		case AnchorFamily::Quadruped:
			familyScale = 0.92f;
			break;
		case AnchorFamily::Hovering:
			familyScale = 0.88f;
			break;
		}

		return std::clamp(bankFactor * morphScale * familyScale, 0.0f, 1.50f);
	}

	static void ApplyToAnchor(AnchorPose& anchor, float rollAmount)
	{
		if (!anchor.bone) {
			return;
		}

		RE::NiMatrix3 baseRotate = anchor.bone->local.rotate;
		if (anchor.hasLastPose && MatrixNearlyEqual(baseRotate, anchor.lastOutputRotate)) {
			baseRotate = anchor.lastBaseRotate;
		}
		anchor.lastBaseRotate = baseRotate;

		if (rollAmount == 0.0f) {
			anchor.bone->local.rotate = baseRotate;
			anchor.hasLastPose = false;
			return;
		}

		RE::NiMatrix3 bankMatrix;
		bankMatrix.SetEulerAnglesXYZ(0.0f, 0.0f, rollAmount);

		anchor.bone->local.rotate = baseRotate * bankMatrix;
		anchor.lastOutputRotate = anchor.bone->local.rotate;
		anchor.hasLastPose = true;
	}

	static void ApplyRollToAnchors(RE::Actor* actor, ActorMotionData& data, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!actor || actor->IsDead()) {
			ResetAnchors(data);
			return;
		}

		auto* root = actor->Get3D();
		if (!root) {
			ResetAnchors(data);
			return;
		}

		RefreshAnchors(data, root, profile);
		if (!data.rearAnchor.bone && !data.frontAnchor.bone) {
			return;
		}

		if (data.rearAnchor.bone) {
			ApplyToAnchor(data.rearAnchor, data.currentRoll * data.rearAnchor.factor);
		}
		if (data.frontAnchor.bone && data.frontAnchor.bone != data.rearAnchor.bone) {
			ApplyToAnchor(data.frontAnchor, data.currentRoll * data.frontAnchor.factor);
		}
	}

	void UpdateActor(RE::Actor* actor)
	{
		if (!ActorDomain::IsFormaNonHuman(actor)) {
			ResetCacheForActor(actor);
			return;
		}

		const bool playerControlledMount = IsPlayerControlledMount(actor);
		if (!IsActorEligible(actor, playerControlledMount)) {
			ResetCacheForActor(actor);
			return;
		}

		const auto* profile = UKClient::GetProfileForActor(actor);
		if (!profile) {
			ResetCacheForActor(actor);
			return;
		}

		auto cache = GetOrCreate(actor->GetFormID());
		if (!cache) {
			return;
		}

		const float dt = ComputeDeltaTime(*cache);
		const AnchorFamily family = GetAnchorFamily(*profile);

		float graphSpeed = 0.0f;
		actor->GetGraphVariableFloat("Speed", graphSpeed);

		const RE::NiPoint3 currentPos = actor->GetPosition();
		const float currentYawDeg = actor->GetAngleZ() * (180.0f / 3.14159265f);

		float engineYawRate = 0.0f;
		auto* process = actor->GetActorRuntimeData().currentProcess;
		if (process && process->middleHigh) {
			engineYawRate = process->middleHigh->rotationSpeed.z;
		}

		if (!cache->initialized) {
			cache->lastYawDeg = currentYawDeg;
			cache->lastPosition = currentPos;
			cache->currentRoll = 0.0f;
			cache->smoothedYawRate = 0.0f;
			cache->smoothedPhysicalSpeed = 0.0f;
			cache->smoothedGovernedSpeed01 = 0.0f;
			cache->stableYawSign = 0;
			cache->playerMountRouteInitialized = true;
			cache->lastPlayerControlledMount = playerControlledMount;
			cache->initialized = true;
			return;
		}

		if (!cache->playerMountRouteInitialized ||
			cache->lastPlayerControlledMount != playerControlledMount) {
			cache->playerMountRouteInitialized = true;
			cache->lastPlayerControlledMount = playerControlledMount;
			cache->lastYawDeg = currentYawDeg;
			cache->lastPosition = currentPos;
			cache->smoothedYawRate = 0.0f;
			cache->stableYawSign = 0;
			cache->lastTelemetryTime = {};
			SKSE::log::info(
				"[FormaMotus][MOUNT-BANK-ROUTE] actor={:08X} route={} yawWitness={}",
				actor->GetFormID(), playerControlledMount ? "PLAYER_CONTROLLED_MOUNT" : "CREATURE_DEFAULT",
				playerControlledMount ? "ACTOR_WORLD_DELTA" : "PROFILE_DEFAULT");
		}

		const float distance = currentPos.GetDistance(cache->lastPosition);
		const float actualSpeed = distance / std::max(dt, 1e-4f);

		const float deltaYawDeg = NormalizeAngleDeg(currentYawDeg - cache->lastYawDeg);
		const float deltaYawRateRad = (deltaYawDeg * DEG2RAD) / std::max(dt, 1e-4f);

		cache->lastYawDeg = currentYawDeg;
		cache->lastPosition = currentPos;

		const bool stableBipedWorldYaw = UsesStableNonHumanBipedYawWitness(actor, *profile);
		const bool stableWorldYaw = stableBipedWorldYaw || playerControlledMount;

		// Player-controlled mounts and selected non-human bipeds use completed world-yaw
		// delta. Other creatures retain the historical engine rotation witness.
		float rawYawRate = stableWorldYaw ? deltaYawRateRad :
			((std::abs(engineYawRate) > 0.0001f) ? engineYawRate : deltaYawRateRad);
		if (!std::isfinite(rawYawRate)) rawYawRate = 0.0f;
		rawYawRate = std::clamp(rawYawRate, -MAX_OBSERVED_YAW_RATE, MAX_OBSERVED_YAW_RATE);
		if (std::abs(rawYawRate) < YAW_RATE_INPUT_DEADZONE) rawYawRate = 0.0f;

		const float yawAlpha = 1.0f - std::exp(-YAW_SMOOTH_RESPONSE * dt);
		cache->smoothedYawRate += (rawYawRate - cache->smoothedYawRate) * yawAlpha;
		if (std::abs(cache->smoothedYawRate) < YAW_SIGN_RELEASE * 0.50f) {
			cache->smoothedYawRate = 0.0f;
		}

		// Schmitt trigger for turn direction: a microscopic + / - oscillation may
		// reduce the bank toward neutral, but cannot flip the body to the other side
		// until a real opposite turn crosses YAW_SIGN_ENTER.
		if (cache->stableYawSign == 0) {
			if (cache->smoothedYawRate >= YAW_SIGN_ENTER) cache->stableYawSign = 1;
			else if (cache->smoothedYawRate <= -YAW_SIGN_ENTER) cache->stableYawSign = -1;
		} else if (cache->stableYawSign > 0) {
			if (cache->smoothedYawRate <= -YAW_SIGN_ENTER) cache->stableYawSign = -1;
			else if (std::abs(cache->smoothedYawRate) <= YAW_SIGN_RELEASE) cache->stableYawSign = 0;
		} else {
			if (cache->smoothedYawRate >= YAW_SIGN_ENTER) cache->stableYawSign = 1;
			else if (std::abs(cache->smoothedYawRate) <= YAW_SIGN_RELEASE) cache->stableYawSign = 0;
		}

		const float speedAlpha = 1.0f - std::exp(-SPEED_SMOOTH_RESPONSE * dt);
		cache->smoothedPhysicalSpeed += (actualSpeed - cache->smoothedPhysicalSpeed) * speedAlpha;
		const float rawGovernedSpeed01 = SpeedGovernorModule::GetGovernedSpeed01(actor);
		const float governedAlpha = 1.0f - std::exp(-GOVERNED_SPEED_SMOOTH_RESPONSE * dt);
		cache->smoothedGovernedSpeed01 +=
			(rawGovernedSpeed01 - cache->smoothedGovernedSpeed01) * governedAlpha;

		const float effectiveSpeed = std::max(graphSpeed, cache->smoothedPhysicalSpeed);
		const float speed01 = Smoothstep(SPEED_START, SPEED_FULL, effectiveSpeed);
		const float moveGate = Smoothstep(MOVE_SPEED_GATE_START, MOVE_SPEED_GATE_FULL, cache->smoothedPhysicalSpeed);
		const float governedResponse01 = Smoothstep(0.10f, 0.72f, cache->smoothedGovernedSpeed01);
		const float bankScale = ComputeBankScale(*profile, family);

		const bool passSpeedGate = stableWorldYaw ? (speed01 > 0.0001f && moveGate > 0.0001f) : true;

		float targetRoll = 0.0f;
		if (passSpeedGate && bankScale > 0.0001f) {
			const float maxBank = std::clamp(std::lerp(MAX_BANK_WALK, MAX_BANK_RUN, speed01) * bankScale, 0.0f, 0.18f);
			const float yawRateForFull = std::lerp(YAW_RATE_FOR_FULL_BANK_WALK, YAW_RATE_FOR_FULL_BANK_RUN, speed01);
			const float stableYawMagnitude = cache->stableYawSign == 0 ? 0.0f : std::abs(cache->smoothedYawRate);
			const float yaw01 = Saturate(stableYawMagnitude / std::max(yawRateForFull, 0.001f));
			const float yawCurve = yaw01 * yaw01 * (3.0f - 2.0f * yaw01);
			const float turnSign = static_cast<float>(cache->stableYawSign);

			float speedWeight = 1.0f;
			float moveWeight = 1.0f;

			if (stableWorldYaw) {
				speedWeight = std::lerp(0.18f, 1.0f, speed01) * speed01;
				moveWeight = std::lerp(0.55f, 1.0f, moveGate);
			}

			switch (family) {
			case AnchorFamily::Biped:
				break;
			case AnchorFamily::Quadruped:
				speedWeight *= 0.96f;
				moveWeight *= 1.02f;
				break;
			case AnchorFamily::Hovering:
				speedWeight *= 0.90f;
				moveWeight *= 0.98f;
				break;
			}

			targetRoll = turnSign * (maxBank * yawCurve);
			targetRoll *= ROLL_SIGN;
			targetRoll *= moveWeight;
			targetRoll *= speedWeight;
			targetRoll *= governedResponse01;
		}

		const auto telemetryNow = Clock::now();
		if (stableWorldYaw && (cache->lastTelemetryTime == Clock::time_point{} ||
			std::chrono::duration<float>(telemetryNow - cache->lastTelemetryTime).count() >= BANK_TELEMETRY_PERIOD)) {
			cache->lastTelemetryTime = telemetryNow;
			const char* role = playerControlledMount ? "PLAYER_MOUNT" : "NONHUMAN_BIPED";
			SKSE::log::info(
				"[FormaMotus][BANK-STABLE] actor={:08X} role={} stableYawWitness=ACTOR_WORLD_DELTA yaw[deltaRate={:+.3f} engine={:+.3f} raw={:+.3f} smooth={:+.3f} sign={}] speed[raw={:.2f} smooth={:.2f} governedRaw={:.3f} governedSmooth={:.3f}] gate[pass={}] roll[targetDeg={:+.3f} currentDeg={:+.3f}]",
				actor->GetFormID(), role, deltaYawRateRad, engineYawRate, rawYawRate, cache->smoothedYawRate,
				cache->stableYawSign, actualSpeed, cache->smoothedPhysicalSpeed,
				rawGovernedSpeed01, cache->smoothedGovernedSpeed01, passSpeedGate,
				targetRoll / DEG2RAD, cache->currentRoll / DEG2RAD);
		}

		const float response = (std::abs(targetRoll) > std::abs(cache->currentRoll)) ? BANK_ACCEL_RESPONSE : BANK_DECEL_RESPONSE;
		const float alpha = 1.0f - std::exp(-response * dt);
		cache->currentRoll += (targetRoll - cache->currentRoll) * alpha;

		if (std::abs(cache->currentRoll) < ROLL_EPSILON) {
			cache->currentRoll = 0.0f;
		}

		if (cache->currentRoll == 0.0f &&
			!cache->rearAnchor.hasLastPose &&
			!cache->frontAnchor.hasLastPose) {
			return;
		}

		ApplyRollToAnchors(actor, *cache, *profile);
	}

	void ClearCache()
	{
		std::unique_lock lock(g_cacheMutex);
		g_cache.clear();
	}
}
