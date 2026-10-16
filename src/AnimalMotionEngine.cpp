#include "SpiderAbdomenPolicy.h"
#include "AxialPosePolicy.h"
#include "PCH.h"
#include "AnimalMotionEngine.h"
#include "ActorDomain.h"
#include <UniversalKinematics.h>
#include "UKClient.h"
#include "MotionSourceClient.h"
#include "FeetOfSkyrimBridge.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BShkbAnimationGraph.h"
#include "RE/H/hkbCharacter.h"
#include "RE/H/hkQsTransform.h"
#include "RE/N/NiNode.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <initializer_list>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace AnimalMotionEngine
{
	using Clock = std::chrono::steady_clock;

	struct BoneEntry
	{
		RE::NiAVObject* bone = nullptr;
		float weight = 0.0f;

		RE::NiMatrix3 lastBaseRotate{};
		RE::NiMatrix3 lastOutputRotate{};
		bool hasLastPose = false;
	};

	struct HavokQuat
	{
		float x = 0.0f;
		float y = 0.0f;
		float z = 0.0f;
		float w = 1.0f;
	};

	struct HavokPoseBinding
	{
		std::vector<std::int32_t> bodyIndices;
		std::vector<std::int32_t> tailIndices;
		bool bodyComplete = false;
		bool successLogged = false;
	};

	struct ActorCache
	{
		MotionData motion;
		float currentFlex = 0.0f;
		float flexVelocity = 0.0f;
		float currentSpeedFlex = 0.0f;
		float currentAngularVelocity = 0.0f;

		// Cauda procedural separada da coluna: contrapeso leve e atrasado.
		float currentTailFlex = 0.0f;

		// Estado da curva dos quadr�pedes. A li��o do ALR � que actor yaw e
		// bhkCharacterController::forwardVec s�o camadas distintas: o engine pode
		// alterar uma antes/sem alterar a outra. O AME observa ambas, prefere a
		// c�psula quando ela cont�m um giro �til e usa actor yaw somente quando o
		// controller est� ausente/estagnado. As duas camadas NUNCA s�o somadas.
		float lastCapsuleYawRad = 0.0f;
		float lastActorYawRad = 0.0f;
		float compositeTurnAngleRad = 0.0f;
		float capsuleTurnIdleTimer = 0.0f;
		bool capsuleYawInitialized = false;
		bool actorYawInitialized = false;
		bool capsuleTurnActive = false;
		bool capsuleControllerMissingLogged = false;

		// R5.5: mounted horses controlled by the Player use a different rotation
		// authority from autonomous quadrupeds. This bit tracks route transitions so
		// accumulated NPC-turn state never leaks across mount/dismount.
		bool playerControlledMountRoute = false;

		// Sinal anat�mico da flex�o por sentido longitudinal f�sico. A r� preserva
		// o sinal hist�rico (-1); o avan�o usa o sinal oposto (+1). Perto de zero
		// mantemos o �ltimo estado v�lido para impedir chatter durante parada/piv�.
		float quadrupedTravelFlexSign = 1.0f;
		bool quadrupedTravelSignInitialized = false;
		float quadrupedTravelPendingSign = 0.0f;
		float quadrupedTravelPendingTimer = 0.0f;

		Clock::time_point lastUpdateTime{};
		bool clockInitialized = false;

		std::vector<BoneEntry> spineAndNeckChain;
		std::vector<BoneEntry> tailChain;

		// V18: a pose axial de criaturas deixa de ser escrita diretamente em NiAVObject.
		// O frame normal calcula os deltas locais e resolve os �ndices do
		// BShkbAnimationGraph; o callback Havok aplica esses deltas ao poseLocal.
		std::vector<HavokQuat> havokBodyDeltas;
		std::vector<HavokQuat> havokTailDeltas;
		std::unordered_map<const RE::hkbCharacter*, HavokPoseBinding> havokBindings;
		float havokFlexSign = 1.0f;
		bool havokPosePrepared = false;
		bool debugHavokBindingLogged = false;
		bool debugHavokApplyLogged = false;
		bool debugHavokMissingLogged = false;

		RE::NiAVObject* lastRoot = nullptr;

		float lastObservedYawDeg = 0.0f;
		bool yawObserved = false;
		AxialPosePolicy::Source axialPoseSource = AxialPosePolicy::Source::Neutral;
		// Physical turn continuity no longer depends on clip-witness edges.

		float diagnosticTimer = 0.0f;
		int diagnosticSamples = 0;
		bool loggedDebug = false;

		// V8 pipeline debug: todos estes campos existem somente para separar
		// acquisition/gating/chain/write/overwrite. Nenhum entra na formula natural.
		std::uint64_t debugUpdateCalls = 0;
		std::uint64_t debugApplyCalls = 0;
		std::uint64_t debugBoneWrites = 0;
		std::uint64_t debugBoneOverwrites = 0;
		float debugHeartbeatTimer = 0.0f;
		float debugSecondsSeen = 0.0f;
		float debugSyntheticTimer = 0.0f;
		float debugSyntheticFlexRad = 0.0f;
		bool debugSyntheticStarted = false;
		bool debugSyntheticFinished = false;
		bool debugNaturalTurnEverSeen = false;
		bool debugProfileLogged = false;
		bool debugChainLogged = false;
		bool debugTailChainLogged = false;

		Clock::time_point debugLastGateLog{};
		Clock::time_point lastBipedTelemetryTime{};
		Clock::time_point lastPlayerMountTelemetryTime{};

		// Protege todos os campos mut�veis deste cache, especialmente o vector
		// de ponteiros de ossos, contra Update/Reset simult�neos em job threads.
		std::mutex mutex;
	};

	static std::unordered_map<RE::FormID, std::shared_ptr<ActorCache>> g_cache;
	static std::shared_mutex g_cacheMutex;

	static constexpr float PI = 3.14159265f;
	static constexpr float DEG2RAD = PI / 180.0f;
	static constexpr float RAD2DEG = 180.0f / PI;

	// ============================================================
	// BUILD: FORMA-MOTUS-R5.7-NONHUMAN-ONLY-2026-09-26
	// ============================================================
	// V18 corrige a fronteira anat�mica revelada pelo Horker:
	// criaturas axiais n�o recebem mais a curvatura como p�s-escrita NiAVObject.
	// Spine/Neck/Tail s�o preparados no frame normal e aplicados ao hkbCharacter
	// generated output pose no PRE_FOS_OUTPUT. Assim Head/Head01/Jaw/Lips e quaisquer
	// outros descendentes permanecem na mesma pose que o Feet of Skyrim receber�.
	//
	// A c�psula continua autoridade f�sica e N�O � amortecida. A anatomia passa a
	// persegui-la com uma mola amortecida, rela��o 1.0x e cap total de 60 graus.
	// GroundedRadial continua explicitamente exclu�do; b�pedes n�o humanos preservam a rota Ni j� validada.
	static constexpr std::string_view BUILD_SIGNATURE = "FORMA-MOTUS-R5.7-NONHUMAN-ONLY-2026-09-26";

	static constexpr float FLEX_GAIN = 0.243f;
	static constexpr float FLEX_RESPONSE = 7.5f;
	static constexpr float SPEED_RESPONSE = 8.0f;
	static constexpr float ANGULAR_RESPONSE = 10.0f;
	static constexpr float FLEX_LIMIT = 0.432f;
	static constexpr float DEFAULT_YAW_SIGN = 1.0f;

	// Rela��o geom�trica 1.0x. Ex.: 30 graus f�sicos acumulados de c�psula =>
	// alvo de 30 graus de flex procedural TOTAL, depois distribu�do pelos pesos.
	static constexpr float CAPSULE_TURN_FLEX_RATIO = 1.00f;

	// Limite absoluto da pose procedural total antes da distribui��o por ossos.
	static constexpr float CAPSULE_TURN_FLEX_LIMIT_RAD = 60.0f * DEG2RAD;

	// Forma Motus R2 separates anticipatory Orientation Warping from the inherited
	// physical-curvature envelope. Incessus error is signed already and gets its
	// own conservative visual cap.
	static constexpr float ORIENTATION_WARP_RATIO = 1.00f;
	static constexpr float ORIENTATION_WARP_DEADZONE_RAD = 2.0f * DEG2RAD;
	static constexpr float ORIENTATION_WARP_LIMIT_RAD = 45.0f * DEG2RAD;

	// IMPORTANTE: estes thresholds detectam apenas o lifecycle da manobra.
	// Eles N�O entram na f�rmula de amplitude e n�o fazem uma curva r�pida
	// curvar mais que uma curva lenta de mesmo �ngulo total.
	static constexpr float CAPSULE_TURN_START_RATE_RAD_S = 2.0f * DEG2RAD;
	static constexpr float CAPSULE_TURN_END_RATE_RAD_S = 0.50f * DEG2RAD;
	static constexpr float CAPSULE_TURN_END_GRACE = 0.20f;

	// O ALR mostrou que actor yaw e controller forwardVec podem divergir. Este
	// piso existe apenas para decidir se uma camada realmente se moveu no frame.
	static constexpr float QUADRUPED_YAW_LAYER_NOISE_RAD = 0.05f * DEG2RAD;

	// Saltos acima de 120 graus em um �nico update continuam sendo tratados como
	// descontinuidade/teleporte, n�o como uma curva anat�mica v�lida.
	static constexpr float CAPSULE_DISCONTINUITY_DELTA_RAD = 120.0f * DEG2RAD;

	// V18: seguidor de segunda ordem. A c�psula pode girar violentamente; a pose
	// possui velocidade pr�pria e persegue o alvo sem copi�-lo no mesmo frame.
	// A mesma mola governa ida, revers�o e retorno ao neutro.
	static constexpr float CAPSULE_FLEX_SPRING_OMEGA = 5.0f;
	static constexpr float CAPSULE_FLEX_SPRING_DAMPING_RATIO = 0.85f;
	static constexpr float CAPSULE_FLEX_MAX_SPEED_RAD_S = 90.0f * DEG2RAD;

	// R5.5 Player-controlled mount path. A mounted horse does not consume yaw like
	// an autonomous NPC quadruped, so its visual curvature is driven by completed
	// actor-world yaw RATE, never by accumulated capsule angle.
	static constexpr float PLAYER_MOUNT_YAW_RATE_DEADZONE_RAD_S = 1.5f * DEG2RAD;
	static constexpr float PLAYER_MOUNT_MAX_YAW_RATE_RAD_S = 240.0f * DEG2RAD;
	static constexpr float PLAYER_MOUNT_YAW_RATE_FLEX_GAIN = 0.243f;
	static constexpr float PLAYER_MOUNT_FLEX_LIMIT_RAD = 28.0f * DEG2RAD;
	static constexpr float PLAYER_MOUNT_FLEX_RESPONSE = 13.0f;
	static constexpr float PLAYER_MOUNT_DISCONTINUITY_DELTA_RAD = 35.0f * DEG2RAD;
	static constexpr float PLAYER_MOUNT_TELEMETRY_PERIOD = 0.40f;

	// Contrapeso de cauda: discreto, limitado e mais lento que a coluna.
	static constexpr float TAIL_COUNTER_FLEX_RATIO = 0.18f;
	static constexpr float TAIL_COUNTER_FLEX_LIMIT_RAD = 10.0f * DEG2RAD;
	static constexpr float TAIL_POSE_RESPONSE = 3.0f;
	static constexpr std::size_t TAIL_MAX_BONES = 12;
	static constexpr int TAIL_MAX_PELVIS_HIERARCHY_DISTANCE = 16;

	// Prote��o num�rica do yaw desembrulhado. O clamp visual continua 60 graus.
	static constexpr float CAPSULE_TURN_ACCUMULATION_LIMIT_RAD = 4.0f * PI;

	// A r� j� estava anatomicamente correta na V9 com sinal -1. No avan�o, o
	// mesmo sinal espelhava a curvatura. O sentido � resolvido fisicamente por
	// velocityXY dot controllerForwardXY; velocidade n�o altera amplitude.
	static constexpr float QUADRUPED_FORWARD_FLEX_SIGN = 1.0f;
	static constexpr float QUADRUPED_BACKWARD_FLEX_SIGN = -1.0f;
	static constexpr float QUADRUPED_TRAVEL_DIRECTION_DEADZONE = 5.0f;
	// R4.4: inverter o sinal anat�mico exige evid�ncia mais forte e sustentada.
	// A dire��o inicial continua imediata; somente FORWARD<->BACKWARD recebe
	// histerese temporal para rejeitar micro-flips do velocity dot forward.
	static constexpr float QUADRUPED_TRAVEL_SWITCH_THRESHOLD = 12.0f;
	static constexpr float QUADRUPED_TRAVEL_SWITCH_CONFIRM_SECONDS = 0.080f;

	// V8 DEBUG ONLY. Se nenhum yaw natural for observado por alguns segundos,
	// injeta uma pose pequena e temporaria. Isso testa o pipeline de ossos sem
	// interferir no controle fisico da capsula.
	// R4.4: telemetria de runtime deixa de ser frame-near. PRE-FOS/Stride/FootLock
	// possuem seus pr�prios eventos; este heartbeat serve apenas como diagn�stico.
	static constexpr float DEBUG_PIPELINE_HEARTBEAT_PERIOD = 2.00f;
	static constexpr float DEBUG_GATE_LOG_PERIOD = 5.00f;
	// O synthetic pose foi �til para provar o pipeline nas V8-V11. Agora que o
	// yaw natural est� funcionando ele fica desativado para n�o criar snaps
	// artificiais ou contaminar a avalia��o visual da suaviza��o real.
	static constexpr bool DEBUG_SYNTHETIC_ENABLED = false;
	static constexpr float DEBUG_SYNTHETIC_DELAY = 2.00f;
	static constexpr float DEBUG_SYNTHETIC_HALF_DURATION = 0.55f;
	static constexpr float DEBUG_SYNTHETIC_FLEX_RAD = 12.0f * DEG2RAD;

	// Extens�o suave da coluna SOMENTE para quadr�pedes.
	// O c�digo tenta incluir apenas UM osso imediatamente abaixo do primeiro
	// osso de coluna conhecido, e somente se o nome ainda parecer de coluna.
	// 0.00f desativa; 0.25f-0.50f � uma faixa discreta.
	static constexpr float QUADRUPED_LOWER_SPINE_WEIGHT = 0.35f;

	static constexpr float SPEED_FLEX_START = 14.0f;
	static constexpr float SPEED_FLEX_WALK = 75.0f;
	static constexpr float SPEED_FLEX_RUN = 140.0f;
	static constexpr float SPEED_FLEX_SPRINT = 230.0f;
	static constexpr float WALK_FLEX_MULT = 0.06f;
	static constexpr float RUN_FLEX_MULT = 0.54f;


	static constexpr float ANGULAR_DEADZONE_RAD_S = 0.045f;
	static constexpr float MAX_ANGULAR_VELOCITY_RAD_S = 5.25f;

	MotionAnalyzer g_analyzer;

	static float ComputeDeltaTime(ActorCache& cache)
	{
		const auto now = Clock::now();
		if (!cache.clockInitialized) {
			cache.lastUpdateTime = now;
			cache.clockInitialized = true;
			return 1.0f / 60.0f;
		}
		const float dt = std::chrono::duration<float>(now - cache.lastUpdateTime).count();
		cache.lastUpdateTime = now;
		return std::clamp(dt, 1.0f / 240.0f, 0.10f);
	}

	static bool MatrixNearlyEqual(const RE::NiMatrix3& a, const RE::NiMatrix3& b, float epsilon = 0.0025f)
	{
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				if (std::abs(a.entry[i][j] - b.entry[i][j]) > epsilon) return false;
			}
		}
		return true;
	}

	static float MatrixMaxAbsDelta(const RE::NiMatrix3& a, const RE::NiMatrix3& b)
	{
		float maximum = 0.0f;
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				maximum = std::max(maximum, std::abs(a.entry[i][j] - b.entry[i][j]));
			}
		}
		return maximum;
	}


	static HavokQuat NormalizeHavokQuat(HavokQuat q)
	{
		const float lengthSq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
		if (!std::isfinite(lengthSq) || lengthSq <= 1.0e-12f) {
			return {};
		}
		const float invLength = 1.0f / std::sqrt(lengthSq);
		q.x *= invLength;
		q.y *= invLength;
		q.z *= invLength;
		q.w *= invLength;
		return q;
	}

	static HavokQuat MultiplyHavokQuat(const HavokQuat& a, const HavokQuat& b)
	{
		return NormalizeHavokQuat({
			a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
			a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
			a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
			a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
		});
	}

	static HavokQuat MatrixToHavokQuat(const RE::NiMatrix3& m)
	{
		HavokQuat q{};
		const float trace = m.entry[0][0] + m.entry[1][1] + m.entry[2][2];

		if (trace > 0.0f) {
			const float s = std::sqrt(std::max(trace + 1.0f, 0.0f)) * 2.0f;
			if (s <= 1.0e-6f) return {};
			q.w = 0.25f * s;
			q.x = (m.entry[2][1] - m.entry[1][2]) / s;
			q.y = (m.entry[0][2] - m.entry[2][0]) / s;
			q.z = (m.entry[1][0] - m.entry[0][1]) / s;
		} else if (m.entry[0][0] > m.entry[1][1] && m.entry[0][0] > m.entry[2][2]) {
			const float s = std::sqrt(std::max(1.0f + m.entry[0][0] - m.entry[1][1] - m.entry[2][2], 0.0f)) * 2.0f;
			if (s <= 1.0e-6f) return {};
			q.w = (m.entry[2][1] - m.entry[1][2]) / s;
			q.x = 0.25f * s;
			q.y = (m.entry[0][1] + m.entry[1][0]) / s;
			q.z = (m.entry[0][2] + m.entry[2][0]) / s;
		} else if (m.entry[1][1] > m.entry[2][2]) {
			const float s = std::sqrt(std::max(1.0f + m.entry[1][1] - m.entry[0][0] - m.entry[2][2], 0.0f)) * 2.0f;
			if (s <= 1.0e-6f) return {};
			q.w = (m.entry[0][2] - m.entry[2][0]) / s;
			q.x = (m.entry[0][1] + m.entry[1][0]) / s;
			q.y = 0.25f * s;
			q.z = (m.entry[1][2] + m.entry[2][1]) / s;
		} else {
			const float s = std::sqrt(std::max(1.0f + m.entry[2][2] - m.entry[0][0] - m.entry[1][1], 0.0f)) * 2.0f;
			if (s <= 1.0e-6f) return {};
			q.w = (m.entry[1][0] - m.entry[0][1]) / s;
			q.x = (m.entry[0][2] + m.entry[2][0]) / s;
			q.y = (m.entry[1][2] + m.entry[2][1]) / s;
			q.z = 0.25f * s;
		}

		return NormalizeHavokQuat(q);
	}

	static HavokQuat ReadHavokQuat(const RE::hkQuaternion& q)
	{
		alignas(16) float values[4]{};
		_mm_store_ps(values, q.vec.quad);
		return NormalizeHavokQuat({ values[0], values[1], values[2], values[3] });
	}

	static void WriteHavokQuat(RE::hkQuaternion& destination, const HavokQuat& q)
	{
		const HavokQuat normalized = NormalizeHavokQuat(q);
		destination.vec = RE::hkVector4(normalized.x, normalized.y, normalized.z, normalized.w);
	}

	static bool ShouldLogDebugGate(ActorCache& cache)
	{
		const auto now = Clock::now();
		if (cache.debugLastGateLog == Clock::time_point{} ||
			std::chrono::duration<float>(now - cache.debugLastGateLog).count() >= DEBUG_GATE_LOG_PERIOD) {
			cache.debugLastGateLog = now;
			return true;
		}
		return false;
	}

	static inline char ToLowerASCII(char c)
	{
		return static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
	}

	static bool ContainsCaseInsensitive(const char* haystack, std::string_view needleLower)
	{
		if (!haystack || needleLower.empty()) return false;
		const size_t n = needleLower.size();
		for (size_t i = 0; haystack[i] != '\0'; ++i) {
			size_t j = 0;
			while (j < n && haystack[i + j] != '\0' && ToLowerASCII(haystack[i + j]) == needleLower[j]) ++j;
			if (j == n) return true;
		}
		return false;
	}

	static RE::NiAVObject* FindBoneFromProfileName(RE::NiAVObject* root, const std::string& name);

	static bool IsFuzzyMatch(const char* haystack, std::string_view needle)
	{
		if (!haystack || needle.empty()) return false;
		std::string cleanH;
		for (int i = 0; haystack[i] != '\0'; ++i) {
			if (haystack[i] != ' ' && haystack[i] != '_') cleanH += ToLowerASCII(haystack[i]);
		}
		std::string cleanN;
		for (char c : needle) {
			if (c != ' ' && c != '_') cleanN += ToLowerASCII(c);
		}
		return cleanH.find(cleanN) != std::string::npos;
	}

	static RE::NiAVObject* FindBoneNodeFast(RE::NiAVObject* root, std::string_view needle)
	{
		if (!root) return nullptr;
		std::vector<RE::NiAVObject*> stack;
		stack.reserve(128);
		stack.push_back(root);

		while (!stack.empty()) {
			RE::NiAVObject* cur = stack.back();
			stack.pop_back();

			const char* nm = cur ? cur->name.c_str() : nullptr;
			if (nm && IsFuzzyMatch(nm, needle)) return cur;

			if (auto asNode = cur->AsNode(); asNode) {
				for (auto& child : asNode->GetChildren()) {
					if (child) stack.push_back(child.get());
				}
			}
		}
		return nullptr;
	}

	static RE::NiAVObject* FindBoneFromProfileName(RE::NiAVObject* root, const std::string& name)
	{
		if (!root || name.empty()) return nullptr;
		if (auto* exact = root->GetObjectByName(name)) return exact;
		return FindBoneNodeFast(root, name);
	}

	static float NormalizeAngleDeg(float angleDeg) { return std::remainder(angleDeg, 360.0f); }
	static float NormalizeAngleRad(float angleRad) { return std::remainder(angleRad, 2.0f * PI); }
	static inline float Saturate(float value) { return std::clamp(value, 0.0f, 1.0f); }

	static inline float Smoothstep(float edge0, float edge1, float value)
	{
		if (edge1 <= edge0) return value >= edge1 ? 1.0f : 0.0f;
		const float t = Saturate((value - edge0) / (edge1 - edge0));
		return t * t * (3.0f - 2.0f * t);
	}

	static inline float ExpAlpha(float response, float dt)
	{
		return 1.0f - std::exp(-std::max(0.0f, response) * std::max(0.0f, dt));
	}

	static void UpdateCapsuleFlexSpring(float targetFlex, float dt, ActorCache& cache, float maxAbsFlex)
	{
		const float step = std::clamp(dt, 0.0f, 0.050f);
		if (step <= 0.0f) return;

		const float omega = CAPSULE_FLEX_SPRING_OMEGA;
		const float stiffness = omega * omega;
		const float damping = 2.0f * CAPSULE_FLEX_SPRING_DAMPING_RATIO * omega;
		const float error = targetFlex - cache.currentFlex;
		const float acceleration = stiffness * error - damping * cache.flexVelocity;

		cache.flexVelocity += acceleration * step;
		cache.flexVelocity = std::clamp(
			cache.flexVelocity,
			-CAPSULE_FLEX_MAX_SPEED_RAD_S,
			CAPSULE_FLEX_MAX_SPEED_RAD_S);
		cache.currentFlex += cache.flexVelocity * step;
		cache.currentFlex = std::clamp(
			cache.currentFlex,
			-std::max(0.0f, maxAbsFlex),
			std::max(0.0f, maxAbsFlex));

		if (std::abs(targetFlex) < 0.00035f &&
			std::abs(cache.currentFlex) < 0.00035f &&
			std::abs(cache.flexVelocity) < 0.0020f) {
			cache.currentFlex = 0.0f;
			cache.flexVelocity = 0.0f;
		}
	}

	static bool ReadGraphFloat(RE::Actor* actor, const char* name, float& outValue)
	{
		outValue = 0.0f;
		if (!actor || !name || !name[0]) return false;
		float value = 0.0f;
		if (!actor->GetGraphVariableFloat(name, value) || !std::isfinite(value)) return false;
		outValue = value;
		return true;
	}

	static bool ReadPhysicalCapsuleYaw(RE::Actor* actor, float& yawRad);

	static float ReadPlanarVelocitySpeed(RE::Actor* actor)
	{
		if (!actor) return 0.0f;
		RE::NiPoint3 velocity{};
		actor->GetLinearVelocity(velocity);
		if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) return 0.0f;
		return std::sqrt(std::max(0.0f, velocity.x * velocity.x + velocity.y * velocity.y));
	}

	static float UpdateQuadrupedTravelFlexSign(RE::Actor* actor, ActorCache& cache, float dt)
	{
		if (!actor) return cache.quadrupedTravelFlexSign;

		RE::NiPoint3 velocity{};
		actor->GetLinearVelocity(velocity);
		if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y)) {
			return cache.quadrupedTravelFlexSign;
		}

		auto* controller = actor->GetCharController();
		if (!controller) return cache.quadrupedTravelFlexSign;

		float capsuleYawRad = 0.0f;
		if (!ReadPhysicalCapsuleYaw(actor, capsuleYawRad)) {
			return cache.quadrupedTravelFlexSign;
		}

		// Usa o mesmo yaw mundial j� convertido para definir o eixo longitudinal.
		// Isso evita o erro da V10, que fazia dot diretamente com forwardVec cru
		// (vetor que nos logs aponta no referencial inverso da locomo��o mundial).
		const float fx = std::cos(capsuleYawRad);
		const float fy = std::sin(capsuleYawRad);
		const float signedLongitudinalSpeed = velocity.x * fx + velocity.y * fy;
		if (!std::isfinite(signedLongitudinalSpeed)) {
			return cache.quadrupedTravelFlexSign;
		}

		float observedSign = 0.0f;
		if (signedLongitudinalSpeed > QUADRUPED_TRAVEL_DIRECTION_DEADZONE) {
			observedSign = QUADRUPED_FORWARD_FLEX_SIGN;
		} else if (signedLongitudinalSpeed < -QUADRUPED_TRAVEL_DIRECTION_DEADZONE) {
			observedSign = QUADRUPED_BACKWARD_FLEX_SIGN;
		}

		// Primeira dire��o v�lida: n�o adicionamos lat�ncia ao spawn/in�cio de marcha.
		if (!cache.quadrupedTravelSignInitialized) {
			if (observedSign != 0.0f) {
				cache.quadrupedTravelFlexSign = observedSign;
				cache.quadrupedTravelSignInitialized = true;
				cache.quadrupedTravelPendingSign = 0.0f;
				cache.quadrupedTravelPendingTimer = 0.0f;
				SKSE::log::info(
					"[AME-R4.4][TRAVEL-SIGN] actor={:08X} name=\"{}\" event=INIT signedLongSpeed={:+.2f} mode={} flexSign={:+.0f}",
					actor->GetFormID(), actor->GetName() ? actor->GetName() : "",
					signedLongitudinalSpeed, observedSign > 0.0f ? "FORWARD" : "BACKWARD", observedSign);
			}
			return cache.quadrupedTravelFlexSign;
		}

		// Mesmo sentido, parada ou ru�do fraco: preserva estado e cancela candidato.
		if (observedSign == 0.0f || observedSign == cache.quadrupedTravelFlexSign ||
			std::abs(signedLongitudinalSpeed) < QUADRUPED_TRAVEL_SWITCH_THRESHOLD) {
			cache.quadrupedTravelPendingSign = 0.0f;
			cache.quadrupedTravelPendingTimer = 0.0f;
			return cache.quadrupedTravelFlexSign;
		}

		// Dire��o oposta forte: s� comita se persistir por uma pequena janela.
		if (cache.quadrupedTravelPendingSign != observedSign) {
			cache.quadrupedTravelPendingSign = observedSign;
			cache.quadrupedTravelPendingTimer = 0.0f;
		}
		cache.quadrupedTravelPendingTimer += std::clamp(dt, 0.0f, 0.10f);

		if (cache.quadrupedTravelPendingTimer >= QUADRUPED_TRAVEL_SWITCH_CONFIRM_SECONDS) {
			cache.quadrupedTravelFlexSign = observedSign;
			cache.quadrupedTravelPendingSign = 0.0f;
			cache.quadrupedTravelPendingTimer = 0.0f;
			SKSE::log::info(
				"[AME-R4.4][TRAVEL-SIGN] actor={:08X} name=\"{}\" event=COMMIT signedLongSpeed={:+.2f} mode={} flexSign={:+.0f} switchThreshold={:.1f} confirmMs={:.0f}",
				actor->GetFormID(), actor->GetName() ? actor->GetName() : "",
				signedLongitudinalSpeed, observedSign > 0.0f ? "FORWARD" : "BACKWARD", observedSign,
				QUADRUPED_TRAVEL_SWITCH_THRESHOLD, QUADRUPED_TRAVEL_SWITCH_CONFIRM_SECONDS * 1000.0f);
		}

		return cache.quadrupedTravelFlexSign;
	}

	static float NormalizeSpeedSampled(float value)
	{
		value = std::abs(value);
		if (value <= 2.0f) return value * SPEED_FLEX_RUN;
		return value;
	}

	static float ReadActorSpeed(RE::Actor* actor)
	{
		if (!actor) return 0.0f;
		float bestSpeed = 0.0f;
		float value = 0.0f;
		if (ReadGraphFloat(actor, "Speed", value)) bestSpeed = std::max(bestSpeed, std::abs(value));
		if (ReadGraphFloat(actor, "SpeedSampled", value)) bestSpeed = std::max(bestSpeed, NormalizeSpeedSampled(value));
		bestSpeed = std::max(bestSpeed, ReadPlanarVelocitySpeed(actor));
		return std::isfinite(bestSpeed) ? bestSpeed : 0.0f;
	}

	static float ComputeSpeedFlexScaleFromSpeed(float speed)
	{
		if (speed <= SPEED_FLEX_START) return 0.0f;
		const float walkPart = Smoothstep(SPEED_FLEX_START, SPEED_FLEX_WALK, speed) * WALK_FLEX_MULT;
		const float runPart = Smoothstep(SPEED_FLEX_WALK, SPEED_FLEX_RUN, speed) * RUN_FLEX_MULT;
		const float sprintPart = Smoothstep(SPEED_FLEX_RUN, SPEED_FLEX_SPRINT, speed) * (1.0f - WALK_FLEX_MULT - RUN_FLEX_MULT);
		return std::clamp(walkPart + runPart + sprintPart, 0.0f, 1.0f);
	}

	static void NormalizeWeights(std::vector<BoneEntry>& chain)
	{
		float total = 0.0f;
		for (const auto& entry : chain) total += entry.weight;
		if (total <= 0.0f) return;
		for (auto& entry : chain) entry.weight /= total;
	}

	static bool ChainContainsBone(const std::vector<BoneEntry>& chain, const RE::NiAVObject* bone)
	{
		if (!bone) return false;
		for (const auto& entry : chain) {
			if (entry.bone == bone) return true;
		}
		return false;
	}

	static bool IsAncestorOf(const RE::NiAVObject* ancestor, const RE::NiAVObject* node)
	{
		if (!ancestor || !node || ancestor == node) return false;
		for (auto* current = node->parent; current; current = current->parent) {
			if (current == ancestor) return true;
		}
		return false;
	}

	static RE::NiAVObject* FindLowestBoneInChain(const std::vector<BoneEntry>& chain)
	{
		// Entre os ossos j� reconhecidos como coluna, encontra o mais baixo:
		// aquele que n�o possui outro osso da pr�pria lista como ancestral.
		for (const auto& candidate : chain) {
			if (!candidate.bone) continue;
			bool hasChainAncestor = false;
			for (const auto& other : chain) {
				if (other.bone && IsAncestorOf(other.bone, candidate.bone)) {
					hasChainAncestor = true;
					break;
				}
			}
			if (!hasChainAncestor) return candidate.bone;
		}
		return chain.empty() ? nullptr : chain.front().bone;
	}

	static bool IsExplicitLowerSpineName(const RE::NiAVObject* bone)
	{
		if (!bone) return false;
		const char* name = bone->name.c_str();
		if (!name || !name[0]) return false;

		// Rejei��es estruturais: mesmo sendo o pai imediato, estes n�s podem
		// carregar pernas, cauda, IK ou o esqueleto inteiro.
		if (ContainsCaseInsensitive(name, "root") ||
		    ContainsCaseInsensitive(name, "pelvis") ||
		    ContainsCaseInsensitive(name, "pelv") ||
		    ContainsCaseInsensitive(name, "com") ||
		    ContainsCaseInsensitive(name, "center of mass") ||
		    ContainsCaseInsensitive(name, "body") ||
		    ContainsCaseInsensitive(name, "hip") ||
		    ContainsCaseInsensitive(name, "leg") ||
		    ContainsCaseInsensitive(name, "thigh") ||
		    ContainsCaseInsensitive(name, "tail") ||
		    ContainsCaseInsensitive(name, "controller") ||
		    ContainsCaseInsensitive(name, "control")) {
			return false;
		}

		// S� aceita nomes que declarem claramente que ainda pertencem ao dorso.
		return ContainsCaseInsensitive(name, "spine") ||
		       ContainsCaseInsensitive(name, "back") ||
		       ContainsCaseInsensitive(name, "lumbar") ||
		       ContainsCaseInsensitive(name, "thoracic") ||
		       ContainsCaseInsensitive(name, "vertebra");
	}

	static void PrependImmediateLowerQuadrupedSpineBone(
		std::vector<BoneEntry>& chain,
		RE::NiAVObject* lowestKnownSpineBone)
	{
		const float weight = std::max(0.0f, QUADRUPED_LOWER_SPINE_WEIGHT);
		if (chain.empty() || !lowestKnownSpineBone || weight <= 0.0f) return;

		// Exatamente UM n�vel abaixo. N�o existe loop e nunca seguimos at� a raiz.
		auto* candidate = lowestKnownSpineBone->parent;
		if (!candidate || !candidate->parent) return;
		if (ChainContainsBone(chain, candidate)) return;
		if (!IsExplicitLowerSpineName(candidate)) return;

		BoneEntry lowerEntry;
		lowerEntry.bone = candidate;
		lowerEntry.weight = weight;
		chain.insert(chain.begin(), lowerEntry);
	}

	static bool AppendUniqueBonePtr(std::vector<BoneEntry>& chain, RE::NiAVObject* bone, float weight)
	{
		if (!bone || weight <= 0.0f) return false;
		for (const auto& existing : chain) {
			if (existing.bone == bone) return false;
		}
		BoneEntry entry;
		entry.bone = bone;
		entry.weight = weight;
		chain.push_back(entry);
		return true;
	}

	static bool AppendUniqueBone(RE::NiAVObject* root, std::vector<BoneEntry>& chain, const char* name, float weight)
	{
		if (!root || !name || !name[0] || weight <= 0.0f) return false;
		auto* bone = FindBoneFromProfileName(root, name);
		return AppendUniqueBonePtr(chain, bone, weight);
	}

	static bool IsBadSpineNodeName(const std::string& name)
	{
		return name == "NPC Root [Root]" ||
		       name == "Scull" ||
		       name == "Skull" ||
		       name == "NPC Pelvis [Pelv]" ||
		       name == "Pelvis" ||
		       name == "[body]" ||
		       name == "Mcrab_Body";
	}


	static bool IsTailBoneName(const RE::NiAVObject* bone)
	{
		if (!bone) return false;
		const char* name = bone->name.c_str();
		if (!name || !name[0]) return false;

		// "detail" cont�m a sequ�ncia "tail" e seria um falso positivo.
		if (ContainsCaseInsensitive(name, "detail")) return false;
		return ContainsCaseInsensitive(name, "tail");
	}

	static void CollectTailBoneCandidates(RE::NiAVObject* root, std::vector<RE::NiAVObject*>& outCandidates)
	{
		outCandidates.clear();
		if (!root) return;

		std::vector<RE::NiAVObject*> stack;
		stack.reserve(128);
		stack.push_back(root);
		while (!stack.empty()) {
			auto* current = stack.back();
			stack.pop_back();
			if (!current) continue;

			if (current != root && IsTailBoneName(current)) {
				outCandidates.push_back(current);
			}

			if (auto* node = current->AsNode(); node) {
				for (auto& child : node->GetChildren()) {
					if (child) stack.push_back(child.get());
				}
			}
		}
	}

	static bool PointerVectorContains(const std::vector<RE::NiAVObject*>& values, const RE::NiAVObject* value)
	{
		return value && std::find(values.begin(), values.end(), value) != values.end();
	}

	static RE::NiAVObject* FindNearestTailAncestor(
		RE::NiAVObject* bone,
		const std::vector<RE::NiAVObject*>& candidates)
	{
		if (!bone) return nullptr;
		for (auto* current = bone->parent; current; current = current->parent) {
			if (PointerVectorContains(candidates, current)) return current;
		}
		return nullptr;
	}

	static int HierarchyDistance(const RE::NiAVObject* ancestor, const RE::NiAVObject* node)
	{
		if (!ancestor || !node) return -1;
		int distance = 0;
		for (auto* current = node; current; current = current->parent, ++distance) {
			if (current == ancestor) return distance;
		}
		return -1;
	}

	static bool BuildTailCounterbalanceChain(
		RE::NiAVObject* root,
		const UniversalKinematics::SkeletonProfile& profile,
		const std::vector<BoneEntry>& axialChain,
		std::vector<BoneEntry>& outChain)
	{
		outChain.clear();
		if (!root) return false;

		std::vector<RE::NiAVObject*> candidates;
		CollectTailBoneCandidates(root, candidates);
		if (candidates.empty()) return false;

		// Nunca reutiliza um n� j� pertencente � coluna/pesco�o. Isso protege
		// perfis ex�ticos que usam "Tail" para descrever o pr�prio eixo corporal.
		candidates.erase(
			std::remove_if(candidates.begin(), candidates.end(), [&](RE::NiAVObject* bone) {
				return !bone || ChainContainsBone(axialChain, bone);
			}),
			candidates.end());
		if (candidates.empty()) return false;

		RE::NiAVObject* pelvis = nullptr;
		if (!profile.pelvisNode.empty()) {
			pelvis = FindBoneFromProfileName(root, profile.pelvisNode);
		}

		// Com pelvis conhecido, a cauda precisa realmente nascer dessa hierarquia.
		if (pelvis) {
			candidates.erase(
				std::remove_if(candidates.begin(), candidates.end(), [&](RE::NiAVObject* bone) {
					const int distance = HierarchyDistance(pelvis, bone);
					return distance <= 0 || distance > TAIL_MAX_PELVIS_HIERARCHY_DISTANCE;
				}),
				candidates.end());
			if (candidates.empty()) return false;
		}

		// Usa uma �nica linhagem: o caminho Tail mais longo do rig.
		RE::NiAVObject* bestLeaf = nullptr;
		std::size_t bestDepth = 0;
		for (auto* candidate : candidates) {
			std::size_t depth = 1;
			for (auto* parent = FindNearestTailAncestor(candidate, candidates);
			     parent;
			     parent = FindNearestTailAncestor(parent, candidates)) {
				++depth;
				if (depth >= TAIL_MAX_BONES) break;
			}
			if (depth > bestDepth) {
				bestDepth = depth;
				bestLeaf = candidate;
			}
		}
		if (!bestLeaf) return false;

		std::vector<RE::NiAVObject*> ordered;
		ordered.reserve(std::min(bestDepth, TAIL_MAX_BONES));
		for (auto* current = bestLeaf; current && ordered.size() < TAIL_MAX_BONES;
		     current = FindNearestTailAncestor(current, candidates)) {
			ordered.push_back(current);
		}
		std::reverse(ordered.begin(), ordered.end());
		if (ordered.empty()) return false;

		float nextWeight = 1.0f;
		for (auto* bone : ordered) {
			if (AppendUniqueBonePtr(outChain, bone, nextWeight)) {
				nextWeight += 0.65f;
			}
		}
		if (outChain.empty()) return false;
		NormalizeWeights(outChain);
		return true;
	}

	static RE::NiAVObject* FindBoneByHints(RE::NiAVObject* root, std::initializer_list<std::string_view> hints)
	{
		if (!root) return nullptr;
		for (const auto& hint : hints) {
			if (hint.empty()) continue;
			if (auto* bone = FindBoneNodeFast(root, hint)) return bone;
		}
		return nullptr;
	}

	static bool BuildHeuristicBipedChain(RE::NiAVObject* root, std::vector<BoneEntry>& outChain)
	{
		outChain.clear();
		if (!root) return false;
		float nextWeight = 1.0f;
		auto appendHints = [&](std::initializer_list<std::string_view> hints, float step) {
			if (auto* bone = FindBoneByHints(root, hints)) {
				if (AppendUniqueBonePtr(outChain, bone, nextWeight)) nextWeight += step;
			}
		};
		appendHints({ "spine2", "spine1", "spine", "chest", "ribcage", "torso", "bip01 spine2", "bip01 spine1", "bip01 spine" }, 0.45f);
		appendHints({ "neck", "neck1", "bip01 neck" }, 0.35f);
		if (outChain.empty()) return false;
		NormalizeWeights(outChain);
		return true;
	}

	static bool BuildBipedChainFromProfile(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile, std::vector<BoneEntry>& outChain)
	{
		outChain.clear();
		float nextWeight = 1.0f;
		for (const auto& nodeName : profile.spineNodes) {
			if (nodeName.empty() || IsBadSpineNodeName(nodeName)) continue;
			if (AppendUniqueBone(root, outChain, nodeName.c_str(), nextWeight)) nextWeight += 0.40f;
		}
		if (!profile.chestNode.empty()) {
			if (AppendUniqueBone(root, outChain, profile.chestNode.c_str(), nextWeight)) nextWeight += 0.45f;
		}
		if (!profile.neckNode.empty()) {
			if (AppendUniqueBone(root, outChain, profile.neckNode.c_str(), nextWeight * 0.55f)) nextWeight += 0.30f;
		}
		if (outChain.empty()) return false;
		NormalizeWeights(outChain);
		return true;
	}

	static bool BuildQuadrupedOverlayChain(RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile, std::vector<BoneEntry>& outChain)
	{
		outChain.clear();
		if (!root) return false;

		// V14: a curvatura axial termina no pesco�o. A cabe�a n�o recebe rota��o
		// local do AME; ela e seus filhos (focinho, mand�bula, tromba, chifres,
		// face etc.) acompanham o pesco�o somente pela hierarquia normal.
		RE::NiAVObject* protectedHeadBone = nullptr;
		if (!profile.headNode.empty()) {
			protectedHeadBone = FindBoneFromProfileName(root, profile.headNode);
		}

		auto appendAxialBone = [&](const std::string& nodeName, float weight) {
			if (nodeName.empty() || weight <= 0.0f || IsBadSpineNodeName(nodeName)) return false;
			auto* bone = FindBoneFromProfileName(root, nodeName);
			if (!bone || bone == protectedHeadBone) return false;
			return AppendUniqueBonePtr(outChain, bone, weight);
		};

		float nextWeight = 1.0f;
		for (const auto& nodeName : profile.spineNodes) {
			if (appendAxialBone(nodeName, nextWeight)) nextWeight += 0.40f;
		}

		// Neste ponto a lista cont�m SOMENTE ossos declarados como coluna/pesco�o,
		// nunca a cabe�a protegida. Guardamos o mais baixo antes dos aliases extras.
		RE::NiAVObject* lowestKnownSpineBone = FindLowestBoneInChain(outChain);

		if (appendAxialBone(profile.chestNode, nextWeight)) nextWeight += 0.45f;
		if (appendAxialBone(profile.neckNode, nextWeight * 0.55f)) nextWeight += 0.30f;
		if (outChain.empty()) return false;

		// Exclusivo desta fun��o de quadr�pedes. BuildBipedChainFromProfile
		// permanece intocado. S� tenta UM pai imediato e validado como coluna.
		PrependImmediateLowerQuadrupedSpineBone(outChain, lowestKnownSpineBone);
		NormalizeWeights(outChain);
		return true;
	}

	static float ComputeMorphScale(const UniversalKinematics::SkeletonProfile& profile)
	{
		if (profile.bodyWidth <= 0.0f) return 1.0f;
		const float aspect = profile.bodyLength / std::max(profile.bodyWidth, 1.0f);
		return std::clamp(0.55f + (aspect - 1.0f) * 0.20f, 0.45f, 1.15f);
	}

	static bool IsAxialAnimalBipedProfile(const UniversalKinematics::SkeletonProfile& profile)
	{
		// Anatomia b�pede n�o implica cinem�tica humanoide. Chicken � o caso
		// can�nico: duas pernas, mas corpo/pesco�o organizados em um eixo
		// longitudinal que deve acompanhar a c�psula como um cavalo.
		return profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped &&
		       profile.upperBodyModel == UniversalKinematics::UpperBodyModel::HeadNeckOnly &&
		       !profile.usesForelimbsAsHands &&
		       !profile.isHumanoidLikeUpperBody;
	}

	static bool IsBipedProfile(const UniversalKinematics::SkeletonProfile& profile)
	{
		return profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped &&
		       !IsAxialAnimalBipedProfile(profile);
	}

	static bool UsesStableNonHumanBipedYawWitness(
		RE::Actor* actor,
		const UniversalKinematics::SkeletonProfile& profile)
	{
		// Some non-human bipeds (for example Troll-family profiles) are anatomically
		// humanoid-like but remain Forma-owned. Preserve their stable completed
		// world-yaw witness without using that anatomical flag as an ownership test.
		return ActorDomain::IsFormaNonHuman(actor) &&
		       profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped &&
		       profile.isHumanoidLikeUpperBody;
	}

	static bool IsNonAxialRadialProfile(const UniversalKinematics::SkeletonProfile& profile)
	{
		// Mudcrab, Frostbite Spider e outros layouts radiais/multi-leg n�o t�m
		// uma coluna longitudinal equivalente � de cavalo/can�deo. N�o aplicar
		// yaw procedural distribu�do nesses corpos.
		return profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedRadial;
	}

    static bool IsSpiderAbdomenProfile(const UniversalKinematics::SkeletonProfile& profile) {
        return IsNonAxialRadialProfile(profile) &&
            std::find(profile.spineNodes.begin(),profile.spineNodes.end(),"Tail1")!=profile.spineNodes.end() &&
            std::find(profile.spineNodes.begin(),profile.spineNodes.end(),"Tail2")!=profile.spineNodes.end();
    }
	static bool IsAxialCapsuleProfile(const UniversalKinematics::SkeletonProfile& profile)
	{
		if (IsNonAxialRadialProfile(profile)) return false;
		if (IsAxialAnimalBipedProfile(profile)) return true;

		return profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped ||
		       profile.locomotionMode == UniversalKinematics::LocomotionMode::GroundedCrawler;
	}

	static bool NameContainsLower(const std::string& value, std::string_view needleLower)
	{
		return !value.empty() && ContainsCaseInsensitive(value.c_str(), needleLower);
	}

	static bool IsHorseLikeProfile(const UniversalKinematics::SkeletonProfile& profile)
	{
		for (const auto& nodeName : profile.spineNodes) {
			if (NameContainsLower(nodeName, "horse")) return true;
		}
		return NameContainsLower(profile.chestNode, "horse") ||
		       NameContainsLower(profile.neckNode, "horse") ||
		       NameContainsLower(profile.headNode, "horse");
	}

	static bool IsPlayerControlledMount(RE::Actor* actor)
	{
		if (!actor) return false;
		RE::NiPointer<RE::Actor> rider;
		return actor->GetMountedBy(rider) && rider && rider->IsPlayerRef();
	}

	static std::string JoinProfileSpineNodes(const UniversalKinematics::SkeletonProfile& profile)
	{
		std::string output;
		for (const auto& nodeName : profile.spineNodes) {
			if (nodeName.empty()) continue;
			if (!output.empty()) output += "|";
			output += nodeName;
		}
		return output.empty() ? std::string{"<none>"} : output;
	}

	static std::string DescribeBoneChain(const std::vector<BoneEntry>& chain)
	{
		std::string output;
		for (const auto& entry : chain) {
			if (!entry.bone) continue;
			if (!output.empty()) output += " | ";
			const char* name = entry.bone->name.c_str();
			output += name && name[0] ? name : "<unnamed>";
			output += "@" + std::to_string(entry.weight);
		}
		return output.empty() ? std::string{"<empty>"} : output;
	}

	static void RefreshBipedChain(ActorCache& cache, RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		// Cauda pertence exclusivamente � rota axial animal. Se um ator/perfil trocar
		// de topologia sem trocar imediatamente o FormID, n�o permita ponteiros/offsets
		// de uma tailChain anterior sobreviverem � rota b�pede.
		cache.tailChain.clear();
		cache.currentTailFlex = 0.0f;
		cache.havokBindings.clear();
		cache.havokBodyDeltas.clear();
		cache.havokTailDeltas.clear();
		cache.havokPosePrepared = false;

		if (!root) {
			cache.spineAndNeckChain.clear();
			cache.lastRoot = nullptr;
			return;
		}
		if (cache.lastRoot == root && !cache.spineAndNeckChain.empty()) return;
		cache.lastRoot = root;
		cache.spineAndNeckChain.clear();
		if (!BuildBipedChainFromProfile(root, profile, cache.spineAndNeckChain) || cache.spineAndNeckChain.empty()) {
			BuildHeuristicBipedChain(root, cache.spineAndNeckChain);
		}
	}

	static void RefreshQuadrupedLikeChain(ActorCache& cache, RE::NiAVObject* root, const UniversalKinematics::SkeletonProfile& profile)
	{
		if (!root) {
			cache.spineAndNeckChain.clear();
			cache.tailChain.clear();
			cache.havokBindings.clear();
			cache.havokBodyDeltas.clear();
			cache.havokTailDeltas.clear();
			cache.havokPosePrepared = false;
			cache.lastRoot = nullptr;
			return;
		}
		if (cache.lastRoot == root && !cache.spineAndNeckChain.empty()) return;
		cache.lastRoot = root;
		cache.spineAndNeckChain.clear();
		cache.tailChain.clear();
		cache.havokBindings.clear();
		cache.havokBodyDeltas.clear();
		cache.havokTailDeltas.clear();
		cache.havokPosePrepared = false;
		cache.debugHavokBindingLogged = false;
		cache.debugHavokApplyLogged = false;
		cache.debugHavokMissingLogged = false;
		BuildQuadrupedOverlayChain(root, profile, cache.spineAndNeckChain);
		BuildTailCounterbalanceChain(root, profile, cache.spineAndNeckChain, cache.tailChain);
	}

    static void RefreshSpiderAbdomenChain(ActorCache& cache,RE::NiAVObject* root) {
        if(root&&cache.lastRoot==root&&!cache.spineAndNeckChain.empty())return;
        cache.lastRoot=root;cache.spineAndNeckChain.clear();cache.tailChain.clear();
        cache.havokBindings.clear();cache.havokBodyDeltas.clear();cache.havokTailDeltas.clear();cache.havokPosePrepared=false;
        cache.debugHavokBindingLogged=false;
        if(!root)return;
        // Only exact abdomen joints; never body, leg roots, pelvis or heuristic bones.
        for(const char* name:{"Tail1","Tail2","Tail3"})if(auto* bone=root->GetObjectByName(name)) {
            BoneEntry entry{};entry.bone=bone;entry.weight=SpiderAbdomenPolicy::Weight(name);
            cache.spineAndNeckChain.push_back(entry);
        }
    }
	static std::int32_t FindGraphBoneIndex(const RE::BShkbAnimationGraph* graph, const RE::NiAVObject* bone)
	{
		if (!graph || !bone) return -1;
		const std::size_t boneNodeCount = graph->boneNodes.size();
		const std::size_t animCount = graph->numAnimBones > 0 ?
			std::min<std::size_t>(boneNodeCount, graph->numAnimBones) :
			boneNodeCount;

		for (std::size_t i = 0; i < animCount; ++i) {
			if (static_cast<const RE::NiAVObject*>(graph->boneNodes[i].node) == bone) {
				return static_cast<std::int32_t>(i);
			}
		}
		return -1;
	}

	static void RefreshHavokPoseBindings(RE::Actor* actor, ActorCache& cache)
	{
		if (!actor || cache.spineAndNeckChain.empty()) return;
		if (!cache.havokBindings.empty()) return;

		RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
		if (!actor->GetAnimationGraphManager(manager) || !manager) {
			return;
		}

		std::size_t graphsBound = 0;
		std::size_t bodyResolvedTotal = 0;
		std::size_t tailResolvedTotal = 0;
		for (const auto& graphPointer : manager->graphs) {
			auto* graph = graphPointer.get();
			if (!graph) continue;
			if (graph->holder && graph->holder != actor) continue;

			auto* character = std::addressof(graph->characterInstance);
			if (!character) continue;

			HavokPoseBinding binding;
			binding.bodyIndices.reserve(cache.spineAndNeckChain.size());
			binding.tailIndices.reserve(cache.tailChain.size());
			std::size_t bodyResolved = 0;

			for (const auto& entry : cache.spineAndNeckChain) {
				const std::int32_t index = FindGraphBoneIndex(graph, entry.bone);
				binding.bodyIndices.push_back(index);
				if (index >= 0) {
					++bodyResolved;
					++bodyResolvedTotal;
				}
			}
			binding.bodyComplete =
				bodyResolved == cache.spineAndNeckChain.size() &&
				!binding.bodyIndices.empty();
			for (const auto& entry : cache.tailChain) {
				const std::int32_t index = FindGraphBoneIndex(graph, entry.bone);
				binding.tailIndices.push_back(index);
				if (index >= 0) ++tailResolvedTotal;
			}

			cache.havokBindings.insert_or_assign(character, std::move(binding));
			++graphsBound;
		}

		if (!cache.debugHavokBindingLogged && !cache.havokBindings.empty()) {
			cache.debugHavokBindingLogged = true;
			const char* actorName = actor->GetName();
			SKSE::log::info(
				"[AME-V18][HAVOK-BIND] actor={:08X} name=\"{}\" graphs={} bodyResolved={}/{} tailResolved={}/{} owner=PRE_FOS_OUTPUT",
				actor->GetFormID(),
				actorName ? actorName : "",
				graphsBound,
				bodyResolvedTotal,
				cache.spineAndNeckChain.size() * graphsBound,
				tailResolvedTotal,
				cache.tailChain.size() * graphsBound);
		}
	}

	static HavokQuat BuildLocalWorldYawDelta(RE::NiAVObject* bone, float angle)
	{
		if (!bone || std::abs(angle) <= 1.0e-7f) {
			return {};
		}

		RE::NiMatrix3 worldYaw;
		worldYaw.SetEulerAnglesXYZ(0.0f, 0.0f, angle);

		RE::NiMatrix3 localDelta = worldYaw;
		if (bone->parent) {
			const RE::NiMatrix3& parentWorld = bone->parent->world.rotate;
			localDelta = parentWorld.Transpose() * worldYaw * parentWorld;
		}
		return MatrixToHavokQuat(localDelta);
	}

	static void PrepareHavokPoseDeltas(ActorCache& cache, float flexSign)
	{
		cache.havokBodyDeltas.clear();
		cache.havokTailDeltas.clear();
		cache.havokBodyDeltas.reserve(cache.spineAndNeckChain.size());
		cache.havokTailDeltas.reserve(cache.tailChain.size());
		cache.havokFlexSign = flexSign;

		for (const auto& entry : cache.spineAndNeckChain) {
			const float turnAngle = cache.currentFlex * entry.weight * flexSign;
			cache.havokBodyDeltas.push_back(BuildLocalWorldYawDelta(entry.bone, turnAngle));
		}
		for (const auto& entry : cache.tailChain) {
			const float tailAngle = cache.currentTailFlex * entry.weight * flexSign;
			cache.havokTailDeltas.push_back(BuildLocalWorldYawDelta(entry.bone, tailAngle));
		}

		cache.havokPosePrepared =
			std::abs(cache.currentFlex) >= 0.00035f ||
			std::abs(cache.currentTailFlex) >= 0.00035f;
	}

	static float ReadProcessRotationSpeedZ(RE::Actor* actor)
	{
		if (!actor) return 0.0f;
		auto* process = actor->GetActorRuntimeData().currentProcess;
		if (!process || !process->middleHigh) return 0.0f;
		const float value = process->middleHigh->rotationSpeed.z;
		return std::isfinite(value) ? value : 0.0f;
	}

	static float ComputeObservedYawRateRadS(RE::Actor* actor, ActorCache& cache, float dt)
	{
		if (!actor) return 0.0f;
		const float currentYawDeg = actor->GetAngleZ() * RAD2DEG;
		float yawRate = 0.0f;
		if (cache.yawObserved) {
			const float deltaYawDeg = NormalizeAngleDeg(currentYawDeg - cache.lastObservedYawDeg);
			yawRate = (deltaYawDeg * DEG2RAD) / std::max(dt, 1.0e-4f);
		}
		cache.lastObservedYawDeg = currentYawDeg;
		cache.yawObserved = true;
		return std::isfinite(yawRate) ? yawRate : 0.0f;
	}

	static float SelectByAbs(float currentValue, float candidateValue)
	{
		if (!std::isfinite(candidateValue)) return currentValue;
		return (std::abs(candidateValue) > std::abs(currentValue)) ? candidateValue : currentValue;
	}

	static float ComputeRawAngularVelocity(
		RE::Actor* actor,
		ActorCache& cache,
		const UniversalKinematics::SkeletonProfile& profile,
		float dt)
	{
		const float observedTurn = ComputeObservedYawRateRadS(actor, cache, dt);
		const float worldTurn = ReadProcessRotationSpeedZ(actor);
		float rawTurn = observedTurn;

		// Non-human humanoid-shaped bipeds may use completed world yaw; all other
		// creature profiles retain the historical rotationSpeed.z fallback.
		if (!UsesStableNonHumanBipedYawWitness(actor, profile) &&
			std::abs(worldTurn) <= MAX_ANGULAR_VELOCITY_RAD_S) {
			rawTurn = SelectByAbs(rawTurn, worldTurn);
		}
		if (!std::isfinite(rawTurn)) rawTurn = 0.0f;
		if (std::abs(rawTurn) < ANGULAR_DEADZONE_RAD_S) rawTurn = 0.0f;
		return std::clamp(rawTurn, -MAX_ANGULAR_VELOCITY_RAD_S, MAX_ANGULAR_VELOCITY_RAD_S);
	}

	static void UpdateMotionData(
		RE::Actor* actor,
		ActorCache& cache,
		MotionData& data,
		const UniversalKinematics::SkeletonProfile& profile,
		float dt)
	{
		const float rawAngularVelocity = ComputeRawAngularVelocity(actor, cache, profile, dt);
		const float angularAlpha = ExpAlpha(ANGULAR_RESPONSE, dt);
		cache.currentAngularVelocity += (rawAngularVelocity - cache.currentAngularVelocity) * angularAlpha;
		if (std::abs(cache.currentAngularVelocity) < ANGULAR_DEADZONE_RAD_S * 0.50f) cache.currentAngularVelocity = 0.0f;
		data.angularVelocity = cache.currentAngularVelocity;
	}

	enum class QuadrupedYawSource
	{
		None,
		Controller,
		ActorFallback
	};

	static const char* QuadrupedYawSourceName(QuadrupedYawSource source)
	{
		switch (source) {
		case QuadrupedYawSource::Controller: return "CONTROLLER";
		case QuadrupedYawSource::ActorFallback: return "ACTOR_FALLBACK";
		default: return "NONE";
		}
	}

	struct QuadrupedCapsuleTurn
	{
		float controllerYawRad = 0.0f;
		float actorYawRad = 0.0f;
		float controllerFrameDeltaRad = 0.0f;
		float actorFrameDeltaRad = 0.0f;
		float effectiveFrameDeltaRad = 0.0f;
		float observedRateRadS = 0.0f;
		float accumulatedAngleRad = 0.0f;
		float targetFlexRad = 0.0f;
		float idleTimer = 0.0f;
		QuadrupedYawSource source = QuadrupedYawSource::None;
		bool controllerValid = false;
		bool layerConflict = false;
		bool discontinuity = false;
		bool active = false;
	};

	struct PlayerMountTurn
	{
		float actorYawRad = 0.0f;
		float frameDeltaRad = 0.0f;
		float yawRateRadS = 0.0f;
		float targetFlexRad = 0.0f;
		bool discontinuity = false;
		bool active = false;
	};


	struct LocomotionOrientationIntent
	{
		Incessus::API::LocomotionSnapshotV1 snapshot{};
		MotionSourceClient::Source source = MotionSourceClient::Source::kNone;
		float targetFlexRad = 0.0f;
		bool snapshotAvailable = false;
		bool supportedActor = false;
		bool stationarySuppressed = false;
		bool authoredTurn = false;
		bool actionBypass = false;
		bool active = false;
	};

	static LocomotionOrientationIntent ComputeLocomotionOrientationIntent(RE::Actor* actor)
	{
		LocomotionOrientationIntent result;
		if (!actor) return result;

		MotionSourceClient::ResolvedSnapshot resolved{};
		result.snapshotAvailable = MotionSourceClient::QueryLocomotion(actor, resolved);
		if (!result.snapshotAvailable) return result;
		result.snapshot = resolved.snapshot;
		result.source = resolved.source;

		const auto flags = result.snapshot.flags;
		result.supportedActor = (flags & Incessus::API::kSupportedActor) != 0;
		if (!result.supportedActor) return result;

		const bool locomotion = (flags & Incessus::API::kGraphLocomotionActive) != 0;
		const bool steering = (flags & Incessus::API::kSteeringValid) != 0;
		const bool pivotBusy = (flags & Incessus::API::kPivotBusy) != 0;
		const bool actionBypass = (flags & Incessus::API::kActionBypassActive) != 0;
		result.actionBypass = actionBypass;
		result.authoredTurn = pivotBusy || (flags & (Incessus::API::kAuthoredRotationObserved |
			Incessus::API::kAuthoredRotationProgressing | Incessus::API::kNativeAuthoredYaw)) != 0;
		const float physicalSpeed = ReadPlanarVelocitySpeed(actor);
		const bool stationaryRelease = physicalSpeed < 8.0f &&
			std::abs(result.snapshot.graphSpeed) < 8.0f && result.snapshot.runtimeCarrierMask == 0;
		result.stationarySuppressed = stationaryRelease;
		// kDirectionalMovement is telemetry, not Orientation-Warp authority. R1
		// proved that AI/Pit-Wolf steering can be fully valid while this bit is not
		// asserted, so gating on it discarded the majority of useful snapshots.
		if (!locomotion || !steering || pivotBusy || actionBypass || stationaryRelease) {
			return result;
		}

		float error = result.snapshot.filteredErrorRadians;
		if (!std::isfinite(error)) return result;

		// The selected motion producer owns locomotion intent. Forma Motus only
		// consumes the read-only steering discrepancy and maps it to visual axial pose.
		// No root/capsule/graph/localTime write occurs here.
		error = NormalizeAngleRad(error);
		if (std::abs(error) < ORIENTATION_WARP_DEADZONE_RAD) error = 0.0f;

		result.targetFlexRad = std::clamp(
			error * ORIENTATION_WARP_RATIO,
			-ORIENTATION_WARP_LIMIT_RAD,
			ORIENTATION_WARP_LIMIT_RAD);
		result.active = std::abs(result.targetFlexRad) > 0.0001f;
		return result;
	}

	static bool ReadPhysicalCapsuleYaw(RE::Actor* actor, float& yawRad)
	{
		auto* controller = actor ? actor->GetCharController() : nullptr;
		if (!controller) return false;

		alignas(16) float values[4]{};
		_mm_store_ps(values, controller->forwardVec.quad);

		const float x = values[0];
		const float y = values[1];
		const float planarLength = std::hypot(x, y);
		if (!std::isfinite(planarLength) || planarLength <= 1.0e-5f) return false;

		// Descoberta confirmada pelos logs de Deer/Elk/Wolf/Fox/Rabbit/Skeever:
		// atan2(forwardVec.y, forwardVec.x) usa uma conven��o inversa e deslocada
		// em -90 graus em rela��o ao yaw mundial do actor. Comparar os dois deltas
		// crus fazia uma mesma rota��o parecer CONFLICT. Convertemos primeiro o
		// vetor Havok para a conven��o mundial usada pela pose procedural.
		const float controllerVectorYaw = std::atan2(y, x);
		yawRad = NormalizeAngleRad(-controllerVectorYaw - (PI * 0.5f));
		return std::isfinite(yawRad);
	}

	static void RebasePlayerMountRoute(RE::Actor* actor, ActorCache& cache, bool playerControlledMount)
	{
		if (!actor || cache.playerControlledMountRoute == playerControlledMount) return;

		cache.playerControlledMountRoute = playerControlledMount;
		cache.lastActorYawRad = NormalizeAngleRad(actor->data.angle.z);
		cache.actorYawInitialized = true;
		cache.lastCapsuleYawRad = 0.0f;
		cache.capsuleYawInitialized = false;
		cache.compositeTurnAngleRad = 0.0f;
		cache.capsuleTurnIdleTimer = 0.0f;
		cache.capsuleTurnActive = false;
		cache.flexVelocity = 0.0f;
		cache.quadrupedTravelFlexSign = QUADRUPED_FORWARD_FLEX_SIGN;
		cache.quadrupedTravelSignInitialized = false;
		cache.quadrupedTravelPendingSign = 0.0f;
		cache.quadrupedTravelPendingTimer = 0.0f;
		cache.lastBipedTelemetryTime = {};
		cache.lastPlayerMountTelemetryTime = {};

		SKSE::log::info(
			"[FormaMotus][MOUNT-ROUTE] actor={:08X} name=\"{}\" route={} yawWitness={} accumulation=false travelSign=false",
			actor->GetFormID(), actor->GetName() ? actor->GetName() : "",
			playerControlledMount ? "PLAYER_CONTROLLED_MOUNT" : "NPC_QUADRUPED",
			playerControlledMount ? "ACTOR_WORLD_DELTA" : "CONTROLLER_THEN_ACTOR_FALLBACK");
	}

	static PlayerMountTurn ComputePlayerMountTurn(
		RE::Actor* actor, ActorCache& cache, float dt, float morphScale, float pitchScale)
	{
		PlayerMountTurn result;
		if (!actor) return result;

		result.actorYawRad = NormalizeAngleRad(actor->data.angle.z);
		if (!cache.actorYawInitialized) {
			cache.lastActorYawRad = result.actorYawRad;
			cache.actorYawInitialized = true;
			return result;
		}

		result.frameDeltaRad = NormalizeAngleRad(result.actorYawRad - cache.lastActorYawRad);
		cache.lastActorYawRad = result.actorYawRad;
		if (!std::isfinite(result.frameDeltaRad)) {
			result.frameDeltaRad = 0.0f;
			return result;
		}

		// Mount/dismount, teleport and animation reattachments can change world yaw in
		// one frame. They are a rebase event, not body curvature.
		if (std::abs(result.frameDeltaRad) >= PLAYER_MOUNT_DISCONTINUITY_DELTA_RAD) {
			result.discontinuity = true;
			return result;
		}

		result.yawRateRadS = result.frameDeltaRad / std::max(dt, 1.0e-4f);
		if (!std::isfinite(result.yawRateRadS)) result.yawRateRadS = 0.0f;
		result.yawRateRadS = std::clamp(
			result.yawRateRadS, -PLAYER_MOUNT_MAX_YAW_RATE_RAD_S, PLAYER_MOUNT_MAX_YAW_RATE_RAD_S);
		if (std::abs(result.yawRateRadS) < PLAYER_MOUNT_YAW_RATE_DEADZONE_RAD_S) {
			result.yawRateRadS = 0.0f;
		}

		result.targetFlexRad = std::clamp(
			result.yawRateRadS * PLAYER_MOUNT_YAW_RATE_FLEX_GAIN * morphScale * pitchScale,
			-PLAYER_MOUNT_FLEX_LIMIT_RAD, PLAYER_MOUNT_FLEX_LIMIT_RAD);
		result.active = std::abs(result.targetFlexRad) > 0.0001f;
		return result;
	}

	static float ResolveQuadrupedEffectiveYawDelta(
		float controllerDeltaRad,
		bool controllerDeltaValid,
		float actorDeltaRad,
		bool actorDeltaValid,
		QuadrupedYawSource& outSource,
		bool& outConflict)
	{
		outSource = QuadrupedYawSource::None;
		outConflict = false;

		const bool controllerMoved = controllerDeltaValid &&
			std::abs(controllerDeltaRad) >= QUADRUPED_YAW_LAYER_NOISE_RAD;
		const bool actorMoved = actorDeltaValid &&
			std::abs(actorDeltaRad) >= QUADRUPED_YAW_LAYER_NOISE_RAD;

		// Autoridade simples: se a c�psula f�sica est� dispon�vel e se moveu,
		// ela � a fonte. Actor yaw nunca veta uma leitura v�lida do controller.
		if (controllerMoved) {
			outSource = QuadrupedYawSource::Controller;
			return controllerDeltaRad;
		}

		// Fallback somente quando a camada f�sica n�o entregou movimento �til.
		// Ap�s a convers�o espacial de ReadPhysicalCapsuleYaw, actor e controller
		// est�o na mesma conven��o mundial, portanto o delta do actor pode ser
		// usado diretamente sem invers�o artificial.
		if (actorMoved) {
			outSource = QuadrupedYawSource::ActorFallback;
			return actorDeltaRad;
		}

		return 0.0f;
	}

	static QuadrupedCapsuleTurn ComputeQuadrupedCapsuleTurn(RE::Actor* actor, ActorCache& cache, float dt)
	{
		QuadrupedCapsuleTurn result;
		if (!actor) return result;

		const float actorYawRad = NormalizeAngleRad(actor->data.angle.z);
		result.actorYawRad = actorYawRad;

		float controllerYawRad = 0.0f;
		result.controllerValid = ReadPhysicalCapsuleYaw(actor, controllerYawRad);
		if (result.controllerValid) {
			result.controllerYawRad = controllerYawRad;
		}

		bool actorDeltaValid = false;
		if (!cache.actorYawInitialized) {
			cache.lastActorYawRad = actorYawRad;
			cache.actorYawInitialized = true;
		} else {
			result.actorFrameDeltaRad = NormalizeAngleRad(actorYawRad - cache.lastActorYawRad);
			cache.lastActorYawRad = actorYawRad;
			actorDeltaValid = std::isfinite(result.actorFrameDeltaRad);
			if (!actorDeltaValid) result.actorFrameDeltaRad = 0.0f;
		}

		bool controllerDeltaValid = false;
		if (result.controllerValid) {
			if (!cache.capsuleYawInitialized) {
				cache.lastCapsuleYawRad = controllerYawRad;
				cache.capsuleYawInitialized = true;
			} else {
				result.controllerFrameDeltaRad =
					NormalizeAngleRad(controllerYawRad - cache.lastCapsuleYawRad);
				cache.lastCapsuleYawRad = controllerYawRad;
				controllerDeltaValid = std::isfinite(result.controllerFrameDeltaRad);
				if (!controllerDeltaValid) result.controllerFrameDeltaRad = 0.0f;
			}
		} else {
			// Se o controller reaparecer depois, o primeiro frame apenas rebasa a
			// camada; actor yaw continua podendo observar a rota��o nesse intervalo.
			cache.capsuleYawInitialized = false;
		}

		result.effectiveFrameDeltaRad = ResolveQuadrupedEffectiveYawDelta(
			result.controllerFrameDeltaRad,
			controllerDeltaValid,
			result.actorFrameDeltaRad,
			actorDeltaValid,
			result.source,
			result.layerConflict);

		if (!std::isfinite(result.effectiveFrameDeltaRad)) {
			result.effectiveFrameDeltaRad = 0.0f;
			result.source = QuadrupedYawSource::None;
			result.layerConflict = false;
		}

		result.observedRateRadS =
			std::abs(result.effectiveFrameDeltaRad) / std::max(dt, 1.0e-4f);

		// Teleporte/reload/snap imposs�vel: n�o contamine a manobra em curso.
		if (std::abs(result.effectiveFrameDeltaRad) >= CAPSULE_DISCONTINUITY_DELTA_RAD) {
			result.discontinuity = true;
			cache.compositeTurnAngleRad = 0.0f;
			cache.capsuleTurnIdleTimer = 0.0f;
			cache.capsuleTurnActive = false;
			return result;
		}

		if (!cache.capsuleTurnActive) {
			cache.compositeTurnAngleRad = 0.0f;

			if (result.observedRateRadS >= CAPSULE_TURN_START_RATE_RAD_S) {
				cache.capsuleTurnActive = true;
				cache.capsuleTurnIdleTimer = 0.0f;
				cache.compositeTurnAngleRad = result.effectiveFrameDeltaRad;
			}
		} else {
			cache.compositeTurnAngleRad += result.effectiveFrameDeltaRad;
			cache.compositeTurnAngleRad = std::clamp(
				cache.compositeTurnAngleRad,
				-CAPSULE_TURN_ACCUMULATION_LIMIT_RAD,
				CAPSULE_TURN_ACCUMULATION_LIMIT_RAD);

			const bool stillTurning =
				result.observedRateRadS > CAPSULE_TURN_END_RATE_RAD_S ||
				std::abs(result.effectiveFrameDeltaRad) >= QUADRUPED_YAW_LAYER_NOISE_RAD;

			if (!stillTurning) {
				cache.capsuleTurnIdleTimer += std::max(0.0f, dt);
			} else {
				cache.capsuleTurnIdleTimer = 0.0f;
			}

			if (cache.capsuleTurnIdleTimer >= CAPSULE_TURN_END_GRACE) {
				cache.capsuleTurnActive = false;
				cache.capsuleTurnIdleTimer = 0.0f;
				cache.compositeTurnAngleRad = 0.0f;
			}
		}

		result.active = cache.capsuleTurnActive;
		result.idleTimer = cache.capsuleTurnIdleTimer;

		if (result.active) {
			result.accumulatedAngleRad = cache.compositeTurnAngleRad;

			// A amplitude continua sendo exclusivamente angular; SPLIT01 usa 1.0x conforme ajuste solicitado.
			// A layer f�sica que fornece o �ngulo observado permanece inalterada.
			result.targetFlexRad = std::clamp(
				result.accumulatedAngleRad * CAPSULE_TURN_FLEX_RATIO,
				-CAPSULE_TURN_FLEX_LIMIT_RAD,
				CAPSULE_TURN_FLEX_LIMIT_RAD);
		}

		return result;
	}


	static void LogQuadrupedPipelineHeartbeat(
		RE::Actor* actor,
		const ActorCache& cache,
		const QuadrupedCapsuleTurn& turn,
		float targetFlex,
		float currentFlex,
		bool syntheticActive)
	{
		if (!actor) return;

		RE::NiPoint3 velocity{};
		actor->GetLinearVelocity(velocity);
		const float planarSpeed = std::hypot(velocity.x, velocity.y);

		const char* actorName = actor->GetName();
		const auto* race = actor->GetRace();
		const char* raceName = race ? race->GetFormEditorID() : nullptr;

		SKSE::log::info(
			"[AME-R4.4][PIPELINE] actor={:08X} name=\"{}\" race={} calls[update={} apply={}] "
			"yaw[source={} active={} conflict={} rateDegS={:.2f} accumDeg={:+.2f}] "
			"motion[speed={:.2f} vel=({:+.1f},{:+.1f},{:+.1f})] "
			"flex[targetDeg={:+.2f} currentDeg={:+.2f} synthetic={}] "
			"axialPose[owner=PRE_FOS_OUTPUT chain={} prepared={} bindings={}]",
			actor->GetFormID(), actorName ? actorName : "",
			raceName && raceName[0] ? raceName : "<unknown>",
			cache.debugUpdateCalls, cache.debugApplyCalls,
			QuadrupedYawSourceName(turn.source), turn.active, turn.layerConflict,
			turn.observedRateRadS * RAD2DEG, turn.accumulatedAngleRad * RAD2DEG,
			planarSpeed, velocity.x, velocity.y, velocity.z,
			targetFlex * RAD2DEG, currentFlex * RAD2DEG, syntheticActive,
			cache.spineAndNeckChain.size(), cache.havokPosePrepared, cache.havokBindings.size());
	}

	void MotionAnalyzer::Update(RE::Actor* actor, MotionData& data)
	{
		if (!actor) return;
		float worldTurn = ReadProcessRotationSpeedZ(actor);
		if (!std::isfinite(worldTurn) || std::abs(worldTurn) < ANGULAR_DEADZONE_RAD_S) worldTurn = 0.0f;
		data.angularVelocity = std::clamp(worldTurn, -MAX_ANGULAR_VELOCITY_RAD_S, MAX_ANGULAR_VELOCITY_RAD_S);
	}

	static bool ReadGraphBool(RE::Actor* actor, const char* name)
	{
		bool value = false;
		if (actor && name && name[0]) actor->GetGraphVariableBool(name, value);
		return value;
	}

	static bool ReadGraphIntNonZero(RE::Actor* actor, const char* name)
	{
		int value = 0;
		if (actor && name && name[0]) actor->GetGraphVariableInt(name, value);
		return value != 0;
	}

	static bool AnyGraphBoolTrue(RE::Actor* actor, std::initializer_list<const char*> names)
	{
		for (const auto* name : names) {
			if (ReadGraphBool(actor, name)) return true;
		}
		return false;
	}

	static bool AnyGraphIntNonZero(RE::Actor* actor, std::initializer_list<const char*> names)
	{
		for (const auto* name : names) {
			if (ReadGraphIntNonZero(actor, name)) return true;
		}
		return false;
	}

	static const char* FindBlockedAnimationReason(RE::Actor* actor, bool isHorse)
	{
        if(actor && actor->AsActorState()->GetKnockState()!=RE::KNOCK_STATE_ENUM::kNormal)return "ENGINE_KNOCK_GETUP";
		if (!actor) return "null-actor";

		static constexpr const char* kCommonBoolNames[] = {
			"bInRagdollState", "bIsRagdoll", "IsRagdoll", "bRagdoll",
			"bInKillMove", "bIsInKillMove", "IsInKillMove", "bKillMove",
			"bInFurniture", "bIsInFurniture", "IsInFurniture", "bFurniture",
			"bInWater", "bIsSwimming", "IsSwimming", "bSwimming",
			"bInJumpState", "bInAir", "bMidair", "IsInMidair"
		};
		for (const auto* name : kCommonBoolNames) {
			if (ReadGraphBool(actor, name)) return name;
		}

		if (!isHorse) {
			static constexpr const char* kMountBoolNames[] = {
				"bIsRiding", "bOnMount", "bMounted", "IsOnMount"
			};
			for (const auto* name : kMountBoolNames) {
				if (ReadGraphBool(actor, name)) return name;
			}
		}

		static constexpr const char* kCommonIntNames[] = {
			"iSitState", "iFurnitureState", "iRagdollState", "iKillMove"
		};
		for (const auto* name : kCommonIntNames) {
			if (ReadGraphIntNonZero(actor, name)) return name;
		}
		if (!isHorse && ReadGraphIntNonZero(actor, "iMountState")) return "iMountState";
		return nullptr;
	}

	static bool IsInBlockedAnimationState(RE::Actor* actor, bool isHorse)
	{
		return FindBlockedAnimationReason(actor, isHorse) != nullptr;
	}

	static void ResetCacheState(ActorCache& cache)
	{
		cache.currentFlex = 0.0f;
		cache.flexVelocity = 0.0f;
		cache.currentSpeedFlex = 0.0f;
		cache.currentAngularVelocity = 0.0f;
		cache.currentTailFlex = 0.0f;
		cache.lastCapsuleYawRad = 0.0f;
		cache.lastActorYawRad = 0.0f;
		cache.compositeTurnAngleRad = 0.0f;
		cache.capsuleTurnIdleTimer = 0.0f;
		cache.capsuleYawInitialized = false;
		cache.actorYawInitialized = false;
		cache.capsuleTurnActive = false;
		cache.capsuleControllerMissingLogged = false;
		cache.playerControlledMountRoute = false;
		cache.quadrupedTravelFlexSign = QUADRUPED_FORWARD_FLEX_SIGN;
		cache.quadrupedTravelSignInitialized = false;
		cache.quadrupedTravelPendingSign = 0.0f;
		cache.quadrupedTravelPendingTimer = 0.0f;
		cache.motion = {};
		cache.clockInitialized = false;
		cache.lastRoot = nullptr;
		cache.spineAndNeckChain.clear();
		cache.tailChain.clear();
		cache.havokBodyDeltas.clear();
		cache.havokTailDeltas.clear();
		cache.havokBindings.clear();
		cache.havokFlexSign = QUADRUPED_FORWARD_FLEX_SIGN;
		cache.havokPosePrepared = false;
		cache.debugHavokBindingLogged = false;
		cache.debugHavokApplyLogged = false;
		cache.debugHavokMissingLogged = false;
		cache.yawObserved = false;
		cache.lastObservedYawDeg = 0.0f;
		cache.diagnosticTimer = 0.0f;
		cache.lastBipedTelemetryTime = {};
		cache.lastPlayerMountTelemetryTime = {};
	}

	static bool IsActorEligible(RE::Actor* actor, bool isHorse)
	{
		if (!actor || actor->IsDead() || !actor->Is3DLoaded()) return false;
		if (actor->Get3D() == nullptr) return false;
		return !IsInBlockedAnimationState(actor, isHorse);
	}

	static void ApplyPhysics(RE::Actor* actor, ActorCache& cache, const UniversalKinematics::SkeletonProfile& profile, float dt)
	{
		++cache.debugApplyCalls;
		cache.debugSecondsSeen += std::max(0.0f, dt);

		const bool isBiped = IsBipedProfile(profile);
		const bool isSpiderAbdomen=IsSpiderAbdomenProfile(profile);
        const bool isAxialCapsule = IsAxialCapsuleProfile(profile)||isSpiderAbdomen;
		// Nunca determine o tipo lendo nomes atrav�s de ponteiros de ossos cacheados:
		// eles podem pertencer a um 3D anterior j� descarregado.
		const bool isHorse = isAxialCapsule && IsHorseLikeProfile(profile);
		const bool isPlayerControlledMount = isHorse && IsPlayerControlledMount(actor);
		RebasePlayerMountRoute(actor, cache, isPlayerControlledMount);

		// Layouts radiais (Mudcrab/aranhas) e outros corpos sem eixo longitudinal
		// expl�cito n�o participam deste m�dulo de tor��o. N�o constroem cadeia,
		// n�o recebem synthetic pose e n�o recebem yaw procedural.
		if (!isBiped && !isAxialCapsule) {
			cache.currentFlex = 0.0f;
			cache.flexVelocity = 0.0f;
				cache.currentTailFlex = 0.0f;
			cache.havokPosePrepared = false;
			cache.havokBodyDeltas.clear();
			cache.havokTailDeltas.clear();
			cache.havokBindings.clear();
			cache.lastRoot = nullptr;
			cache.spineAndNeckChain.clear();
			cache.tailChain.clear();
			if (!cache.loggedDebug) {
				cache.loggedDebug = true;
				SKSE::log::info(
					"[AnimalMotionEngine] actor={:08X} model=non-axial-no-spine-yaw locomotionMode={}",
					actor->GetFormID(), static_cast<int>(profile.locomotionMode));
			}
			return;
		}


		if (!IsActorEligible(actor, isHorse)) {
			cache.lastRoot = nullptr;
			cache.spineAndNeckChain.clear();
			cache.tailChain.clear();
			cache.havokBindings.clear();
			cache.havokBodyDeltas.clear();
			cache.havokTailDeltas.clear();
			cache.havokPosePrepared = false;
			cache.currentTailFlex = 0.0f;
			cache.flexVelocity = 0.0f;
			return;
		}

		auto* root = actor->Get3D();
		if (isBiped) RefreshBipedChain(cache, root, profile);
		else if(isSpiderAbdomen)RefreshSpiderAbdomenChain(cache,root);
        else RefreshQuadrupedLikeChain(cache, root, profile);

		if (cache.spineAndNeckChain.empty()) {
			if (!isBiped && ShouldLogDebugGate(cache)) {
				const char* actorName = actor->GetName();
				SKSE::log::error(
					"[AME-V8][T2-CHAIN-EMPTY] actor={:08X} name=\"{}\" spineNodes=\"{}\" chest=\"{}\" neck=\"{}\" head=\"{}\"",
					actor->GetFormID(), actorName ? actorName : "",
					JoinProfileSpineNodes(profile), profile.chestNode, profile.neckNode, profile.headNode);
			}
			return;
		}

		if (!isBiped && !cache.debugChainLogged) {
			cache.debugChainLogged = true;
			const char* actorName = actor->GetName();
			SKSE::log::info(
				"[AME-V8][T2-CHAIN] actor={:08X} name=\"{}\" bones={} detail=\"{}\"",
				actor->GetFormID(), actorName ? actorName : "", cache.spineAndNeckChain.size(),
				DescribeBoneChain(cache.spineAndNeckChain));
		}

		if (!isBiped && !cache.debugTailChainLogged) {
			cache.debugTailChainLogged = true;
			const char* actorName = actor->GetName();
			SKSE::log::info(
				"[AME-V18][TAIL-CHAIN] actor={:08X} name=\"{}\" bones={} detail=\"{}\"",
				actor->GetFormID(), actorName ? actorName : "", cache.tailChain.size(),
				DescribeBoneChain(cache.tailChain));
		}

		if (!cache.loggedDebug) {
			cache.loggedDebug = true;
			SKSE::log::info(
				"[AnimalMotionEngine] actor={:08X} model={} horse={} playerControlledMount={} chainBones={} tailBones={}",
				actor->GetFormID(),
				isBiped ? "biped-angular-velocity" :
					(isPlayerControlledMount ? "player-mount-world-yaw-rate" : "axial-capsule-world-yaw"),
				isHorse,
				isPlayerControlledMount,
				cache.spineAndNeckChain.size(),
				cache.tailChain.size());
		}

		const float morphScale = ComputeMorphScale(profile);
		const float pitchScale = (profile.pitchFactor > 0.0f) ? profile.pitchFactor : 1.0f;

		float targetFlex = 0.0f;
		QuadrupedCapsuleTurn quadrupedTurn;
		AxialPosePolicy::Source axialSource = AxialPosePolicy::Source::Neutral;
		PlayerMountTurn playerMountTurn;
		LocomotionOrientationIntent locomotionIntent;
		bool debugHeartbeat = false;
		bool debugSyntheticPoseActive = false;
		if (isBiped) {
			// Caminho original dos b�pedes, preservado sem altera��o funcional.
			const float actorSpeed = ReadActorSpeed(actor);
			const float speedFlexTarget = ComputeSpeedFlexScaleFromSpeed(actorSpeed);
			const float speedAlpha = ExpAlpha(SPEED_RESPONSE, dt);
			cache.currentSpeedFlex += (speedFlexTarget - cache.currentSpeedFlex) * speedAlpha;

			targetFlex = cache.motion.angularVelocity * FLEX_GAIN * morphScale * pitchScale;
			targetFlex *= cache.currentSpeedFlex;
			targetFlex = std::clamp(targetFlex, -FLEX_LIMIT, FLEX_LIMIT);
		} else {
			locomotionIntent = ComputeLocomotionOrientationIntent(actor);

			if (isPlayerControlledMount) {
				// Mounted-by-Player authority is deliberately independent from the autonomous
				// quadruped capsule observer. The actor's completed world-yaw delta is already
				// the rotation that Skyrim consumed for the mount this frame.
				playerMountTurn = ComputePlayerMountTurn(actor, cache, dt, morphScale, pitchScale);
				targetFlex = playerMountTurn.targetFlexRad;
			} else {
                // The physical observer owns its continuous turn episode. Sparse
                // authored-clip flags must not reset its accumulated body angle.
				quadrupedTurn = ComputeQuadrupedCapsuleTurn(actor, cache, dt);

				// Standing authored turns use measured capsule/actor yaw, never the
				// requested AI angle. Moving orientation remains producer-owned.
				const auto previousAxialSource = cache.axialPoseSource;
				axialSource = AxialPosePolicy::Select(locomotionIntent.supportedActor,
					locomotionIntent.authoredTurn, locomotionIntent.actionBypass,
					locomotionIntent.active, quadrupedTurn.active);
				targetFlex = axialSource == AxialPosePolicy::Source::PhysicalTurn ?
					quadrupedTurn.targetFlexRad :
					(axialSource == AxialPosePolicy::Source::Steering ? locomotionIntent.targetFlexRad : 0.0f);
				cache.axialPoseSource = axialSource;
				if (previousAxialSource != axialSource) {
					SKSE::log::info("[FORMA_R5_11][AXIAL_SOURCE] actor={:08X} source={} authored={} action={} physicalActive={} physicalDeltaDeg={:+.3f} targetDeg={:+.3f} rootWrites=ZERO",
						actor->GetFormID(), static_cast<int>(axialSource), locomotionIntent.authoredTurn,
						locomotionIntent.actionBypass, quadrupedTurn.active,
						quadrupedTurn.effectiveFrameDeltaRad * RAD2DEG, targetFlex * RAD2DEG);
				}
			}

			if ((isPlayerControlledMount &&
					(playerMountTurn.active || std::abs(playerMountTurn.frameDeltaRad) >= QUADRUPED_YAW_LAYER_NOISE_RAD)) ||
				(!isPlayerControlledMount &&
					(quadrupedTurn.active || std::abs(quadrupedTurn.effectiveFrameDeltaRad) >= QUADRUPED_YAW_LAYER_NOISE_RAD))) {
				cache.debugNaturalTurnEverSeen = true;
			}

			cache.debugHeartbeatTimer += std::max(0.0f, dt);
			if (cache.debugHeartbeatTimer >= DEBUG_PIPELINE_HEARTBEAT_PERIOD) {
				cache.debugHeartbeatTimer = 0.0f;
				debugHeartbeat = true;
			}

			if (debugHeartbeat && locomotionIntent.supportedActor && !isPlayerControlledMount) {
				const auto flags = locomotionIntent.snapshot.flags;
				const bool locomotion = (flags & Incessus::API::kGraphLocomotionActive) != 0;
				const bool steering = (flags & Incessus::API::kSteeringValid) != 0;
				const bool pivot = (flags & Incessus::API::kPivotBusy) != 0;
				const bool action = (flags & Incessus::API::kActionBypassActive) != 0;
				const bool directional = (flags & Incessus::API::kDirectionalMovement) != 0;
				const char* gate = !locomotion ? "NO_LOCOMOTION" : !steering ? "NO_STEERING" : pivot ? "PIVOT" : action ? "ACTION" : locomotionIntent.stationarySuppressed ? "STATIONARY" : locomotionIntent.active ? "ACTIVE" : "DEADZONE";
				SKSE::log::info(
					"[FormaMotus][ORIENTATION] actor={:08X} source={} gate={} profile=\"{}\" active={} rawErrorDeg={:+.2f} filteredErrorDeg={:+.2f} targetDeg={:+.2f} clamped={} gait={} graphSpeed={:.2f} directionalTelemetry={} carrierMask=0x{:02X} flags=0x{:08X} serial={} poseSign=+1",
					actor->GetFormID(), MotionSourceClient::SourceName(locomotionIntent.source), gate, locomotionIntent.snapshot.profile, locomotionIntent.active,
					locomotionIntent.snapshot.rawErrorRadians * RAD2DEG,
					locomotionIntent.snapshot.filteredErrorRadians * RAD2DEG,
					locomotionIntent.targetFlexRad * RAD2DEG,
					std::abs(locomotionIntent.snapshot.filteredErrorRadians) > ORIENTATION_WARP_LIMIT_RAD,
					static_cast<unsigned>(locomotionIntent.snapshot.desiredGait),
					locomotionIntent.snapshot.graphSpeed, directional,
					static_cast<unsigned>(locomotionIntent.snapshot.runtimeCarrierMask),
					locomotionIntent.snapshot.flags, locomotionIntent.snapshot.snapshotSerial);
			}

			// T5: injeta UMA pose procedural temporaria quando nao existe curva ativa.
			// Nao escreve yaw, rotMod, translation ou graph. O teste ocorre mesmo se
			// houver micro-ruido nos sensores, para provar o pipeline de pose.
			if (DEBUG_SYNTHETIC_ENABLED && !cache.debugSyntheticFinished &&
				cache.debugSecondsSeen >= DEBUG_SYNTHETIC_DELAY &&
				(!(isPlayerControlledMount ? playerMountTurn.active : quadrupedTurn.active) || cache.debugSyntheticStarted)) {
				if (!cache.debugSyntheticStarted) {
					cache.debugSyntheticStarted = true;
					cache.debugSyntheticTimer = 0.0f;
					SKSE::log::warn(
						"[AME-V8][T5-SYNTHETIC-START] actor={:08X} seen={:.2f}s naturalTurnEverSeen={} flexDeg=+{:.1f} then -{:.1f}",
						actor->GetFormID(), cache.debugSecondsSeen, cache.debugNaturalTurnEverSeen,
						DEBUG_SYNTHETIC_FLEX_RAD * RAD2DEG, DEBUG_SYNTHETIC_FLEX_RAD * RAD2DEG);
				}

				cache.debugSyntheticTimer += std::max(0.0f, dt);
				if (cache.debugSyntheticTimer < DEBUG_SYNTHETIC_HALF_DURATION) {
					cache.debugSyntheticFlexRad = DEBUG_SYNTHETIC_FLEX_RAD;
					debugSyntheticPoseActive = true;
				} else if (cache.debugSyntheticTimer < DEBUG_SYNTHETIC_HALF_DURATION * 2.0f) {
					cache.debugSyntheticFlexRad = -DEBUG_SYNTHETIC_FLEX_RAD;
					debugSyntheticPoseActive = true;
				} else {
					cache.debugSyntheticFlexRad = 0.0f;
					cache.debugSyntheticFinished = true;
					SKSE::log::warn(
						"[AME-V8][T5-SYNTHETIC-END] actor={:08X} boneWrites={} overwrites={}",
						actor->GetFormID(), cache.debugBoneWrites, cache.debugBoneOverwrites);
				}

				if (debugSyntheticPoseActive) {
					targetFlex = cache.debugSyntheticFlexRad;
				}
			}

			cache.diagnosticTimer += std::max(0.0f, dt);

			if (isPlayerControlledMount) {
				const auto now = Clock::now();
				if (cache.lastPlayerMountTelemetryTime == Clock::time_point{} ||
					std::chrono::duration<float>(now - cache.lastPlayerMountTelemetryTime).count() >= PLAYER_MOUNT_TELEMETRY_PERIOD) {
					cache.lastPlayerMountTelemetryTime = now;
					SKSE::log::info(
						"[FormaMotus][MOUNT-AME] actor={:08X} name=\"{}\" route=PLAYER_CONTROLLED_MOUNT witness=ACTOR_WORLD_DELTA accumulated=false travelSign=false yaw[deltaDeg={:+.3f} rateDegS={:+.2f} discontinuity={}] flex[targetDeg={:+.2f} currentDeg={:+.2f}]",
						actor->GetFormID(), actor->GetName() ? actor->GetName() : "",
						playerMountTurn.frameDeltaRad * RAD2DEG, playerMountTurn.yawRateRadS * RAD2DEG,
						playerMountTurn.discontinuity, targetFlex * RAD2DEG, cache.currentFlex * RAD2DEG);
				}
			}

			if (!isPlayerControlledMount && !quadrupedTurn.controllerValid && !cache.capsuleControllerMissingLogged) {
				cache.capsuleControllerMissingLogged = true;
				const char* actorName = actor->GetName();
				SKSE::log::warn(
					"[AnimalMotionEngine] controller-yaw unavailable actor={:08X} name=\"{}\"; world-convention actor-yaw fallback is active",
					actor->GetFormID(),
					actorName ? actorName : "");
			}
			if (!isPlayerControlledMount && quadrupedTurn.controllerValid) {
				cache.capsuleControllerMissingLogged = false;
			}

			const bool meaningfulTurn =
				!isPlayerControlledMount &&
				(quadrupedTurn.active ||
				 quadrupedTurn.discontinuity ||
				 std::abs(cache.currentFlex) >= 1.0f * DEG2RAD);
			if (meaningfulTurn && cache.diagnosticSamples < 14 && cache.diagnosticTimer >= 0.40f) {
				cache.diagnosticTimer = 0.0f;
				++cache.diagnosticSamples;
				const char* actorName = actor->GetName();
				const float controllerActorOffset = quadrupedTurn.controllerValid ?
					NormalizeAngleRad(quadrupedTurn.controllerYawRad - quadrupedTurn.actorYawRad) :
					0.0f;
				SKSE::log::info(
					"[AnimalMotionEngine] quadruped-yaw actor={:08X} name=\"{}\" horse={} controllerValid={} source={} conflict={} active={} discontinuity={} controllerYawDeg={:.1f} actorYawDeg={:.1f} offsetDeg={:+.1f} controllerDeltaDeg={:+.3f} actorDeltaDeg={:+.3f} effectiveDeltaDeg={:+.3f} gateRateDegS={:.2f} turnAngleDeg={:+.1f} ratio={:.2f} targetFlexDeg={:+.1f} idle={:.3f}",
					actor->GetFormID(),
					actorName ? actorName : "",
					isHorse,
					quadrupedTurn.controllerValid,
					QuadrupedYawSourceName(quadrupedTurn.source),
					quadrupedTurn.layerConflict,
					quadrupedTurn.active,
					quadrupedTurn.discontinuity,
					quadrupedTurn.controllerYawRad * RAD2DEG,
					quadrupedTurn.actorYawRad * RAD2DEG,
					controllerActorOffset * RAD2DEG,
					quadrupedTurn.controllerFrameDeltaRad * RAD2DEG,
					quadrupedTurn.actorFrameDeltaRad * RAD2DEG,
					quadrupedTurn.effectiveFrameDeltaRad * RAD2DEG,
					quadrupedTurn.observedRateRadS * RAD2DEG,
					quadrupedTurn.accumulatedAngleRad * RAD2DEG,
					CAPSULE_TURN_FLEX_RATIO,
					targetFlex * RAD2DEG,
					quadrupedTurn.idleTimer);
			}
		}

		if (isBiped) {
			const float flexAlpha = ExpAlpha(FLEX_RESPONSE, dt);
			cache.currentFlex += (targetFlex - cache.currentFlex) * flexAlpha;
			cache.flexVelocity = 0.0f;
		} else if (isPlayerControlledMount) {
			// No angle accumulator and no second-order spring: the mounted horse follows
			// completed yaw rate quickly while still filtering frame noise.
			const float flexAlpha = ExpAlpha(PLAYER_MOUNT_FLEX_RESPONSE, dt);
			const float mountTarget =
				(playerMountTurn.active || debugSyntheticPoseActive) ? targetFlex : 0.0f;
			cache.currentFlex += (mountTarget - cache.currentFlex) * flexAlpha;
			cache.currentFlex = std::clamp(
				cache.currentFlex, -PLAYER_MOUNT_FLEX_LIMIT_RAD, PLAYER_MOUNT_FLEX_LIMIT_RAD);
			cache.flexVelocity = 0.0f;
		} else {
			// V18: uma �nica mola governa ida, revers�o e retorno. Enquanto a c�psula
			// est� em manobra, o alvo � o yaw acumulado * 1.0. Quando a manobra termina,
			// o alvo torna-se zero e a MESMA velocidade de pose conduz o retorno.
			const bool orientationActive = axialSource != AxialPosePolicy::Source::Neutral;
			const float springTarget =
				(orientationActive || debugSyntheticPoseActive) ? targetFlex : 0.0f;
			const float springLimit = isSpiderAbdomen?SpiderAbdomenPolicy::limitDegrees*DEG2RAD:
                (axialSource == AxialPosePolicy::Source::Steering ? ORIENTATION_WARP_LIMIT_RAD : CAPSULE_TURN_FLEX_LIMIT_RAD);
			UpdateCapsuleFlexSpring(springTarget, dt, cache, springLimit);
		}
		if (isBiped && std::abs(cache.currentFlex) < 0.0005f) cache.currentFlex = 0.0f;

		// Contrapeso de cauda: usa o flex visual j� suavizado do tronco em sinal
		// oposto e responde mais lentamente, criando atraso sem tocar na c�psula.
		if (!isBiped && !cache.tailChain.empty()) {
			const float tailTarget = std::clamp(
				-cache.currentFlex * TAIL_COUNTER_FLEX_RATIO,
				-TAIL_COUNTER_FLEX_LIMIT_RAD,
				TAIL_COUNTER_FLEX_LIMIT_RAD);
			const float tailAlpha = ExpAlpha(TAIL_POSE_RESPONSE, dt);
			cache.currentTailFlex += (tailTarget - cache.currentTailFlex) * tailAlpha;
		} else {
			cache.currentTailFlex = 0.0f;
		}
		if (std::abs(cache.currentTailFlex) < 0.00035f) cache.currentTailFlex = 0.0f;

		const bool zeroBodyPose = cache.currentFlex == 0.0f;

		// A amplitude continua exclusivamente angular. Somente o SINAL anat�mico
		// depende do sentido longitudinal f�sico do quadr�pede. A r� preserva -1
		// (comportamento que j� estava correto); o avan�o usa +1.
		float flexSign = DEFAULT_YAW_SIGN;
		if (!isBiped) {
			// Player-controlled mounts and provider steering are already signed in world
			// yaw space. FORWARD/BACKWARD sign hysteresis belongs only to autonomous
			// physical-yaw fallback.
			if (isPlayerControlledMount || locomotionIntent.supportedActor) {
				flexSign = 1.0f;
			} else {
				flexSign = UpdateQuadrupedTravelFlexSign(actor, cache, dt);
			}
		}

		if (!isBiped) {
			// V18: a rota axial termina AQUI no dom�nio Ni. Nenhum Spine/Neck/Tail
			// recebe local.rotate nem Update() p�s-Havok. O frame apenas prepara os
			// deltas locais e a associa��o boneNodes -> poseLocal para o pr�ximo
			// PRE_FOS_OUTPUT. Descendentes faciais continuam animation/FoS-owned.
			RefreshHavokPoseBindings(actor, cache);
			PrepareHavokPoseDeltas(cache, flexSign);

			if (cache.havokPosePrepared && cache.havokBindings.empty() && !cache.debugHavokMissingLogged) {
				cache.debugHavokMissingLogged = true;
				const char* actorName = actor->GetName();
				SKSE::log::warn(
					"[AME-V18][HAVOK-BIND-MISSING] actor={:08X} name=\"{}\" chainBones={} tailBones={} action=SKIP_NI_FALLBACK",
					actor->GetFormID(),
					actorName ? actorName : "",
					cache.spineAndNeckChain.size(),
					cache.tailChain.size());
			}

			if (debugHeartbeat && !isPlayerControlledMount) {
				LogQuadrupedPipelineHeartbeat(
					actor, cache, quadrupedTurn, targetFlex, cache.currentFlex,
					debugSyntheticPoseActive);
			}
			return;
		}

		// B�pedes n�o humanos preservam a rota Ni j� validada. Nenhum comportamento b�pede �
		// movido para o bridge Havok nesta build.
		if (zeroBodyPose) {
			bool anyApplied = false;
			for (const auto& entry : cache.spineAndNeckChain) {
				if (entry.hasLastPose) {
					anyApplied = true;
					break;
				}
			}
			if (!anyApplied) {
				return;
			}
		}

		std::size_t bipedPoseWrites = 0;
		for (auto& entry : cache.spineAndNeckChain) {
			if (!entry.bone) continue;

			RE::NiMatrix3 baseRotate = entry.bone->local.rotate;
			if (entry.hasLastPose && MatrixNearlyEqual(baseRotate, entry.lastOutputRotate)) {
				baseRotate = entry.lastBaseRotate;
			}
			entry.lastBaseRotate = baseRotate;

			if (zeroBodyPose) {
				entry.bone->local.rotate = baseRotate;
				entry.hasLastPose = false;
				continue;
			}

			RE::NiMatrix3 delta;
			const float turnAngle = cache.currentFlex * entry.weight * flexSign;
			delta.SetEulerAnglesXYZ(0.0f, turnAngle, 0.0f);
			entry.bone->local.rotate = baseRotate * delta;
			entry.lastOutputRotate = entry.bone->local.rotate;
			entry.hasLastPose = true;
			++bipedPoseWrites;
		}

		RE::NiUpdateData updateData;
		for (auto& entry : cache.spineAndNeckChain) {
			if (entry.bone) {
				entry.bone->Update(updateData);
			}
		}

		if (UsesStableNonHumanBipedYawWitness(actor, profile) && bipedPoseWrites > 0) {
			const auto now = Clock::now();
			if (cache.lastBipedTelemetryTime == Clock::time_point{} ||
				std::chrono::duration<float>(now - cache.lastBipedTelemetryTime).count() >= 1.0f) {
				cache.lastBipedTelemetryTime = now;
				SKSE::log::info(
					"[FormaMotus][BIPED-AME] actor={:08X} role=NONHUMAN_BIPED stableYawWitness=ACTOR_WORLD_DELTA route=NI_POST_UPDATE chain={} angularRateDegS={:+.2f} flexDeg={:+.2f} poseWrites={}",
					actor->GetFormID(), cache.spineAndNeckChain.size(),
					cache.motion.angularVelocity * RAD2DEG, cache.currentFlex * RAD2DEG,
					bipedPoseWrites);
			}
		}

	}



	void ApplyPreFootIkPose(
		RE::FormID actorID,
		RE::BShkbAnimationGraph* graph,
		FeetOfSkyrimBridge::PoseContext& context)
	{
		if (actorID == 0 || !graph || !context.pose || context.poseCount <= 0) return;

		std::shared_ptr<ActorCache> cachePtr;
		{
			std::shared_lock lock(g_cacheMutex);
			const auto it = g_cache.find(actorID);
			if (it == g_cache.end()) return;
			cachePtr = it->second;
		}
		if (!cachePtr) return;

		std::scoped_lock cacheLock(cachePtr->mutex);
		if (!cachePtr->havokPosePrepared) return;

		auto* character = std::addressof(graph->characterInstance);
		const auto bindingIt = cachePtr->havokBindings.find(character);
		if (bindingIt == cachePtr->havokBindings.end()) return;
		auto& binding = bindingIt->second;
		if (!binding.bodyComplete) return;

		std::size_t bodyWrites = 0;
		std::size_t tailWrites = 0;
		const std::size_t bodyCount = std::min(binding.bodyIndices.size(), cachePtr->havokBodyDeltas.size());
		for (std::size_t i = 0; i < bodyCount; ++i) {
			const int index = binding.bodyIndices[i];
			if (index < 0 || index >= context.poseCount) continue;
			float* q = context.pose + index * 12 + 4;
			const HavokQuat base = NormalizeHavokQuat({ q[0], q[1], q[2], q[3] });
			const HavokQuat warped = MultiplyHavokQuat(cachePtr->havokBodyDeltas[i], base);
			q[0] = warped.x; q[1] = warped.y; q[2] = warped.z; q[3] = warped.w;
			FeetOfSkyrimBridge::MarkBoneValid(context, index);
			++bodyWrites;
		}

		const std::size_t tailCount = std::min(binding.tailIndices.size(), cachePtr->havokTailDeltas.size());
		for (std::size_t i = 0; i < tailCount; ++i) {
			const int index = binding.tailIndices[i];
			if (index < 0 || index >= context.poseCount) continue;
			float* q = context.pose + index * 12 + 4;
			const HavokQuat base = NormalizeHavokQuat({ q[0], q[1], q[2], q[3] });
			const HavokQuat warped = MultiplyHavokQuat(cachePtr->havokTailDeltas[i], base);
			q[0] = warped.x; q[1] = warped.y; q[2] = warped.z; q[3] = warped.w;
			FeetOfSkyrimBridge::MarkBoneValid(context, index);
			++tailWrites;
		}

		if (!binding.successLogged && bodyWrites > 0) {
			binding.successLogged = true;
			cachePtr->debugHavokApplyLogged = true;
			SKSE::log::info(
				"[FormaMotus][PRE-FOS-POSE] actor={:08X} poseCount={} bodyWrites={} tailWrites={} flexDeg={:+.2f} flexVelDegS={:+.2f} poseSign={:+.0f} next=FEET_OF_SKYRIM",
				actorID, context.poseCount, bodyWrites, tailWrites,
				cachePtr->currentFlex * RAD2DEG, cachePtr->flexVelocity * RAD2DEG,
				cachePtr->havokFlexSign);
		}
	}

	void ApplyHavokPose(RE::FormID actorID, RE::hkbCharacter* character)
	{
		if (actorID == 0 || !character) return;

		std::shared_ptr<ActorCache> cachePtr;
		{
			std::shared_lock lock(g_cacheMutex);
			const auto it = g_cache.find(actorID);
			if (it == g_cache.end()) return;
			cachePtr = it->second;
		}
		if (!cachePtr) return;

		std::scoped_lock cacheLock(cachePtr->mutex);
		if (!cachePtr->havokPosePrepared) return;

		const auto bindingIt = cachePtr->havokBindings.find(character);
		if (bindingIt == cachePtr->havokBindings.end()) return;

		auto& binding = bindingIt->second;
		if (!binding.bodyComplete) {
			if (!cachePtr->debugHavokMissingLogged) {
				cachePtr->debugHavokMissingLogged = true;
				SKSE::log::warn(
					"[AME-V18][HAVOK-INCOMPLETE] actor={:08X} bodyTargets={} action=SKIP_NI_FALLBACK",
					actorID,
					binding.bodyIndices.size());
			}
			return;
		}

		if (!character->poseLocal || character->numPoseLocal <= 0 ||
			character->numPoseLocal > 2048) {
			return;
		}

		// CommonLib keeps poseLocal opaque (const void**) even though Skyrim's
		// hkbCharacter stores a contiguous hkQsTransform array at this address.
		// We mutate rotation only; translation and scale remain animation-owned.
		auto* pose = const_cast<RE::hkQsTransform*>(
			reinterpret_cast<const RE::hkQsTransform*>(character->poseLocal));
		if (!pose) return;

		const std::int32_t poseCount = character->numPoseLocal;
		std::size_t bodyWrites = 0;
		std::size_t tailWrites = 0;

		const std::size_t bodyCount = std::min(
			binding.bodyIndices.size(),
			cachePtr->havokBodyDeltas.size());
		for (std::size_t i = 0; i < bodyCount; ++i) {
			const std::int32_t index = binding.bodyIndices[i];
			if (index < 0 || index >= poseCount) continue;

			const auto& delta = cachePtr->havokBodyDeltas[i];
			const HavokQuat base = ReadHavokQuat(pose[index].rotation);
			WriteHavokQuat(pose[index].rotation, MultiplyHavokQuat(delta, base));
			++bodyWrites;
		}

		const std::size_t tailCount = std::min(
			binding.tailIndices.size(),
			cachePtr->havokTailDeltas.size());
		for (std::size_t i = 0; i < tailCount; ++i) {
			const std::int32_t index = binding.tailIndices[i];
			if (index < 0 || index >= poseCount) continue;

			const auto& delta = cachePtr->havokTailDeltas[i];
			const HavokQuat base = ReadHavokQuat(pose[index].rotation);
			WriteHavokQuat(pose[index].rotation, MultiplyHavokQuat(delta, base));
			++tailWrites;
		}

		if (!binding.successLogged && bodyWrites > 0) {
			binding.successLogged = true;
			cachePtr->debugHavokApplyLogged = true;
			SKSE::log::info(
				"[AME-V18][HAVOK-POSE] actor={:08X} poseCount={} bodyWrites={} tailWrites={} flexDeg={:+.2f} flexVelDegS={:+.2f} ratio={:.2f}",
				actorID,
				poseCount,
				bodyWrites,
				tailWrites,
				cachePtr->currentFlex * RAD2DEG,
				cachePtr->flexVelocity * RAD2DEG,
				CAPSULE_TURN_FLEX_RATIO);
		}
	}


	void Update(RE::Actor* actor)
	{
		if (!ActorDomain::IsFormaNonHuman(actor)) return;

		static std::once_flag buildLogOnce;
		std::call_once(buildLogOnce, []() {
			SKSE::log::info(
				"[AnimalMotionEngine] BUILD {} | heritage=UK_PROCEDURAL_ANIMATIONS_SPLIT01 ownership=NON_HUMAN_ONLY creatureLocomotionIntent=true physicalYawFallback=true axialOnly=true radialExcluded=true axialPoseOwner=PRE_FOS_OUTPUT orientationCapDeg={:.1f} physicalRatio={:.2f} physicalCapDeg={:.1f} springOmega={:.2f} springZeta={:.2f} maxFlexSpeedDegS={:.1f} tailCounter=true tailResponse={:.2f}",
				BUILD_SIGNATURE,
				ORIENTATION_WARP_LIMIT_RAD * RAD2DEG,
				CAPSULE_TURN_FLEX_RATIO,
				CAPSULE_TURN_FLEX_LIMIT_RAD * RAD2DEG,
				CAPSULE_FLEX_SPRING_OMEGA,
				CAPSULE_FLEX_SPRING_DAMPING_RATIO,
				CAPSULE_FLEX_MAX_SPEED_RAD_S * RAD2DEG,
				TAIL_POSE_RESPONSE);
			SKSE::log::info(
				"[FormaMotus][R5.7] AME ownership=NON_HUMAN_ONLY; Player/ActorTypeNPC actors never enter this module; non-human Spine/Neck/Tail deltas remain PRE_FOS_OUTPUT; FullBodyIK owns foot lock");
		});

		const RE::FormID id = actor->GetFormID();
		std::shared_ptr<ActorCache> cachePtr;
		{
			std::unique_lock lock(g_cacheMutex);
			auto& slot = g_cache[id];
			if (!slot) slot = std::make_shared<ActorCache>();
			cachePtr = slot;
		}
		if (!cachePtr) return;

		// O CrashLogger mostrou esta rotina numa ScrapHeap JobThread. O mutex
		// impede que outro Update/Reset limpe ou percorra o mesmo vector ao mesmo tempo.
		std::scoped_lock cacheLock(cachePtr->mutex);
		++cachePtr->debugUpdateCalls;

		// Fa�a a valida��o b�sica ANTES de pedir o perfil. A identifica��o do
		// esqueleto pode percorrer nomes como "ElkLFrontHoof"; n�o devemos iniciar
		// essa leitura enquanto o 3D est� ausente ou sendo descartado.
		if (actor->IsDead() || !actor->Is3DLoaded() || actor->Get3D() == nullptr) {
			if (ShouldLogDebugGate(*cachePtr)) {
				const char* actorName = actor->GetName();
				SKSE::log::warn(
					"[AME-V8][T1-ENTRY-GATE] actor={:08X} name=\"{}\" dead={} loaded3D={} root={} updateCalls={}",
					actor->GetFormID(), actorName ? actorName : "", actor->IsDead(),
					actor->Is3DLoaded(), actor->Get3D() != nullptr, cachePtr->debugUpdateCalls);
			}
			ResetCacheState(*cachePtr);
			return;
		}

		const auto* profile = UKClient::GetProfileForActor(actor);
		if (!profile) {
			if (ShouldLogDebugGate(*cachePtr)) {
				const char* actorName = actor->GetName();
				const auto* race = actor->GetRace();
				const char* raceName = race ? race->GetFormEditorID() : nullptr;
				SKSE::log::error(
					"[AME-V8][T1-PROFILE-MISSING] actor={:08X} name=\"{}\" race={} updateCalls={}",
					actor->GetFormID(), actorName ? actorName : "",
					raceName && raceName[0] ? raceName : "<unknown>", cachePtr->debugUpdateCalls);
			}
			ResetCacheState(*cachePtr);
			return;
		}

		const bool isBiped = IsBipedProfile(*profile);
		const bool isAxialCapsule = IsAxialCapsuleProfile(*profile);
		// IMPORTANTE: n�o chamar IsHorseLikeChain aqui. A cadeia pode conter
		// NiAVObject* do 3D anterior; at� ler bone->name j� seria use-after-free.
		const bool isHorse = isAxialCapsule && IsHorseLikeProfile(*profile);

		if (!cachePtr->debugProfileLogged) {
			cachePtr->debugProfileLogged = true;
			const char* actorName = actor->GetName();
			const auto* race = actor->GetRace();
			const char* raceName = race ? race->GetFormEditorID() : nullptr;
			const bool playerControlledMount = isHorse && IsPlayerControlledMount(actor);
			SKSE::log::info(
				"[AME-V11][T1-PROFILE] actor={:08X} name=\"{}\" race={} locomotionMode={} bipedMotion={} humanoidLike={} stableBipedWorldYaw={} axialCapsule={} radialExcluded={} horse={} playerControlledMount={} bodyLength={:.1f} bodyWidth={:.1f} pitchFactor={:.3f} spine=\"{}\" chest=\"{}\" neck=\"{}\" head=\"{}\"",
				actor->GetFormID(), actorName ? actorName : "",
				raceName && raceName[0] ? raceName : "<unknown>",
				static_cast<int>(profile->locomotionMode), isBiped, profile->isHumanoidLikeUpperBody, UsesStableNonHumanBipedYawWitness(actor, *profile),
				isAxialCapsule, IsNonAxialRadialProfile(*profile), isHorse, playerControlledMount,
				profile->bodyLength, profile->bodyWidth, profile->pitchFactor,
				JoinProfileSpineNodes(*profile), profile->chestNode, profile->neckNode, profile->headNode);
		}

		if (!IsActorEligible(actor, isHorse)) {
			if (ShouldLogDebugGate(*cachePtr)) {
				const char* actorName = actor->GetName();
				const char* reason = FindBlockedAnimationReason(actor, isHorse);
				SKSE::log::debug(
					"[AME-R4.4][ELIGIBILITY-GATE] actor={:08X} name=\"{}\" reason={} dead={} loaded3D={} root={} updateCalls={}",
					actor->GetFormID(), actorName ? actorName : "", reason ? reason : "basic-gate",
					actor->IsDead(), actor->Is3DLoaded(), actor->Get3D() != nullptr, cachePtr->debugUpdateCalls);
			}
			ResetCacheState(*cachePtr);
			return;
		}

		const float dt = ComputeDeltaTime(*cachePtr);
		UpdateMotionData(actor, *cachePtr, cachePtr->motion, *profile, dt);
		ApplyPhysics(actor, *cachePtr, *profile, dt);
	}

	void Clear()
	{
		std::unique_lock lock(g_cacheMutex);
		g_cache.clear();
	}
}
