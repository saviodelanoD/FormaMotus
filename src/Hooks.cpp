#include "PCH.h"
#include "Hooks.h"
#include "ActorDomain.h"
#include "AnimalMotionEngine.h"
#include "AngularCurveModule.h"
#include "SpeedGovernorModule.h"
#include "UKClient.h"
#include "MotionSourceClient.h"
#include "MotusClient.h"
#include "FeetOfSkyrimBridge.h"
#include "StrideWarpModule.h"
#include "SlopeWarpModule.h"
#include "RE/B/BShkbAnimationGraph.h"
#include <cstring>
#include <Windows.h>
#include <mutex>
#include <unordered_set>

namespace Hooks
{
    namespace
    {

        bool IsInSpecialInteraction(RE::Actor* actor)
        {
            if (!actor) return false;

            auto* actorState = actor->AsActorState();
            if (actorState && actorState->GetSitSleepState() != RE::SIT_SLEEP_STATE::kNormal) {
                return true;
            }
            if (actor->GetOccupiedFurniture().get() != nullptr) {
                return true;
            }
            bool isSynced = false;
            if (actor->GetGraphVariableBool("bIsSyncedAnim", isSynced) && isSynced) {
                return true;
            }
            if (actor->IsInKillMove() || actor->IsDead()) {
                return true;
            }
            return actorState && actorState->IsBleedingOut();
        }

        bool IsActorValidForStack(RE::Actor* actor)
        {
            if (!actor || !actor->Is3DLoaded() || actor->IsDisabled()) {
                return false;
            }
            return UKClient::GetProfileForActor(actor) != nullptr;
        }

        void RunProceduralStack(RE::Actor* actor)
        {
            if (!ActorDomain::IsFormaNonHuman(actor)) return;
            if (!IsActorValidForStack(actor)) return;
            if (IsInSpecialInteraction(actor)) return;

            auto* actorState = actor->AsActorState();
            const bool isSwimming = actorState && actorState->IsSwimming();

            // Grounded boundary witness: proves whether grounded bipeds/quadrupeds ever
            // reach the Foot IK driver boundary used by our PRE_FOS_OUTPUT stack.
            FeetOfSkyrimBridge::ObserveGroundedActor(actor);

            // Heritage ordering retained from UK_ProceduralAnimations SPLIT_01.
            AngularCurveModule::UpdateActor(actor);
            if (!isSwimming) {
                // This prepares the NEXT generated-output warp. The inherited boundary intentionally
                // consumes the last completed frame at PRE_FOS_OUTPUT, avoiding
                // mutation of Incessus root/capsule state during Character::Update.
                AnimalMotionEngine::Update(actor);
                // R4.5: upper-body obstacle/shoulder posing was detached from Forma Motus.
            }
        }

        // Skyrim 1.6.640 hkbBehaviorGraph::Generate, vslot 0x17:
        // +A27A30 copies activeChildrenOutput[0] to output (R8 -> R9).
        // CommonLib's declaration omits these native parameters; do not call it.
        struct RadialBehaviorGenerate
        {
            static void thunk(RE::hkbBehaviorGraph* node, const RE::hkbContext& context,
                              const void* const* children, void* output, float timeOffset)
            {
                func(node, context, children, output, timeOffset);
                if (!context.character || !output || context.character->behaviorGraph.get() != node) return;
                auto* graph = SKSE::stl::adjust_pointer<RE::BShkbAnimationGraph>(context.character, -0xC0);
                auto* reference = static_cast<RE::TESObjectREFR*>(graph->holder);
                auto* actor = reference ? reference->As<RE::Actor>() : nullptr;
                if (!ActorDomain::IsFormaNonHuman(actor)) return;
                const auto* profile = UKClient::GetProfileForActor(actor);
                if (!profile || profile->locomotionMode != UniversalKinematics::LocomotionMode::GroundedRadial) return;
                FeetOfSkyrimBridge::PoseContext pose{};
                const bool ready=FeetOfSkyrimBridge::AcquirePoseContext(graph,output,pose,false);
                {
                    std::scoped_lock lock(diagMutex);
                    if(seen.insert(actor->GetFormID()).second)
                        SKSE::log::info("[FORMA_R5_13][RADIAL_OUTPUT] actor={:08X} callback=true poseReady={} poseCount={} footDriverRequired=false",actor->GetFormID(),ready,pose.poseCount);
                }
                if(ready)AnimalMotionEngine::ApplyPreFootIkPose(actor->GetFormID(),graph,pose);
            }
            static inline std::mutex diagMutex;
            static inline std::unordered_set<RE::FormID> seen;
            static void ClearDiagnostics(){std::scoped_lock lock(diagMutex);seen.clear();}
            static inline REL::Relocation<decltype(thunk)> func;
            static bool Install()
            {
                if (REL::Module::get().version() != REL::Version{1,6,640,0}) return false;
                REL::Relocation<std::uintptr_t> vt{ RE::VTABLE_hkbBehaviorGraph[0] };
                const auto target = *reinterpret_cast<const std::uintptr_t*>(vt.address()+0x17*sizeof(void*));
                constexpr std::uint8_t expected[]{0x49,0x8B,0x08,0x4C,0x3B,0xC9};
                const bool native=target==REL::Module::get().base()+0xA27A30 &&
                    std::memcmp(reinterpret_cast<const void*>(target),expected,sizeof(expected))==0;
                HMODULE owner=nullptr;
                GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS|GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                                   reinterpret_cast<LPCWSTR>(target),&owner);
                const auto oarModule=GetModuleHandleW(L"OpenAnimationReplacer.dll");
                // OAR Hooks.h installs this exact slot and Hooks.cpp forwards
                // the same five-argument Generate ABI before its preview callback.
                // Keep OAR's owner; accept only that known module or native code.
                const bool oar=oarModule && owner==oarModule;
                if(!native&&!oar){
                    SKSE::log::warn("[FORMA_R5_13][RADIAL_GENERATE] unavailable target=0x{:X} owner=0x{:X} reason=UNKNOWN_OWNER writes=ZERO",target,reinterpret_cast<std::uintptr_t>(owner));return false;
                }
                func=vt.write_vfunc(0x17,thunk);
                SKSE::log::info("[FORMA_R5_13][RADIAL_GENERATE] installed owner={} target=0x{:X} topBehaviorOnly=true footDriverRequired=false rootWrites=ZERO",native?"NATIVE":"OpenAnimationReplacer",target);
                return true;
            }
        };

        struct Character_Update
        {
            static void thunk(RE::Character* a_this, float a_delta)
            {
                func(a_this, a_delta);
                RunProceduralStack(a_this);
            }
            static inline REL::Relocation<decltype(thunk)> func;
        };
    }

    void ResetRuntimeState()
    {
        RadialBehaviorGenerate::ClearDiagnostics();
        AnimalMotionEngine::Clear();
        AngularCurveModule::ClearCache();
        SpeedGovernorModule::Clear();
        StrideWarpModule::Clear();
        SlopeWarpModule::Clear();
        FeetOfSkyrimBridge::Reset();
    }

    void Install()
    {
        // R5 full-product contract. Fail closed: a partial pose stack would make
        // visual diagnosis impossible and could compete with the final IK owner.
        if (!UKClient::Initialize()) {
            SKSE::log::critical("[FormaMotus][DEPENDENCY] UniversalKinematics unavailable; hooks not installed.");
            return;
        }
        const bool creatureMotionReady = MotionSourceClient::Initialize();
        if (!creatureMotionReady) {
            SKSE::log::warn(
                "[FormaMotus][DEPENDENCY] Incessus unavailable; non-human AME keeps its physical-yaw fallback, but provider-authored creature Orientation/Stride references are unavailable.");
        }
        if (!MotusClient::Initialize()) {
            SKSE::log::critical("[FormaMotus][DEPENDENCY] Motus Continuus Pose Kinematics SDK unavailable; hooks not installed.");
            return;
        }
        const bool slopeReady = SlopeWarpModule::Initialize();
        if (!slopeReady) {
            SKSE::log::warn("[FormaMotus][SLOPE] Terrain SDK unavailable; core Forma layers remain active and Slope Warp will retry lazily.");
        }
        if (!FeetOfSkyrimBridge::Initialize()) {
            SKSE::log::critical("[FormaMotus][DEPENDENCY] Feet of Skyrim unavailable; hooks not installed.");
            return;
        }

        ResetRuntimeState();

        // Feet of Skyrim installs its driver hooks from SKSEPlugin_Load. Forma
        // Motus installs at kDataLoaded and chains immediately before those
        // existing targets. No FoS binary/source is replaced.
        if (!FeetOfSkyrimBridge::InstallPreHooks()) {
            SKSE::log::critical("[FormaMotus][DEPENDENCY] Feet of Skyrim call-chain integration failed; hooks not installed.");
            return;
        }

        RadialBehaviorGenerate::Install();

        REL::Relocation<std::uintptr_t> vtblCharacter{ RE::VTABLE_Character[0] };
        Character_Update::func = vtblCharacter.write_vfunc(0xAD, Character_Update::thunk);

        // R1's hkbBehaviorGraph::Update POST hook is deliberately gone. The
        // complete creature order is now one-way and deterministic:
        // generated pose -> Orientation -> Stride -> Slope -> Feet of Skyrim. Foot lock is external.
        SKSE::log::info(
            "[FormaMotus] R5.8 hooks installed | ownership=NON_HUMAN_ONLY | Player/ActorTypeNPC=EXCLUDED | creatureProvider[Incessus={}] | UK=READY | MotusPoseSDK=READY | TerrainSDK={} | FeetOfSkyrim=READY | poseOrder=ORIENTATION->STRIDE->SLOPE->FOS | footLockOwner=FULLBODYIK | axialOwner=PRE_FOS_OUTPUT",
            MotionSourceClient::HasIncessus() ? "READY" : "MISSING_SOFT",
            slopeReady ? "READY" : "MISSING_SOFT");
    }
}

