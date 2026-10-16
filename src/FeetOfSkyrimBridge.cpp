#include "PCH.h"
#include "FeetOfSkyrimBridge.h"
#include "AnimalMotionEngine.h"
#include "StrideWarpModule.h"
#include "UKClient.h"

#include "RE/B/BShkbAnimationGraph.h"
#include <UniversalKinematics.h>

#include <Windows.h>
#include <array>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cctype>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <unordered_set>

namespace FeetOfSkyrimBridge
{
    namespace
    {
        // Current Feet of Skyrim / hkbFootIk layout. These values are deliberately
        // isolated in this bridge so a future FoS layout update never leaks into
        // the pose-warp algorithms themselves.
        constexpr std::uint64_t kCharacterInGraph = 0xC0;
        constexpr std::uint64_t kGraphSetup = 0x110;
        constexpr std::uint64_t kGraphHolder = 0x210;
        constexpr std::uint64_t kHolderFormType = 0x1A;
        constexpr std::uint8_t kActorCharacterFormType = 62;
        constexpr std::uint64_t kSetupSkeleton = 0x20;
        constexpr std::uint64_t kSetupCharData = 0x40;
        constexpr std::uint64_t kCharDataDriverInfo = 0x88;
        constexpr std::uint64_t kCharDataModelForward = 0x40;
        constexpr std::uint64_t kCharDataModelRight = 0x50;
        constexpr std::uint64_t kInfoLegData = 0x10;
        constexpr std::uint64_t kInfoLegCount = 0x18;
        constexpr std::uint64_t kInfoLockFeetWhenPlanted = 0x40;
        constexpr std::uint64_t kLegStride = 0x60;
        constexpr std::uint64_t kLegKneeAxis = 0x10;
        constexpr std::uint64_t kLegPlantedHeight = 0x30;
        constexpr std::uint64_t kLegRaisedHeight = 0x34;
        constexpr std::uint64_t kLegMaxHeight = 0x38;
        constexpr std::uint64_t kLegMinHeight = 0x3C;
        constexpr std::uint64_t kLegHipIndex = 0x4C;
        constexpr std::uint64_t kLegKneeIndex = 0x4E;
        constexpr std::uint64_t kLegAnkleIndex = 0x50;
        constexpr std::uint64_t kSkeletonParentData = 0x18;
        constexpr std::uint64_t kSkeletonParentCount = 0x20;
        constexpr std::uint64_t kSkeletonBoneData = 0x28;
        constexpr std::uint64_t kSkeletonBoneCount = 0x30;
        constexpr std::uint64_t kBoneStride = 0x10;
        constexpr std::uint64_t kBoneNameMask = ~1ull;

        constexpr std::uint64_t kTrackTable = 0x10;
        constexpr std::uint64_t kTrackEntryBytes = 0x10;
        constexpr int kPoseTrack = 2;

        constexpr REL::RelocationID kSceneModifierCaller{ 58795, 59458 };
        constexpr REL::RelocationID kPairedSyncCaller{ 62636, 63581 };

        using DriverUpdateFn = void (*)(void*, void*, void*);
        std::array<DriverUpdateFn, 3> g_original{};
        std::atomic_bool g_available{ false };
        std::atomic_bool g_installed{ false };
        HMODULE g_fosModule = nullptr;
        using Clock = std::chrono::steady_clock;
        struct GroundedDiag
        {
            Clock::time_point firstSeen{};
            bool callbackSeen{ false };
            bool missingCallbackLogged{ false };
            bool callbackLogged{ false };
        };
        std::mutex g_poseDiagMutex;
        std::unordered_set<RE::FormID> g_poseContextMissingLogged;
        std::unordered_map<RE::FormID, GroundedDiag> g_groundedDiag;

        std::ptrdiff_t RuntimeOffset(std::ptrdiff_t se, std::ptrdiff_t ae)
        {
            return REL::Module::IsAE() ? ae : se;
        }

        bool AddressInsideFoS(std::uintptr_t address)
        {
            if (!g_fosModule || !address) return false;
            __try {
                const auto base = reinterpret_cast<std::uintptr_t>(g_fosModule);
                const auto* dos = reinterpret_cast<const IMAGE_DOS_HEADER*>(base);
                if (dos->e_magic != IMAGE_DOS_SIGNATURE) return false;
                const auto* nt = reinterpret_cast<const IMAGE_NT_HEADERS64*>(base + dos->e_lfanew);
                if (nt->Signature != IMAGE_NT_SIGNATURE) return false;
                const auto end = base + nt->OptionalHeader.SizeOfImage;
                return address >= base && address < end;
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return false;
            }
        }

        std::uintptr_t ResolveThunkTarget(std::uintptr_t address)
        {
            std::uintptr_t current = address;
            for (int depth = 0; depth < 6 && current; ++depth) {
                if (AddressInsideFoS(current)) return current;
                __try {
                    const auto* p = reinterpret_cast<const std::uint8_t*>(current);
                    // CommonLib far branch island: FF 25 disp32 -> [RIP+disp32]
                    if (p[0] == 0xFF && p[1] == 0x25) {
                        const auto disp = *reinterpret_cast<const std::int32_t*>(p + 2);
                        const auto slot = current + 6 + static_cast<std::intptr_t>(disp);
                        current = *reinterpret_cast<const std::uintptr_t*>(slot);
                        continue;
                    }
                    // near jmp rel32
                    if (p[0] == 0xE9) {
                        const auto disp = *reinterpret_cast<const std::int32_t*>(p + 1);
                        current = current + 5 + static_cast<std::intptr_t>(disp);
                        continue;
                    }
                    // mov rax, imm64 ; jmp rax
                    if (p[0] == 0x48 && p[1] == 0xB8 && p[10] == 0xFF && p[11] == 0xE0) {
                        current = *reinterpret_cast<const std::uintptr_t*>(p + 2);
                        continue;
                    }
                } __except (EXCEPTION_EXECUTE_HANDLER) {
                    return 0;
                }
                return current;
            }
            return current;
        }


        bool Normalize3(float* v)
        {
            const float len2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if (!std::isfinite(len2) || len2 < 1.0e-8f) return false;
            const float inv = 1.0f / std::sqrt(len2);
            v[0] *= inv;
            v[1] *= inv;
            v[2] *= inv;
            return true;
        }

        std::uint64_t At(std::uint64_t address)
        {
            return *reinterpret_cast<std::uint64_t*>(address);
        }

        void CopyBoneName(std::uint64_t bones, int boneCount, int index, char* dst)
        {
            dst[0] = '\0';
            if (!bones || index < 0 || index >= boneCount) return;
            const auto namePtr = At(bones + static_cast<std::uint64_t>(index) * kBoneStride) & kBoneNameMask;
            if (!namePtr) return;
            const char* src = reinterpret_cast<const char*>(namePtr);
            int i = 0;
            for (; i < kBoneNameCap - 1 && src[i]; ++i) dst[i] = src[i];
            dst[i] = '\0';
        }

        RE::Actor* ResolveActor(void* context, RE::BShkbAnimationGraph*& graph) noexcept
        {
            graph = nullptr;
            if (!context) return nullptr;
            __try {
                const auto character = *reinterpret_cast<std::uint64_t*>(context);
                if (!character) return nullptr;
                const auto graphAddress = character - kCharacterInGraph;
                const auto holder = At(graphAddress + kGraphHolder);
                if (!holder) return nullptr;
                if (*reinterpret_cast<std::uint8_t*>(holder + kHolderFormType) != kActorCharacterFormType) return nullptr;
                graph = reinterpret_cast<RE::BShkbAnimationGraph*>(graphAddress);
                return reinterpret_cast<RE::Actor*>(holder);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                graph = nullptr;
                return nullptr;
            }
        }

        void Dispatch(std::size_t siteIndex, void* driver, void* context, void* output)
        {
            RE::BShkbAnimationGraph* graph = nullptr;
            if (auto* actor = ResolveActor(context, graph); actor && graph && output) {
                PoseContext pose;
                const bool poseOk = AcquirePoseContext(graph, output, pose);
                if (const auto* profile = UKClient::GetProfileForActor(actor); profile &&
                    (profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped ||
                     profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped)) {
                    std::scoped_lock diagLock(g_poseDiagMutex);
                    auto& diag = g_groundedDiag[actor->GetFormID()];
                    if (diag.firstSeen == Clock::time_point{}) diag.firstSeen = Clock::now();
                    diag.callbackSeen = true;
                    if (profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped && !diag.callbackLogged) {
                        diag.callbackLogged = true;
                        SKSE::log::info(
                            "[FormaMotus][FoS][PRE-FOS-BIPED] actor={:08X} callbackSeen=true poseContext={} legs={} reason={}",
                            actor->GetFormID(), poseOk, poseOk ? pose.legCount : 0,
                            poseOk ? "READY" : "POSE_OR_DRIVER_CONTEXT_UNAVAILABLE");
                    }
                }
                if (poseOk) {
                    // Order is intentional: axial orientation establishes the body
                    // frame first, then the leg module applies swing Stride and world-space
                    // Foot Lock. Feet of Skyrim receives this output immediately
                    // afterwards and remains the
                    // final terrain/contact/leg IK owner.
                    // Radial abdomen is applied once at the top behavior output.
                    const auto* profile = UKClient::GetProfileForActor(actor);
                    if (!profile || profile->locomotionMode != UniversalKinematics::LocomotionMode::GroundedRadial)
                        AnimalMotionEngine::ApplyPreFootIkPose(actor->GetFormID(), graph, pose);
                    StrideWarpModule::ApplyPreFootIkPose(actor, graph, pose);
                } else if (const auto* profile = UKClient::GetProfileForActor(actor); profile &&
                    (profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped ||
                     profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped)) {
                    std::scoped_lock diagLock(g_poseDiagMutex);
                    if (g_poseContextMissingLogged.insert(actor->GetFormID()).second) {
                        SKSE::log::info(
                            "[FormaMotus][FoS][NO-POSE-CONTEXT] actor={:08X} locomotionMode={} reason=FOOT_IK_OUTPUT_OR_DRIVER_INFO_UNAVAILABLE_AT_THIS_CALLSITE",
                            actor->GetFormID(), static_cast<unsigned>(profile->locomotionMode));
                    }
                }
            }

            if (siteIndex < g_original.size() && g_original[siteIndex]) {
                g_original[siteIndex](driver, context, output);
            }
        }

        void Hook0(void* d, void* c, void* o) { Dispatch(0, d, c, o); }
        void Hook1(void* d, void* c, void* o) { Dispatch(1, d, c, o); }
        void Hook2(void* d, void* c, void* o) { Dispatch(2, d, c, o); }

        std::uintptr_t ReadNearCallTarget(std::uintptr_t site)
        {
            __try {
                const auto* p = reinterpret_cast<const std::uint8_t*>(site);
                if (p[0] != 0xE8) return 0;
                const auto disp = *reinterpret_cast<const std::int32_t*>(p + 1);
                return site + 5 + static_cast<std::intptr_t>(disp);
            } __except (EXCEPTION_EXECUTE_HANDLER) {
                return 0;
            }
        }

        bool CanChainSite(REL::RelocationID id, std::ptrdiff_t se, std::ptrdiff_t ae)
        {
            const REL::Relocation<std::uintptr_t> site{ id, RuntimeOffset(se, ae) };
            const auto currentTarget = ReadNearCallTarget(site.address());
            const auto resolved = ResolveThunkTarget(currentTarget);
            return currentTarget != 0 && AddressInsideFoS(resolved);
        }

        bool PatchSite(std::size_t index, REL::RelocationID id, std::ptrdiff_t se, std::ptrdiff_t ae, DriverUpdateFn hook)
        {
            const REL::Relocation<std::uintptr_t> site{ id, RuntimeOffset(se, ae) };
            const auto currentTarget = ReadNearCallTarget(site.address());
            const auto resolved = ResolveThunkTarget(currentTarget);
            if (!currentTarget || !AddressInsideFoS(resolved)) {
                SKSE::log::critical(
                    "[FormaMotus][FoS][CHAIN-REJECT] site={} address=0x{:X} currentTarget=0x{:X} resolved=0x{:X} reason=EXISTING_TARGET_NOT_FOS",
                    index, site.address(), currentTarget, resolved);
                return false;
            }

            const auto previous = SKSE::GetTrampoline().write_call<5>(
                site.address(), reinterpret_cast<std::uintptr_t>(hook));
            if (!previous) return false;
            g_original[index] = reinterpret_cast<DriverUpdateFn>(previous);
            SKSE::log::info(
                "[FormaMotus][FoS][CHAIN] site={} address=0x{:X} previous=0x{:X} resolvedFoS=0x{:X} owner=PRE_FOS_OUTPUT",
                index, site.address(), previous, resolved);
            return true;
        }
    }

    bool Initialize()
    {
        if (g_available.load(std::memory_order_acquire)) return true;
        const HMODULE module = ::GetModuleHandleW(L"FeetOfSkyrim.dll");
        if (!module) {
            SKSE::log::critical("[FormaMotus][FoS] FeetOfSkyrim.dll not loaded; hard dependency unavailable.");
            return false;
        }
        g_fosModule = module;
        g_available.store(true, std::memory_order_release);
        SKSE::log::info("[FormaMotus][FoS] FeetOfSkyrim.dll detected; final leg IK owner is available.");
        return true;
    }

    bool IsAvailable()
    {
        return g_available.load(std::memory_order_acquire) || Initialize();
    }

    bool InstallPreHooks()
    {
        if (g_installed.load(std::memory_order_acquire)) return true;
        if (!Initialize() || REL::Module::IsVR()) return false;

        // Validate the whole existing chain before touching any callsite.
        if (!CanChainSite(kSceneModifierCaller, 0x62, 0x62) ||
            !CanChainSite(kPairedSyncCaller, 0x316, 0x2D6) ||
            !CanChainSite(kPairedSyncCaller, 0x4AE, 0x43F)) {
            SKSE::log::critical("[FormaMotus][FoS] one or more driver callsites are not currently owned by Feet of Skyrim; no callsite was modified.");
            return false;
        }

        SKSE::AllocTrampoline(1u << 10);
        const bool ok0 = PatchSite(0, kSceneModifierCaller, 0x62, 0x62, Hook0);
        const bool ok1 = PatchSite(1, kPairedSyncCaller, 0x316, 0x2D6, Hook1);
        const bool ok2 = PatchSite(2, kPairedSyncCaller, 0x4AE, 0x43F, Hook2);
        if (!(ok0 && ok1 && ok2)) {
            SKSE::log::critical("[FormaMotus][FoS] failed to chain all Foot IK driver callsites; Forma Motus remains inert.");
            g_original = {};
            return false;
        }

        g_installed.store(true, std::memory_order_release);
        SKSE::log::info("[FormaMotus][FoS][READY] chained 3/3 driver callsites | order=ORIENTATION->STRIDE->FOOT_LOCK->FEET_OF_SKYRIM");
        return true;
    }

    void Reset()
    {
        // Hook installation is process-lifetime. Diagnostics are per save/runtime.
        std::scoped_lock lock(g_poseDiagMutex);
        g_poseContextMissingLogged.clear();
        g_groundedDiag.clear();
    }

    void ObserveGroundedActor(RE::Actor* actor)
    {
        if (!actor) return;
        const auto* profile = UKClient::GetProfileForActor(actor);
        if (!profile) return;
        const bool biped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedBiped;
        const bool quadruped = profile->locomotionMode == UniversalKinematics::LocomotionMode::GroundedQuadruped;
        if (!biped && !quadruped) return;

        std::scoped_lock lock(g_poseDiagMutex);
        auto& diag = g_groundedDiag[actor->GetFormID()];
        const auto now = Clock::now();
        if (diag.firstSeen == Clock::time_point{}) diag.firstSeen = now;
        if (biped && !diag.callbackSeen && !diag.missingCallbackLogged &&
            std::chrono::duration<float>(now - diag.firstSeen).count() >= 1.0f) {
            diag.missingCallbackLogged = true;
            const auto* race = actor->GetRace();
            SKSE::log::info(
                "[FormaMotus][FoS][PRE-FOS-BIPED] actor={:08X} name=\"{}\" race=\"{}\" callbackSeen=false poseContext=false legs=0 reason=NO_FOS_DRIVER_CALLBACK_WITHIN_1S",
                actor->GetFormID(), actor->GetName(), race ? race->GetFormEditorID() : "");
        }
    }

    bool AcquirePoseContext(RE::BShkbAnimationGraph* graph, void* output, PoseContext& out, bool requireFootIK) noexcept
    {
        out = {};
        if (!graph || !output) return false;

        __try {
            const auto graphAddress = reinterpret_cast<std::uint64_t>(graph);
            const auto setup = At(graphAddress + kGraphSetup);
            if (!setup) return false;
            const auto charData = At(setup + kSetupCharData);
            const auto skeleton = At(setup + kSetupSkeleton);
            if (!charData || !skeleton) return false;
            const auto driverInfo = At(charData + kCharDataDriverInfo);
            if (requireFootIK && !driverInfo) return false;

            const int boneCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonBoneCount);
            const int parentCount = *reinterpret_cast<std::int32_t*>(skeleton + kSkeletonParentCount);
            const auto parents = At(skeleton + kSkeletonParentData);
            const auto bones = At(skeleton + kSkeletonBoneData);
            if (boneCount < 1 || boneCount > kBoneCap || parentCount != boneCount || !parents || !bones) return false;

            const auto data = *reinterpret_cast<std::uint64_t*>(output);
            if (!data || *reinterpret_cast<std::int32_t*>(data + 0x04) <= kPoseTrack) return false;
            const auto poseEntry = data + kTrackTable + kPoseTrack * kTrackEntryBytes;
            const auto poseCap = *reinterpret_cast<std::int16_t*>(poseEntry + 0x0);
            const auto poseCount = *reinterpret_cast<std::int16_t*>(poseEntry + 0x2);
            const auto poseOff = *reinterpret_cast<std::int16_t*>(poseEntry + 0x4);
            const auto poseElem = *reinterpret_cast<std::int16_t*>(poseEntry + 0x6);
            if (poseCap != boneCount || poseElem != 0x30 || poseOff <= 0 || poseCount <= 0 || poseCount > poseCap) return false;

            out.data = data;
            out.charData = charData;
            out.skeleton = skeleton;
            out.driverInfo = driverInfo;
            out.pose = reinterpret_cast<float*>(data + static_cast<std::uint16_t>(poseOff));
            out.poseOffset = static_cast<std::uint16_t>(poseOff);
            out.poseCapacity = poseCap;
            out.poseCount = poseCount;
            out.boneCount = boneCount;

            for (int i = 0; i < boneCount; ++i) {
                out.parents[i] = *reinterpret_cast<std::int16_t*>(parents + static_cast<std::uint64_t>(i) * 2);
                if (out.parents[i] >= i) return false;
            }

            if (!requireFootIK) return true; // validated axial output needs no leg driver

            const auto* fwd = reinterpret_cast<const float*>(charData + kCharDataModelForward);
            const auto* right = reinterpret_cast<const float*>(charData + kCharDataModelRight);
            for (int i = 0; i < 3; ++i) {
                out.modelForward[i] = fwd[i];
                out.modelRight[i] = right[i];
            }
            if (!Normalize3(out.modelForward) || !Normalize3(out.modelRight)) return false;

            const int legCount = *reinterpret_cast<std::int32_t*>(driverInfo + kInfoLegCount);
            const auto legs = At(driverInfo + kInfoLegData);
            if (legCount < 1 || legCount > kMaxLegs || !legs) return false;
            out.legCount = legCount;
            out.lockFeetWhenPlanted =
                *reinterpret_cast<const std::uint8_t*>(driverInfo + kInfoLockFeetWhenPlanted) != 0;

            for (int i = 0; i < legCount; ++i) {
                const auto leg = legs + static_cast<std::uint64_t>(i) * kLegStride;
                auto& slot = out.legs[i];
                slot.hipIndex = static_cast<std::int16_t>(*reinterpret_cast<std::uint16_t*>(leg + kLegHipIndex));
                slot.kneeIndex = static_cast<std::int16_t>(*reinterpret_cast<std::uint16_t*>(leg + kLegKneeIndex));
                slot.ankleIndex = static_cast<std::int16_t>(*reinterpret_cast<std::uint16_t*>(leg + kLegAnkleIndex));
                const auto* kneeAxis = reinterpret_cast<const float*>(leg + kLegKneeAxis);
                slot.kneeAxis[0] = kneeAxis[0];
                slot.kneeAxis[1] = kneeAxis[1];
                slot.kneeAxis[2] = kneeAxis[2];
                slot.plantedHeight = *reinterpret_cast<const float*>(leg + kLegPlantedHeight);
                slot.raisedHeight = *reinterpret_cast<const float*>(leg + kLegRaisedHeight);
                slot.maxAnkleHeight = *reinterpret_cast<const float*>(leg + kLegMaxHeight);
                slot.minAnkleHeight = *reinterpret_cast<const float*>(leg + kLegMinHeight);
                if (slot.hipIndex < 0 || slot.kneeIndex < 0 || slot.ankleIndex < 0 ||
                    slot.hipIndex >= boneCount || slot.kneeIndex >= boneCount || slot.ankleIndex >= boneCount) {
                    return false;
                }
                CopyBoneName(bones, boneCount, slot.hipIndex, slot.hipBone);
                CopyBoneName(bones, boneCount, slot.kneeIndex, slot.kneeBone);
                CopyBoneName(bones, boneCount, slot.ankleIndex, slot.ankleBone);
            }
            return true;
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            out = {};
            return false;
        }
    }

    int FindBoneIndexByNameContains(const PoseContext& context, const char* token) noexcept
    {
        if (!context.skeleton || !token || !*token || context.boneCount < 1) return -1;

        auto containsInsensitive = [](const char* text, const char* needle) noexcept {
            if (!text || !needle || !*needle) return false;
            const std::size_t textLen = std::strlen(text);
            const std::size_t needleLen = std::strlen(needle);
            if (needleLen > textLen) return false;
            for (std::size_t i = 0; i + needleLen <= textLen; ++i) {
                bool match = true;
                for (std::size_t j = 0; j < needleLen; ++j) {
                    const auto a = static_cast<unsigned char>(text[i + j]);
                    const auto b = static_cast<unsigned char>(needle[j]);
                    if (std::tolower(a) != std::tolower(b)) {
                        match = false;
                        break;
                    }
                }
                if (match) return true;
            }
            return false;
        };

        __try {
            const auto bones = At(context.skeleton + kSkeletonBoneData);
            if (!bones) return -1;
            char name[kBoneNameCap]{};
            for (int i = 0; i < context.boneCount; ++i) {
                CopyBoneName(bones, context.boneCount, i, name);
                if (containsInsensitive(name, token)) return i;
            }
        } __except (EXCEPTION_EXECUTE_HANDLER) {
            return -1;
        }
        return -1;
    }

    void MarkBoneValid(const PoseContext& context, int boneIndex) noexcept
    {
        if (!context.data || boneIndex < 0 || boneIndex >= context.poseCapacity) return;
        __try {
            auto* mask = reinterpret_cast<std::uint32_t*>(
                context.data + context.poseOffset + static_cast<std::uint64_t>(context.poseCapacity) * 0x30);
            mask[boneIndex >> 5] |= (1u << (boneIndex & 31));
        } __except (EXCEPTION_EXECUTE_HANDLER) {
        }
    }
}

