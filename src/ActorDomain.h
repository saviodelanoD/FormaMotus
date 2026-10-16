#pragma once

#include "PCH.h"

namespace ActorDomain
{
    // Forma Motus R5.7 is creature/non-human only.
    //
    // Do not use UniversalKinematics::isHumanoidLikeUpperBody as the ownership
    // boundary: some creatures (notably Troll-family profiles) can legitimately
    // look humanoid anatomically while still belonging to the creature pipeline.
    //
    // ActorTypeNPC is the Skyrim race-domain boundary used here for the ALS lane.
    // Player is excluded explicitly as well. Unknown/non-NPC races fail open to
    // Forma so creature support is not silently lost.
    inline bool IsALSManagedHumanoid(RE::Actor* actor) noexcept
    {
        if (!actor) {
            return false;
        }
        if (actor->IsPlayerRef()) {
            return true;
        }

        const auto* race = actor->GetRace();
        if (!race) {
            return false;
        }

        // Explicit creature domains win over ActorTypeNPC. This protects
        // humanoid-shaped non-humans (Draugr/Daedra/etc.) from being routed out of
        // Forma merely because a modded race also carries NPC-like metadata.
        if (race->HasKeywordString("ActorTypeCreature") ||
            race->HasKeywordString("ActorTypeAnimal") ||
            race->HasKeywordString("ActorTypeDragon") ||
            race->HasKeywordString("ActorTypeUndead") ||
            race->HasKeywordString("ActorTypeDaedra") ||
            race->HasKeywordString("ActorTypeDwarven")) {
            return false;
        }

        return race->HasKeywordString("ActorTypeNPC");
    }

    inline bool IsFormaNonHuman(RE::Actor* actor) noexcept
    {
        return actor && !IsALSManagedHumanoid(actor);
    }
}

