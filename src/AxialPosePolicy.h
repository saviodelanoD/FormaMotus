#pragma once
namespace AxialPosePolicy {
enum class Source { Neutral, Steering, PhysicalTurn };
inline Source Select(bool supported,bool authoredTurn,bool action,bool steeringActive,bool physicalActive){
    if(!supported)return physicalActive?Source::PhysicalTurn:Source::Neutral;
    if(action)return Source::Neutral;
    if(authoredTurn)return physicalActive?Source::PhysicalTurn:Source::Neutral;
    // A missing clip witness does not invalidate measured physical rotation.
    return steeringActive?Source::Steering:(physicalActive?Source::PhysicalTurn:Source::Neutral);
}
}

