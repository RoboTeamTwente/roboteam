//
// Created by Martin Miksik on 17/02/2023.
//

#ifndef RTT_STATEVECTOR_H
#define RTT_STATEVECTOR_H

#include <roboteam_utils/Vector2.h>

#include "utilities/StpInfoEnums.h"

namespace rtt::ai::control {

struct StateVector {
    Vector2 position;
    Vector2 velocity;
};


struct PositionControlInput {
    const int robotId;
    const StateVector& state;
    const Vector2& targetPos;
    const double maxVel;
    const stp::AvoidObjects& avoidObjects;
};

}

#endif  // RTT_STATEVECTOR_H
