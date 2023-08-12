//
// Created by rolf on 30-7-23.
//

#include <utility>

#include "observer/filters/vision/RobotTrajectorySegment.h"
namespace rtt {

    RobotTrajectorySegment::RobotTrajectorySegment(RobotShape  startPos,
                                                   Vector2 vel,
                                                   double angVel,
                                                   double dt,
                                                   bool isBlue,
                                                   int id) :
          startPos{std::move(startPos)},
          vel{vel},
          angVel{angVel},
          dt{dt},
          isBlue{isBlue},
          id{id}{
    }
}