//
// Created by rolf on 30-7-23.
//

#ifndef RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOTTRAJECTORYSEGMENT_H
#define RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOTTRAJECTORYSEGMENT_H

#include <roboteam_utils/RobotShape.h>
namespace rtt {

    //This class contains the data necessary to describe a robot travelling at a constant velocity
    //For a small period of time.
    //This is an approximation of the robots' behavior used for computing ball collisions
    struct RobotTrajectorySegment {
        RobotTrajectorySegment(RobotShape  startPos,
                               Vector2 vel,
                               double angVel,
                               double dt,
                               bool isBlue,
                               int id
                               );
        RobotShape startPos; //includes robots' starting angle
        Vector2 vel;
        double angVel;
        double dt;

        bool isBlue;
        int id;
    };

}

#endif  // RTT_ROBOTEAM_WORLD_OBSERVER_SRC_FILTERS_VISION_ROBOTTRAJECTORYSEGMENT_H
