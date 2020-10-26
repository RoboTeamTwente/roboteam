//
// Created by rolf on 26-10-20.
//


#include "RobotParameters.h"

RobotParameters RobotParameters::DEFAULT() {
    //TODO: fix these to sensical values
    RobotParameters parameters;
    parameters.radius = 0.09;
    parameters.height = 0.15;
    parameters.frontWidth = 0.1;
    parameters.dribblerWidth = parameters.frontWidth;
    parameters.angleOffset = 0.0;
    return parameters;
}

RobotParameters RobotParameters::RTT_2020() {
    //TODO: fix these
    RobotParameters parameters;
    parameters.radius = 0.09;
    parameters.height = 0.15;
    parameters.frontWidth = 0.1;
    parameters.dribblerWidth = parameters.frontWidth;
    parameters.angleOffset = 0.0;
    return parameters;
}

proto::RobotParameters RobotParameters::toProto() const {
    proto::RobotParameters protoMsg;
    protoMsg.set_radius(radius);
    protoMsg.set_height(height);
    protoMsg.set_front_width(frontWidth);
    protoMsg.set_dribbler_width(dribblerWidth);
    protoMsg.set_angle_offset(angleOffset);
    return protoMsg;
}

RobotParameters::RobotParameters(const proto::RobotParameters &protoParams) :
radius{protoParams.radius()},
height{protoParams.height()},
frontWidth{protoParams.front_width()},
dribblerWidth{protoParams.dribbler_width()},
angleOffset{protoParams.angle_offset()}{
}

RobotParameters::RobotParameters() {
    auto defParams = RobotParameters::DEFAULT();
    radius = defParams.radius;
    height = defParams.height;
    frontWidth = defParams.frontWidth;
    dribblerWidth = defParams.dribblerWidth;
    angleOffset = defParams.angleOffset;
}
