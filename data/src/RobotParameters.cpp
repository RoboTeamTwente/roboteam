//
// Created by rolf on 26-10-20.
//


#include "RobotParameters.h"

RobotParameters RobotParameters::DEFAULT() {
    RobotParameters parameters; //default constructor should be roughly correct
    return parameters;
}

RobotParameters RobotParameters::RTT_2020() {
    //TODO: fix these
    RobotParameters parameters(0.09,0.15,0.1,0.1,0.0);
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
        angleOffset{protoParams.angle_offset()} {
}

//TODO fix correct values
RobotParameters::RobotParameters() : radius{0.09}, height{0.15},frontWidth{0.1}, dribblerWidth{0.1}, angleOffset{0.0} {

}

RobotParameters::RobotParameters(double radius, double height, double frontWidth, double dribblerWidth,
                                 double angleOffset) : radius{radius}, height{height}, frontWidth{frontWidth},
                                                       dribblerWidth{dribblerWidth}, angleOffset{angleOffset} {

}
