//
// Created by rolf on 26-10-20.
//

#ifndef RTT_ROBOTPARAMETERS_H
#define RTT_ROBOTPARAMETERS_H

#include <roboteam_proto/RobotParameters.pb.h>

class RobotParameters {
public:
    RobotParameters();
    explicit RobotParameters(const proto::RobotParameters& protoParams);
    [[nodiscard]] proto::RobotParameters toProto() const;
    static RobotParameters DEFAULT();
    static RobotParameters RTT_2020();
private:
    double radius;
    double height;
    double frontWidth;
    double dribblerWidth;
    double angleOffset;

};


#endif //RTT_ROBOTPARAMETERS_H
