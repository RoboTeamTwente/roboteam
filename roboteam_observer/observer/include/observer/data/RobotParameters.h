//
// Created by rolf on 26-10-20.
//

#ifndef RTT_ROBOTPARAMETERS_H
#define RTT_ROBOTPARAMETERS_H

#include <proto/RobotParameters.pb.h>

class RobotParameters {
   public:
    RobotParameters();
    RobotParameters(double radius, double height, double frontWidth, double dribblerWidth, double angleOffset);
    explicit RobotParameters(const proto::RobotParameters& protoParams);
    [[nodiscard]] proto::RobotParameters toProto() const;
    static RobotParameters from_default();
    static RobotParameters from_rtt2020();

   private:
    double radius;
    double height;
    double frontWidth;
    double dribblerWidth;
    double angleOffset;
};

#endif  // RTT_ROBOTPARAMETERS_H
