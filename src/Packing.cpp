//
// Created by rolf on 03-03-21.
//

#include "Packing.h"
#include "utilities.h"
#include <cmath>

RobotCommandPayload createEmbeddedCommand(const proto::RobotCommand& proto, const proto::World& world, bool isYellow){
    float rho = sqrtf(proto.vel().x() * proto.vel().x() + proto.vel().y() * proto.vel().y());
    float theta = atan2f(proto.vel().y(), proto.vel().x());
    auto bot = rtt::robothub::utils::getWorldBot(proto.id(),isYellow,world);

    RobotCommand command;
    command.header = PACKET_TYPE_ROBOT_COMMAND;
    command.id = proto.id();

    command.doKick = proto.kicker();
    command.doChip = proto.chipper();
    command.doForce = proto.chip_kick_forced();

    command.rho = rho;
    command.theta = theta;
    command.angle = proto.w();

    if(bot){
        command.useCameraAngle = true;
        command.cameraAngle = bot->angle();
    }else{
        command.useCameraAngle = false;
        command.cameraAngle = 0.0;
    }
    command.kickChipPower = proto.chip_kick_vel();
    command.dribbler = proto.dribbler();
    command.angularControl = proto.use_angle();
    command.feedback = false;

    RobotCommandPayload payload;
    encodeRobotCommand(&payload,&command);
    return payload;
}

proto::RobotFeedback feedbackFromRaw(RobotFeedbackPayload * payload){
    // TODO UPDATE FUNCTION TO LATEST REM VERSION
    proto::RobotFeedback _packet;
    return _packet;


    RobotFeedback feedback;
    decodeRobotFeedback(&feedback,payload);

    proto::RobotFeedback packet;

    packet.set_id(feedback.id);
    packet.set_xsenscalibrated(feedback.XsensCalibrated);
    packet.set_ballsensorisworking(feedback.ballSensorWorking);
    packet.set_batterylow(feedback.batteryLevel <=2 );//TODO: change in protobuf; this field is a uint8!
    packet.set_hasball(feedback.hasBall);
    packet.set_ballpos(feedback.ballPos);

    packet.set_x_vel(feedback.rho*cosf(feedback.theta));
    packet.set_y_vel(feedback.rho*sinf(feedback.theta));
    packet.set_yaw(feedback.angle);

    packet.set_haslockedwheel(feedback.wheelLocked>0); //TODO: change in protobuf
    packet.set_signalstrength(feedback.rssi); //TODO: this is not a float but a uint?

    //not used for now;
    //feedback.wheelBraking;
    //feedback.capacitorCharged;
    return packet;
}