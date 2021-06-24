//
// Created by rolf on 24-06-21.
//
#include "filters/vision/robot/RobotObservation.h"

RobotObservation::RobotObservation(int cameraID, Time timeCaptured, Time timeSent, const proto::SSL_DetectionRobot& detectionRobot) :
    cameraID(cameraID),
    timeCaptured(timeCaptured),
    timeSent(timeSent),
    robotID(detectionRobot.robot_id()),
    position(detectionRobot.x()/1000.0,detectionRobot.y()/1000.0), //Position by SSL-Vision is given in millimeters
    pixelPosition(detectionRobot.pixel_x(),detectionRobot.pixel_y()),
    orientation(detectionRobot.orientation()),
    confidence(detectionRobot.confidence()),
    height(detectionRobot.height()/1000.0)
{

}