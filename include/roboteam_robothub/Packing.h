//
// Created by rolf on 03-03-21.
//

#ifndef RTT_ROBOTEAM_ROBOTHUB_SRC_PACKING_H_
#define RTT_ROBOTEAM_ROBOTHUB_SRC_PACKING_H_
#include <roboteam_proto/RobotCommand.pb.h>
#include <roboteam_proto/RobotFeedback.pb.h>
#include <roboteam_proto/World.pb.h>

#include "RobotCommand.h"
#include "RobotFeedback.h"

RobotCommandPayload createEmbeddedCommand(const proto::RobotCommand& proto, const proto::World& world, bool isYellow);

proto::RobotFeedback feedbackFromRaw(RobotFeedbackPayload* payload);
#endif  // RTT_ROBOTEAM_ROBOTHUB_SRC_PACKING_H_