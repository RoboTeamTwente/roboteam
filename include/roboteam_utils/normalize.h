#pragma once

#include <roboteam_proto/RobotCommand.pb.h>
#include <roboteam_proto/GeometryData.pb.h>
#include <roboteam_proto/Referee.pb.h>
#include <roboteam_proto/WorldBall.pb.h>
#include <roboteam_proto/WorldRobot.pb.h>
#include <roboteam_proto/World.pb.h>
#include <roboteam_proto/messages_robocup_ssl_referee.pb.h>
#include <roboteam_proto/messages_robocup_ssl_geometry.pb.h>
#include "roboteam_proto/FieldLineSegment.pb.h"
#include "roboteam_proto/FieldCircularArc.pb.h"
#include "Mathematics.h"

namespace roboteam_utils {

  void rotate(proto::WorldBall *ball) ;
  void rotate(proto::WorldRobot *robot);
  void rotate(proto::World *world);
  void rotate(proto::SSL_Referee *refereeData);
  void rotate(proto::SSL_FieldCicularArc *arc);
  void toMeters(proto::SSL_FieldCicularArc *arc);
  void rotate(proto::SSL_FieldLineSegment *line);
  void toMeters(proto::SSL_FieldLineSegment * line);
  void rotate(proto::SSL_GeometryFieldSize * field);
//  void toMeters(proto::SSL_GeometryFieldSize * field);
  void rotate(proto::RobotCommand * command);

}
