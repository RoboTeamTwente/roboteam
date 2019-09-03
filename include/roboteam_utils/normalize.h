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

  void rotate(roboteam_proto::WorldBall *ball) ;
  void rotate(roboteam_proto::WorldRobot *robot);
  void rotate(roboteam_proto::World *world);
  void rotate(roboteam_proto::SSL_Referee *refereeData);
  void rotate(roboteam_proto::SSL_FieldCicularArc *arc);
  void toMeters(roboteam_proto::SSL_FieldCicularArc *arc);
  void rotate(roboteam_proto::SSL_FieldLineSegment *line);
  void toMeters(roboteam_proto::SSL_FieldLineSegment * line);
  void rotate(roboteam_proto::SSL_GeometryFieldSize * field);
  void toMeters(roboteam_proto::SSL_GeometryFieldSize * field);

}
