#include "roboteam_utils/constants.h"

namespace rtt {

template<typename T, long unsigned int N>
static bool has(std::array<T, N> arr, T val) {
  for (unsigned int i = 0; i < N; i++) {
    if (arr[i] == val) {
      return true;
    }
  }
  return false;
}

const std::string ROBOTEAM_WORLD_TCP_PUBLISHER = "tcp://127.0.0.1:5555";
const std::string ROBOTEAM_AI_TCP_PUBLISHER = "tcp://127.0.0.1:5556";
const std::string ROBOTEAM_AI_2_TCP_PUBLISHER = "tcp://127.0.0.1:5557"; // for a eventual second ai
const std::string ROBOTEAM_ROBOTHUB_TCP_PUBLISHER = "tcp://127.0.0.1:5558";
const std::string ROBOTEAM_ROBOTHUB_TCP_2_PUBLISHER = "tcp://127.0.0.1:5559"; // for a eventual second robothub

const std::string NODE_AVOID_ROBOTS_TEST = "AvoidRobotsTest";

const std::string NODE_DO_STRATEGY = "doStrategy";

const std::string NODE_DRIBBLE_TEST = "DribbleTest";

const std::string NODE_FOLLOW_PATH_TEST = "FollowPathTest";

const std::string NODE_GET_BALL_TEST = "GetBallTest";

const std::string NODE_GO_TO_POS_TEST = "GoToPosTest";

const std::string NODE_KEEPER_TEST = "KeeperTactic";

const std::string NODE_KICK_AT_GOAL_TEST = "KickAtGoal";

const std::string NODE_KICK_TEST = "KickTest";

const std::string NODE_ROTATE_AROUND_POINT_TEST = "RotateAroundPoint";

const std::string NODE_STAND_FREE_TEST = "StandFreeTest";

const std::string NODE_TACTICS = "tactics";

const std::string SERVICE_NAVSIM = "navsim";

const std::string SERVICE_OPPONENT_TRACKER = "tracker";

const std::string SERVICE_WORLD_RESET = "world_reset";

const std::string TOPIC_COMMANDS = "robotcommands";

const std::string TOPIC_SETTINGS = "settings";

const std::string TOPIC_GEOMETRY = "vision_geometry";

const std::string TOPIC_REFEREE = "vision_referee";

const std::string TOPIC_WORLD_STATE = "world_state";

const std::string TOPIC_FEEDBACK = "feedback";


const std::string TRACKER_TYPE_ACCEL = "Acceleration";

const std::string TRACKER_TYPE_SPEED = "Speed";


} // namespace rtt
