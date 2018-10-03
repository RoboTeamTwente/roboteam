
#include <ros/ros.h>
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

void get_PARAM_ITERATIONS_PER_SECOND(int& tgt) {
  ros::param::get("role_iterations_per_second", tgt);
}

void set_PARAM_ITERATIONS_PER_SECOND(const int& val) {
  ros::param::set("role_iterations_per_second", val);
}

bool has_PARAM_ITERATIONS_PER_SECOND() {
  return ros::param::has("role_iterations_per_second");
}

void get_PARAM_KICKING(bool& tgt) {
  ros::param::get("/kickingTheBall", tgt);
}

void set_PARAM_KICKING(const bool& val) {
  ros::param::set("/kickingTheBall", val);
}

bool has_PARAM_KICKING() {
  return ros::param::has("/kickingTheBall");
}

void get_PARAM_NORMALIZE_FIELD(bool& tgt) {
  ros::param::get("normalize_field", tgt);
}

void set_PARAM_NORMALIZE_FIELD(const bool& val) {
  ros::param::set("normalize_field", val);
}

bool has_PARAM_NORMALIZE_FIELD() {
  return ros::param::has("normalize_field");
}

void get_PARAM_NUM_ROLE_NODES(int& tgt) {
  ros::param::get("num_role_nodes", tgt);
}

void set_PARAM_NUM_ROLE_NODES(const int& val) {
  ros::param::set("num_role_nodes", val);
}

bool has_PARAM_NUM_ROLE_NODES() {
  return ros::param::has("num_role_nodes");
}

const std::array<string, 2> PARAM_OUR_COLOR_valid_values = {"yellow", "blue"};

void get_PARAM_OUR_COLOR(string& tgt, bool error_on_invalid) {
  string val;
  ros::param::get("our_color", val);
  if(!has(PARAM_OUR_COLOR_valid_values, val)) {
    if (error_on_invalid) {
      throw std::runtime_error("Param fetched with invalid value.");
    } else {
      ROS_WARN("Param PARAM_OUR_COLOR fetched with invalid value.");
    }
  }
  tgt = val;
}

void set_PARAM_OUR_COLOR(const string& val, bool error_on_invalid) {
  if(!has(PARAM_OUR_COLOR_valid_values, val)) {
    if (error_on_invalid) {
      throw new std::runtime_error("Tried to set invalid value to a param.");
    } else {
      ROS_WARN("Setting invalid value to param PARAM_OUR_COLOR");
    }
  }
  ros::param::set("our_color", val);
}

bool has_PARAM_OUR_COLOR() {
  return ros::param::has("our_color");
}

const std::array<string, 2> PARAM_OUR_SIDE_valid_values = {"left", "right"};

void get_PARAM_OUR_SIDE(string& tgt, bool error_on_invalid) {
  string val;
  ros::param::get("our_side", val);
  if(!has(PARAM_OUR_SIDE_valid_values, val)) {
    if (error_on_invalid) {
      throw std::runtime_error("Param fetched with invalid value.");
    } else {
      ROS_WARN("Param PARAM_OUR_SIDE fetched with invalid value.");
    }
  }
  tgt = val;
}

void set_PARAM_OUR_SIDE(const string& val, bool error_on_invalid) {
  if(!has(PARAM_OUR_SIDE_valid_values, val)) {
    if (error_on_invalid) {
      throw new std::runtime_error("Tried to set invalid value to a param.");
    } else {
      ROS_WARN("Setting invalid value to param PARAM_OUR_SIDE");
    }
  }
  ros::param::set("our_side", val);
}

bool has_PARAM_OUR_SIDE() {
  return ros::param::has("our_side");
}

const std::string SERVICE_NAVSIM = "navsim";

const std::string SERVICE_OPPONENT_TRACKER = "tracker";

const std::string SERVICE_WORLD_RESET = "world_reset";

const std::string TOPIC_COMMANDS = "robotcommands";

const std::string TOPIC_DEBUG_LINES = "view_debug_lines";

const std::string TOPIC_DEBUG_POINTS = "view_debug_points";

const std::string TOPIC_DETECTION = "vision_detection";

const std::string TOPIC_GEOMETRY = "vision_geometry";

const std::string TOPIC_REFEREE = "vision_referee";

const std::string TOPIC_ROLE_DIRECTIVE = "role_directive";

const std::string TOPIC_ROLE_FEEDBACK = "role_feedback";

const std::string TOPIC_WORLD_STATE = "world_state";

const std::string TRACKER_TYPE_ACCEL = "Acceleration";

const std::string TRACKER_TYPE_SPEED = "Speed";


} // namespace rtt