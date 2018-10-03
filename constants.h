#pragma once

#include <array>
#include <string>

// generated from String Constants

namespace rtt {

using string = std::string;

extern const std::string NODE_AVOID_ROBOTS_TEST;

extern const std::string NODE_DO_STRATEGY;

extern const std::string NODE_DRIBBLE_TEST;

extern const std::string NODE_FOLLOW_PATH_TEST;

extern const std::string NODE_GET_BALL_TEST;

extern const std::string NODE_GO_TO_POS_TEST;

extern const std::string NODE_KEEPER_TEST;

extern const std::string NODE_KICK_AT_GOAL_TEST;

extern const std::string NODE_KICK_TEST;

extern const std::string NODE_ROTATE_AROUND_POINT_TEST;

extern const std::string NODE_STAND_FREE_TEST;

extern const std::string NODE_TACTICS;

void get_PARAM_ITERATIONS_PER_SECOND(int&);
void set_PARAM_ITERATIONS_PER_SECOND(const int&);
bool has_PARAM_ITERATIONS_PER_SECOND();

void get_PARAM_KICKING(bool&);
void set_PARAM_KICKING(const bool&);
bool has_PARAM_KICKING();

void get_PARAM_NORMALIZE_FIELD(bool&);
void set_PARAM_NORMALIZE_FIELD(const bool&);
bool has_PARAM_NORMALIZE_FIELD();

void get_PARAM_NUM_ROLE_NODES(int&);
void set_PARAM_NUM_ROLE_NODES(const int&);
bool has_PARAM_NUM_ROLE_NODES();

extern const std::array<string, 2> PARAM_OUR_COLOR_valid_values;
void get_PARAM_OUR_COLOR(string& tgt, bool error_on_invalid = false);
void set_PARAM_OUR_COLOR(const string& val, bool error_on_invalid = true);
bool has_PARAM_OUR_COLOR();

extern const std::array<string, 2> PARAM_OUR_SIDE_valid_values;
void get_PARAM_OUR_SIDE(string& tgt, bool error_on_invalid = false);
void set_PARAM_OUR_SIDE(const string& val, bool error_on_invalid = true);
bool has_PARAM_OUR_SIDE();

extern const std::string SERVICE_NAVSIM;

extern const std::string SERVICE_OPPONENT_TRACKER;

extern const std::string SERVICE_WORLD_RESET;

extern const std::string TOPIC_COMMANDS;

extern const std::string TOPIC_DEBUG_LINES;

extern const std::string TOPIC_DEBUG_POINTS;

extern const std::string TOPIC_DETECTION;

extern const std::string TOPIC_GEOMETRY;

extern const std::string TOPIC_REFEREE;

extern const std::string TOPIC_ROLE_DIRECTIVE;

extern const std::string TOPIC_ROLE_FEEDBACK;

extern const std::string TOPIC_WORLD_STATE;

extern const std::string TRACKER_TYPE_ACCEL;

extern const std::string TRACKER_TYPE_SPEED;


} // namespace rtt