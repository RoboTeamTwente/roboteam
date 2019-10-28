#pragma once

#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_utils/Position.h"

#define INVALID_ROBOT_ID 99999

namespace rtt {

    class Robot {

    private:
        uint id;
        // bool our_team;
        float x;
        float y;
        float angle;

        float x_vel;
        float y_vel;
        float w;

        double last_detection_time;

    public:
        Robot();
        explicit Robot(uint id);
        Robot(uint id, float x, float y, float w);

        void set_id(uint id);
        void move_to(float x, float y);
        void rotate_to(float w);
        void set_vel(float x_vel, float y_vel, float w_vel);
        void update_last_detection_time(double time);
        bool is_detection_old(double time, double threshold);
        bool is_detection_from_future(double time);
        Position get_position() const;
        Position get_velocity() const;
        uint get_id() const;

       proto::WorldRobot as_message() const;
    };

}
