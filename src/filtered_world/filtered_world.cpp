
#include "roboteam_world/world/filtered_world.h"
#include <vector>
#include "roboteam_utils/Vector2.h"
#include <math.h>

namespace rtt {

    FilteredWorld::FilteredWorld(Predictor predictor) : fresh{false} {
        reset();
        this->predictor = std::move(predictor);
    }

    /// Reset the world, clear all the buffers and states
    void FilteredWorld::reset() {

        // Clear the input buffers
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();

        // These are used for the final world state
        robots_blue_world.clear();
        robots_yellow_world.clear();
        ball_world = rtt::Ball();

        // Initialize the input buffers.
        robots_blue_buffer = RobotMultiCamBuffer();
        robots_yellow_buffer = RobotMultiCamBuffer();

        // The cameras
        world_cams = std::map<int, bool>();

    }

    /// Create a message that has the world in it
    roboteam_proto::World FilteredWorld::as_message() const {

        roboteam_proto::World returnMsg;
        for (auto &robot : robots_blue_world) {
            returnMsg.mutable_blue()->Add(robot.second.as_message());
        }

        for (auto &robot : robots_yellow_world) {
            returnMsg.mutable_yellow()->Add(robot.second.as_message());
        }

        auto worldBall = ball_world.as_message();
        returnMsg.set_allocated_ball(&worldBall);
        returnMsg.set_time(timeLastUpdated);
        return returnMsg;
    }

    /// To be called when a detection_frame message is received.
    void FilteredWorld::detection_callback(const roboteam_proto::SSL_DetectionFrame msg) {
        buffer_detection_frame(msg);
        //std::cout<<msg.camera_id<<std::endl;
        if (is_calculation_needed()) {
            //std::cout<<"MERGE"<<std::endl;
            // Reset the camera update flags.
            for (auto &cam : world_cams) {
                cam.second = false;
            }
            double time_now = msg.t_capture();
            merge_frames(time_now);
            timeLastUpdated = time_now;
            fresh = true;
        }
    }

    /// Consume a message if it is fresh
    std::optional<roboteam_proto::World> FilteredWorld::consumeMsg() {
        if (isFresh()) {
            setFresh(false);
            return as_message();
        }
        return {};
    }

    /// Adds a received detection frame to the buffers.
    void FilteredWorld::buffer_detection_frame(const roboteam_proto::SSL_DetectionFrame msg) {

        auto cam_id = msg.camera_id();

        // Set this cameras updated flag.
        // If this camera hasn't sent frames before, it is now added to the list of cameras. We reset the other camera's to true as their information is still mergeable now.
        if (world_cams.find(cam_id) == world_cams.end()) {
            for (auto &cam :world_cams) {
                cam.second = true;
            }
        }
        world_cams[cam_id] = true;
        timeFrameCaptured[cam_id] = msg.t_capture();

        // Add the robot data to the buffers
        for (const roboteam_proto::SSL_DetectionRobot robot : msg.robots_blue()) {
            int bot_id = robot.robot_id();

            robots_blue_buffer[bot_id][cam_id] = roboteam_proto::SSL_DetectionRobot(robot);
        }
        for (const roboteam_proto::SSL_DetectionRobot robot : msg.robots_yellow()) {
            int bot_id = robot.robot_id();

            robots_yellow_buffer[bot_id][cam_id] = roboteam_proto::SSL_DetectionRobot(robot);
        }

        // ==== Ball ====
        // There can be multiple balls in one frame:
        // Find the ball with the smallest distance from the linear extrapolation of last known position
        // and add that to the buffer.
        if (!msg.balls().empty()) {
            Position previousBallPos = ball_world.get_position();
            Position previousBallVel = ball_world.get_velocity();
            roboteam_proto::SSL_DetectionBall closestBall = msg.balls()[0];
            Position predictedPosition = previousBallPos + previousBallVel * (msg.t_capture() - timeLastUpdated);
            double closestDist2 = Vector2(closestBall.x(), closestBall.y()).dist2(Vector2(predictedPosition.x, predictedPosition.y));

            for (auto const &ball : msg.balls()) {

                double dist2 = Vector2(ball.x(), ball.y()).dist2(Vector2(predictedPosition.x, predictedPosition.y));
                if (dist2 < closestDist2) {
                    closestBall = ball;
                    closestDist2 = dist2;
                }
            }

            ball_buffer[cam_id] = closestBall; // msg.balls[0];
        } else {
            ball_buffer.erase(cam_id);
        }
    }


    /// Returns true when every camera's frame has updated.
    /// When there are no cameras, this function will always return false.
    bool FilteredWorld::is_calculation_needed() const {
        if (world_cams.empty()) {
            // No cameras? No use doing a frame merge calculation.
            return false;
        }
        for (auto &cam : world_cams) {

            if (!cam.second) {
                return false;
            }
        }
        return true;
    }

    /**
     * Merges the frames of multiple camera's into a world state.
     * Merges the robots using FilteredWorld::merge_robots
     * Picks the best option for the ball.
     * Clears the buffers for the frames afterwards.
     */
    void FilteredWorld::merge_frames(double timestamp) {
        // merge all the robots in the buffers
        std::string s;
     //   get_PARAM_OUR_COLOR(s);
        bool isBlueOurTeam = s == "blue";
        merge_robots(robots_blue_buffer, robots_blue_world, old_blue, timestamp, isBlueOurTeam);
        merge_robots(robots_yellow_buffer, robots_yellow_world, old_yellow, timestamp, !isBlueOurTeam);
        // merge the balls in the buffer
        merge_balls(timestamp);
        // Clear the buffers.
        robots_blue_buffer.clear();
        robots_yellow_buffer.clear();
    }

    /// Merges the balls from different frames
    void FilteredWorld::merge_balls(double timestamp) {
        // Take the ball from the camera where extrapolation with respect to the world is closest to
        // the extrapolation of the last world state's velocity and position
        if (!ball_buffer.empty()) {
            // extrapolation of the last world's state velocity and position
            Position previousBallPos = ball_world.get_position();
            Position previousBallVel = ball_world.get_velocity();
            Position predictedPosition = previousBallPos + previousBallVel * (timestamp - timeLastUpdated);

            // First extrapolation
            roboteam_proto::SSL_DetectionBall closestBall = ball_buffer.begin()->second;
            int best_camera = ball_buffer.begin()->first;
            // Initial extrapolation. Does the same as below
            double closestDist2 = Vector2(closestBall.x(), closestBall.y()).dist2(
                    Vector2(predictedPosition.x, predictedPosition.y));

            for (auto const &detectedBall : ball_buffer) {
                // Extrapolate from detectionframe's capture time to current time.
                Position detectedBallPos = Position(Vector2(detectedBall.second.x(), detectedBall.second.y()));
                double dist2 = Vector2(detectedBallPos.x,detectedBallPos.y).dist2(
                        Vector2(predictedPosition.x, predictedPosition.y));
                // Pick the Extrapolation which works best
                if (dist2 < closestDist2) {
                    //best_camera=detectedBall.first;
                    closestBall = detectedBall.second;
                    closestBall.set_x(detectedBallPos.x);
                    closestBall.set_y(detectedBallPos.y);
                    closestDist2 = dist2;
                }
            }
            // Move the ball to the one that has best extrapolation
            //ball_world.move_to(ball_buffer[best_camera].pos.x,ball_buffer[best_camera].pos.y,ball_buffer[best_camera].z);
            ball_world.move_to(closestBall.x(), closestBall.y(), closestBall.z());
            ball_world.set_existence(closestBall.area());
            ball_world.set_visible(true);
        } else {
            ball_world.set_visible(false);
        }

        // Update the predictor and get a speed vector for the ball
        predictor.update(ball_world, timestamp);
        boost::optional<Position> ballVel = predictor.computeBallVelocity();

        if (ballVel) {
            Position vel = *ballVel;
            ball_world.set_velocity(static_cast<float>(vel.x), static_cast<float>(vel.y));
        }
    }
    /// Merges the robots from different frames
    void FilteredWorld::merge_robots(RobotMultiCamBuffer &robots_buffer, std::map<int,
            rtt::Robot> &robots_output, std::map<int, rtt::Robot> &old_buffer, double timestamp, bool our_team) {
        //For every robot buffer
        for (auto &robot_buffer : robots_buffer) {
            auto bot_id = (uint) robot_buffer.first;

            Robot robot;
            robot.set_id(bot_id);

            // Places the robot to the extrapolation of the last frame we saw it in. (assumes good camera calibration)
            //TODO: A position is 'bad' if the internal velocity of the robot is exceedingly large. This is hard to implement for different scenario's
            //TODO: Catch case if time measurement is off.
            Vector2 previousPosition;
            //If the robot was on last world, get the previous position. If not, initialize it to position(0,0)
            if (robots_output.find(bot_id) != robots_output.end()) {
                previousPosition = Vector2(robots_output[bot_id].get_position().x,
                                           robots_output[bot_id].get_position().y);
            } else {
                std::cout << "Adding bot " << bot_id << std::endl;
                previousPosition = Vector2(0, 0);
            }

            float w = 0;

            Vector2 robotPos;
            //TODO: Rotation is now fixed to last frames information. Could still be extrapolated?
            Vector2 zero = {0, 0};
            double last_frame = timeLastUpdated;
            for (auto &buf : robot_buffer.second) {
                if (timeFrameCaptured[buf.first] > last_frame) {
                    last_frame = timeFrameCaptured[buf.first];
                    Vector2 bufPosition = Vector2(buf.second.x(), buf.second.y());
                    robotPos=bufPosition;
                    w = buf.second.orientation();
                    if (w>=M_PI){
                        while(w>=M_PI){
                            w=w-2*M_PI;
                        }
                    }
                    else if(w<-M_PI){
                        while(w<-M_PI){
                            w=w+2*M_PI;
                        }
                    }
                }
            }
            // Assign the robot position and rotation to the extrapolation calculated.
            robot.move_to((float) robotPos.x, (float) robotPos.y);
            robot.rotate_to(w);

            // Send an update and discard old data for buffers used for calculations
            predictor.update(robot, our_team, timestamp);
            // compute velocity of the robot and update if received
            boost::optional<Position> robotVel = predictor.computeRobotVelocity(bot_id, our_team);
            if (robotVel) {
                Position vel = *robotVel;

                robot.set_vel(static_cast<float>  (vel.x),
                              static_cast<float>  (vel.y),
                              static_cast<float>  (vel.rot));
            }

            // Update the last detection time used in calculations.
            robot.update_last_detection_time(timestamp);
            // add the updated robot to robot_output and old_buffer.
            robots_output[bot_id] = robot;
            old_buffer[bot_id] = robot;
        }

        // Remove old robots.
        auto botIter = robots_output.begin();

        while (botIter != robots_output.end()) {
            // Remove robots that are not detected for 0.5 seconds.
            if (botIter->second.is_detection_old(timestamp, 0.5)) {
                std::cout << "Removing bot : " << botIter->second.get_id() << ". Too old" << std::endl;
                botIter = robots_output.erase(botIter);
            } else if (botIter->second.is_detection_from_future(timestamp)) {
                std::cout << "Removing bot : " << botIter->second.get_id() << ". It's from the future?" << std::endl;
                botIter = robots_output.erase(botIter);
            } else {
                ++botIter;
            }
        }
    }

    bool FilteredWorld::isFresh() {
        return fresh;
    }

    void FilteredWorld::setFresh(bool newFresh) {
        fresh = newFresh;
    }
}
