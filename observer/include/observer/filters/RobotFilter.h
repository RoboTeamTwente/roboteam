//
// Created by rolf on 05-11-19.
//

#ifndef RTT_ROBOTFILTER_H
#define RTT_ROBOTFILTER_H

#include <roboteam_proto/WorldRobot.pb.h>
#include <roboteam_proto/messages_robocup_ssl_detection.pb.h>

#include "CameraFilter.h"
#include "KalmanFilter.h"
#include "RobotObservation.h"

/**
 * A class that can filter robots and predict where they will be based on observations.
 * @author Rolf
 * @date 5 November 2019
 */
class RobotFilter : public CameraFilter {
    typedef KalmanFilter<6, 3> Kalman;

   public:
    /**
     * Construct a RobotFilter.
     * @param detectionRobot Initial observation of the robot we start our filter with.
     * @param detectTime Point in time we start the filter at.
     */
    explicit RobotFilter(const proto::SSL_DetectionRobot &detectionRobot, double detectTime, int cameraID);
    /**
     * Predicts the state of the robot based on past observations
     * @param time The time at which we wish to have a prediction of where the robot will be
     * @param permanentUpdate If set to true, the update is applied permanently to the filter.
     * @param cameraSwitched Set to true if we just switched our main camera, gives a bit more leeway for offsets in position
     * If not, we may still add new observations from after the last time the Filter was between the variable time
     * and the last time the filter was permanently updated.
     */
    void predict(double time, bool permanentUpdate, bool cameraSwitched);
    /**
     * Updates the Filter until the specified time, applying observations of the robot and predicting the state along the way.
     * @param time Time until which we want to update.
     * @param doLastPredict In the very last step after applying all the observations, we can choose to not do the last
     * prediction if we do not immediately want to read the filter's data.
     */
    void update(double time, bool doLastPredict);
    /**
     * Adds an observation of the robot to the filter.
     * @param detectionRobot State of the robot that was observed
     * @param time Time the robot was observed
     * @param cameraID ID of the camera that saw the robot
     */
    void addObservation(const proto::SSL_DetectionRobot &detectionRobot, double time, int cameraID);
    /**
     * Distance of the state of the filter to a point.
     * @param x xCoordinate (in millimeters!)
     * @param y yCoordinate (in millimeters!)
     * @return Distance from the state to the point (x,y)
     */
    [[nodiscard]] double distanceTo(double x, double y) const;
    /**
     * Outputs the current filter state in proto format.
     * @return The Proto message associated with the state of the filter
     */
    [[nodiscard]] proto::WorldRobot asWorldRobot() const;

   private:
    /**
     * Applies the observation to the kalman Filter at the current time the filter is at.
     * This changes the z and r matrices.
     * Make sure you have predicted until the correct time before calling this!
     * @param detectionRobot Robot to be applied
     */
    void applyObservation(const RobotObservation &observation);
    /**
     * A function that casts any angle to the range [-PI,PI)
     * @param angle angle to be limited
     * @return Limited angle in range [-PI,PI)
     */
    [[nodiscard]] double limitAngle(double angle) const;
    /**
     * Initializes the kalman Filter structures
     * @param detectionRobot Contains the initial state of the Filter.
     */
    void KalmanInit(const proto::SSL_DetectionRobot &detectionRobot);
    std::unique_ptr<Kalman> kalman = nullptr;
    int botId;
    std::vector<RobotObservation> observations;
};

#endif  // RTT_ROBOTFILTER_H
