# Robot #

## General ##
The structure that represents a robot, it represents the team it's on, and some constants. POD.

## Team ##
The enum Team in [team.hpp](https://github.com/RoboTeamTwente/roboteam_ai/blob/development/include/roboteam_ai/world_new/Team.hpp) is fairly simple. A robot is either `us`, meaning it belongs to us, `them`, meaning it belongs to them or `both`, which is invalid. 

## Members ##
`id` -> The ID of the robot (between 1 and 11, 0 is invalid iirc)

`team` -> Enum value on which team it is, read the [section above](#team)

`pos` -> Current position of the robot

`vel` -> Current velocity of the robot

`angle` -> Current angle of the robot

`distanceToBall` -> Distance from the robot to the ball

`angleDiffToBall` -> Angle offset between the kicker and the ball.

`angularVelocity` -> Angular velocity of the robot.

`batteryLow` -> Flag that indicates whether the battery of the robot is low.

`workingDribbler` -> True if dribbler works, false if not.

`workingBallSensor` -> Dito but for sensor.

`BallSensorSeesBall` -> True if the ball sensor picks up the ball, false if not.

`DribblerSeesBall` -> True if the dribbler picks up the ball, false if not.

`ballPos` -> Position of the ball (if seesBall == true)


## Functions ## 

### Trivial getters and setters, therefore will **not** be covered ##

## Advice ##
Honestly it's POD I don't have any advice for this one.
