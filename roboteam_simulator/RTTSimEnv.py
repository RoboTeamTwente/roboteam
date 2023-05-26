import gym
from gym import spaces
import numpy as np
import cv2 as cv
from numpy import size, floor

from collisions import check_collisions, ball_field_collision
from static_classes import Field


# Robot commands (accessed as robot_command[i])
# index | action            | low   | high
# 0     | ID                | 0     | +10
# 1     | x acceleration    | -1    | +1
# 2     | y acceleration    | -1    | +1
# 3     | angle             | 0     | +360
# 4     | angular velocity  | -10   | +10
# 5     | use ang vel       | 0     | 1
# 6     | camera angle      | 0     | +360
# 7     | camera angle set  | 0     | 1
# 8     | kick speed        | 0     | 7
# 9     | wait for ball     | 0     | 1
# 10    | kick at angle     | 0     | 360
# 11    | kick type         | 0     | 2
# 12    | dribbler speed    | 0     | 1
# 13    | ignore_packet     | 0     | 1
# Low and high artefacts of original ML ideas

def get_observation(field):
    obs = []
    # Ball
    obs.append(field.ball.pos_x)
    obs.append(field.ball.pos_y)
    obs.append(field.ball.vel_x)
    obs.append(field.ball.vel_y)
    # Yellow robots
    for robot in field.yellowTeam.robots:
        obs.append(robot.id)
        obs.append(0)
        obs.append(robot.pos_x)
        obs.append(robot.pos_y)
        obs.append(robot.vel_x)
        obs.append(robot.vel_y)
        obs.append(robot.acc_x)
        obs.append(robot.acc_y)
        obs.append(robot.angle)
        obs.append(robot.ang_vel)
        obs.append(robot.kick_speed)
        obs.append(int(robot.has_ball))
        obs.append(robot.dribbler_speed)
    # Blue robots
    for robot in field.blueTeam.robots:
        obs.append(robot.id)
        obs.append(1)
        obs.append(robot.pos_x)
        obs.append(robot.pos_y)
        obs.append(robot.vel_x)
        obs.append(robot.vel_y)
        obs.append(robot.acc_x)
        obs.append(robot.acc_y)
        obs.append(robot.angle)
        obs.append(robot.ang_vel)
        obs.append(robot.kick_speed)
        obs.append(int(robot.has_ball))
        obs.append(robot.dribbler_speed)
    return np.asarray(obs)


class RTTSimEnv(gym.Env):
    def __init__(self):
        super(RTTSimEnv, self).__init__()
        # Initialize the field
        self.field = Field(1340, 1040)
        cv.namedWindow("RTTSim", cv.WND_PROP_FULLSCREEN)
        cv.resizeWindow("RTTSim", 1920, 1080)

        # define observation space
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(size(get_observation(self.field)),),
                                            dtype=np.float64)

    def update(self):
        # Update location of objects
        # Location of ball
        self.field.ball.update()

        # Location of robots
        for robot in self.field.yellowTeam.robots + self.field.blueTeam.robots:
            robot.update()

        # Check and handle collisions of objects
        ball_field_collision(self.field)
        for robot in self.field.yellowTeam.robots + self.field.blueTeam.robots:
            check_collisions(self.field, robot)

        return

    def reset(self, **kwargs):
        # reset simulator to initial position
        self.field = Field(1340, 1040)
        obs = get_observation(self.field)
        return obs

    def render(self, mode="human"):
        # open the visual simulator
        img = np.zeros((self.field.size_y, self.field.size_x, 3), np.uint8)
        img = self.field.render(img)
        cv.imshow("RTTSim", img)
        k = cv.waitKey(1)
        if k == 27:
            self.steps = np.inf
        return 0

    def close(self):
        # close the simulator
        return 0
