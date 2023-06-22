import gym
import time
from gym import spaces
import numpy as np
import cv2 as cv
from numpy import size, floor

from collisions import check_collisions, ball_field_collision
from static_classes import Field

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
        self.frame_number = 0

        super(RTTSimEnv, self).__init__()

        # Initialize the field
        self.field = Field()

        # Create window to show simulator state in
        cv.namedWindow("RTTSim", cv.WND_PROP_FULLSCREEN)
        cv.resizeWindow("RTTSim", 1920, 1080)

        # Define observation space (required for machine learning)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(size(get_observation(self.field)),),
                                            dtype=np.float64)

        # Hack to be able to access Publisher
        self.pub = None

    def update(self):
        # Update location of objects
        # Location of ball
        self.field.ball.update()

        # Location of robots
        for robot in self.field.yellowTeam.robots + self.field.blueTeam.robots:
            robot.step()

        # Check and handle collisions of objects
        ball_field_collision(self.field)
        for robot in self.field.yellowTeam.robots + self.field.blueTeam.robots:
            check_collisions(self.field, robot)

        self.pub.send(self.pub.env.field)

        self.frame_number += 1

        time.sleep(0.3)

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

    def set_pub(self, pub):
        self.pub = pub