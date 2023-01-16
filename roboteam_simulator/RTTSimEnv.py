import gym
from gym import spaces
import numpy as np
import cv2 as cv
from numpy import size, floor

from collisions import check_collisions, ball_field_collision, ball_goal_collision
from static_classes import Field


# robot commands
# id = robot_command[0]
# velocity_x = robot_command[1]
# velocity_y = robot_command[2]
# angle = robot_command[3]
# angular_velocity = robot_command[4]
# use_angular_velocity = robot_command[5]
# camera_angle_of_robot = robot_command[6]
# camera_angle_of_robot_is_set = robot_command[7]
# kick_speed = robot_command[8]
# wait_for_ball = robot_command[9]
# kick_at_angle = robot_command[10]
# kick_type = robot_command[11]
# dribbler_speed = robot_command[12]
# ignore_packet = robot_command[13]

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
        self.steps = 0
        # Initialize the field
        self.field = Field(1340, 1040)
        cv.namedWindow("RTTSim", cv.WND_PROP_FULLSCREEN)
        cv.resizeWindow("RTTSim", 1920, 1080)

        # define action space
        # shape --> 11X14
        # for each robot:
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
        self.action_space = spaces.Box(low=np.array([[0, -1, -1, 0, -10, 0, 0, 0, 0, 0, 0, 0, 1, 0]] * 11),
                                       high=np.array(
                                           [[+10, +1, +1, +360, +10, +1, +360, +1, +65, +0, +360, +2, +1, +1]] * 11),
                                       shape=(11, 14), dtype=np.int)
        # define observation space
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(size(get_observation(self.field)),),
                                            dtype=np.float64)

    def step(self, action):
        self.steps += 1
        done = False if self.steps < 100000 else True
        reward = 0
        self.field.ball.update()
        ball_field_collision(self.field)
        reward += ball_goal_collision(self.field)
        if reward:
            done = True
        for command in action:
            self.field.yellowTeam.robots[int(command[0])].command(command)
        for robot in self.field.yellowTeam.robots:
            temp_x, temp_y = robot.pos_x, robot.pos_y
            robot.update()
            reward += check_collisions(self.field, robot)
            # if robot.pos_x == temp_x and robot.pos_y == temp_y:
            #     reward += -10

        # blue team takes random actions
        action = self.action_space.sample()
        for command in action:
            self.field.blueTeam.robots[command[0]].command(command)
        for robot in self.field.blueTeam.robots:
            robot.update()
            check_collisions(self.field, robot)
        obs = get_observation(self.field)

        info = {}

        return obs, reward, done, info

    def reset(self, **kwargs):
        # reset simulator to initial position
        self.field = Field(1340, 1040)
        self.steps = 0
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
