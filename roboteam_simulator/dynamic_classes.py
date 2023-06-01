"""! @brief Dynamic classes used in the RoboTeam Simulator

@section description_dynamic_classes Description
Defines the classes for various dynamic parts of an SSL Robocup game
- Ball
- Team
- Robot

@section author_dynamic_classes Author
- Created by Jibbe Andringa on 30/11/2022
"""
import math

import cv2 as cv


class Ball:
    """! The ball class

    Defines the ball class also referred to as "oranjebal"
    """

    def __init__(self, size_x, size_y):
        """! Initializes the ball

        @param size_x   X-component of the field size
        @param size_y   Y-component of the field size
        """
        # initial position
        self.pos_x = size_x / 2
        self.pos_y = size_y / 2
        # initial velocity
        self.vel_x = 0
        self.vel_y = 0
        # color in bgr (orange)
        self.color = (0, 123, 255)
        # radius of the ball
        self.size = 4

    def render(self, img):
        """! Render the ball on the screen

        @param img  Image as numpy array, which the ball should be rendered on
        @return Updated image as numpy array
        """
        img = cv.circle(img, (int(self.pos_x), int(self.pos_y)), self.size, self.color, -1)
        return img

    def update(self):
        """! Update the status of the ball
        """
        self.pos_x += self.vel_x
        self.pos_y += self.vel_y

        self.vel_x = self.vel_x - self.vel_x / 100
        self.vel_y = self.vel_y - self.vel_y / 100


class Robot:
    """! The robot class

    Defines the robot class used to play the game with
    """

    def __init__(self, side, id, x, y):
        """! Initializes the robot

        @param side The color of the team the robot belongs to
        @para id    ID of the robot
        @param x    X-component of initial position of the robot
        @param y    Y-component of initial position of the robot
        """
        # ID of the robot
        self.id = id
        # Color of the team the robot belongs to
        self.side = side
        self.color = (0, 255, 255) if side == "yellow" else (255, 0, 0)
        # Initial position of the robot
        self.pos_x = x
        self.pos_y = y
        # Initial velocity of the robot
        self.vel_x = 0
        self.vel_y = 0
        # Initial acceleration of the robot
        self.acc_x = 0
        self.acc_y = 0
        # Size of the robot
        self.radius = 9
        # Initial angle of the robot
        self.angle = 0
        self.x_angle = math.cos(math.radians(self.angle))
        self.y_angle = math.sin(math.radians(self.angle))
        # Initial angular velocity
        self.ang_vel = 0
        # Initial kick speed
        self.kick_speed = 0
        # Does the robot have the ball
        self.has_ball = False
        # Initial dribbler speed
        self.dribbler_speed = 0

        self.last_command_received = None

    def render(self, img):
        """! Render the robot on the screen

        @param img  Image as numpy array, which the robot should be rendered on
        @return Updated image as numpy array
        """
        # Render robot
        cv.circle(img, (int(self.pos_x), int(self.pos_y)), self.radius, self.color, -1)
        # Render angle indicator
        self.x_angle = math.cos(math.radians(self.angle))
        self.y_angle = math.sin(math.radians(self.angle))
        cv.line(img, (int(self.pos_x), int(self.pos_y)), (int(self.pos_x + 20 * self.x_angle), int(self.pos_y + 20 * self.y_angle)), self.color, 1)
        return img

    def update(self):
        """! Update the status of the robot
        """
        # Update position
        self.pos_x += self.vel_x
        self.pos_y += self.vel_y
        # Update angle
        self.angle = (self.angle + self.ang_vel) % 360

    def command(self, robot_command):
        """! Process robot command

        @param robot_command    List of attributes the robot should adhere to
        """
        # Check for correct ID
        if robot_command.id == self.id or robot_command.id == -1:
            self.vel_x = robot_command.velocity_x
            self.vel_y = robot_command.velocity_y
            if robot_command.use_angular_velocity:
                self.ang_vel = robot_command.angular_velocity
            self.kick_speed = robot_command.kick_speed
            self.dribbler_speed = robot_command.dribbler_speed

        # Exit if ID is wrong
        else:
            print("ERROR: Wrong id")
            exit()

    def step(self):
        # Set course of action to be last command received
        if self.last_command_received is not None:
            self.command(self.last_command_received)

        self.update()


class Team:
    """! The Team class

    Defines the teams that play on the field, either yellow or blue
    """

    def __init__(self, size=11, side="yellow"):
        """! Initialize the team

        @param size The number of robots in the team
        @param side The color of the Team
        """
        self.size = size
        self.side = side
        self.robots = []
        for i in range(self.size):
            self.robots.append(Robot(self.side, i, 670 - 20 * (i + 1) if side == "yellow" else 670 + 20 * (i + 1), 100))

    def render(self, img):
        """! Render all robots in the team

        @param img  Image as numpy array, which the team should be rendered on
        @return Updated image as numpy array
        """
        for robot in self.robots:
            img = robot.render(img)
        return img
