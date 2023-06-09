"""! @brief Static classes used in the RoboTeam Simulator

@section description_static_classes Description
Defines the classes for various static parts of an SSL Robocup game
- Goal
- CenterCircle
- DefenseArea
- Border
- Field

@section author_static_classes Author
- Created by Jibbe Andringa on 30/11/2022
"""

import math
import numpy as np
import cv2 as cv
from dynamic_classes import Team, Ball


class Goal:
    """! The goal class

    defines the goals and penalty  marks
    """

    def __init__(self, size_y, margin_left, margin_right, side="yellow"):
        """! Initializes the goal

        @param size_y       Y-component of the field size
        @param margin_left  Left margin of the border
        @param margin_right Right margin of the border
        @param side         Color of the team the goal belongs to
        """
        # Size of the goal
        self.size_y = int(100 / 1040 * size_y)
        # Top margin of the goal
        self.margin_top = int((size_y - self.size_y) / 2)
        # Bottom margin of the goal
        self.margin_bottom = int(size_y - ((size_y - self.size_y) / 2))
        # Y-component of the penalty location
        self.penalty_y = int(size_y / 2)
        # Color of the team the goal belongs to
        self.side = side

        # Side specific initializations
        if side == "yellow":
            # position of the goal
            self.margin_left = margin_left
            self.margin_right = margin_left
            # X-component of the penalty location
            self.penalty_x = margin_left + 80
            # The color yellow in bgr
            self.color = (0, 255, 255)
        elif side == "blue":
            # position of the goal
            self.margin_left = margin_right
            self.margin_right = margin_right
            # X-component of the penalty location
            self.penalty_x = margin_right - 80
            # The color blue in bgr
            self.color = (255, 0, 0)

    def render(self, img):
        """! Render the goal and penalty mark onto the screen

        @param img  Image as numpy array, which the goal and penalty mark should be rendered on
        @return     Updated image as numpy array
        """
        # Draw the goal
        cv.line(img, (self.margin_left, self.margin_top), (self.margin_right, self.margin_bottom), self.color, 20)
        # Draw the penalty mark
        cv.circle(img, (self.penalty_x, self.penalty_y), 4, (255, 255, 255), 10)
        return img


class CenterCircle:
    """! The CenterCircle class

    defines the center circle and kick-off mark
    """

    def __init__(self, size_x, size_y):
        """! Initializes the CenterCircle

        @param size_x   X-component of the field size
        @param size_y   Y-component of the field size
        """
        # Radius of the center circle
        self.radius = int(50 / math.sqrt(pow(1340, 2) + pow(1040, 2)) * math.sqrt(pow(size_x, 2) + pow(size_y, 2)))
        # Position of the center circle and kick-off mark
        self.x = int(size_x / 2)
        self.y = int(size_y / 2)

    def render(self, img):
        """! Renders the center circle and kick-off mark

        @param img  Image as numpy array, which the center circle and kick-off mark should be drawn on
        @return     Updated image as numpy array
        """
        # Draw the center circle
        cv.circle(img, (self.x, self.y), self.radius, (255, 255, 255), 10)
        # Draw the kick-off mark
        cv.circle(img, (self.x, self.y), 4, (255, 255, 255), 10)
        return img


class DefenseArea:
    """! The DefenseArea class

    Defines the defense area class
    """

    def __init__(self, size_x, size_y, margin_left, margin_right, side="yellow"):
        """! Initializes the defense area

        @param size_x       X-component of the size of the field
        @param size_y       Y-component of the size of the field
        @param margin_left  Left margin of the border
        @param margin_right Right margin of the border
        @param side         Color of the team the defense area belongs to
        """
        # Size of the defense area
        self.size_x = int(180 / 1340 * size_x)
        self.size_y = int(360 / 1040 * size_y)
        # Color of the team the defense area belongs to
        self.side = side
        # Margins of the defense area
        self.margin_top = int((size_y - self.size_y) / 2)
        self.margin_bottom = int((size_y - (size_y - self.size_y) / 2))
        if side == "yellow":
            self.margin_left = margin_left
            self.margin_right = margin_left + self.size_x
        elif side == "blue":
            self.margin_left = margin_right - self.size_x
            self.margin_right = margin_right

    def render(self, img):
        """! Render the defense area

        @param img  Image as numpy array, which the defense area should be rendered on
        @return     Updated image as numpy array
        """
        cv.rectangle(img, (self.margin_left, self.margin_top), (self.margin_right, self.margin_bottom), (255, 255, 255),
                     10)
        return img


class Border:
    """! The Border class

    Defines the borders of the playing area
    """

    def __init__(self, size_x, size_y):
        """! Initializes the border

        @param size_x   X-component of the size of the field
        @param size_y   Y-component of the size of the field
        """
        # Size of the border
        self.size_x = int(1200 / 1340 * size_x)
        self.size_y = int(900 / 1040 * size_y)
        # Margins of the border
        self.margin_left = int((size_x - self.size_x) / 2)
        self.margin_top = int((size_y - self.size_y) / 2)
        self.margin_right = int(size_x - ((size_x - self.size_x) / 2))
        self.margin_bottom = int(size_y - ((size_y - self.size_y) / 2))

    def render(self, img):
        """! Render the border

        @param img  Image as numpy array which the border should be rendered on
        @return     Updated image as numpy array
        """
        cv.rectangle(img, (self.margin_left, self.margin_top), (self.margin_right, self.margin_bottom), (255, 255, 255),
                     10)
        return img


class Field:
    """! The Field class

    Defines the Field including everything that belongs to the field
    """

    def __init__(self, size_x, size_y):
        """! Initializes the field

        @param size_x   X-component of the size of the screen
        @param size_y   Y-component of the size of the screen
        """
        # Size of the field
        self.size_x = size_x
        self.size_y = size_y
        # Border of the playing area
        self.border = Border(self.size_x, self.size_y)
        # Defense areas
        self.yellowDefenseArea = DefenseArea(self.size_x, self.size_y, self.border.margin_left,
                                             self.border.margin_right, "yellow")
        self.blueDefenseArea = DefenseArea(self.size_x, self.size_y, self.border.margin_left, self.border.margin_right,
                                           "blue")
        # Center circle
        self.centerCircle = CenterCircle(self.size_x, self.size_y)
        # Goals
        self.yellowGoal = Goal(self.size_y, self.border.margin_left, self.border.margin_right, "yellow")
        self.blueGoal = Goal(self.size_y, self.border.margin_left, self.border.margin_right, "blue")
        # Ball
        self.ball = Ball(self.size_x, self.size_y)
        # Teams
        self.yellowTeam = Team(11, "yellow")
        self.blueTeam = Team(11, "blue")
        cv.namedWindow("RTTSim", cv.WND_PROP_FULLSCREEN)
        cv.resizeWindow("RTTSim", 1920, 1080)

    def render(self, img):
        """! Render the field
        """
        img = self.border.render(img)
        img = self.yellowDefenseArea.render(img)
        img = self.blueDefenseArea.render(img)
        img = self.centerCircle.render(img)
        img = self.yellowGoal.render(img)
        img = self.blueGoal.render(img)
        cv.line(img, (self.centerCircle.x, self.border.margin_top), (self.centerCircle.x, self.border.margin_bottom),
                (255, 255, 255), 10)
        cv.line(img, (self.border.margin_left, self.centerCircle.y), (self.border.margin_right, self.centerCircle.y),
                (255, 255, 255), 10)
        img = self.ball.render(img)
        img = self.yellowTeam.render(img)
        img = self.blueTeam.render(img)
        return img
