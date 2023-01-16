"""! @brief Dynamic classes used in the RoboTeam Simulator

@section description_collisions
Checks for collisions and updates robots accordingly

@section author_collisions Author
- Created by Jibbe Andringa on 02/12/2022
"""
from math import sqrt


def robot_field_collision(field, robot):
    """! The robot_field_collision function

    Detects collisions between current robot and the field edges
    @param field    The field which the game is played on
    @param robot    The robot for which to detect collisions
    """
    reward = 0
    if robot.pos_y - robot.radius <= 0:
        robot.pos_y = 0 + robot.radius
        robot.vel_y //= -2
        reward = -2
    if robot.pos_y + robot.radius >= field.size_y:
        robot.pos_y = field.size_y - robot.radius
        robot.vel_y //= -2
        reward = -2
    if robot.pos_x - robot.radius <= 0:
        robot.pos_x = 0 + robot.radius
        robot.vel_x //= -2
        reward = -2
    if robot.pos_x + robot.radius >= field.size_x:
        robot.pos_x = field.size_x - robot.radius
        robot.vel_x //= -2
        reward = -2
    return reward


def robot_border_collision(border, robot):
    """! The robot_field_collision function

    Detects collisions between current robot and the field edges
    @param border   The border of the field
    @param robot    The robot for which to detect collisions
    """
    reward = 0
    if robot.pos_y - robot.radius <= border.margin_top:
        reward = -2
    if robot.pos_y + robot.radius >= border.margin_bottom:
        reward = -2
    if robot.pos_x - robot.radius <= border.margin_left:
        reward = -2
    if robot.pos_x + robot.radius >= border.margin_right:
        reward = -2
    # print(robot.side, reward)
    return reward


def robot_robot_collision(field, robot):
    """! The robot_robot_collision function

    Detects collisions between current robot and all other robots on the field
    @param field    The field which the game is played on
    @param robot    The robot for which to detect collisions
    """
    reward = 0
    for bot in field.yellowTeam.robots:
        distance = max(sqrt(pow(robot.pos_x - bot.pos_x, 2) + pow(robot.pos_y - bot.pos_y, 2)), 1)
        if (robot.id != bot.id or robot.side != bot.side) and distance < robot.radius + bot.radius:
            midpoint_x = (robot.pos_x + bot.pos_x) / 2
            midpoint_y = (robot.pos_y + bot.pos_y) / 2

            robot.pos_x = int(midpoint_x + robot.radius * (robot.pos_x - bot.pos_x) / distance)
            robot.pos_y = int(midpoint_y + robot.radius * (robot.pos_y - bot.pos_y) / distance)
            bot.pos_x = int(midpoint_x + bot.radius * (bot.pos_x - robot.pos_x) / distance)
            bot.pos_y = int(midpoint_y + bot.radius * (bot.pos_y - robot.pos_y) / distance)
            reward = -5

    for bot in field.blueTeam.robots:
        distance = max(sqrt(pow(robot.pos_x - bot.pos_x, 2) + pow(robot.pos_y - bot.pos_y, 2)), 1)
        if (robot.id != bot.id or robot.side != bot.side) and distance < robot.radius + bot.radius:
            midpoint_x = (robot.pos_x + bot.pos_x) / 2
            midpoint_y = (robot.pos_y + bot.pos_y) / 2

            robot.pos_x = int(midpoint_x + robot.radius * (robot.pos_x - bot.pos_x) / distance)
            robot.pos_y = int(midpoint_y + robot.radius * (robot.pos_y - bot.pos_y) / distance)
            bot.pos_x = int(midpoint_x + bot.radius * (bot.pos_x - robot.pos_x) / distance)
            bot.pos_y = int(midpoint_y + bot.radius * (bot.pos_y - robot.pos_y) / distance)
            reward = -5
    return reward


def robot_ball_collision(ball, robot):
    """! The robot_robot_collision function

    Detects collisions between current robot and the ball
    @param ball     The ball used in the game
    @param robot    The robot for which to detect collisions
    """
    reward = 0
    distance = max(1, int(sqrt(pow(robot.pos_x - ball.pos_x, 2) + pow(robot.pos_y - ball.pos_y, 2))))
    if robot.has_ball:
        ball.pos_x = robot.pos_x + robot.x_angle * (robot.radius + ball.size)
        ball.pos_y = robot.pos_y + robot.y_angle * (robot.radius + ball.size)
        print(robot.kick_speed)
        if robot.kick_speed:
            ball.vel_x = int(robot.x_angle * robot.kick_speed)
            ball.vel_y = int(robot.x_angle * robot.kick_speed)
            robot.has_ball = False
        else:
            ball.vel_x = robot.vel_x
            ball.vel_y = robot.vel_y
    if distance <= robot.radius + ball.size:
        midpoint_x = (robot.pos_x + ball.pos_x) / 2
        midpoint_y = (robot.pos_y + ball.pos_y) / 2

        robot.pos_x = int(midpoint_x + robot.radius * (robot.pos_x - ball.pos_x) / distance)
        robot.pos_y = int(midpoint_y + robot.radius * (robot.pos_y - ball.pos_y) / distance)
        ball.pos_x = int(midpoint_x + ball.size * (ball.pos_x - robot.pos_x) / distance)
        ball.pos_y = int(midpoint_y + ball.size * (ball.pos_y - robot.pos_y) / distance)
        if robot.dribbler_speed and (
                robot.pos_x > ball.pos_x > robot.pos_x + robot.x_angle * (robot.radius + ball.size + 1) and
                robot.pos_y > ball.pos_y > robot.pos_y + robot.y_angle * (robot.radius + ball.size + 1) or

                robot.pos_x < ball.pos_x < robot.pos_x + robot.x_angle * (robot.radius + ball.size + 1) and
                robot.pos_y > ball.pos_y > robot.pos_y + robot.y_angle * (robot.radius + ball.size + 1) or

                robot.pos_x > ball.pos_x > robot.pos_x + robot.x_angle * (robot.radius + ball.size + 1) and
                robot.pos_y > ball.pos_y > robot.pos_y + robot.y_angle * (robot.radius + ball.size + 1) or

                robot.pos_x > ball.pos_x > robot.pos_x + robot.x_angle * (robot.radius + ball.size + 1) and
                robot.pos_y < ball.pos_y < robot.pos_y + robot.y_angle * (robot.radius + ball.size + 1)):
            robot.has_ball = True
            ball.vel_x = robot.vel_x
            ball.vel_y = robot.vel_y
        else:
            ball.vel_x = int(ball.size * (ball.pos_x - robot.pos_x) / distance)
            ball.vel_y = int(ball.size * (ball.pos_y - robot.pos_y) / distance)
        reward = 1 if robot.side == "yellow" else -1
    return reward


def ball_field_collision(field):
    """! The robot_robot_collision function

    Detects collisions between the ball and the edges of the field
    @param field    The field which the game is played on
    """
    ball = field.ball
    if ball.pos_y - ball.size <= 0:
        ball.pos_y = 0 + ball.size
        ball.vel_y //= -2
    if ball.pos_y + ball.size >= field.size_y:
        ball.pos_y = field.size_y - ball.size
        ball.vel_y //= -2
    if ball.pos_x - ball.size <= 0:
        ball.pos_x = 0 + ball.size
        ball.vel_x //= -2
    if ball.pos_x + ball.size >= field.size_x:
        ball.pos_x = field.size_x - ball.size
        ball.vel_x //= -2


def ball_goal_collision(field):
    """! The robot_robot_collision function

    Detects collisions between the ball and the edges of the field
    @param field    The field which the game is played on
    """
    ball = field.ball
    yellowGoal = field.yellowGoal
    blueGoal = field.blueGoal
    reward = 0
    if ball.pos_y - ball.size >= yellowGoal.margin_top and \
            ball.pos_y + ball.size <= yellowGoal.margin_bottom and \
            ball.pos_x - ball.size <= yellowGoal.margin_left and \
            ball.pos_x + ball.size >= yellowGoal.margin_right:
        reward = -1000000

    if ball.pos_y - ball.size <= blueGoal.margin_top and \
            ball.pos_y + ball.size >= blueGoal.margin_bottom and \
            ball.pos_x - ball.size <= blueGoal.margin_left and \
            ball.pos_x + ball.size >= blueGoal.margin_right:
        reward = 1000000
    return reward

def check_collisions(field, robot):
    """! The check_collisions function

    Checks all collisions where a robot is involved
    @param field    The field which the game is played on
    @param robot    The robot for which to detect collisions
    """
    reward = robot_field_collision(field, robot) + \
             robot_robot_collision(field, robot) + \
             robot_ball_collision(field.ball, robot) + \
             robot_border_collision(field.border, robot)
    return reward
