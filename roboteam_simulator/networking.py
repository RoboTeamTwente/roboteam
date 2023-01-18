"""! @brief networking classes used for communication with the teams

@section description_networking Description
Defines the classes used for publishing the SSL packages and receiving commands from the teams. The SSL packages are
published as multicast. Meaning that multiple applications will be able to listen to these packages.
-Publisher
-Subscriber

@section author_static_classes Author
- Created by Jibbe Andringa on 16/01/2023
"""
import zmq
import time
from PyQt5.QtCore import QByteArray

from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.messages_robocup_ssl_detection_pb2 import SSL_DetectionFrame, SSL_DetectionBall, SSL_DetectionRobot

# Addresses and ports used to communicate with the teams
PUBLISH_ADDRESS = '224.5.23.2'  # The address the packages should be published on
PUBLISH_PORT = '10006'  # The port the packages should be published on
SUB_ADDRESS = '224.5.23.2'  # The address the packages should be received on
SUB_PORT = '10006'  # The port the packages should be received on


class Publisher:
    """! The Publisher class

    The Publisher is responsible for multicasting the data by the simulator in custom protobuf packages
    """

    def __init__(self):
        """! Initializes the Publisher by creating a socket and binding it to the defined publishing address/port
        """
        self.context = zmq.Context()  # The zmq context that manages the sockets
        self.socket = self.context.socket(zmq.PUB)  # The socket we will be publishing with
        self.socket.bind(f'tcp://{PUBLISH_ADDRESS}:{PUBLISH_PORT}')  # Bind the socket to the publishing address/port
        self.ball = SSL_DetectionBall()  # The SSL_DetectionBall package containing data about the ball
        self.robots_yellow = []  # List of SSL_DetectionRobot packages containing data about the yellow robots
        self.robots_blue = []  # List of SSL_DetectionRobot packages containing data about the blue robots
        self.wrapper = SSL_WrapperPacket()  # The SSL_WrapperPacket package containing all data to be published
        self.frame_number = 0  # Frame number of the current SSL_DetectionFrame
        self.detection_frame = SSL_DetectionFrame()  # The SSL_DetectionFrame package containing data from the simulator

    def send(self, field):
        # noinspection GrazieInspection
        """! Sends the package to the earlier initialized socket

        @param field Field data that needs to be sent
        """
        self.wrap_ball(field.ball)  # Wrap the ball in na SSL_DetectionBall package
        self.wrap_robots(field.yellowTeam.robots, field.blueTeam.robots)  # Wrap robots in SSL_DetectionRobot packages
        self.wrap_detection_frame()  # Wrap all data in an SSL_DetectionFrame package
        self.wrap_packet()
        packet = self.wrapper.SerializeToString()  # Serializes the wrapper packet
        self.socket.send(packet)  # Sends the serialized packet to the socket

    def wrap_ball(self, ball):
        """! Wraps the ball data in an SSL_DetectionBall package

        @param ball The ball data to be wrapped
        """
        self.ball.confidence = 1  # Factor of how sure the camara is about the ball data, is always 1 in simulation
        self.ball.x = ball.pos_x  # X-position of the ball
        self.ball.y = ball.pos_y  # Y-position of the ball
        self.ball.pixel_x = ball.pos_x  # Pixelated x-position of the ball
        self.ball.pixel_y = ball.pos_y  # Pixelated y-position of the ball

    def wrap_robots(self, yellow_robots, blue_robots):
        """! Wraps the robots data in an SSL_DetectionRobot package and adds it to the list of its respective team.

        @param yellow_robots A list of all yellow team robots to be wrapped
        @param blue_robots A list of all blue team robots to be wrapped
        """
        self.robots_yellow.clear()  # Make sure there are no yellow robots left from previous frame
        # Wraps the yellow team robots one by one and puts them into the robots_yellow list
        for robot in yellow_robots:
            temp_bot = SSL_DetectionRobot()  # Temporary SSL_DetectionRobot package to add to the yellow robots list
            temp_bot.confidence = 1  # Factor of how sure the camera is about the robot data, is always 1 in simulation
            temp_bot.robot_id = robot.id  # ID of the robot
            temp_bot.x = robot.pos_x  # X-position of the robot
            temp_bot.y = robot.pos_y  # Y-position of the robot
            temp_bot.orientation = robot.angle  # Angle of the robot
            temp_bot.pixel_x = robot.pos_x  # Pixelated x-position of the robot
            temp_bot.pixel_y = robot.pos_y  # Pixelated y-position of the robot
            self.robots_yellow.append(temp_bot)  # Puts the SSL_DetectionRobot package in the yellow robots list

        self.robots_blue.clear()  # Make sure there are no blue robots left from previous frame
        # Wraps the blue team robots one by one and puts the into the robots_blue list
        for robot in blue_robots:
            temp_bot = SSL_DetectionRobot()  # Temporary SSL_DetectionRobot package to add to the blue robots list
            temp_bot.confidence = 1  # Factor of how sure the camera is about the robot data, is always 1 in simulation
            temp_bot.robot_id = robot.id  # ID of the robot
            temp_bot.x = robot.pos_x  # X-position of the robot
            temp_bot.y = robot.pos_y  # Y-position of the robot
            temp_bot.orientation = robot.angle  # Angle of the robot
            temp_bot.pixel_x = robot.pos_x  # Pixelated x-position of the robot
            temp_bot.pixel_y = robot.pos_y  # Pixelated y-position of the robot
            self.robots_blue.append(temp_bot)  # Puts the SSL_DetectionRobot package in the blue robots list

    def wrap_detection_frame(self):
        """! Wraps all data from the simulation into the SSL_DetectionFrame package
        """
        self.detection_frame.frame_number = ++self.frame_number  # The number of this frame, is incremented by one.
        self.detection_frame.t_capture = time.time()  # The time at which the frame was captured
        self.detection_frame.t_sent = time.time()  # The time at which the frame is sent
        self.detection_frame.camera_id = 0  # ID of the camera which captured the frame
        del self.detection_frame.balls[:]  # Make sure there are no balls left from previous frame
        self.detection_frame.balls.append(self.ball)  # The SSL_DetectionBall package
        del self.detection_frame.robots_yellow[:]  # Make sure there are no yellow robots left from previous frame
        for robot in self.robots_yellow:  # Add yellow robots to the detection frame
            self.detection_frame.robots_yellow.append(robot)
        del self.detection_frame.robots_blue[:]  # Make sure there are no blue robots left from previous frame
        for robot in self.robots_blue:  # Add blue robots to the detection frame
            self.detection_frame.robots_blue.append(robot)

    def wrap_packet(self):
        """! Wraps the detection frame and geometry data into the SSL_WrapperPacket package
        """
        self.wrapper.detection.CopyFrom(self.detection_frame)  # Add the detection frame the SSL_WrapperPacket message


class Subscriber:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f'tcp://{SUB_ADDRESS}:{SUB_PORT}')

    def receive(self):
        msg = self.socket.recv()
        print(msg)
        return msg
