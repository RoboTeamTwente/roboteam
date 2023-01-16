import zmq
from proto.messages_robocup_ssl_wrapper_pb2 import SSL_WrapperPacket
from proto.messages_robocup_ssl_detection_pb2 import SSL_DetectionFrame, SSL_DetectionBall, SSL_DetectionRobot
import time


class Publisher:
    def __init__(self):
        ADDRESS = '256.4.5.23.2'
        PORT = '10006'
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind('tcp://' + ADDRESS + ':' + PORT)
        self.ball = SSL_DetectionBall()
        self.robots_yellow = []
        self.robots_blue = []
        self.wrapper = SSL_WrapperPacket()
        self.frame_number = 0
        self.detection_frame = SSL_DetectionFrame()

    def send(self):
        packet = self.wrapper.SerializeToString()
        self.socket.send(packet)

    def wrap_ball(self, ball):
        self.ball.confidence = 1
        self.ball.x = ball.pos_x
        self.ball.y = ball.pos_y
        self.ball.pixel_x = ball.pos_x
        self.ball.pixel_y = ball.pos_y

    def wrap_robots(self, yellow_robots, blue_robots):
        for robot in yellow_robots:
            temp_bot = SSL_DetectionRobot()
            temp_bot.confidence = 1
            temp_bot.robot_id = robot.id
            temp_bot.x = robot.pos_x
            temp_bot.y = robot.pos_y
            temp_bot.orientation = robot.angle
            temp_bot.pixel_x = robot.pos_x
            temp_bot.pixel_y = robot.pos_y
            self.robots_yellow.append(temp_bot)

        for robot in blue_robots:
            temp_bot = SSL_DetectionRobot()
            temp_bot.confidence = 1
            temp_bot.robot_id = robot.id
            temp_bot.x = robot.pos_x
            temp_bot.y = robot.pos_y
            temp_bot.orientation = robot.angle
            temp_bot.pixel_x = robot.pos_x
            temp_bot.pixel_y = robot.pos_y
            self.robots_blue.append(temp_bot)

    def wrap_detection_frame(self):
        self.detection_frame.frame_number = ++self.frame_number
        self.detection_frame.t_capture = time.time()
        self.detection_frame.t_sent = time.time()
        self.detection_frame.camera_id = 0
        self.detection_frame.balls = self.ball
        self.detection_frame.robots_yellow = self.robots_yellow
        self.detection_frame.robots_blue = self.robots_blue


publisher = Publisher()
publisher.wrap_detection_frame()
