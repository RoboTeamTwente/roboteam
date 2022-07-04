# AUTOGENERATED. Run generator/main.py to regenerate
"""
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ] [  6   ] [  7   ] [  8   ] [  9   ] [  10  ] [  11  ] [  12  ] [  13  ] [  14  ]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- remVersion
-------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- id
-------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- messageId
-------- -------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- doKick
-------- -------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- doChip
-------- -------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- kickAtAngle
-------- -------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- doForce
-------- -------- -------- 1------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- useCameraAngle
-------- -------- -------- -1111111 11111111 1------- -------- -------- -------- -------- -------- -------- -------- -------- -------- rho
-------- -------- -------- -------- -------- -1111111 11111111 1------- -------- -------- -------- -------- -------- -------- -------- theta
-------- -------- -------- -------- -------- -------- -------- -1111111 11111111 1------- -------- -------- -------- -------- -------- angle
-------- -------- -------- -------- -------- -------- -------- -------- -------- -1111111 11111111 1------- -------- -------- -------- angularVelocity
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -1111111 11111111 1------- -------- cameraAngle
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -111---- -------- dribbler
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- ----111- -------- kickChipPower
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------1 -------- useAbsoluteAngle
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 1------- feedback
"""

import numpy as np
from . import REM_BaseTypes



class REM_RobotCommand:
    header = 0                # integer [0, 255]             Header byte indicating the type of packet
    remVersion = 0            # integer [0, 15]              Version of roboteam_embedded_messages
    id = 0                    # integer [0, 15]              Id of the robot
    messageId = 0             # integer [0, 15]              Id of the message
    doKick = 0                # integer [0, 1]               Do a kick if ballsensor
    doChip = 0                # integer [0, 1]               Do a chip if ballsensor
    kickAtAngle = 0           # integer [0, 1]               Do a kick once angle is reached
    doForce = 0               # integer [0, 1]               Do regardless of ballsensor
    useCameraAngle = 0        # integer [0, 1]               Use the info in 'cameraAngle'
    rho = 0                   # float   [0.000, 8.000]       Magnitude of movement (m/s)
    theta = 0                 # float   [-3.142, 3.142]      Direction of movement (radians)
    angle = 0                 # float   [-3.142, 3.142]      Absolute angle (rad)
    angularVelocity = 0       # float   [-12.566, 12.566]    Angular velocity (rad/s)
    cameraAngle = 0           # float   [-3.142, 3.142]      Angle of the robot as seen by camera (rad)
    dribbler = 0              # float   [0.000, 1.000]       Dribbler speed
    kickChipPower = 0         # float   [0.000, 6.500]       Speed of the ball in m/s
    useAbsoluteAngle = 0      # integer [0, 1]               0 = angular velocity, 1 = absolute angle
    feedback = 0              # integer [0, 1]               Ignore the packet. Just send feedback



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

    @staticmethod
    def get_remVersion(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    def get_id(payload):
        return ((payload[1] & 0b00001111));

    @staticmethod
    def get_messageId(payload):
        return ((payload[2] & 0b11110000) >> 4);

    @staticmethod
    def get_doKick(payload):
        return (payload[2] & 0b00001000) > 0;

    @staticmethod
    def get_doChip(payload):
        return (payload[2] & 0b00000100) > 0;

    @staticmethod
    def get_kickAtAngle(payload):
        return (payload[2] & 0b00000010) > 0;

    @staticmethod
    def get_doForce(payload):
        return (payload[2] & 0b00000001) > 0;

    @staticmethod
    def get_useCameraAngle(payload):
        return (payload[3] & 0b10000000) > 0;

    @staticmethod
    def get_rho(payload):
        _rho = ((payload[3] & 0b01111111) << 9) | ((payload[4]) << 1) | ((payload[5] & 0b10000000) >> 7);
        return (_rho * 0.0001220721751736) + 0.0000000000000000;

    @staticmethod
    def get_theta(payload):
        _theta = ((payload[5] & 0b01111111) << 9) | ((payload[6]) << 1) | ((payload[7] & 0b10000000) >> 7);
        return (_theta * 0.0000958752621833) + -3.1415926535897931;

    @staticmethod
    def get_angle(payload):
        _angle = ((payload[7] & 0b01111111) << 9) | ((payload[8]) << 1) | ((payload[9] & 0b10000000) >> 7);
        return (_angle * 0.0000958752621833) + -3.1415926535897931;

    @staticmethod
    def get_angularVelocity(payload):
        _angularVelocity = ((payload[9] & 0b01111111) << 9) | ((payload[10]) << 1) | ((payload[11] & 0b10000000) >> 7);
        return (_angularVelocity * 0.0003835010487330) + -12.5663706143591725;

    @staticmethod
    def get_cameraAngle(payload):
        _cameraAngle = ((payload[11] & 0b01111111) << 9) | ((payload[12]) << 1) | ((payload[13] & 0b10000000) >> 7);
        return (_cameraAngle * 0.0000958752621833) + -3.1415926535897931;

    @staticmethod
    def get_dribbler(payload):
        _dribbler = ((payload[13] & 0b01110000) >> 4);
        return (_dribbler * 0.1428571428571428) + 0.0000000000000000;

    @staticmethod
    def get_kickChipPower(payload):
        _kickChipPower = ((payload[13] & 0b00001110) >> 1);
        return (_kickChipPower * 0.9285714285714286) + 0.0000000000000000;

    @staticmethod
    def get_useAbsoluteAngle(payload):
        return (payload[13] & 0b00000001) > 0;

    @staticmethod
    def get_feedback(payload):
        return (payload[14] & 0b10000000) > 0;

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

    @staticmethod
    def set_remVersion(payload, remVersion):
        payload[1] = ((remVersion << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    def set_id(payload, id):
        payload[1] = (id & 0b00001111) | (payload[1] & 0b11110000);

    @staticmethod
    def set_messageId(payload, messageId):
        payload[2] = ((messageId << 4) & 0b11110000) | (payload[2] & 0b00001111);

    @staticmethod
    def set_doKick(payload, doKick):
        payload[2] = ((doKick << 3) & 0b00001000) | (payload[2] & 0b11110111);

    @staticmethod
    def set_doChip(payload, doChip):
        payload[2] = ((doChip << 2) & 0b00000100) | (payload[2] & 0b11111011);

    @staticmethod
    def set_kickAtAngle(payload, kickAtAngle):
        payload[2] = ((kickAtAngle << 1) & 0b00000010) | (payload[2] & 0b11111101);

    @staticmethod
    def set_doForce(payload, doForce):
        payload[2] = (doForce & 0b00000001) | (payload[2] & 0b11111110);

    @staticmethod
    def set_useCameraAngle(payload, useCameraAngle):
        payload[3] = ((useCameraAngle << 7) & 0b10000000) | (payload[3] & 0b01111111);

    @staticmethod
    def set_rho(payload, rho):
        _rho = int(rho / 0.0001220721751736);
        payload[3] = ((_rho >> 9) & 0b01111111) | (payload[3] & 0b10000000);
        payload[4] = (_rho >> 1);
        payload[5] = ((_rho << 7) & 0b10000000) | (payload[5] & 0b01111111);

    @staticmethod
    def set_theta(payload, theta):
        _theta = int((theta +3.1415926535897931) / 0.0000958752621833);
        payload[5] = ((_theta >> 9) & 0b01111111) | (payload[5] & 0b10000000);
        payload[6] = (_theta >> 1);
        payload[7] = ((_theta << 7) & 0b10000000) | (payload[7] & 0b01111111);

    @staticmethod
    def set_angle(payload, angle):
        _angle = int((angle +3.1415926535897931) / 0.0000958752621833);
        payload[7] = ((_angle >> 9) & 0b01111111) | (payload[7] & 0b10000000);
        payload[8] = (_angle >> 1);
        payload[9] = ((_angle << 7) & 0b10000000) | (payload[9] & 0b01111111);

    @staticmethod
    def set_angularVelocity(payload, angularVelocity):
        _angularVelocity = int((angularVelocity +12.5663706143591725) / 0.0003835010487330);
        payload[9] = ((_angularVelocity >> 9) & 0b01111111) | (payload[9] & 0b10000000);
        payload[10] = (_angularVelocity >> 1);
        payload[11] = ((_angularVelocity << 7) & 0b10000000) | (payload[11] & 0b01111111);

    @staticmethod
    def set_cameraAngle(payload, cameraAngle):
        _cameraAngle = int((cameraAngle +3.1415926535897931) / 0.0000958752621833);
        payload[11] = ((_cameraAngle >> 9) & 0b01111111) | (payload[11] & 0b10000000);
        payload[12] = (_cameraAngle >> 1);
        payload[13] = ((_cameraAngle << 7) & 0b10000000) | (payload[13] & 0b01111111);

    @staticmethod
    def set_dribbler(payload, dribbler):
        _dribbler = int(dribbler / 0.1428571428571428);
        payload[13] = ((_dribbler << 4) & 0b01110000) | (payload[13] & 0b10001111);

    @staticmethod
    def set_kickChipPower(payload, kickChipPower):
        _kickChipPower = int(kickChipPower / 0.9285714285714286);
        payload[13] = ((_kickChipPower << 1) & 0b00001110) | (payload[13] & 0b11110001);

    @staticmethod
    def set_useAbsoluteAngle(payload, useAbsoluteAngle):
        payload[13] = (useAbsoluteAngle & 0b00000001) | (payload[13] & 0b11111110);

    @staticmethod
    def set_feedback(payload, feedback):
        payload[14] = ((feedback << 7) & 0b10000000) | (payload[14] & 0b01111111);

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(REM_BaseTypes.PACKET_SIZE_REM_ROBOT_COMMAND, dtype=np.uint8)
        REM_RobotCommand.set_header              (payload, self.header)
        REM_RobotCommand.set_remVersion          (payload, self.remVersion)
        REM_RobotCommand.set_id                  (payload, self.id)
        REM_RobotCommand.set_messageId           (payload, self.messageId)
        REM_RobotCommand.set_doKick              (payload, self.doKick)
        REM_RobotCommand.set_doChip              (payload, self.doChip)
        REM_RobotCommand.set_kickAtAngle         (payload, self.kickAtAngle)
        REM_RobotCommand.set_doForce             (payload, self.doForce)
        REM_RobotCommand.set_useCameraAngle      (payload, self.useCameraAngle)
        REM_RobotCommand.set_rho                 (payload, self.rho)
        REM_RobotCommand.set_theta               (payload, self.theta)
        REM_RobotCommand.set_angle               (payload, self.angle)
        REM_RobotCommand.set_angularVelocity     (payload, self.angularVelocity)
        REM_RobotCommand.set_cameraAngle         (payload, self.cameraAngle)
        REM_RobotCommand.set_dribbler            (payload, self.dribbler)
        REM_RobotCommand.set_kickChipPower       (payload, self.kickChipPower)
        REM_RobotCommand.set_useAbsoluteAngle    (payload, self.useAbsoluteAngle)
        REM_RobotCommand.set_feedback            (payload, self.feedback)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = REM_RobotCommand.get_header(payload)
        self.remVersion       = REM_RobotCommand.get_remVersion(payload)
        self.id               = REM_RobotCommand.get_id(payload)
        self.messageId        = REM_RobotCommand.get_messageId(payload)
        self.doKick           = REM_RobotCommand.get_doKick(payload)
        self.doChip           = REM_RobotCommand.get_doChip(payload)
        self.kickAtAngle      = REM_RobotCommand.get_kickAtAngle(payload)
        self.doForce          = REM_RobotCommand.get_doForce(payload)
        self.useCameraAngle   = REM_RobotCommand.get_useCameraAngle(payload)
        self.rho              = REM_RobotCommand.get_rho(payload)
        self.theta            = REM_RobotCommand.get_theta(payload)
        self.angle            = REM_RobotCommand.get_angle(payload)
        self.angularVelocity  = REM_RobotCommand.get_angularVelocity(payload)
        self.cameraAngle      = REM_RobotCommand.get_cameraAngle(payload)
        self.dribbler         = REM_RobotCommand.get_dribbler(payload)
        self.kickChipPower    = REM_RobotCommand.get_kickChipPower(payload)
        self.useAbsoluteAngle = REM_RobotCommand.get_useAbsoluteAngle(payload)
        self.feedback         = REM_RobotCommand.get_feedback(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
