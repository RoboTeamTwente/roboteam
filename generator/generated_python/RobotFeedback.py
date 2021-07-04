# AUTOGENERATED. Run generator/main.py to regenerate
# Generated on July 04 2021, 22:21:31

"""
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ] [  6   ] [  7   ] [  8   ] [  9   ] [  10  ]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- id
-------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- -------- messageId
-------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- batteryLevel
-------- -------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- XsensCalibrated
-------- -------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- ballSensorWorking
-------- -------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- hasBall
-------- -------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- capacitorCharged
-------- -------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- ballPos
-------- -------- -------- ----1111 11111111 1111---- -------- -------- -------- -------- -------- rho
-------- -------- -------- -------- -------- ----1111 11111111 1111---- -------- -------- -------- theta
-------- -------- -------- -------- -------- -------- -------- ----1111 11111111 1111---- -------- angle
-------- -------- -------- -------- -------- -------- -------- -------- -------- ----1111 -------- wheelLocked
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 1111---- wheelBraking
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- ----1111 rssi
"""




class RobotFeedback:
    header = 0                # Header byte indicating the type of packet
    id = 0                    # Id of the robot 
    messageId = 0             # Id of the message
    batteryLevel = 0          # The voltage level of the battery
    XsensCalibrated = 0       # Indicates if the XSens IMU is calibrated
    ballSensorWorking = 0     # Indicates if the ballsensor is working
    hasBall = 0               # Indicates if the ball is somewhere in front of the ballsensor
    capacitorCharged = 0      # Indicates if the capacitor for kicking and chipping is charged
    ballPos = 0               # Indicates where in front of the ballsensor the ball is
    rho = 0                   # The estimated magnitude of movement (m/s)
    theta = 0                 # The estimated direction of movement (rad)
    angle = 0                 # The estimated angle (rad)
    wheelLocked = 0           # Indicates if a wheel is locked. One bit per wheel
    wheelBraking = 0          # Indicates if a wheel is slipping. One bit per wheel
    rssi = 0                  # Signal strength of the last packet received by the robot



# ================================ GETTERS ================================
    @staticmethod
    get_header(payload):
        return ((payload[0]));

    @staticmethod
    get_id(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    get_messageId(payload):
        return ((payload[1] & 0b00001111));

# ================================ SETTERS ================================
    @staticmethod
    set_header(payload, header):
        payload[0] = header;

    @staticmethod
    set_id(payload, id):
        payload[1] = ((id << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    set_messageId(payload, messageId):
        payload[1] = (messageId & 0b00001111) | (payload[1] & 0b11110000);

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(BaseTypes.PACKET_SIZE_ROBOT_FEEDBACK, dtype=np.uint8)
        RobotFeedback.set_header              (payload, self.header)
        RobotFeedback.set_id                  (payload, self.id)
        RobotFeedback.set_messageId           (payload, self.messageId)
        RobotFeedback.set_batteryLevel        (payload, self.batteryLevel)
        RobotFeedback.set_XsensCalibrated     (payload, self.XsensCalibrated)
        RobotFeedback.set_ballSensorWorking   (payload, self.ballSensorWorking)
        RobotFeedback.set_hasBall             (payload, self.hasBall)
        RobotFeedback.set_capacitorCharged    (payload, self.capacitorCharged)
        RobotFeedback.set_ballPos             (payload, self.ballPos)
        RobotFeedback.set_rho                 (payload, self.rho)
        RobotFeedback.set_theta               (payload, self.theta)
        RobotFeedback.set_angle               (payload, self.angle)
        RobotFeedback.set_wheelLocked         (payload, self.wheelLocked)
        RobotFeedback.set_wheelBraking        (payload, self.wheelBraking)
        RobotFeedback.set_rssi                (payload, self.rssi)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = RobotFeedback.get_header(payload)
        self.id               = RobotFeedback.get_id(payload)
        self.messageId        = RobotFeedback.get_messageId(payload)
        self.batteryLevel     = RobotFeedback.get_batteryLevel(payload)
        self.XsensCalibrated  = RobotFeedback.get_XsensCalibrated(payload)
        self.ballSensorWorking= RobotFeedback.get_ballSensorWorking(payload)
        self.hasBall          = RobotFeedback.get_hasBall(payload)
        self.capacitorCharged = RobotFeedback.get_capacitorCharged(payload)
        self.ballPos          = RobotFeedback.get_ballPos(payload)
        self.rho              = RobotFeedback.get_rho(payload)
        self.theta            = RobotFeedback.get_theta(payload)
        self.angle            = RobotFeedback.get_angle(payload)
        self.wheelLocked      = RobotFeedback.get_wheelLocked(payload)
        self.wheelBraking     = RobotFeedback.get_wheelBraking(payload)
        self.rssi             = RobotFeedback.get_rssi(payload)


    def printBitString(self):        payload = self.encode()        for i in range(len(payload)):            print(format(payload[i], '08b'), end=" ")        print()