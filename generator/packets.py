import math

packets = {
    "REM_RobotCommand" : [
        ["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["id",                  4,  None, "Id of the robot"],
        ["messageId",           4,  None, "Id of the message"],
        ["doKick",              1,  None, "Do a kick if ballsensor"],
        ["doChip",              1,  None, "Do a chip if ballsensor"],
        ["kickAtAngle",         1,  None, "Do a kick once angle is reached"],
        ["doForce",             1,  None, "Do regardless of ballsensor"],
        ["useCameraAngle",      1,  None, "Use the info in 'cameraAngle'"],
        ["rho",                16, [0, 8], "Magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi], "Direction of movement (radians)"],
        ["angle",              16, [-math.pi, math.pi], "Absolute angle (rad)"],
        ["angularVelocity",    16, [-4*math.pi, 4*math.pi], "Angular velocity (rad/s)"],
        ["cameraAngle",        16, [-math.pi, math.pi], "Angle of the robot as seen by camera (rad)"],
        ["dribbler",            3,  [0, 1], "Dribbler speed"],
        ["kickChipPower",       4,  [0, 6.5], "Speed of the ball in m/s"],
        ["useAbsoluteAngle",    1,  None, "0 = angular velocity, 1 = absolute angle"],
        ["feedback",            1,  None, "Ignore the packet. Just send feedback"],
    ],
    "REM_RobotFeedback" : [
        ["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["id",                  4,  None, "Id of the robot "],
        ["messageId",           4,  None, "Id of the message"],
        ["batteryLevel",        4,  None, "The voltage level of the battery"],
        ["XsensCalibrated",     1,  None, "Indicates if the XSens IMU is calibrated"],
        ["ballSensorWorking",   1,  None, "Indicates if the ballsensor is working"],
        ["ballSensorSeesBall",  1,  None, "Indicates if the ballsensor sees the ball"],
        ["dribblerSeesBall",    1,  None, "Indicates if the dribbler sees the ball"],
        ["capacitorCharged",    1,  None, "Indicates if the capacitor for kicking and chipping is charged"],
        ["ballPos",             4,  [-0.5, 0.5],  "Indicates where in front of the ballsensor the ball is"],
        ["rho",                16, [0, 8],                 "The estimated magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi],    "The estimated direction of movement (rad)"],
        ["angle",              16, [-math.pi, math.pi],    "The estimated angle (rad)"],
        ["wheelLocked",         4,  None, "Indicates if a wheel is locked. One bit per wheel"],
        ["wheelBraking",        4,  None, "Indicates if a wheel is slipping. One bit per wheel"],
        ["rssi",                8,  None, "Signal strength of the last packet received by the robot"]
    ],
    "REM_RobotStateInfo" : [
        ["header",              	8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          	4,  None, "Version of roboteam_embedded_messages"],
        ["id",                 	4,  None, "Id of the robot "],
        ["messageId",          	4,  None, "Id of the message"],
        ["xsensAcc1",         	32, [-50000., 50000.], "xsensAcc1"],
        ["xsensAcc2",         	32, [-50000., 50000.], "xsensAcc2"],
        ["xsensYaw",          	32, [-50000., 50000.], "xsensYaw"],
        ["rateOfTurn",        	16, [-20., 20.], "rateOfTurn"],
        ["wheelSpeed1",       	16, [-1000., 1000.], "wheelSpeed1"],
        ["wheelSpeed2",       	16, [-1000., 1000.], "wheelSpeed2"],
        ["wheelSpeed3",        	16, [-1000., 1000.], "wheelSpeed3"],
        ["wheelSpeed4",        	16, [-1000., 1000.], "wheelSpeed4"],
        ["dribbleSpeed",       	16, [0.     , 5000.], "dribblerSpeed"],
        ["filteredDribbleSpeed",	16, [0.     , 5000.], "filtered dribblerSpeed"],
        ["dribblespeedBeforeGotBall",	16, [0.     , 5000.], "dribblerSpeed at at the time dribbler thinks it got the ball"],
        ["bodyXIntegral",      	16, [-5000., 5000.], "Integral value from the PID for body_x"],
        ["bodyYIntegral", 		16, [-5000., 5000.], "Integral value from the PID for body_y"],
        ["bodyWIntegral", 		16, [-5000., 5000.], "Integral value from the PID for body_w"],
        ["bodyYawIntegral",		16, [-5000., 5000.], "Integral value from the PID for body_Yaw"],
        ["wheel1Integral",     	16, [-5000., 5000.], "Integral value from the PID for Wheel_1"],
        ["wheel2Integral",    	16, [-5000., 5000.], "Integral value from the PID for Wheel_2"],
        ["wheel3Integral",   		16, [-5000., 5000.], "Integral value from the PID for Wheel_3"],
        ["wheel4Integral",  		16, [-5000., 5000.], "Integral value from the PID for Wheel_4"]
    ],
    "REM_RobotBuzzer" : [
    	["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["id",                  4,  None, "Id of the robot"],
        ["messageId",           4,  None, "Id of the message"],
    	["period",             12, None, "Sound that the buzzer makes."],
    	["duration",           16, [0., 5.], "Duration of the sound"]
    ],
    "REM_BasestationLog" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["messageLength",       8, None, "Length of the following message"]
    ],
    "REM_RobotLog" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["id",                  4, None, "Id of the robot"],
        ["messageLength",       8, None, "Length of the following message"]
    ],
    "REM_BasestationGetConfiguration" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
    ],
    "REM_BasestationConfiguration" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["channel",             1, None, "Channel on which the basestation and robots communicate"]
    ],
    "REM_BasestationSetConfiguration" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["channel",             1, None, "Channel on which the basestation and robots communicate"]
    ],
    "REM_RobotGetPIDGains" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["id",                  4, None, "Id of the robot"]
    ],
    "REM_RobotPIDGains" : [
        ["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["id",                  4,  None, "Id of the robot"],
        ["PbodyX",             16, [0.,40.], "Received P gain of the PID for body_x (x-direction)"],
        ["IbodyX",             16, [0.,20.], "Received I gain of the PID for body_x (x-direction)"],
        ["DbodyX",             16, [0.,10.], "Received D gain of the PID for body_x (x-direction)"],
        ["PbodyY",             16, [0.,40.], "Received P gain of the PID for body_y (y-direction)"],
        ["IbodyY",             16, [0.,20.], "Received I gain of the PID for body_y (y-direction)"],
        ["DbodyY",             16, [0.,10.], "Received D gain of the PID for body_y (y-direction)"],
        ["PbodyW",             16, [0.,40.], "Received P gain of the PID for body_w (Angular velocity)"],
        ["IbodyW",             16, [0.,20.], "Received I gain of the PID for body_w (Angular velocity)"],
        ["DbodyW",             16, [0.,10.], "Received D gain of the PID for body_w (Angular velocity)"],
        ["PbodyYaw",           16, [0.,40.], "Received P gain of the PID for body_yaw (Absolute angle)"],
        ["IbodyYaw",           16, [0.,20.], "Received I gain of the PID for body_yaw (Absolute angle)"],
        ["DbodyYaw",           16, [0.,10.], "Received D gain of the PID for body_yaw (Absolute angle)"],
        ["Pwheels",            16, [0.,40.], "Received P gain of the PID for the wheels"],
        ["Iwheels",            16, [0.,20.], "Received I gain of the PID for the wheels"],
        ["Dwheels",            16, [0.,10.], "Received D gain of the PID for the wheels"]
    ],
    "REM_RobotSetPIDGains" : [
        ["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["id",                  4,  None, "Id of the robot"],
        ["PbodyX",             16, [0.,40.], "Commanded P gain of the PID for body_x (x-direction)"],
        ["IbodyX",             16, [0.,20.], "Commanded I gain of the PID for body_x (x-direction)"],
        ["DbodyX",             16, [0.,10.], "Commanded D gain of the PID for body_x (x-direction)"],
        ["PbodyY",             16, [0.,40.], "Commanded P gain of the PID for body_y (y-direction)"],
        ["IbodyY",             16, [0.,20.], "Commanded I gain of the PID for body_y (y-direction)"],
        ["DbodyY",             16, [0.,10.], "Commanded D gain of the PID for body_y (y-direction)"],
        ["PbodyW",             16, [0.,40.], "Commanded P gain of the PID for body_w (Angular velocity)"],
        ["IbodyW",             16, [0.,20.], "Commanded I gain of the PID for body_w (Angular velocity)"],
        ["DbodyW",             16, [0.,10.], "Commanded D gain of the PID for body_w (Angular velocity)"],
        ["PbodyYaw",           16, [0.,40.], "Commanded P gain of the PID for body_yaw (Absolute angle)"],
        ["IbodyYaw",           16, [0.,20.], "Commanded I gain of the PID for body_yaw (Absolute angle)"],
        ["DbodyYaw",           16, [0.,10.], "Commanded D gain of the PID for body_yaw (Absolute angle)"],
        ["Pwheels",            16, [0.,40.], "Commanded P gain of the PID for the wheels"],
        ["Iwheels",            16, [0.,20.], "Commanded I gain of the PID for the wheels"],
        ["Dwheels",            16, [0.,10.], "Commanded D gain of the PID for the wheels"]
    ],
    "REM_RobotAssuredPacket" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["id",                  4, None, "Id of the robot"],
        ["sequenceNumber",      8, None, "Number to match this packet with AssuredAck"],
        ["messageLength",       8, None, "Length of the following message"]
    ],
    "REM_RobotAssuredAck" : [
        ["header",              8, None, "Header byte indicating the type of packet"],
        ["remVersion",          4, None, "Version of roboteam_embedded_messages"],
        ["id",                  4, None, "Id of the robot"],
        ["sequenceNumber",      8, None, "Number to match this packet with AssuredPacket"],
    ],
    # https://media.digikey.com/pdf/Data%20Sheets/Semtech%20PDFs/SX1280-81_Rev3.2_Mar2020.pdf see page 124. 
    # Minimum payload length is 6 bytes
    "REM_SX1280Filler" : [
        ["header",              8,  None, "Header byte indicating the type of packet"],
        ["remVersion",          4,  None, "Version of roboteam_embedded_messages"],
        ["fillerBits",         36,  None, "SX1280 requires a minimum of 6 bytes payload. See documentation page 124."]
    ]

}
