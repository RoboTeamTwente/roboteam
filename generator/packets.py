import math

""" This generic header will autmatically be added to every packet defined below. """
generic_packet_header = [
    ["header",      8,      None, "Header byte indicating the type of packet"],
    # Destination
    ["toRobotId",   4,      None, "Id of the receiving robot"],
    ["toColor",     1,      None, "Color of the receiving robot / basestation. Yellow = 0, Blue = 1"],
    ["toBC",        1,      None, "Bit indicating this packet has to be broadcasted to all robots"],
    ["toBS",        1,      None, "Bit indicating this packet is meant for the basestation"],
    ["toPC",        1,      None, "Bit indicating this packet is meant for the PC"],
    # Source
    ["fromRobotId", 4,      None, "Id of the transmitting robot"],
    ["fromColor",   1,      None, "Color of the transmitting robot / basestation. Yellow = 0, Blue = 1"],
    ["reserved",    1,      None, "reserved"],
    ["fromBS",      1,      None, "Bit indicating this packet is coming from the basestation"],
    ["fromPC",      1,      None, "Bit indicating this packet is coming from the PC"],

    ["remVersion",  4,      None, "Version of roboteam_embedded_messages"],
    ["messageId",   4,      None, "messageId. Can be used for aligning packets"],
    ["payloadSize", 8,      None, "Size of the payload. At most 255 bytes including the generic_packet_header. Keep the 127 byte SX1280 limit in mind"]
]

packets = {
    "REM_Packet" : [],
    "REM_RobotCommand" : [
        # Movement
        ["rho",                16, [0, 8], "Magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi], "Direction of movement (radians)"],
        ["angle",              16, [-math.pi, math.pi], "Absolute angle (rad)"],
        ["angularVelocity",    16, [-4*math.pi, 4*math.pi], "Angular velocity (rad/s)"],
        ["cameraAngle",        16, [-math.pi, math.pi], "Angle of the robot as seen by camera (rad)"],
        ["useCameraAngle",      1,  None, "Use the info in 'cameraAngle'"],
        ["useAbsoluteAngle",    1,  None, "0 = angular velocity, 1 = absolute angle"],
        # Dribbler
        ["dribbler",            3,  [0, 1], "Dribbler speed"],
        # Kicker / Chipper
        ["doKick",              1,  None, "Do a kick if ballsensor"],
        ["doChip",              1,  None, "Do a chip if ballsensor"],
        ["kickAtAngle",         1,  None, "Do a kick once angle is reached"],
        ["kickChipPower",       4,  [0, 6.5], "Speed of the ball in m/s"],
        ["doForce",             1,  None, "Do regardless of ballsensor"],
        ["feedback",            1,  None, "Ignore the packet. Just send feedback"],
    ],
    "REM_RobotFeedback" : [
        # Movement
        ["rho",                16, [0, 8],                 "The estimated magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi],    "The estimated direction of movement (rad)"],
        ["angle",              16, [-math.pi, math.pi],    "The estimated angle (rad)"],
        
        ["batteryLevel",        4,  None, "The voltage level of the battery"],
        ["XsensCalibrated",     1,  None, "Indicates if the XSens IMU is calibrated"],
        ["capacitorCharged",    1,  None, "Indicates if the capacitor for kicking and chipping is charged"],
        # Ball handling
        ["ballSensorWorking",   1,  None, "Indicates if the ballsensor is working"],
        ["ballSensorSeesBall",  1,  None, "Indicates if the ballsensor sees the ball"],
        ["ballPos",             4,  [-0.5, 0.5],  "Indicates where in front of the ballsensor the ball is"],
        ["dribblerSeesBall",    1,  None, "Indicates if the dribbler sees the ball"],
        ["reserved1",           3,  None, "reserved1"],

        ["wheelLocked",         4,  None, "Indicates if a wheel is locked. One bit per wheel"],
        ["wheelBraking",        4,  None, "Indicates if a wheel is slipping. One bit per wheel"],
        ["rssi",                8,  None, "Signal strength of the last packet received by the robot"]
    
    ],
    "REM_RobotStateInfo" : [
        ["xsensAcc1",          16, [-16 * 9.81, 16 * 9.81], "xsensAcc1"],
        ["xsensAcc2",          16, [-16 * 9.81, 16 * 9.81], "xsensAcc2"],
        ["xsensYaw",           32, [-50000., 50000.], "xsensYaw"],
        ["rateOfTurn",         16, [-20., 20.], "rateOfTurn"],
        ["wheelSpeed1",        16, [-1000., 1000.], "wheelSpeed1"],
        ["wheelSpeed2",        16, [-1000., 1000.], "wheelSpeed2"],
        ["wheelSpeed3",        16, [-1000., 1000.], "wheelSpeed3"],
        ["wheelSpeed4",        16, [-1000., 1000.], "wheelSpeed4"],
        ["dribbleSpeed",       16, [0.     , 5000.], "dribblerSpeed"],
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
    	["period",             12, None, "Sound that the buzzer makes."],
    	["duration",           16, [0., 5.], "Duration of the sound"]
    ],
    "REM_BasestationLog"              : [],
    "REM_RobotLog"                    : [],
    "REM_BasestationGetConfiguration" : [],
    "REM_BasestationConfiguration"    : [
        ["channel",             1, None, "Channel on which the basestation and robots communicate"]
    ],
    "REM_RobotGetPIDGains"            : [],
    "REM_RobotPIDGains" : [
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
        ["sequenceNumber",      8, None, "Number to match this packet with AssuredAck"],
        ["messageLength",       8, None, "Length of the following message"]
    ],
    "REM_RobotAssuredAck" : [
        ["sequenceNumber",      8, None, "Number to match this packet with AssuredPacket"],
    ],
    
    "REM_RobotMusicCommand" : [
        ["play",                1,  None, "Set to play the current song"],
        ["pause",               1,  None, "Set to pause the current song"],
        ["stop",                1,  None, "Set to stop the current song"],

        ["previousSong",        1,  None, "Set to stop the current song"],        
        ["nextSong",            1,  None, "Set to stop the current song"],        

        ["volume",              5,  None, "Set the volume. Value between 1 and 31. 0 is ignored"],
        ["volumeUp",            1,  None, "Set to increase the volume"],
        ["volumeDown",          1,  None, "Set to decrease the volume"],
        
        ["folderId",            4,  None, "The id of the folder, from which to pick a song"],
        ["songId",              8,  None, "Id of the song, given the folder"],
    ],
    # https://media.digikey.com/pdf/Data%20Sheets/Semtech%20PDFs/SX1280-81_Rev3.2_Mar2020.pdf see page 124. 
    # Minimum payload length is 6 bytes
    "REM_SX1280Filler" : [
        ["fillerBits",         36,  None, "SX1280 requires a minimum of 6 bytes payload. See documentation page 124."]
    ]

}

# For each packet, insert 1 header byte to ensure that it's present
# header_field = ["header", 8, None, "Header byte indicating the type of packet"]
# for packet_name in packets:
#     packets[packet_name] = [header_field] + packets[packet_name]

# For each packet, add the generic_header_packet
for packet_name in packets:
    packets[packet_name] = generic_packet_header + packets[packet_name]


















