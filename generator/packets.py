import math

packets = {
    "RobotCommand" : [
        ["header",             8,  None, "Header indicating packet type"],
        ["id",                 4,  None, "Id of the robot"],
        ["doKick",             1,  None, "Do a kick if ballsensor"],
        ["doChip",             1,  None, "Do a chip if ballsensor"],
        ["doForce",            1,  None, "Do regardless of ballsensor"],
        ["useCameraAngle",     1,  None, "Use the info in 'cameraAngle'"],
        ["rho",                16, [0, 8], "Magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi], "Direction of movement (radians)"],
        ["angle",              16, [-math.pi, math.pi], "Absolute angle (rad) / angular velocity (rad/s)"],
        ["cameraAngle",        16, [-math.pi, math.pi], "Angle of the robot as seen by camera (rad)"],
        ["dribbler",           3,  None, "Dribbler speed"],
        ["kickChipPower",      3,  None, "Power of the kick or chip"],
        ["angularControl",     1,  None, "0 = angular velocity, 1 = absolute angle"],
        ["feedback",           1,  None, "Ignore the packet. Just send feedback"],
    ],
    "RobotFeedback" : [
        ["header",             8,  None, "Header byte indicating the type of packet"],
        ["id",                 4,  None, "Id of the robot "],
        ["batteryLevel",       4,  None, "The voltage level of the battery"],
        ["XsensCalibrated",    1,  None, "Indicates if the XSens IMU is calibrated"],
        ["ballSensorWorking",  1,  None, "Indicates if the ballsensor is working"],
        ["hasBall",            1,  None, "Indicates if the ball is somewhere in front of the ballsensor"],
        ["capacitorCharged",   1,  None, "Indicates if the capacitor for kicking and chipping is charged"],
        ["ballPos",            4,  [-0.5, 0.5],  "Indicates where in front of the ballsensor the ball is"],
        ["rho",                16, [0, 8],                 "The estimated magnitude of movement (m/s)"],
        ["theta",              16, [-math.pi, math.pi],    "The estimated direction of movement (rad)"],
        ["angle",              16, [-math.pi, math.pi],    "The estimated angle (rad)"],
        ["wheelLocked",        4,  None, "Indicates if a wheel is locked. One bit per wheel"],
        ["wheelBraking",       4,  None, "Indicates if a wheel is slipping. One bit per wheel"],
        ["rssi",               4,  None, "Signal strength of the last packet received by the robot"]
    ]
}    