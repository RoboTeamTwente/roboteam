# roboteam_embedded_messages
This repository holds all definitions and implementations of messages that are sent between (currently) the robothub, the basestation, and the robot.
This repository is header-only, and is to stay that way.

### Suggested packet formats
Below are the currently implemented packet formats. These have not yet been tested
##### RobotCommand (not yet tested)
```
FIELD               BITS  RANGE     DESCRIPTION
header              :  8            Header indicating packet type
id                  :  4            Id of the robot
doKick              :  1            Do a kick if ballsensor
doChip              :  1            Do a chip if ballsensor
doForce             :  1            Do regardless of ballsensor
useCameraAngle      :  1            Use the info in 'cameraAngle'
rho                 : 16  [ 0, 8)   Magnitude of movement (m/s)
theta               : 16  [-π, π)   Direction of movement (radians)
angle               : 16  [-π, π)   Absolute angle (rad) / angular velocity (rad/s)
cameraAngle         : 16  [-π, π)   Angle of the robot as seen by camera (rad)
dribbler            :  3            Dribbler speed
kickChipPower       :  3            Power of the kick or chip
angularControl      :  1            0 = angular velocity 1 = absolute angle
feedback            :  1            Ignore the packet. Just send feedback
                    ----+    
                      88

last revision : January 26th 2021 by Emiel Steerneman
```

##### RobotFeedback (not yet tested)
```
FIELD               BITS  RANGE     DESCRIPTION
header              :  8            Header byte indicating the type of packet
id                  :  4            Id of the robot 
batteryLevel        :  4            The voltage level of the battery
XsensCalibrated     :  1            Indicates if the XSens IMU is calibrated
ballSensorWorking   :  1            Indicates if the ballsensor is working
hasBall             :  1            Indicates if the ball is somewhere in front of the ballsensor
capacitorCharged    :  1            Indicates if the capacitor for kicking and chipping is charged
ballPos             :  4  [-.5, .5] Indicates where in front of the ballsensor the ball is
rho                 : 16  [ 0, 8]   The estimated magnitude of movement (m/s)
theta               : 16  [-π, π]   The estimated direction of movement (rad)
angle               : 16  [-π, π]   The estimated angle (rad)
wheelLocked         :  4            Indicates if a wheel is locked. One bit per wheel
wheelBraking        :  4            Indicates if a wheel is slipping. One bit per wheel
rssi                :  4            Signal strength of the last packet received by the robot
                    ----+
                      84

last revision : January 26th 2021 by Emiel Steerneman
```

### Generate python bindings
The .h files come with Python bindings, so that messages can be sent using Python. To generate the Python bindings, go into the folder ```python_bindings``` and run ```setup.sh```. Then, in the same folder, run the command ```python generate.py --includes ../include/*  --name rem --output ../../rem```. This will create a folder named ```rem``` next to the folder ```roboteam_embedded_messages```. Following is a small example to send a RobotCommand via basestation to a robot.

```
import time
import serial
from rem import rem # import the roboteam_embedded_messages (rem) python bindings

port = [BASESTATION PATH HERE. Probably something like "/dev/serial/by-id/usb-RTT_BaseStation...."
connection = serial.Serial(port=port, baudrate=115200, parity=serial.PARITY_ODD, stopbits=serial.STOPBITS_TWO, bytesize=serial.SEVENBITS, timeout=0.01)

# These structs are defined in RobotCommand.h
cmd = rem.ffi.new("RobotCommand*")
cmdPayload = rem.ffi.new("RobotCommandPayload*")

# Turn on the dribbler, and send packets at around +- 60Hz
while True:
    time.sleep(0.017)

    cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
    cmd.id = 3       # Set the ID of the robot
    cmd.dribbler = 4 # Turn on the dribbler

    # The encodeRobotCommand function is defined in RobotCommand.h
    rem.lib.encodeRobotCommand(cmdPayload, cmd)
    connection.write(cmdPayload.payload)
```