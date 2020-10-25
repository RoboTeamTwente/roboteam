# roboteam_embedded_messages
This repository holds all definitions and implementations of messages that are sent between (currently) the robothub, the basestation, and the robot.
This repository is header-only, and is to stay that way.

### Suggested packet formats
Below are the currently implemented packet formats. These have not yet been tested
##### RobotCommand (not yet tested)
```
header          :  8 bits  Header byte indicating the type of packet
id              :  4 bits  Id of the robot 
K C F Cam       :  4 bits  (Kick / Chip / Forced / Use camera rotation)
rho             : 16 bits  direction of movement
theta           : 16 bits  magnitude of movement (speed) 
angle           : 16 bits  absolute ? direction to face; relative ? angular velocity
camera_rotation : 16 bits  angle of the robot as seen by the cameras
dribbler        :  3 bits  dribbler speed
kick_chip_power :  3 bits  power of the kick or chip
angle_absolute  :  1 bits  indicates either absolute angle or angular velocity
feedback only   :  1 bits  indicates that the packet may be ignored. Do send feedback though
                ---------+	
                88 bits

last revision : Oktober 22nd 2020 by Emiel Steerneman
```

##### RobotFeedback (not yet tested)
```
header              :  8 bits  Header byte indicating the type of packet
id                  :  4 bits  Id of the robot 
battery_level       :  4 bits  The voltage level of the battery
xsens_calibrated    :  1 bits  Indicates if the XSens IMU is calibrated
ballsensor_working  :  1 bits  Indicates if the ballsensor is working
has_ball            :  1 bits  Indicates if the ball is somewhere in front of the ballsensor
capacitor_charged   :  1 bits  Indicates if the capacitor for kicking and chipping is charged
ball_position       :  4 bits  Indicates where in front of the ballsensor the ball is
rho                 : 16 bits  The estimated direction of movement
theta               : 16 bits  The estimated magnitude of movement (speed)
angle               : 16 bits  The estimated angle
wheel_locked        :  4 bits  Indicates if a wheel is locked. One bit per wheel
wheel_slipping      :  4 bits  Indicates if a wheel is slipping. One bit per wheel
rssi                :  4 bits  Signal strength of the last packet received by the robot
                    ---------+
                      84 bits

last revision : Oktober 22nd 2020 by Emiel Steerneman
```