# roboteam_embedded_messages (REM)
This repository holds all definitions and implementations of messages that are sent between (currently) the robothub, the basestation, and the robot.
This repository is header-only, and is to stay that way.

## Generating packets
To generate the code for the packets, run the file `main.py` in the folder `./generator`. Files are generated for each packet, in the languages C, Python, and Protobuf. These are placed in the corresponding folders ./include, ./python, and ./proto. When `main.py` is executed, the version number `REM_LOCAL_VERSION` will be incremented by 1. To generate with a specific version, use the `version` flag. For example: `python main.py --version 1`. The version goes up to at most 15, and will loop around back to 0. For more information regarding packet generation, read the README in the `./generator` folder.

### Packet instances and payloads
Each packet has two representations. The first representation is an instance of that packet, which is a Struct in C and a class instance in Python. While these are simple to use, they can't be sent over a wire. The second representation is compressed / encoded, which is simply an array of bytes. 


## Defining a new packet
All packet definitions can be found in the file `generator/packets.py`. New packets can be created by adding a definition of the packet to the `packets` dictionary. 

### Packet structure
A single definition packet is a 2D list, where each inner list defines a single field of the packet. For example, a cropped version of the `REM_RobotCommand`:
```
"REM_RobotCommand" : [
    # Movement
    ["rho",                16, [0, 8], "Magnitude of movement (m/s)"],
    ["theta",              16, [-math.pi, math.pi], "Direction of movement (radians)"],
    # Dribbler
    ["dribbler",            3,  [0, 1], "Dribbler speed"],
    # Kicker / Chipper
    ["doKick",              1,  None, "Do a kick if ballsensor"],
    ["kickChipPower",       4,  [0, 6.5], "Speed of the ball in m/s"],
    ["doForce",             1,  None, "Do regardless of ballsensor"]
],
```
Each variable consists of the four elements [NAME, N BITS, RANGE, DESCRIPTION];
1. The first element (e.g. `rho`, `theta`) indicates the name of the variable. 
2. The second element (`3`, `4`, `16`) indicates the number of bits to be allocated for this variable. An arbitraty number of bits can be allocated, such as 7, 23, 123, or any other number.
3. The third element indicates the value range the variable can take. This is only used for floating point variables. For example, the variable `rho` can take any value between 0 and 8. The generated code automatically transforms a floating point variable to an integer and back.
4. The fourth element is the description of the variable.

A packet must always start with a `header` variable! This variable is used to indicate the type of the packet. For example, `RobotCommand` has header `15` and `RobotFeedback` has header `51`.

### Default fields
Some fields will automatically be added to each packet definition during generation. These fields can be found in the variable `generic_packet_header` in `packets.py`. These fields contain important information regarding packet type, routing, payload size, and REM version. It's the REM equivalent of a TCP packet header. Each packet is required to have these fields (with the exception of the REM_SX1280Filler packet) to ensure that basestation code and logging code works properly. 

### Design patterns
All packet names are prepended with `REM_`. All fields, both packet names and variable names, are in camelCase. The name `BaseTypes` is reserved and can not be used as a packet name.

## Floating point variables
Floating point variables cannot directly be sent between two systems (e.g. robot and computer), because their implementations might differ. Instead, a float is first converted to an integer, sent to the other system, and then converted back to a float. This of course leads to a loss of precision, which depends on the number of bits that are reserved for the float. For example, take a floating point variable that can take a value between 0 and 1. 4 bits are reserved for the variable, resulting in a total of 2^4 = 16 different values. 
```
FLOAT IN   0   .1  .2  .3  .4  .5  .6  .7  .8  .9  1 
INT        0   2   3   5   6   8   9   11  12  14  15
FLOAT OUT  0  .13 .2  .33  .4  .53 .6  .73 .8  .93 1 
```

## Python example
```
# It is assumed that this code runs next to the roboteam_embedded_messages folder, not within it
from roboteam_embedded_messages.python import REM_BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand  import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback

cmd = REM_RobotCommand()
# Set the required fields
cmd.header = REM_BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
cmd.toRobotId = 7
cmd.fromPC = 1
cmd.remVersion = REM_BaseTypes.REM_LOCAL_VERSION
cmd.payloadSize = REM_BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND
# Set the fields to trigger the kicker
cmd.doKick = True
cmd.kickChipPower = 0.5
cmd.doForce = True

# Convert the command to bytes
cmd_in_bytes = cmd.encode()

### Send the packet to the robot somehow, such as by using the Pyserial library
### Receive bytes back from the robot representing a REM_RobotFeedback packet

feedback_in_bytes = receive_something_somehow()
fb = REM_RobotFeedback()
fb.decode(feedback_in_bytes)
print("Received feedback packet from robot", fb.fromRobotId)
if fb.remVersion != REM_BaseTypes.REM_LOCAL_VERSION:
	print("Warning! The REM versions do not match. Somewhere something requires an update")
```



