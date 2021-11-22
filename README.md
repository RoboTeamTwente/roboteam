# roboteam_embedded_messages
This repository holds all definitions and implementations of messages that are sent between (currently) the robothub, the basestation, and the robot.
This repository is header-only, and is to stay that way.

## Generating packets
To generate the code for the packets, run the file `generator/main.py`. A version number `remVersion` is incremented each time `main.py` is run. It resets back to 0 after it hit 15. This versions is embedded into the code, in the variable `LOCAL_REM_VERSION` (look in `BaseTypes.h` and `BaseTypes.py`). The current version is stored in the file `latest_rem_version`. Versioning has been added to be able to check that all robots / basestations / scripts run the same version of REM. The REM version is added to most packet, via the `remVersion` variable. To generate a specific version, run `main.py --version <your version number here>`. An example; If the basestation receives a packet from a robot which contains `remVersion = 5`, but its `LOCAL_REM_VERSION = 6`, then you know that the robot runs an outdated REM version. 


## Defining a new packet
All packet definitions can be found in the file `generator/packets.py`. New packets can be created by adding a definition of the packet to the `packets` dictionary. A packet defintion is a list of lists, where each inner list defines a single variable. For example, a small section of the `RobotCommand`:
```
"RobotCommand" : [
    ["header",             8,  None, "Header byte indicating the type of packet"],
    ["remVersion",         4,  None, "Version of roboteam_embedded_messages"],
    ["id",                 4,  None, "Id of the robot"],
    ["rho",                16, [0, 8], "Magnitude of movement (m/s)"],
    ["theta",              16, [-math.pi, math.pi], "Direction of movement (radians)"]
]
```
Each variable consists of four elements;
* The first element (`header`, `remVersion`, `id`) indicates the name of the variable. 
* The second element (`8`, `4`, `16`) indicates the number of bits to be allocated for this variable. An arbitraty number of bits can be allocated, such as 7, 23, 123, or any other number.
* The third element indicates the value range the variable can take. This is only used for floating point variables. For example, the variable `rho` can take any value between 0 and 8. The generated code automatically transforms a floating point variable to an integer and back.
* The fourth element is the description of the variable.

A packet must always start with a `header` variable! This variable is used to indicate the type of the packet. For example, `RobotCommand` has header `15` and `RobotFeedback` has header `51`.

### Floating point variables
Floating point variables cannot directly be sent between two systems (e.g. robot and computer), because their implementations might differ. Instead, a float is first converted to an integer, sent to the other system, and then converted back to a float. This of course leads to a loss of precision, which depends on the number of bits that are reserved for the float. For example, take a floating point variable that can take a value between 0 and 1. 4 bits are reserved for the variable, resulting in a total of 2^4 = 16 different values. 
```
FLOAT IN   0   .1  .2  .3  .4  .5  .6  .7  .8  .9  1 
INT        0   2   3   5   6   8   9   11  12  14  15
FLOAT OUT  0  .13 .2  .33  .4  .53 .6  .73 .8  .93 1 
```

## Python example
```
import roboteam_embedded_messages.python.BaseTypes as BaseTypes
from roboteam_embedded_messages.python.RobotCommand import RobotCommand

# Send to robot
robotCommand = RobotCommand()
robotCommand.header = BaseTypes.PACKET_TYPE_ROBOT_COMMAND
robotCommand.remVersion = BaseTypes.LOCAL_REM_VERSION
robotCommand.id = 10
send_to_robot(robotCommand.encode())

# Receive from robot
robotFeedback = RobotFeedback()
robotFeedback.decode(<list of bytes with>)
print(robotFeedback.robotId)

if robotFeedback.remVersion != BaseTypes.LOCAL_REM_VERSION:
    print("REM versions are different")
```

## C example
todo



