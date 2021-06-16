import sys
import math
import time
import signal
from xbox360controller import Xbox360Controller
import utils
import serial 
import numpy as np

# try:
#     from rem import rem
# except ImportError:
#     print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
#     print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
#     print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
#     exit()


last_written = time.time()
packet_Hz = 60
basestation = None

# buzzer = rem.ffi.new("RobotBuzzer*")
# buzzerPayload = rem.ffi.new("RobotBuzzerPayload*")

# command = rem.ffi.new("RobotCommand*")
# commandPayload = rem.ffi.new("RobotCommandPayload*")

controller = Xbox360Controller(0)

robot_id = 0
absolute_angle = 0

class ButtonState:
    def __init__(self):
        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.HAT_X = 0
        self.HAT_Y = 0

button_state = ButtonState()


while True:
    # Open basestation with the basestation
    if basestation is None or not basestation.isOpen():
        basestation = utils.openContinuous(timeout=0.001)

    try:
        while True:
            # Run at {packet_Hz}fps
            if 1./packet_Hz <= time.time() - last_written:
                # Timing stuff
                last_written += 1./packet_Hz

                if controller.button_mode._value:
                    print("Exiting!")
                    basestation = None
                    controller = None
                    sys.exit()


                # buzzer.header = rem.lib.PACKET_TYPE_ROBOT_BUZZER
                # buzzer.id = robot_id

                # buzzer.period = 570 + int(350 * controller.axis_l.x)
                # buzzer.duration = 1
                # rem.lib.encodeRobotBuzzer(buzzerPayload, buzzer)
                # basestation.write(buzzerPayload.payload)

                # print(robot_id, controller.hat.x, controller.hat.y)
                if button_state.HAT_X != controller.hat.x:
                    button_state.HAT_X = controller.hat.x
                    robot_id += controller.hat.x
                    print(f"Switched to ID {robot_id}")

                if robot_id < 0: robot_id = 0
                if 15 < robot_id : robot_id = 15

                # command.id = robot_id
                # command.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND

                # Angle
                if 0.3 < abs(controller.axis_r.x): absolute_angle -= controller.axis_r.x * 0.1
                
                # Forward backward left right
                y = controller.axis_l.y if 0.3 < abs(controller.axis_l.y) else 0 
                x = controller.axis_l.x if 0.3 < abs(controller.axis_l.x) else 0
                rho = math.sqrt(x * x + y * y);
                theta = math.atan2(-x, -y);
                
                # command.rho = rho
                # command.theta = theta + absolute_angle
                # command.angle = absolute_angle

                # rem.lib.encodeRobotCommand(commandPayload, command)
                # basestation.write(commandPayload.payload)




                robotcommand = np.zeros(11, dtype=np.uint8)
                robotcommand[0] = 15 # 15 = RobotCommand

                robotcommand[1] = ((robot_id << 4) & 0b11110000) | (robotcommand[1] & 0b00001111);

                _rho = int((rho - 0.0000000000000000) * 8192.0000000000000000)
                robotcommand[2] = (_rho >> 8);
                robotcommand[3] = _rho;

                _theta = int((theta + absolute_angle - -3.1415926535897931) * 10430.3783504704533698)
                robotcommand[4] = (_theta >> 8);
                robotcommand[5] = _theta;

                _absolute_angle = int((absolute_angle - -3.1415926535897931) * 10430.3783504704533698)
                robotcommand[6] = (_absolute_angle >> 8);
                robotcommand[7] = _absolute_angle;

                cameraAngle = int((0 - -3.1415926535897931) * 10430.3783504704533698)
                robotcommand[8] = (cameraAngle >> 8);
                robotcommand[9] = cameraAngle;


                myBytes = bytearray(robotcommand)
                # print("\n\n")
                # for i in range(11):
                #     print(commandPayload.payload[i], myBytes[i])

                basestation.write(myBytes)


                time.sleep(0.005)
        signal.pause()

    except serial.SerialException as se:
        print("SerialException", se)
        basestation = None
    except serial.SerialTimeoutException as ste:
        print("SerialTimeoutException", ste)
    except KeyError:
        print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
    except Exception as e:
        print("[Error]", e)

print("Exiting")