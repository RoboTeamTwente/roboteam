import sys
import math
import time
import signal
from xbox360controller import Xbox360Controller
import utils
import serial 
import numpy as np

try:
    from rem import rem
except ImportError:
    print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
    print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
    print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
    exit()
buzzer = rem.ffi.new("RobotBuzzer*")
buzzerPayload = rem.ffi.new("RobotBuzzerPayload*")
command = rem.ffi.new("RobotCommand*")
commandPayload = rem.ffi.new("RobotCommandPayload*")


last_written = time.time()
packet_Hz = 60
basestation = None

robot_id = 15
absolute_angle = 0

class JoystickWrapper:
    def __init__(self, controller):
        self.controller = controller
        self.robot_id = 0
        self.absolute_angle = 0

        self.A = False
        self.B = False
        self.X = False
        self.Y = False
        self.HAT_X = 0
        self.HAT_Y = 0
        self.command = rem.ffi.new("RobotCommand*")
        self.payload = rem.ffi.new("RobotCommandPayload*")

    def get_payload(self):

        if self.controller.button_mode._value:
            print("Exiting!")
            sys.exit()

        if self.HAT_X != self.controller.hat.x:
            self.HAT_X = self.controller.hat.x
            self.robot_id += self.controller.hat.x
            print(f"Switched to ID {self.robot_id}")

        # Toggle dribbler
        self.command.dribbler = 0
        if self.controller.button_y._value and not self.Y:
            self.Y = True
            if self.command.dribbler == 0: 
                self.command.dribbler = 7
            else:
                self.command.dribbler = 0
        else:
            self.Y = False

        if self.robot_id < 0: self.robot_id = 0
        if 15 < self.robot_id : self.robot_id = 15

        self.command.doKick = False
        self.command.doChip = False

        if self.controller.button_a._value and not self.A:
            self.command.kickChipPower = 5
            self.command.doChip = True
            self.command.doForce = True
        self.A = self.controller.button_a._value

        if self.controller.button_b._value and not self.B:
            self.command.kickChipPower = 5
            self.command.doKick = True
            self.command.doForce = True
        self.B = self.controller.button_b._value


        # Angle
        if 0.3 < abs(self.controller.axis_r.x): self.absolute_angle -= self.controller.axis_r.x * 0.1
        
        # Forward backward left right
        deadzone = 0.3

        velocity_x = 0
        if deadzone < abs(self.controller.axis_l.x):
            velocity_x = ( abs(self.controller.axis_l.x) - deadzone) / (1 - deadzone)
            velocity_x *= np.sign(self.controller.axis_l.x)

        velocity_y = 0
        if deadzone < abs(self.controller.axis_l.y):
            velocity_y = ( abs(self.controller.axis_l.y) - deadzone) / (1 - deadzone)
            velocity_y *= np.sign(self.controller.axis_l.y)

        rho = math.sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
        theta = math.atan2(-velocity_x, -velocity_y);

        self.command.id = self.robot_id
        self.command.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND

        self.command.rho = rho
        self.command.theta = theta + self.absolute_angle
        self.command.angle = self.absolute_angle

        rem.lib.encodeRobotCommand(self.payload, self.command)
        return self.payload


print(Xbox360Controller.get_available())

# wrappers = [JoystickWrapper(Xbox360Controller(i)) for i in range(len(Xbox360Controller.get_available()))]
wrappers = [JoystickWrapper(controller) for controller in Xbox360Controller.get_available()]

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


                # payloads = [wrapper.get_payload() for wrapper in wrappers]
                
                for wrapper in wrappers:
                    basestation.write(wrapper.get_payload().payload)

                # robotcommand = np.zeros(11, dtype=np.uint8)
                # robotcommand[0] = 15 # 15 = RobotCommand

                # robotcommand[1] = ((robot_id << 4) & 0b11110000) | (robotcommand[1] & 0b00001111);

                # _rho = int((rho - 0.0000000000000000) * 8192.0000000000000000)
                # robotcommand[2] = (_rho >> 8);
                # robotcommand[3] = _rho;

                # _theta = int((theta + absolute_angle - -3.1415926535897931) * 10430.3783504704533698)
                # robotcommand[4] = (_theta >> 8);
                # robotcommand[5] = _theta;

                # _absolute_angle = int((absolute_angle - -3.1415926535897931) * 10430.3783504704533698)
                # robotcommand[6] = (_absolute_angle >> 8);
                # robotcommand[7] = _absolute_angle;

                # cameraAngle = int((0 - -3.1415926535897931) * 10430.3783504704533698)
                # robotcommand[8] = (cameraAngle >> 8);
                # robotcommand[9] = cameraAngle;

                # basestation.write(bytearray(robotcommand))


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