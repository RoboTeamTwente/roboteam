# AUTOGENERATED. Run generator/main.py to regenerate
"""
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ] [  6   ] [  7   ] [  8   ] [  9   ] [  10  ]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- toRobotId
-------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- -------- toColor
-------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- -------- toBC
-------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- -------- toBS
-------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- -------- toPC
-------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- fromRobotId
-------- -------- ----1--- -------- -------- -------- -------- -------- -------- -------- -------- fromColor
-------- -------- -----1-- -------- -------- -------- -------- -------- -------- -------- -------- reserved
-------- -------- ------1- -------- -------- -------- -------- -------- -------- -------- -------- fromBS
-------- -------- -------1 -------- -------- -------- -------- -------- -------- -------- -------- fromPC
-------- -------- -------- 1111---- -------- -------- -------- -------- -------- -------- -------- remVersion
-------- -------- -------- ----1111 -------- -------- -------- -------- -------- -------- -------- messageId
-------- -------- -------- -------- 11111111 -------- -------- -------- -------- -------- -------- payloadSize
-------- -------- -------- -------- -------- 11111111 -------- -------- -------- -------- -------- header
-------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 1111---- fillerBits
"""

import numpy as np
from . import REM_BaseTypes



class REM_SX1280Filler:
    header = 0                # integer [0, 255]             Header byte indicating the type of packet
    toRobotId = 0             # integer [0, 15]              Id of the receiving robot
    toColor = 0               # integer [0, 1]               Color of the receiving robot / basestation. Yellow = 0, Blue = 1
    toBC = 0                  # integer [0, 1]               Bit indicating this packet has to be broadcasted to all robots
    toBS = 0                  # integer [0, 1]               Bit indicating this packet is meant for the basestation
    toPC = 0                  # integer [0, 1]               Bit indicating this packet is meant for the PC
    fromRobotId = 0           # integer [0, 15]              Id of the transmitting robot
    fromColor = 0             # integer [0, 1]               Color of the transmitting robot / basestation. Yellow = 0, Blue = 1
    reserved = 0              # integer [0, 1]               reserved
    fromBS = 0                # integer [0, 1]               Bit indicating this packet is coming from the basestation
    fromPC = 0                # integer [0, 1]               Bit indicating this packet is coming from the PC
    remVersion = 0            # integer [0, 15]              Version of roboteam_embedded_messages
    messageId = 0             # integer [0, 15]              messageId. Can be used for aligning packets
    payloadSize = 0           # integer [0, 255]             Size of the payload. At most 255 bytes including the generic_packet_header. Keep the 127 byte SX1280 limit in mind
    header = 0                # integer [0, 255]             Header byte indicating the type of packet
    fillerBits = 0            # integer [0, 68719476735]     SX1280 requires a minimum of 6 bytes payload. See documentation page 124.



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

    @staticmethod
    def get_toRobotId(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    def get_toColor(payload):
        return (payload[1] & 0b00001000) > 0;

    @staticmethod
    def get_toBC(payload):
        return (payload[1] & 0b00000100) > 0;

    @staticmethod
    def get_toBS(payload):
        return (payload[1] & 0b00000010) > 0;

    @staticmethod
    def get_toPC(payload):
        return (payload[1] & 0b00000001) > 0;

    @staticmethod
    def get_fromRobotId(payload):
        return ((payload[2] & 0b11110000) >> 4);

    @staticmethod
    def get_fromColor(payload):
        return (payload[2] & 0b00001000) > 0;

    @staticmethod
    def get_reserved(payload):
        return (payload[2] & 0b00000100) > 0;

    @staticmethod
    def get_fromBS(payload):
        return (payload[2] & 0b00000010) > 0;

    @staticmethod
    def get_fromPC(payload):
        return (payload[2] & 0b00000001) > 0;

    @staticmethod
    def get_remVersion(payload):
        return ((payload[3] & 0b11110000) >> 4);

    @staticmethod
    def get_messageId(payload):
        return ((payload[3] & 0b00001111));

    @staticmethod
    def get_payloadSize(payload):
        return ((payload[4]));

    @staticmethod
    def get_header(payload):
        return ((payload[5]));

    @staticmethod
    def get_fillerBits(payload):
        return ((payload[6]) << 28) | ((payload[7]) << 20) | ((payload[8]) << 12) | ((payload[9]) << 4) | ((payload[10] & 0b11110000) >> 4);

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

    @staticmethod
    def set_toRobotId(payload, toRobotId):
        payload[1] = ((toRobotId << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    def set_toColor(payload, toColor):
        payload[1] = ((toColor << 3) & 0b00001000) | (payload[1] & 0b11110111);

    @staticmethod
    def set_toBC(payload, toBC):
        payload[1] = ((toBC << 2) & 0b00000100) | (payload[1] & 0b11111011);

    @staticmethod
    def set_toBS(payload, toBS):
        payload[1] = ((toBS << 1) & 0b00000010) | (payload[1] & 0b11111101);

    @staticmethod
    def set_toPC(payload, toPC):
        payload[1] = (toPC & 0b00000001) | (payload[1] & 0b11111110);

    @staticmethod
    def set_fromRobotId(payload, fromRobotId):
        payload[2] = ((fromRobotId << 4) & 0b11110000) | (payload[2] & 0b00001111);

    @staticmethod
    def set_fromColor(payload, fromColor):
        payload[2] = ((fromColor << 3) & 0b00001000) | (payload[2] & 0b11110111);

    @staticmethod
    def set_reserved(payload, reserved):
        payload[2] = ((reserved << 2) & 0b00000100) | (payload[2] & 0b11111011);

    @staticmethod
    def set_fromBS(payload, fromBS):
        payload[2] = ((fromBS << 1) & 0b00000010) | (payload[2] & 0b11111101);

    @staticmethod
    def set_fromPC(payload, fromPC):
        payload[2] = (fromPC & 0b00000001) | (payload[2] & 0b11111110);

    @staticmethod
    def set_remVersion(payload, remVersion):
        payload[3] = ((remVersion << 4) & 0b11110000) | (payload[3] & 0b00001111);

    @staticmethod
    def set_messageId(payload, messageId):
        payload[3] = (messageId & 0b00001111) | (payload[3] & 0b11110000);

    @staticmethod
    def set_payloadSize(payload, payloadSize):
        payload[4] = payloadSize;

    @staticmethod
    def set_header(payload, header):
        payload[5] = header;

    @staticmethod
    def set_fillerBits(payload, fillerBits):
        payload[6] = (fillerBits >> 28);
        payload[7] = (fillerBits >> 20);
        payload[8] = (fillerBits >> 12);
        payload[9] = (fillerBits >> 4);
        payload[10] = ((fillerBits << 4) & 0b11110000) | (payload[10] & 0b00001111);

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(REM_BaseTypes.PACKET_SIZE_REM_SX1280FILLER, dtype=np.uint8)
        REM_SX1280Filler.set_header              (payload, self.header)
        REM_SX1280Filler.set_toRobotId           (payload, self.toRobotId)
        REM_SX1280Filler.set_toColor             (payload, self.toColor)
        REM_SX1280Filler.set_toBC                (payload, self.toBC)
        REM_SX1280Filler.set_toBS                (payload, self.toBS)
        REM_SX1280Filler.set_toPC                (payload, self.toPC)
        REM_SX1280Filler.set_fromRobotId         (payload, self.fromRobotId)
        REM_SX1280Filler.set_fromColor           (payload, self.fromColor)
        REM_SX1280Filler.set_reserved            (payload, self.reserved)
        REM_SX1280Filler.set_fromBS              (payload, self.fromBS)
        REM_SX1280Filler.set_fromPC              (payload, self.fromPC)
        REM_SX1280Filler.set_remVersion          (payload, self.remVersion)
        REM_SX1280Filler.set_messageId           (payload, self.messageId)
        REM_SX1280Filler.set_payloadSize         (payload, self.payloadSize)
        REM_SX1280Filler.set_header              (payload, self.header)
        REM_SX1280Filler.set_fillerBits          (payload, self.fillerBits)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = REM_SX1280Filler.get_header(payload)
        self.toRobotId        = REM_SX1280Filler.get_toRobotId(payload)
        self.toColor          = REM_SX1280Filler.get_toColor(payload)
        self.toBC             = REM_SX1280Filler.get_toBC(payload)
        self.toBS             = REM_SX1280Filler.get_toBS(payload)
        self.toPC             = REM_SX1280Filler.get_toPC(payload)
        self.fromRobotId      = REM_SX1280Filler.get_fromRobotId(payload)
        self.fromColor        = REM_SX1280Filler.get_fromColor(payload)
        self.reserved         = REM_SX1280Filler.get_reserved(payload)
        self.fromBS           = REM_SX1280Filler.get_fromBS(payload)
        self.fromPC           = REM_SX1280Filler.get_fromPC(payload)
        self.remVersion       = REM_SX1280Filler.get_remVersion(payload)
        self.messageId        = REM_SX1280Filler.get_messageId(payload)
        self.payloadSize      = REM_SX1280Filler.get_payloadSize(payload)
        self.header           = REM_SX1280Filler.get_header(payload)
        self.fillerBits       = REM_SX1280Filler.get_fillerBits(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
