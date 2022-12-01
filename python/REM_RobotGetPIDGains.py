# AUTOGENERATED. Run generator/main.py to regenerate
"""
[  0   ] [  1   ]
11111111 -------- header
-------- 1111---- remVersion
-------- ----1111 id
"""

import numpy as np
from . import REM_BaseTypes



class REM_RobotGetPIDGains:
    header = 0                # integer [0, 255]             Header byte indicating the type of packet
    remVersion = 0            # integer [0, 15]              Version of roboteam_embedded_messages
    id = 0                    # integer [0, 15]              Id of the robot



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

    @staticmethod
    def get_remVersion(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    def get_id(payload):
        return ((payload[1] & 0b00001111));

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

    @staticmethod
    def set_remVersion(payload, remVersion):
        payload[1] = ((remVersion << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    def set_id(payload, id):
        payload[1] = (id & 0b00001111) | (payload[1] & 0b11110000);

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(REM_BaseTypes.PACKET_SIZE_REM_ROBOT_GET_PIDGAINS, dtype=np.uint8)
        REM_RobotGetPIDGains.set_header              (payload, self.header)
        REM_RobotGetPIDGains.set_remVersion          (payload, self.remVersion)
        REM_RobotGetPIDGains.set_id                  (payload, self.id)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = REM_RobotGetPIDGains.get_header(payload)
        self.remVersion       = REM_RobotGetPIDGains.get_remVersion(payload)
        self.id               = REM_RobotGetPIDGains.get_id(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
