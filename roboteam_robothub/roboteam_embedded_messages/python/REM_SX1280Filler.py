# AUTOGENERATED. Run generator/main.py to regenerate
"""
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ]
11111111 -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- remVersion
-------- ----1111 11111111 11111111 11111111 11111111 fillerBits
"""

import numpy as np
from . import REM_BaseTypes



class REM_SX1280Filler:
    header = 0                # integer [0, 255]             Header byte indicating the type of packet
    remVersion = 0            # integer [0, 15]              Version of roboteam_embedded_messages
    fillerBits = 0            # integer [0, 68719476735]     SX1280 requires a minimum of 6 bytes payload. See documentation page 124.



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

    @staticmethod
    def get_remVersion(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    def get_fillerBits(payload):
        return ((payload[1] & 0b00001111) << 32) | ((payload[2]) << 24) | ((payload[3]) << 16) | ((payload[4]) << 8) | ((payload[5]));

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

    @staticmethod
    def set_remVersion(payload, remVersion):
        payload[1] = ((remVersion << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    def set_fillerBits(payload, fillerBits):
        payload[1] = ((fillerBits >> 32) & 0b00001111) | (payload[1] & 0b11110000);
        payload[2] = (fillerBits >> 24);
        payload[3] = (fillerBits >> 16);
        payload[4] = (fillerBits >> 8);
        payload[5] = fillerBits;

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(REM_BaseTypes.REM_PACKET_SIZE_REM_SX1280FILLER, dtype=np.uint8)
        REM_SX1280Filler.set_header              (payload, self.header)
        REM_SX1280Filler.set_remVersion          (payload, self.remVersion)
        REM_SX1280Filler.set_fillerBits          (payload, self.fillerBits)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = REM_SX1280Filler.get_header(payload)
        self.remVersion       = REM_SX1280Filler.get_remVersion(payload)
        self.fillerBits       = REM_SX1280Filler.get_fillerBits(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
