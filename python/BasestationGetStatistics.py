# AUTOGENERATED. Run generator/main.py to regenerate
"""
[  0   ]
11111111 header
"""

import numpy as np
from . import BaseTypes



class BasestationGetStatistics:
    header = 0                # integer [0, 255]             Header byte indicating the type of packet



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(BaseTypes.PACKET_SIZE_BASESTATION_GET_STATISTICS, dtype=np.uint8)
        BasestationGetStatistics.set_header              (payload, self.header)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = BasestationGetStatistics.get_header(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
