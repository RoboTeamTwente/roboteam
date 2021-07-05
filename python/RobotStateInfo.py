# AUTOGENERATED. Run generator/main.py to regenerate
# Generated on July 05 2021, 02:19:27

"""
[  0   ] [  1   ] [  2   ] [  3   ] [  4   ] [  5   ] [  6   ] [  7   ] [  8   ] [  9   ] [  10  ] [  11  ] [  12  ] [  13  ] [  14  ] [  15  ] [  16  ] [  17  ] [  18  ] [  19  ] [  20  ] [  21  ] [  22  ] [  23  ] [  24  ] [  25  ] [  26  ] [  27  ] [  28  ] [  29  ] [  30  ] [  31  ] [  32  ] [  33  ]
11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- header
-------- 1111---- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- id
-------- ----1111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- messageId
-------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- xsensAcc1
-------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- xsensAcc2
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- xsensYaw
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- rateOfTurn
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- wheelSpeed1
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- -------- -------- -------- -------- wheelSpeed2
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 -------- -------- -------- -------- wheelSpeed3
-------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- -------- 11111111 11111111 11111111 11111111 wheelSpeed4
"""

import numpy as np
import BaseTypes



class RobotStateInfo:
    header = 0                # Header byte indicating the type of packet
    id = 0                    # Id of the robot 
    messageId = 0             # Id of the message
    xsensAcc1 = 0             # xsensAcc1
    xsensAcc2 = 0             # xsensAcc2
    xsensYaw = 0              # xsensYaw
    rateOfTurn = 0            # rateOfTurn
    wheelSpeed1 = 0           # wheelSpeed1
    wheelSpeed2 = 0           # wheelSpeed2
    wheelSpeed3 = 0           # wheelSpeed3
    wheelSpeed4 = 0           # wheelSpeed4



# ================================ GETTERS ================================
    @staticmethod
    def get_header(payload):
        return ((payload[0]));

    @staticmethod
    def get_id(payload):
        return ((payload[1] & 0b11110000) >> 4);

    @staticmethod
    def get_messageId(payload):
        return ((payload[1] & 0b00001111));

    @staticmethod
    def get_xsensAcc1(payload):
        _xsensAcc1 = ((payload[2]) << 24) | ((payload[3]) << 16) | ((payload[4]) << 8) | ((payload[5]));
        return (_xsensAcc1 * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_xsensAcc2(payload):
        _xsensAcc2 = ((payload[6]) << 24) | ((payload[7]) << 16) | ((payload[8]) << 8) | ((payload[9]));
        return (_xsensAcc2 * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_xsensYaw(payload):
        _xsensYaw = ((payload[10]) << 24) | ((payload[11]) << 16) | ((payload[12]) << 8) | ((payload[13]));
        return (_xsensYaw * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_rateOfTurn(payload):
        _rateOfTurn = ((payload[14]) << 24) | ((payload[15]) << 16) | ((payload[16]) << 8) | ((payload[17]));
        return (_rateOfTurn * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_wheelSpeed1(payload):
        _wheelSpeed1 = ((payload[18]) << 24) | ((payload[19]) << 16) | ((payload[20]) << 8) | ((payload[21]));
        return (_wheelSpeed1 * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_wheelSpeed2(payload):
        _wheelSpeed2 = ((payload[22]) << 24) | ((payload[23]) << 16) | ((payload[24]) << 8) | ((payload[25]));
        return (_wheelSpeed2 * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_wheelSpeed3(payload):
        _wheelSpeed3 = ((payload[26]) << 24) | ((payload[27]) << 16) | ((payload[28]) << 8) | ((payload[29]));
        return (_wheelSpeed3 * 0.0000232830643708) + -50000.0000000000000000;

    @staticmethod
    def get_wheelSpeed4(payload):
        _wheelSpeed4 = ((payload[30]) << 24) | ((payload[31]) << 16) | ((payload[32]) << 8) | ((payload[33]));
        return (_wheelSpeed4 * 0.0000232830643708) + -50000.0000000000000000;

# ================================ SETTERS ================================
    @staticmethod
    def set_header(payload, header):
        payload[0] = header;

    @staticmethod
    def set_id(payload, id):
        payload[1] = ((id << 4) & 0b11110000) | (payload[1] & 0b00001111);

    @staticmethod
    def set_messageId(payload, messageId):
        payload[1] = (messageId & 0b00001111) | (payload[1] & 0b11110000);

    @staticmethod
    def set_xsensAcc1(payload, xsensAcc1):
        _xsensAcc1 = int((xsensAcc1 +50000.0000000000000000) / 0.0000232830643708);
        payload[2] = (_xsensAcc1 >> 24);
        payload[3] = (_xsensAcc1 >> 16);
        payload[4] = (_xsensAcc1 >> 8);
        payload[5] = _xsensAcc1;

    @staticmethod
    def set_xsensAcc2(payload, xsensAcc2):
        _xsensAcc2 = int((xsensAcc2 +50000.0000000000000000) / 0.0000232830643708);
        payload[6] = (_xsensAcc2 >> 24);
        payload[7] = (_xsensAcc2 >> 16);
        payload[8] = (_xsensAcc2 >> 8);
        payload[9] = _xsensAcc2;

    @staticmethod
    def set_xsensYaw(payload, xsensYaw):
        _xsensYaw = int((xsensYaw +50000.0000000000000000) / 0.0000232830643708);
        payload[10] = (_xsensYaw >> 24);
        payload[11] = (_xsensYaw >> 16);
        payload[12] = (_xsensYaw >> 8);
        payload[13] = _xsensYaw;

    @staticmethod
    def set_rateOfTurn(payload, rateOfTurn):
        _rateOfTurn = int((rateOfTurn +50000.0000000000000000) / 0.0000232830643708);
        payload[14] = (_rateOfTurn >> 24);
        payload[15] = (_rateOfTurn >> 16);
        payload[16] = (_rateOfTurn >> 8);
        payload[17] = _rateOfTurn;

    @staticmethod
    def set_wheelSpeed1(payload, wheelSpeed1):
        _wheelSpeed1 = int((wheelSpeed1 +50000.0000000000000000) / 0.0000232830643708);
        payload[18] = (_wheelSpeed1 >> 24);
        payload[19] = (_wheelSpeed1 >> 16);
        payload[20] = (_wheelSpeed1 >> 8);
        payload[21] = _wheelSpeed1;

    @staticmethod
    def set_wheelSpeed2(payload, wheelSpeed2):
        _wheelSpeed2 = int((wheelSpeed2 +50000.0000000000000000) / 0.0000232830643708);
        payload[22] = (_wheelSpeed2 >> 24);
        payload[23] = (_wheelSpeed2 >> 16);
        payload[24] = (_wheelSpeed2 >> 8);
        payload[25] = _wheelSpeed2;

    @staticmethod
    def set_wheelSpeed3(payload, wheelSpeed3):
        _wheelSpeed3 = int((wheelSpeed3 +50000.0000000000000000) / 0.0000232830643708);
        payload[26] = (_wheelSpeed3 >> 24);
        payload[27] = (_wheelSpeed3 >> 16);
        payload[28] = (_wheelSpeed3 >> 8);
        payload[29] = _wheelSpeed3;

    @staticmethod
    def set_wheelSpeed4(payload, wheelSpeed4):
        _wheelSpeed4 = int((wheelSpeed4 +50000.0000000000000000) / 0.0000232830643708);
        payload[30] = (_wheelSpeed4 >> 24);
        payload[31] = (_wheelSpeed4 >> 16);
        payload[32] = (_wheelSpeed4 >> 8);
        payload[33] = _wheelSpeed4;

# ================================ ENCODE ================================
    def encode(self):
        payload = np.zeros(BaseTypes.PACKET_SIZE_ROBOT_STATE_INFO, dtype=np.uint8)
        RobotStateInfo.set_header              (payload, self.header)
        RobotStateInfo.set_id                  (payload, self.id)
        RobotStateInfo.set_messageId           (payload, self.messageId)
        RobotStateInfo.set_xsensAcc1           (payload, self.xsensAcc1)
        RobotStateInfo.set_xsensAcc2           (payload, self.xsensAcc2)
        RobotStateInfo.set_xsensYaw            (payload, self.xsensYaw)
        RobotStateInfo.set_rateOfTurn          (payload, self.rateOfTurn)
        RobotStateInfo.set_wheelSpeed1         (payload, self.wheelSpeed1)
        RobotStateInfo.set_wheelSpeed2         (payload, self.wheelSpeed2)
        RobotStateInfo.set_wheelSpeed3         (payload, self.wheelSpeed3)
        RobotStateInfo.set_wheelSpeed4         (payload, self.wheelSpeed4)
        return payload


# ================================ DECODE ================================
    def decode(self, payload):
        self.header           = RobotStateInfo.get_header(payload)
        self.id               = RobotStateInfo.get_id(payload)
        self.messageId        = RobotStateInfo.get_messageId(payload)
        self.xsensAcc1        = RobotStateInfo.get_xsensAcc1(payload)
        self.xsensAcc2        = RobotStateInfo.get_xsensAcc2(payload)
        self.xsensYaw         = RobotStateInfo.get_xsensYaw(payload)
        self.rateOfTurn       = RobotStateInfo.get_rateOfTurn(payload)
        self.wheelSpeed1      = RobotStateInfo.get_wheelSpeed1(payload)
        self.wheelSpeed2      = RobotStateInfo.get_wheelSpeed2(payload)
        self.wheelSpeed3      = RobotStateInfo.get_wheelSpeed3(payload)
        self.wheelSpeed4      = RobotStateInfo.get_wheelSpeed4(payload)


    def print_bit_string(self):
        payload = self.encode()
        for i in range(len(payload)):
            print(format(payload[i], '08b'), end=" ")
        print()
