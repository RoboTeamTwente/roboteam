# AUTOGENERATED. Run generator/main.py to regenerate
# Generated on August 30 2021, 22:48:16

# Warning : Check the unicode table before assigning a byte, to make sure that the byte isn't used for anything special : https://unicode-table.com/
# For example, don't use the following bytes
# 0b00000000 : The null-terminator, used to signal the end of strings / arrays / etc.
# 0b00001010 : The byte for newline, used for line termination.

LOCAL_REM_VERSION = 0

PACKET_TYPE_ROBOT_COMMAND                                    = 0b00001111 # 15 
PACKET_SIZE_ROBOT_COMMAND                                    = 12
PACKET_RANGE_ROBOT_COMMAND_RHO_MIN                           = 0.
PACKET_RANGE_ROBOT_COMMAND_RHO_MAX                           = 8.
PACKET_RANGE_ROBOT_COMMAND_THETA_MIN                         = -3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_THETA_MAX                         = 3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_ANGLE_MIN                         = -3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_ANGLE_MAX                         = 3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_CAMERA_ANGLE_MIN                  = -3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_CAMERA_ANGLE_MAX                  = 3.1415926535897931
PACKET_RANGE_ROBOT_COMMAND_KICK_CHIP_POWER_MIN               = 0.
PACKET_RANGE_ROBOT_COMMAND_KICK_CHIP_POWER_MAX               = 1.

PACKET_TYPE_ROBOT_FEEDBACK                                   = 0b00110011 # 51 
PACKET_SIZE_ROBOT_FEEDBACK                                   = 12
PACKET_RANGE_ROBOT_FEEDBACK_BALL_POS_MIN                     = -0.5
PACKET_RANGE_ROBOT_FEEDBACK_BALL_POS_MAX                     = 0.5
PACKET_RANGE_ROBOT_FEEDBACK_RHO_MIN                          = 0.
PACKET_RANGE_ROBOT_FEEDBACK_RHO_MAX                          = 8.
PACKET_RANGE_ROBOT_FEEDBACK_THETA_MIN                        = -3.1415926535897931
PACKET_RANGE_ROBOT_FEEDBACK_THETA_MAX                        = 3.1415926535897931
PACKET_RANGE_ROBOT_FEEDBACK_ANGLE_MIN                        = -3.1415926535897931
PACKET_RANGE_ROBOT_FEEDBACK_ANGLE_MAX                        = 3.1415926535897931

PACKET_TYPE_ROBOT_STATE_INFO                                 = 0b00111100 # 60 
PACKET_SIZE_ROBOT_STATE_INFO                                 = 35
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_ACC1_MIN                 = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_ACC1_MAX                 = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_ACC2_MIN                 = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_ACC2_MAX                 = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_YAW_MIN                  = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_XSENS_YAW_MAX                  = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_RATE_OF_TURN_MIN               = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_RATE_OF_TURN_MAX               = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED1_MIN               = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED1_MAX               = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED2_MIN               = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED2_MAX               = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED3_MIN               = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED3_MAX               = 50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED4_MIN               = -50000.
PACKET_RANGE_ROBOT_STATE_INFO_WHEEL_SPEED4_MAX               = 50000.

PACKET_TYPE_ROBOT_BUZZER                                     = 0b01010101 # 85 
PACKET_SIZE_ROBOT_BUZZER                                     = 6
PACKET_RANGE_ROBOT_BUZZER_DURATION_MIN                       = 0.
PACKET_RANGE_ROBOT_BUZZER_DURATION_MAX                       = 5.

PACKET_TYPE_BASESTATION_STATISTICS                           = 0b01011010 # 90 
PACKET_SIZE_BASESTATION_STATISTICS                           = 34

PACKET_TYPE_BASESTATION_GET_STATISTICS                       = 0b01100110 # 102 
PACKET_SIZE_BASESTATION_GET_STATISTICS                       = 1

PACKET_TYPE_BASESTATION_LOG                                  = 0b01101001 # 105 
PACKET_SIZE_BASESTATION_LOG                                  = 1

