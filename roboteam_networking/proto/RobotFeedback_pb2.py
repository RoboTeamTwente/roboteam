# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: RobotFeedback.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()




DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x13RobotFeedback.proto\x12\x05proto\"\x9b\x02\n\rRobotFeedback\x12\n\n\x02id\x18\x01 \x01(\x05\x12\x1d\n\x15\x62\x61ll_sensor_sees_ball\x18\x02 \x01(\x08\x12\x1e\n\x16\x62\x61ll_sensor_is_working\x18\x04 \x01(\x08\x12\x1a\n\x12\x64ribbler_sees_ball\x18\x05 \x01(\x08\x12\x1c\n\x14\x65stimated_velocity_x\x18\x06 \x01(\x01\x12\x1c\n\x14\x65stimated_velocity_y\x18\x07 \x01(\x01\x12\x15\n\restimated_yaw\x18\x08 \x01(\x01\x12\x1b\n\x13xsens_is_calibrated\x18\t \x01(\x08\x12\x1c\n\x14\x63\x61pacitor_is_charged\x18\n \x01(\x08\x12\x15\n\rbattery_level\x18\r \x01(\x02\"\x8b\x01\n\x0eRobotsFeedback\x12\x1e\n\x04team\x18\x01 \x01(\x0e\x32\x10.proto.RobotTeam\x12*\n\x06source\x18\x02 \x01(\x0e\x32\x1a.proto.RobotFeedbackSource\x12-\n\x0frobots_feedback\x18\x03 \x03(\x0b\x32\x14.proto.RobotFeedback*+\n\tRobotTeam\x12\x0f\n\x0bYELLOW_TEAM\x10\x00\x12\r\n\tBLUE_TEAM\x10\x01*5\n\x13RobotFeedbackSource\x12\r\n\tSIMULATOR\x10\x00\x12\x0f\n\x0b\x42\x41SESTATION\x10\x01\x62\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'RobotFeedback_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_ROBOTTEAM']._serialized_start=458
  _globals['_ROBOTTEAM']._serialized_end=501
  _globals['_ROBOTFEEDBACKSOURCE']._serialized_start=503
  _globals['_ROBOTFEEDBACKSOURCE']._serialized_end=556
  _globals['_ROBOTFEEDBACK']._serialized_start=31
  _globals['_ROBOTFEEDBACK']._serialized_end=314
  _globals['_ROBOTSFEEDBACK']._serialized_start=317
  _globals['_ROBOTSFEEDBACK']._serialized_end=456
# @@protoc_insertion_point(module_scope)