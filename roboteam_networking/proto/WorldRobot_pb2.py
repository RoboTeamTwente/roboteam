# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: WorldRobot.proto
"""Generated protocol buffer code."""
from google.protobuf.internal import builder as _builder
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import Vector2f_pb2 as Vector2f__pb2
import RobotProcessedFeedback_pb2 as RobotProcessedFeedback__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x10WorldRobot.proto\x12\x05proto\x1a\x0eVector2f.proto\x1a\x1cRobotProcessedFeedback.proto\"\xa1\x01\n\nWorldRobot\x12\n\n\x02id\x18\x01 \x01(\r\x12\x1c\n\x03pos\x18\x02 \x01(\x0b\x32\x0f.proto.Vector2f\x12\x0b\n\x03yaw\x18\x03 \x01(\x02\x12\x1c\n\x03vel\x18\x04 \x01(\x0b\x32\x0f.proto.Vector2f\x12\t\n\x01w\x18\x05 \x01(\x02\x12\x33\n\x0c\x66\x65\x65\x64\x62\x61\x63kInfo\x18\x06 \x01(\x0b\x32\x1d.proto.RobotProcessedFeedbackb\x06proto3')

_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, globals())
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'WorldRobot_pb2', globals())
if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _WORLDROBOT._serialized_start=74
  _WORLDROBOT._serialized_end=235
# @@protoc_insertion_point(module_scope)
