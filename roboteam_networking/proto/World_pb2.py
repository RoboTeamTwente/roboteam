# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: World.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import WorldBall_pb2 as WorldBall__pb2
from . import WorldRobot_pb2 as WorldRobot__pb2
from . import RobotProcessedFeedback_pb2 as RobotProcessedFeedback__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\x0bWorld.proto\x12\x05proto\x1a\x0fWorldBall.proto\x1a\x10WorldRobot.proto\x1a\x1cRobotProcessedFeedback.proto\"\xf3\x01\n\x05World\x12\x0c\n\x04time\x18\x01 \x01(\x04\x12\n\n\x02id\x18\x02 \x01(\r\x12\x1e\n\x04\x62\x61ll\x18\x03 \x01(\x0b\x32\x10.proto.WorldBall\x12!\n\x06yellow\x18\x04 \x03(\x0b\x32\x11.proto.WorldRobot\x12\x1f\n\x04\x62lue\x18\x05 \x03(\x0b\x32\x11.proto.WorldRobot\x12\x36\n\x14yellow_unseen_robots\x18\x06 \x03(\x0b\x32\x18.proto.FeedbackOnlyRobot\x12\x34\n\x12\x62lue_unseen_robots\x18\x07 \x03(\x0b\x32\x18.proto.FeedbackOnlyRobotb\x06proto3')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'World_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_WORLD']._serialized_start=88
  _globals['_WORLD']._serialized_end=331
# @@protoc_insertion_point(module_scope)