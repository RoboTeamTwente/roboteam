# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: ssl_vision_wrapper_tracked.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import ssl_vision_detection_tracked_pb2 as ssl__vision__detection__tracked__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n ssl_vision_wrapper_tracked.proto\x1a\"ssl_vision_detection_tracked.proto\"_\n\x14TrackerWrapperPacket\x12\x0c\n\x04uuid\x18\x01 \x02(\t\x12\x13\n\x0bsource_name\x18\x02 \x01(\t\x12$\n\rtracked_frame\x18\x03 \x01(\x0b\x32\r.TrackedFrameBAZ?github.com/RoboCup-SSL/ssl-game-controller/internal/app/tracker')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'ssl_vision_wrapper_tracked_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  _globals['DESCRIPTOR']._options = None
  _globals['DESCRIPTOR']._serialized_options = b'Z?github.com/RoboCup-SSL/ssl-game-controller/internal/app/tracker'
  _globals['_TRACKERWRAPPERPACKET']._serialized_start=72
  _globals['_TRACKERWRAPPERPACKET']._serialized_end=167
# @@protoc_insertion_point(module_scope)
