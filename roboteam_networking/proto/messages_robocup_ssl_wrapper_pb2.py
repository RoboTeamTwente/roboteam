# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: messages_robocup_ssl_wrapper.proto
# Protobuf Python Version: 4.25.3
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import symbol_database as _symbol_database
from google.protobuf.internal import builder as _builder
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


from . import messages_robocup_ssl_detection_pb2 as messages__robocup__ssl__detection__pb2
from . import messages_robocup_ssl_geometry_pb2 as messages__robocup__ssl__geometry__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n\"messages_robocup_ssl_wrapper.proto\x12\x05proto\x1a$messages_robocup_ssl_detection.proto\x1a#messages_robocup_ssl_geometry.proto\"l\n\x11SSL_WrapperPacket\x12,\n\tdetection\x18\x01 \x01(\x0b\x32\x19.proto.SSL_DetectionFrame\x12)\n\x08geometry\x18\x02 \x01(\x0b\x32\x17.proto.SSL_GeometryData')

_globals = globals()
_builder.BuildMessageAndEnumDescriptors(DESCRIPTOR, _globals)
_builder.BuildTopDescriptorsAndMessages(DESCRIPTOR, 'messages_robocup_ssl_wrapper_pb2', _globals)
if _descriptor._USE_C_DESCRIPTORS == False:
  DESCRIPTOR._options = None
  _globals['_SSL_WRAPPERPACKET']._serialized_start=120
  _globals['_SSL_WRAPPERPACKET']._serialized_end=228
# @@protoc_insertion_point(module_scope)
