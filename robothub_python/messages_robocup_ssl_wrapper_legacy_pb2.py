# -*- coding: utf-8 -*-
# Generated by the protocol buffer compiler.  DO NOT EDIT!
# source: messages_robocup_ssl_wrapper_legacy.proto
"""Generated protocol buffer code."""
from google.protobuf import descriptor as _descriptor
from google.protobuf import descriptor_pool as _descriptor_pool
from google.protobuf import message as _message
from google.protobuf import reflection as _reflection
from google.protobuf import symbol_database as _symbol_database
# @@protoc_insertion_point(imports)

_sym_db = _symbol_database.Default()


import messages_robocup_ssl_detection_pb2 as messages__robocup__ssl__detection__pb2
import messages_robocup_ssl_geometry_legacy_pb2 as messages__robocup__ssl__geometry__legacy__pb2


DESCRIPTOR = _descriptor_pool.Default().AddSerializedFile(b'\n)messages_robocup_ssl_wrapper_legacy.proto\x12\x1fproto.RoboCup2014Legacy.Wrapper\x1a$messages_robocup_ssl_detection.proto\x1a*messages_robocup_ssl_geometry_legacy.proto\"\x87\x01\n\x11SSL_WrapperPacket\x12,\n\tdetection\x18\x01 \x01(\x0b\x32\x19.proto.SSL_DetectionFrame\x12\x44\n\x08geometry\x18\x02 \x01(\x0b\x32\x32.proto.RoboCup2014Legacy.Geometry.SSL_GeometryDatab\x06proto3')



_SSL_WRAPPERPACKET = DESCRIPTOR.message_types_by_name['SSL_WrapperPacket']
SSL_WrapperPacket = _reflection.GeneratedProtocolMessageType('SSL_WrapperPacket', (_message.Message,), {
  'DESCRIPTOR' : _SSL_WRAPPERPACKET,
  '__module__' : 'messages_robocup_ssl_wrapper_legacy_pb2'
  # @@protoc_insertion_point(class_scope:proto.RoboCup2014Legacy.Wrapper.SSL_WrapperPacket)
  })
_sym_db.RegisterMessage(SSL_WrapperPacket)

if _descriptor._USE_C_DESCRIPTORS == False:

  DESCRIPTOR._options = None
  _SSL_WRAPPERPACKET._serialized_start=161
  _SSL_WRAPPERPACKET._serialized_end=296
# @@protoc_insertion_point(module_scope)
