import numpy as np
from collections import deque
import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotLog import REM_RobotLog
from roboteam_embedded_messages.python.REM_BasestationLog import REM_BasestationLog

class REMParser():
	
	def __init__(self, device, output_file=None):
		print(f"[REMParser] New REMParser")
		if device: print(f"[REMParser] Device {device.port}")

		self.device = device
		self.byte_buffer = bytes()
		self.packet_buffer = deque()
		self.output_file = None

		if type(output_file) == str:
			print(f"[REMParser] Creating output file {output_file}")
			self.output_file = open(output_file, "wb")



	def read(self):

		bytes_in_waiting = self.device.inWaiting()
		if bytes_in_waiting == 0: return
		# print(f"[read] {bytes_in_waiting} bytes in waiting")
		self.byte_buffer += self.device.read(bytes_in_waiting)
		# print(f"Read {bytes_in_waiting} bytes")

	def process(self):
		# No bytes in the buffer, so nothing to process
		if len(self.byte_buffer) == 0: return

		# Process as many bytes / packets as possible
		while True:
			# Stop when there are no more bytes to process
			if len(self.byte_buffer) == 0: break
			# Get the packet type of the packet at the front of the buffer
			packet_type = self.byte_buffer[0]

			# BasestationLog or RobotLog. Assumption that these are the only two packets with dynamic size
			if packet_type in [BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG, BaseTypes.PACKET_TYPE_REM_ROBOT_LOG]:
				idx = 0 if packet_type == BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG else 1
				packet_size = [BaseTypes.PACKET_SIZE_REM_BASESTATION_LOG, BaseTypes.PACKET_SIZE_REM_ROBOT_LOG][idx]
				# Check if the entire packet is in the buffer
				if len(self.byte_buffer) < packet_size:	break
				
				packet_bytes = self.byte_buffer[:packet_size]
				# Get required bytes from buffer and process
				packet = [REM_BasestationLog, REM_RobotLog][idx]()
				packet.decode(packet_bytes)
				# Check if the entire message is in the buffer
				if len(self.byte_buffer) < packet_size + packet.messageLength: break
				# Get the message from the buffer
				message = self.byte_buffer[packet_size : packet_size + packet.messageLength]
				packet.message = message.decode()
				# Add packet to buffer
				self.addPacket(packet)
				# Write bytes to output file
				self.writeBytes(packet_bytes + message)
				# Remove processed bytes from buffer
				self.byte_buffer = self.byte_buffer[packet_size + packet.messageLength:]
				
			# Basestation get configuration
			else:
				packet_size = BaseTypes.PACKET_TYPE_TO_SIZE(packet_type)
				
				if len(self.byte_buffer) < packet_size: break
				# Create packet instance
				packet_obj = BaseTypes.PACKET_TYPE_TO_OBJ(packet_type)()
				# print(f"[process] Packet of size {packet_size} : {packet_type} : {type(packet_obj).__name__}")
				packet_bytes = self.byte_buffer[:packet_size]
				# Get required bytes from buffer and process
				packet_obj.decode(packet_bytes)
				# Add packet to buffer
				self.addPacket(packet_obj)
				# Write bytes to output file
				self.writeBytes(packet_bytes)
				# Remove processed bytes from buffer
				self.byte_buffer = self.byte_buffer[packet_size:]

	def addPacket(self, packet):
		self.packet_buffer.append(packet)
		
	def writeBytes(self, _bytes):
		if self.output_file is not None: 
			self.output_file.write(_bytes)

	def hasPackets(self):
		return 0 < len(self.packet_buffer)
	
	def getNextPacket(self):
		if self.hasPackets(): return self.packet_buffer.popleft()

	def parseFile(self, filepath):
		print(f"[REMParser] Parsing file {filepath}")
		with open(filepath, "rb") as file:
			self.byte_buffer = file.read()
			self.process()

		packet_counts = {}
		for packet in self.packet_buffer:
			packet_type = type(packet)
			if packet_type not in packet_counts:
				packet_counts[packet_type] = 0
			packet_counts[packet_type] += 1

		for packet_type in packet_counts:
			print(packet_type.__name__.ljust(30), packet_counts[packet_type])