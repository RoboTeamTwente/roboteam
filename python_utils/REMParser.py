import numpy as np
from collections import deque
import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_Packet import REM_Packet
from roboteam_embedded_messages.python.REM_Log import REM_Log

DEBUG = False

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
		# print(f"[read] Read {bytes_in_waiting} bytes")

	def process(self):
		# No bytes in the buffer, so nothing to process
		if len(self.byte_buffer) == 0: return
		
		if DEBUG: print("--process()-----------")
		
		# Process as many bytes / packets as possible
		while True:
			# Stop when there are no more bytes to process
			if len(self.byte_buffer) == 0: break

			if DEBUG: print(f"- while True | {len(self.byte_buffer)} bytes in buffer")

			# Check if the packet type is valid according to REM
			packet_type = self.byte_buffer[0]
			packet_valid = BaseTypes.REM_PACKET_TYPE_TO_VALID(packet_type)
			# If the packet type is not valid / unknown
			if not packet_valid:
				self.byte_buffer = bytes()
				raise Exception(f"[REMParser][process] Error! Received invalid packet type {packet_type}!")

			# Make sure that at least the entire default REM_Packet header is in the buffer
			# This is need to call functions such as get_remVersion()and get_payloadSize()
			if len(self.byte_buffer) < BaseTypes.REM_PACKET_SIZE_REM_PACKET:
				if DEBUG: print(f"- Complete REM_Packet not yet in buffer. {len(self.byte_buffer)}/{BaseTypes.REM_PACKET_SIZE_REM_PACKET} bytes")
				break				

			# At least the REM_Packet headers are in the buffer. Decode it
			packet = REM_Log()
			packet.decode(self.byte_buffer[:BaseTypes.REM_PACKET_SIZE_REM_PACKET])

			# Get the expected packet size as expected by REM
			rem_packet_size = BaseTypes.REM_PACKET_TYPE_TO_SIZE(packet.header)

			if DEBUG: print(f"- type={packet.header} ({BaseTypes.REM_PACKET_TYPE_TO_OBJ(packet.header).__name__}), size={packet.payloadSize}, REM_size={rem_packet_size}")

			# Ensure that the entire payload is in the byte buffer
			if len(self.byte_buffer) < packet.payloadSize: 
				if DEBUG: print(f"- Complete packet not yet in buffer. {len(self.byte_buffer)}/{rem_packet_size} bytes")
				break

			# if not REM_log, packet->payloadSize should be equal to expected REM_PACKET_SIZE
			if packet.header != BaseTypes.REM_PACKET_TYPE_REM_LOG:
				if packet.payloadSize != rem_packet_size:
					self.byte_buffer = bytes()
					raise Exception(f"[REMParser][process] Error! REM_Packet->payloadSize={packet.payloadSize} does not equal expected REM_PACKET_SIZE_*={rem_packet_size}! ")

			# BasestationLog or RobotLog. Assumption that these are the only two packets with dynamic size
			try:
				if packet.header == BaseTypes.REM_PACKET_TYPE_REM_LOG:

					# Retrieve the bytes from the byte buffer
					packet_bytes = self.byte_buffer[:packet.payloadSize]
					# Decode the packet as REM_Log
					packet = REM_Log()
					packet.decode(packet_bytes)
					# Get the message from the buffer. The message is everything after the REM_Packet header
					message = packet_bytes[BaseTypes.REM_PACKET_SIZE_REM_LOG:]
					# Convert bytes into string, and store in REM_Log object
					packet.message = message.decode()
					# Add packet to buffer
					self.addPacket(packet)
					# Write bytes to output file
					self.writeBytes(packet_bytes + message)
					# Remove processed bytes from buffer
					self.byte_buffer = self.byte_buffer[packet.payloadSize:]
					
				# Basestation get configuration
				else:
					packet_size = BaseTypes.REM_PACKET_TYPE_TO_SIZE(packet.header)
					# print(f"[process] Packet of size {packet_size} : type {packet_type} ", end="")
					if len(self.byte_buffer) < packet_size: break
					# Create packet instance= 
					packet_obj = BaseTypes.REM_PACKET_TYPE_TO_OBJ(packet.header)()
					# print(f"\r[process] Packet of size {packet_size} : type {packet_type} : {type(packet_obj).__name__}")
					packet_bytes = self.byte_buffer[:packet_size]
					# Get required bytes from buffer and process
					packet_obj.decode(packet_bytes)
					# Add packet to buffer
					self.addPacket(packet_obj)
					# Write bytes to output file
					self.writeBytes(packet_bytes)
					# Remove processed bytes from buffer
					self.byte_buffer = self.byte_buffer[packet_size:]
			except Exception as e:
				print("EXCEPTION", e)
				self.byte_buffer = bytes()

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