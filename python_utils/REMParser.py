import numpy as np
from collections import deque
import argparse
import utils
import json
import copy
import time 
from datetime import datetime, timedelta
import os

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_Packet import REM_Packet
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
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
			try:
				# Create symlink
				os.remove("latest.rembin")
			except Exception as e:
				print("\n")
				print(e)
			os.symlink(output_file, "latest.rembin")
	def read(self):
		bytes_in_waiting = self.device.inWaiting()
		if bytes_in_waiting == 0: return
		if DEBUG: print(f"[read] {bytes_in_waiting} bytes in waiting")
		self.byte_buffer += self.device.read(bytes_in_waiting)
		if DEBUG: print(f"[read] Read {bytes_in_waiting} bytes")

	def process(self, parse_file=False):
		# No bytes in the buffer, so nothing to process
		if len(self.byte_buffer) == 0: return
		
		if DEBUG: print("--process()-----------")
		
		# Process as many bytes / packets as possible
		while True:
			# Stop when there are no more bytes to process
			if len(self.byte_buffer) == 0: break

			timestamp_parser_ms = None
			if parse_file:
				timestamp_ms_bytes = self.byte_buffer[:8]
				timestamp_parser_ms = int.from_bytes(timestamp_ms_bytes, 'little')
				self.byte_buffer = self.byte_buffer[8:]

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
			packet = REM_Packet()
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

			# Retrieve the bytes of the entire packet from the byte buffer
			packet_bytes = self.byte_buffer[:packet.payloadSize]
			# Create packet instance
			packet = BaseTypes.REM_PACKET_TYPE_TO_OBJ(packet_type)()
			# Decode the packet
			packet.decode(packet_bytes)

			if packet.header == BaseTypes.REM_PACKET_TYPE_REM_LOG:
				# Get the message from the buffer. The message is everything after the REM_Packet header
				message = packet_bytes[BaseTypes.REM_PACKET_SIZE_REM_LOG:]
				# Convert bytes into string, and store in REM_Log object
				packet.message = message.decode()
			
			if timestamp_parser_ms is not None:
				packet.timestamp_parser_ms = timestamp_parser_ms

			# Add packet to buffer
			self.addPacket(packet)
			if DEBUG: print(f"- Added packet type={type(packet)}")
			# Write bytes to output file
			self.writeBytes(packet_bytes)
			# Remove processed bytes from buffer
			self.byte_buffer = self.byte_buffer[packet.payloadSize:]

	def addPacket(self, packet):
		self.packet_buffer.append(packet)
		
	def writeBytes(self, _bytes):
		if self.output_file is not None:
			time_ms_bytes = int(time.time()*1000).to_bytes(8, 'little')
			self.output_file.write(time_ms_bytes)
			self.output_file.write(_bytes)

	def hasPackets(self):
		return 0 < len(self.packet_buffer)
	
	def getNextPacket(self):
		if self.hasPackets(): return self.packet_buffer.popleft()

	def parseFile(self, filepath, print_statistics=True):
		print(f"[REMParser] Parsing file {filepath}")
		with open(filepath, "rb") as file:
			self.byte_buffer = file.read()
			self.process(parse_file=True)

		if not print_statistics: return

		# Print file statistics

		packet_counts = {}
		packet_timestamps = {}
		
		print("   ", "PACKET TYPE".ljust(20), "COUNT", " ", "START DATE".ljust(23), " ", "STOP DATE".ljust(23), " ", "DURATION H:M:S:MS")
		for packet in self.packet_buffer:
			packet_type = type(packet)
			if packet_type not in packet_counts:
				packet_counts[packet_type] = 0
				packet_timestamps[packet_type] = {'start' : packet.timestamp_parser_ms / 1000}
			packet_counts[packet_type] += 1
			packet_timestamps[packet_type]['stop'] = packet.timestamp_parser_ms / 1000

		for packet_type in packet_counts:
			start_sec, stop_sec = packet_timestamps[packet_type].values()
			start_msec, stop_msec = start_sec % 1, stop_sec % 1
			datetime_str_start = datetime.fromtimestamp(np.floor(start_sec)).strftime("%Y-%m-%d %H:%M:%S")  + (f".{start_msec:.3f}"[2:])
			datetime_str_stop  = datetime.fromtimestamp(np.floor(stop_sec )).strftime("%Y-%m-%d %H:%M:%S")  + (f".{stop_msec :.3f}"[2:])
			duration_sec = stop_sec - start_sec
			duration_str = str(timedelta(seconds=duration_sec))[:-3]

			print("   ", packet_type.__name__.ljust(20), str(packet_counts[packet_type]).rjust(5), " ", datetime_str_start, " ", datetime_str_stop, " ", duration_str)

if __name__ == "__main__":
	print("Running REMParser directly")

	argparser = argparse.ArgumentParser()
	argparser.add_argument('input_file', help='File to parse')
	args = argparser.parse_args()

	print("Parsing file", args.input_file)

	parser = REMParser(device=None)
	parser.parseFile(args.input_file)

	packet_dicts = []
	for packet in parser.packet_buffer:
		if type(packet) in [REM_RobotFeedback]:
			packet_dict = utils.packetToDict(packet)
			packet_dicts.append(packet_dict)

	# Split up packets into types
	packets_by_type = {}
	for packet in parser.packet_buffer:
		type_str = type(packet).__name__
		if type_str not in packets_by_type:
			packets_by_type[type_str] = []
		packets_by_type[type_str].append(utils.packetToDict(packet))

	output_file_no_ext = os.path.splitext(args.input_file)[0]

	for type_str in packets_by_type:
		packets = packets_by_type[type_str]
		
		# json
		output_file_json = f"{output_file_no_ext}_{type_str}.json"
		with open(output_file_json, 'w') as file:
			file.write(json.dumps(packets))

		# CSV
		output_file_csv = f"{output_file_no_ext}_{type_str}.csv"
		with open(output_file_csv, 'w') as file:
			header = ",".join(list(packets[0].keys()))
			file.write(header + "\n")
			for packet in packets:
				values = list(packet.values())
				string = ",".join([str(v) for v in values])
				file.write(string + "\n")

	print("Done!")









