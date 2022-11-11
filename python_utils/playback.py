import argparse
import copy
from collections import deque
from datetime import datetime, timedelta
import json
import numpy as np
import os
import time 

import utils
from REMParser import REMParser
import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_Packet import REM_Packet
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from roboteam_embedded_messages.python.REM_Log import REM_Log

if __name__ == "__main__":
	print("Running REMParser directly")

	argparser = argparse.ArgumentParser()
	argparser.add_argument('input_file', help='File to parse')
	args = argparser.parse_args()

	print("Parsing file", args.input_file)

	parser = REMParser(device=None)
	parser.parseFile(args.input_file)

	# Get all RobotCommands
	rcs = [ packet for packet in parser.packet_buffer if type(packet) == BaseTypes.REM_RobotCommand ]
	t_start, t_stop = rcs[0].timestamp_parser_ms, rcs[-1].timestamp_parser_ms
	print(t_start, t_stop)

	t_now = time.time() + 1

	for command in rcs:
		command.timestamp_parser_ms = (command.timestamp_parser_ms - t_start)/1000 + t_now
	
	serial = utils.openContinuous(timeout=0.001)

	rc_index = 0
	while True:
		time.sleep(0.001)
		t_now = time.time()
		if rcs[rc_index].timestamp_parser_ms < t_now:
			# print(rc_index)
			rc_index += 1
			serial.write(rcs[rc_index].encode())