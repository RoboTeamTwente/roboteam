#ifndef NAVSIM_H_
#define NAVSIM_H_

#include <string>
#include <sstream>

const char RESULT_SUCCESS 		= 0b00000000;
const char RESULT_CLOSE			= 0b00000001;
const char RESULT_VELOCITY_DISREGARD 	= 0b00000010;
const char RESULT_ORIENTATION_DISREGARD = 0b00000100;
const char RESULT_FAIL_GOAL_OCCUPIED 	= 0b00001000;
const char RESULT_FAIL_NO_PATH		= 0b00010000;
const char RESULT_FAIL_JAVA_EXCEPTION	= 0b00100000;
const char RESULT_FAIL_UNKOWN		= 0b01000000;
const char RESULT_FAIL			= 0b10000000;

std::string describe_flags(char flags) {
	std::ostringstream ss;
	if (flags & RESULT_FAIL) {
		ss << "The pathfinding failed: ";
		if (flags & RESULT_FAIL_GOAL_OCCUPIED) {
			ss << "<The goal is occupied>";
		}
		if (flags & RESULT_FAIL_NO_PATH) {
			ss << "<No valid path exists between start and goal>";
		}
		if (flags & RESULT_FAIL_JAVA_EXCEPTION) {
			ss << "<An exception was thrown from Java code>";
		}
		if (flags & RESULT_FAIL_UNKOWN) {
			ss << "<Unkown cause>";
		}
		ss << "\n";
	} else {
		ss << "The pathfinding succeeded.\n";
		if (flags & RESULT_CLOSE) {
			ss << "The start position is very close to the goal.\n";
		}	
		if (flags & RESULT_ORIENTATION_DISREGARD) {
			ss << "The resulting path disregards the requested goal orientation.\n";
		}
		if (flags & RESULT_VELOCITY_DISREGARD) {
			ss << "The resulting path disregards the requested goal velocity.\n";
		}		
	}
	return ss.str();
}

#endif
