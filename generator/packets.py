packets = {
	"RobotCommand" : [
		["uint8_t", 	"header", 				8,	"Header indicating packet type"],
		["uint8_t", 	"id", 					4,	"Id of the robot"],
		["bool", 		"doKick", 				1,	"Do a kick if ballsensor"],
		["bool", 		"doChip", 				1,	"Do a chip if ballsensor"],
		["bool", 		"doForce", 				1,	"Do regardless of ballsensor"],
		["bool", 		"useCameraAngle", 		1,	"Use the info in 'cameraAngle'"],
		["uint16_t",	"rho", 					16,	"Direction of movement"],
		["uint16_t",	"theta", 				16,	"Magnitude of movement (speed)"],
		["uint16_t",	"angle", 				16,	"Absolute angle / angular velocity"],
		["uint16_t",	"cameraAngle", 			16,	"Angle of the robot as seen by camera"],
		["uint8_t", 	"dribbler", 			3,	"Dribbler speed"],
		["uint8_t", 	"kickChipPower", 		3,	"Power of the kick or chip"],
		["bool", 		"angularControl", 		1,	"0 = angular velocity, 1 = absolute angle"],
		["bool", 		"feedback", 			1,	"Ignore the packet. Just send feedback"],
	],
	"RobotFeedback" : [
		["uint8_t",		"header",          		8,	"Header byte indicating the type of packet"],
		["uint8_t",		"id",               	4,  "Id of the robot "],
		["uint8_t",		"battery_level",       	4,  "The voltage level of the battery"],
		["bool", 		"xsens_calibrated",  	1,  "Indicates if the XSens IMU is calibrated"],
		["bool",		"ballsensor_working", 	1,  "Indicates if the ballsensor is working"],
		["bool",		"has_ball",            	1,  "Indicates if the ball is somewhere in front of the ballsensor"],
		["bool",		"capacitor_charged", 	1,  "Indicates if the capacitor for kicking and chipping is charged"],
		["uint8_t",		"ball_position",      	4,  "Indicates where in front of the ballsensor the ball is"],
		["uint16_t",	"rho",               	16, "The estimated direction of movement"],
		["uint16_t",	"theta",            	16, "The estimated magnitude of movement (speed)"],
		["uint16_t",	"angle",          	  	16, "The estimated angle"],
		["uint8_t",		"wheel_locked",     	4,  "Indicates if a wheel is locked. One bit per wheel"],
		["uint8_t",		"wheel_slipping",    	4,  "Indicates if a wheel is slipping. One bit per wheel"],
		["uint8_t",		"rssi",            		4,  "Signal strength of the last packet received by the robot"]
	]
}