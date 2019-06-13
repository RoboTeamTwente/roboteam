#include "ros/ros.h"
#include "Application.h"

int main(int argc, char *argv[]) {
	ros::init(argc, argv, "robothub");
	rtt::robothub::Application app;
	app.loop();
	return 0;
}