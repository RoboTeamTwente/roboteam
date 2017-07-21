#include <ros/ros.h>
#include "roboteam_msgs/RobotCommand.h"

int main(int argc, char *argv[]) {
    auto lawineCount = 150;
    if (argc > 1) {
        lawineCount = std::stoi(std::string(argv[1]));
    }
    // Create ROS node called robothub and subscribe to topic 'robotcommands'
    ros::init(argc, argv, "barrage");
    ros::NodeHandle n;

    auto pub = n.advertise<roboteam_msgs::RobotCommand>("robotcommands", 10000);

    ros::Rate fps60(20);

    std::cout << "Lawine count: " << lawineCount << "\n";

    while (ros::ok()) {
        fps60.sleep();
        ros::spinOnce();

        roboteam_msgs::RobotCommand command;
        for (int i = 0; i < lawineCount; i++) {
            pub.publish(command);
        }
    }

    return 0;
}

