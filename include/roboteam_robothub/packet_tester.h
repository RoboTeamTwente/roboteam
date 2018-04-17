//
// Created by emiel on 30/03/18.
//

#ifndef ROBOTEAM_ROBOTHUB_PACKET_TESTER_H
#define ROBOTEAM_ROBOTHUB_PACKET_TESTER_H

namespace rtt {
    void printcmd(roboteam_msgs::RobotCommand cmd);
    void printllrc(LowLevelRobotCommand llrc);
    void test();
    void test2();
}

#endif //ROBOTEAM_ROBOTHUB_PACKET_TESTER_H
