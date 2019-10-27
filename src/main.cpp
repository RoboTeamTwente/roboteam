//
// Created by Lukas Bos on 29/08/2019.
//

#include "RobotHub.h"

int main(int argc, char *argv[]) {

    // get the id of the ai from the init
    int id = 0;
    if (argc == 2) {
        id = *argv[1] - '0';
    }

    // this is for multiple AI support
    rtt::robothub::RobotHub app;
    if (id == 1) {
        app.set_robot_command_channel(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL);
        app.set_feedback_channel(proto::FEEDBACK_SECONDARY_CHANNEL);
        app.set_settings_channel(proto::SETTINGS_SECONDARY_CHANNEL);
    } else {
        app.set_robot_command_channel(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
        app.set_feedback_channel(proto::FEEDBACK_PRIMARY_CHANNEL);
        app.set_settings_channel(proto::SETTINGS_PRIMARY_CHANNEL);
    }
  app.subscribeToTopics();

  app.start();
  return 0;
}