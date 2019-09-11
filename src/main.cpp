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
        app.setAiPublisher(rtt::ROBOTEAM_AI_2_TCP_PUBLISHER);
        app.setRobothubPublisher(rtt::ROBOTEAM_ROBOTHUB_TCP_2_PUBLISHER);
    } else {
        app.setAiPublisher(rtt::ROBOTEAM_AI_TCP_PUBLISHER);
        app.setRobothubPublisher(rtt::ROBOTEAM_ROBOTHUB_TCP_PUBLISHER);
    }
  app.subscribeToTopics();

  app.loop();
  return 0;
}