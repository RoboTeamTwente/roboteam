//
// Created by Lukas Bos on 29/08/2019.
//

#include "RobotHub.h"

int main(int argc, char *argv[]) {
  rtt::robothub::RobotHub app;
  app.loop();
  return 0;
}