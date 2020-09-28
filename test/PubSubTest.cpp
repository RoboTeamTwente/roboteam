#include <gtest/gtest.h>
#include <roboteam_proto/RobotCommand.pb.h>
#include <roboteam_utils/networking/include/Subscriber.h>
#include <roboteam_utils/networking/include/Publisher.h>
#include "roboteam_utils/Timer.h"

bool messageReceivedSuccesFully = false;
double receivedTime = 0;

void handleRobotCommand(proto::RobotCommand & robot_command) {
  EXPECT_EQ(robot_command.geneva_state(), 4);
  receivedTime = robot_command.id();
  messageReceivedSuccesFully = true;
}

TEST(PubSubTest, function_subscription) {
  auto sub = std::make_shared<proto::Subscriber<proto::RobotCommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL, &handleRobotCommand);
  auto pub = std::make_shared<proto::Publisher<proto::RobotCommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);

  proto::RobotCommand cmd;
  cmd.set_geneva_state(4);

  auto reference_time = roboteam_utils::Timer::getCurrentTime().count();
  while(!messageReceivedSuccesFully) {
    auto now = roboteam_utils::Timer::getCurrentTime().count();
    if (now > reference_time + 1000) {
      ASSERT_TRUE(false); // it takes too long. test should fail instead of deadlock
    }
    cmd.set_id(roboteam_utils::Timer::getCurrentTime().count()); // we store the time in the id
    pub->send(cmd);
  }

  // the communication should be fast (<10ms)
  EXPECT_LE( roboteam_utils::Timer::getCurrentTime().count() - receivedTime,  10);
}


TEST(PubSubTest, method_subscription) {
  struct Dummy {
    proto::RobotCommand cmd;
    std::shared_ptr<proto::Subscriber<proto::RobotCommand>> sub;
    bool got_command = false;
    int receivedTime = 0;

    Dummy() {
      const proto::Channel DUMMY_CHANNEL = {"dummy_channel", "127.0.0.1", "5555"};
      sub = std::make_shared<proto::Subscriber<proto::RobotCommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL, &Dummy::handle_message, this);
    }

    void handle_message(proto::RobotCommand & robotcommand) {
      cmd = robotcommand;
      receivedTime = robotcommand.id();
      got_command = true;
    }
  };

  Dummy dummy;
  
  auto pub = std::make_shared<proto::Publisher<proto::RobotCommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL);
  proto::RobotCommand cmd;
  cmd.set_geneva_state(2);

  auto reference_time = roboteam_utils::Timer::getCurrentTime().count();
  while(!dummy.got_command) {
    auto now = roboteam_utils::Timer::getCurrentTime().count();
    if (now > reference_time + 1000) {
      ASSERT_TRUE(false); // it takes too long. test should fail instead of deadlock
    }
    cmd.set_id(roboteam_utils::Timer::getCurrentTime().count()); // we store the time in the id
    pub->send(cmd);
  }

  // the communication should be fast (<10ms)
  EXPECT_LE( roboteam_utils::Timer::getCurrentTime().count() - dummy.receivedTime,  10);
  EXPECT_EQ(dummy.cmd.geneva_state(), 2);
}