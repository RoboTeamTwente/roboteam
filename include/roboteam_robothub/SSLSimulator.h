//
// Created by rolf on 01-05-21.
//

#ifndef RTT_SSLSIMULATOR_H
#define RTT_SSLSIMULATOR_H

#include <roboteam_proto/RobotCommand.pb.h>
#include <roboteam_proto/ssl_simulation_robot_control.pb.h>
#include <QUdpSocket>
#include <roboteam_proto/World.pb.h>

class SSLSimulator {
public:
    void set_ip(std::string ip);
    void set_color(bool is_yellow);
    void add_robot_command(const proto::RobotCommand& command, const proto::World& world);
    void send_commands();
    [[nodiscard]] proto::sim::RobotCommand convert_command(const proto::RobotCommand& command,const proto::World& world) const;
private:
    std::vector<proto::sim::RobotCommand> commands;
    QUdpSocket socket;
    std::string ip;
    quint16 port;
    bool is_yellow;

};


#endif //RTT_SSLSIMULATOR_H
