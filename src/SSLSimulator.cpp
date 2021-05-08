//
// Created by rolf on 01-05-21.
//

#include "SSLSimulator.h"
#include <roboteam_proto/ssl_simulation_robot_control.pb.h>
#include <roboteam_proto/ssl_simulation_robot_feedback.pb.h>



proto::sim::RobotCommand SSLSimulator::convert_command(const proto::RobotCommand &command) const {
    proto::sim::RobotCommand sim_command;
    sim_command.set_id(command.id());


    float kick_vel = command.chipper() || command.kicker() ?  command.chip_kick_vel() : 0.0f;
    float kick_angle = command.chipper() ? 45.0f : 0.0f;
    sim_command.set_kick_speed(kick_vel); //kick speed in m/s
    sim_command.set_kick_angle(kick_angle); //TODO: what is the normal kick angle of our robots?

    sim_command.set_dribbler_speed(command.dribbler());//TODO: find suitable conversion (especially for GrSim)

    proto::sim::RobotMoveCommand move_command;
    auto *  global_vel = move_command.mutable_global_velocity();
    global_vel->set_x(command.vel().x());
    global_vel->set_y(command.vel().y());
    global_vel->set_angular(command.w()); //TODO: fix w in this case!!

    sim_command.mutable_move_command()->CopyFrom(move_command);

    return sim_command;
}

void SSLSimulator::set_ip(std::string _ip) {
    ip = _ip;

}

void SSLSimulator::set_color(bool _is_yellow) {
    is_yellow = _is_yellow;
    port = is_yellow ? 10302 : 10301; //TODO: hardcoded, make it settable?
}

void SSLSimulator::add_robot_command(const proto::RobotCommand &command) {
    commands.push_back(convert_command(command));
}

void SSLSimulator::send_commands() {

    proto::sim::RobotControl packet;
    for(const auto& command : commands){
        packet.mutable_robot_commands()->Add()->CopyFrom(command);
    }
    QByteArray datagram;
    datagram.resize(packet.ByteSizeLong());
    packet.SerializeToArray(datagram.data(),datagram.size());

    socket.writeDatagram(datagram,QHostAddress(QString::fromStdString(ip)),port);

}


