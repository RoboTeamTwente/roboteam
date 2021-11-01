//
// Created by rolf on 01-05-21.
//

#include "SSLSimulator.h"
#include <roboteam_proto/ssl_simulation_robot_control.pb.h>
#include <roboteam_proto/ssl_simulation_robot_feedback.pb.h>
#include <roboteam_proto/World.pb.h>
#include "utilities.h"
#include <cmath>


proto::sim::RobotCommand SSLSimulator::convert_command(const proto::RobotCommand &command,const proto::World& world) const {
    proto::sim::RobotCommand sim_command;
    sim_command.set_id(command.id());


    float kick_vel = command.chipper() || command.kicker() ?  command.chip_kick_vel() : 0.0f;
    float kick_angle = command.chipper() ? 45.0f : 0.0f;
    sim_command.set_kick_speed(kick_vel); //kick speed in m/s
    sim_command.set_kick_angle(kick_angle);

    if(command.dribbler()>0){
        sim_command.set_dribbler_speed(1021);//Theoretical max rpm according to alejandro, probably a bit quick..
    }else{
        sim_command.set_dribbler_speed(0.0);
    }
    proto::sim::RobotMoveCommand move_command;

    //below is for if erforce decides not to support global velocity. Above works fine in grsim... >.<
    {
        float robot_angle = 0.0;
        std::shared_ptr<proto::WorldRobot> findBot = rtt::robothub::utils::getWorldBot(command.id(),is_yellow, world);
        if(findBot){
            robot_angle = - findBot->angle();
        }else{
            std::cout<<"could not find robot!"<<std::endl;
        }
        float forward = cosf(robot_angle) * command.vel().x() - sinf(robot_angle) * command.vel().y();
        float left = sinf(robot_angle) * command.vel().x() + cosf(robot_angle) * command.vel().y();
        auto * local_vel = move_command.mutable_local_velocity();
        local_vel->set_forward(forward);
        local_vel->set_left(left);
        local_vel->set_angular(command.w());

    }

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

void SSLSimulator::add_robot_command(const proto::RobotCommand &command, const proto::World& world) {
    commands.push_back(convert_command(command,world));
}

void SSLSimulator::send_commands() {

    proto::sim::RobotControl packet;
    for(const auto& command : commands){
        packet.mutable_robot_commands()->Add()->CopyFrom(command);
    }
    QByteArray datagram;
    datagram.resize(packet.ByteSizeLong());
    packet.SerializeToArray(datagram.data(),datagram.size());

    auto bytes_sent = socket.writeDatagram(datagram,QHostAddress(QString::fromStdString(ip)),port);


    commands.clear();
}


