#include <AIDataNetworker.hpp>

#include <proto/AIData.pb.h>

namespace rtt::net {

proto::RobotPath robotPathToProto(const RobotPath& path) {
    proto::RobotPath proto;
    proto.set_robot_id(path.robotId);
    for (const auto& point : path.path) {
        auto protoPoint = proto.add_points();
        protoPoint->set_x(point.x);
        protoPoint->set_y(point.y);

    }
    return proto;
}

RobotPath protoToRobotPath(const proto::RobotPath& proto) {
    RobotPath path;
    path.robotId = proto.robot_id();

    for (const auto& protoPoint : proto.points()) {
        path.path.push_back({protoPoint.x(), protoPoint.y()});
    }

    return path;
}

proto::RobotSTP robotSTPToProto(const RobotSTP& stp) {
    proto::RobotSTP proto;
    proto.set_robot_id(stp.robotId);
    proto.set_role(stp.role);
    proto.set_role_status(stp.roleStatus);
    proto.set_tactic(stp.tactic);
    proto.set_tactic_status(stp.tacticStatus);
    proto.set_skill(stp.skill);
    proto.set_skill_status(stp.skillStatus);
    return proto;
}

RobotSTP protoToRobotSTP(const proto::RobotSTP& proto) {
    RobotSTP stp = {
        .robotId = proto.robot_id(),
        .role = proto.role(),
        .roleStatus = proto.role_status(),
        .tactic = proto.tactic(),
        .tacticStatus = proto.tactic_status(),
        .skill = proto.skill(),
        .skillStatus = proto.skill_status(),
    };
    return stp;
}

proto::AIData aiDataToProto(const AIData& data) {
    proto::AIData proto;
    for (const auto& path : data.robotPaths) {
        proto.mutable_robot_paths()->Add(robotPathToProto(path));
    }
    for (const auto& stp : data.robotStps) {
        proto.mutable_robot_stps()->Add(robotSTPToProto(stp));
    }
    return proto;
}

AIData protoToAIData(const proto::AIData& proto) {
    AIData data;
    for (const auto& protoPath : proto.robot_paths()) {
        data.robotPaths.emplace_back(protoToRobotPath(protoPath));
    }
    for (const auto& protoSTP : proto.robot_stps()) {
        data.robotStps.emplace_back(protoToRobotSTP(protoSTP));
    }
    return data;
}

AIYellowDataPublisher::AIYellowDataPublisher() : utils::Publisher(utils::ChannelType::AI_YELLOW_CHANNEL) {}

bool AIYellowDataPublisher::publish(const AIData& data) {
    auto proto = aiDataToProto(data);
    return this->send(proto.SerializeAsString());
}

AIYellowDataSubscriber::AIYellowDataSubscriber(const std::function<void(const AIData&)>& callback)
    : utils::Subscriber(utils::ChannelType::AI_YELLOW_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void AIYellowDataSubscriber::onPublishedMessage(const std::string& message) {
    proto::AIData proto;
    proto.ParseFromString(message);
    this->callback(protoToAIData(proto));
}

AIBlueDataPublisher::AIBlueDataPublisher() : utils::Publisher(utils::ChannelType::AI_BLUE_CHANNEL) {}

bool AIBlueDataPublisher::publish(const AIData& data) {
    auto proto = aiDataToProto(data);
    return this->send(proto.SerializeAsString());
}

AIBlueDataSubscriber::AIBlueDataSubscriber(const std::function<void(const AIData&)>& callback)
    : utils::Subscriber(utils::ChannelType::AI_YELLOW_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void AIBlueDataSubscriber::onPublishedMessage(const std::string& message) {
    proto::AIData proto;
    proto.ParseFromString(message);
    this->callback(protoToAIData(proto));
}

}  // namespace rtt::net