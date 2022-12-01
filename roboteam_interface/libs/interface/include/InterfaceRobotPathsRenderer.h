#pragma once

#include <QGraphicsItem>
#include <QGraphicsPathItem>
#include <InterfaceFieldStateStore.h>
#include "InterfaceFieldStateStore.h"
#include <proto/AIData.pb.h>

namespace rtt::Interface {
    class InterfaceRobotPathItem: public QGraphicsPathItem {
    public:
        InterfaceRobotPathItem(rtt::Team, int, QGraphicsItem* parent = nullptr);

        void triggerUpdate(const proto::RobotPath& path);
        void updateScale(double fieldWidth, double fieldHeight);

        rtt::Team getTeam();
        int getRobotId();


    private:
        double scale;
        int robot_id;
        rtt::Team team;

//        std::vector<rtt::RobotPath> paths;
    };
}



