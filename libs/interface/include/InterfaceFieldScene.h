//
// Created by Dawid Kulikowski on 01/12/2021.
//

#ifndef RTT_INTERFACEFIELDSCENE_H
#define RTT_INTERFACEFIELDSCENE_H
#include <InterfaceRobotPathsRenderer.h>
#include <proto/AIData.pb.h>
#include <roboteam_interface_utils/MessageCache.h>

#include <QGraphicsView>
#include <QTimer>
#include <memory>
#include <roboteam_utils/AIData.hpp>

#include "InterfaceBallItem.h"
#include "InterfaceFieldRenderer.h"
#include "InterfaceFieldStateStore.h"
#include "InterfaceRobotItem.h"
#include "proto/WorldRobot.pb.h"

namespace rtt::Interface {
    class InterfaceFieldScene: public QGraphicsScene {
        Q_OBJECT
    public:
        InterfaceFieldScene(std::map<rtt::Team, std::weak_ptr<MessageCache<proto::AIData>>>, std::weak_ptr<MessageCache<proto::State>> state, QObject* parent = nullptr);

    private slots:
        void triggerUpdate();

    private:
        InterfaceFieldRenderer renderer;

        std::map<rtt::Team, std::weak_ptr<MessageCache<proto::AIData>>> pathsData;
        std::weak_ptr<MessageCache<proto::State>> state;
        QTimer *timer;

        std::vector<InterfaceRobotItem*> robots;
        std::vector<InterfaceRobotPathItem*> paths;

        InterfaceBallItem *ball = nullptr;

        void doUpdateRobot(const proto::WorldRobot&, bool);

        void doUpdatePaths(std::weak_ptr<MessageCache<proto::AIData>>);
    };
}




#endif  // RTT_INTERFACEFIELDSCENE_H
