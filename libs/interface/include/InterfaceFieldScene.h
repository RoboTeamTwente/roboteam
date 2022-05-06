//
// Created by Dawid Kulikowski on 01/12/2021.
//

#ifndef RTT_INTERFACEFIELDSCENE_H
#define RTT_INTERFACEFIELDSCENE_H
#include <QGraphicsView>
#include <QTimer>
#include <memory>

#include <roboteam_utils/AIData.hpp>

#include "proto/WorldRobot.pb.h"
#include "InterfaceFieldRenderer.h"
#include <InterfaceRobotPathsRenderer.h>
#include "InterfaceFieldStateStore.h"
#include "InterfaceRobotItem.h"
#include "InterfaceBallItem.h"

class InterfaceFieldScene: public QGraphicsScene {
    Q_OBJECT
public:
    InterfaceFieldScene(std::weak_ptr<InterfaceFieldStateStore> state, QObject* parent = nullptr);

private slots:
    void triggerUpdate();

private:
    InterfaceFieldRenderer renderer;

    std::weak_ptr<InterfaceFieldStateStore> state;
    QTimer *timer;

    std::vector<InterfaceRobotItem*> robots;
    InterfaceBallItem *ball = nullptr;
    InterfaceRobotPathItem *yellowPaths = nullptr;
    InterfaceRobotPathItem *bluePaths = nullptr;

    void doUpdateRobot(const proto::WorldRobot&, bool);
};


#endif  // RTT_INTERFACEFIELDSCENE_H
