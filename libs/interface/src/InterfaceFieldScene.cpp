//
// Created by Dawid Kulikowski on 01/12/2021.
//

#include "InterfaceFieldScene.h"
#include <QGraphicsScene>
#include <fstream>
#include <proto/State.pb.h>

InterfaceFieldScene::InterfaceFieldScene(std::weak_ptr<InterfaceFieldStateStore> state, QObject* parent): QGraphicsScene(parent), state(state), ball(new InterfaceBallItem()) {
    this->timer = new QTimer(this);
// TODO: Centralize timer
    this->addItem(this->ball);

    QObject::connect(timer, &QTimer::timeout, this, &InterfaceFieldScene::triggerUpdate);
    QObject::connect(this, &QGraphicsScene::sceneRectChanged, this, &InterfaceFieldScene::triggerUpdate);

    timer->setInterval(20);
    timer->start();
}

void InterfaceFieldScene::triggerUpdate() {
    if (auto stateHolder = this->state.lock()) {
        auto currentFieldState = stateHolder->getState();

        for (const auto& robot : currentFieldState.last_seen_world().yellow()) {
            doUpdateRobot(robot, true);
        }

        for (const auto& robot : currentFieldState.last_seen_world().blue()) {
            doUpdateRobot(robot, false);
        }

        for (const auto& robot : this->robots) {
            robot->triggerUpdate(currentFieldState);
        }

        this->ball->trigger_update(currentFieldState);
    }
}

void InterfaceFieldScene::doUpdateRobot(const proto::WorldRobot& robot, bool isYellow) {

    auto hasRobot = std::find_if(this->robots.begin(), this->robots.end(), [&] (const auto& itm) {
                        return itm->getRobotId() == robot.id() && itm->getIsYellow() == isYellow;
                    }) != this->robots.end();

    if (!hasRobot) {
        auto newRobot = new InterfaceRobotItem((int)(robot.id()), isYellow);
        newRobot->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        this->addItem(newRobot);
        this->robots.push_back(newRobot);
    }


}