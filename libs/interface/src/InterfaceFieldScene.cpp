//
// Created by Dawid Kulikowski on 01/12/2021.
//

#include "InterfaceFieldScene.h"
#include <QGraphicsScene>
#include <fstream>
#include <proto/State.pb.h>

InterfaceFieldScene::InterfaceFieldScene(std::weak_ptr<InterfaceFieldStateStore> state, QObject* parent):
      QGraphicsScene(parent),
      renderer(state),
      state(state),
      ball(new InterfaceBallItem()),
      yellowPaths(new InterfaceRobotPathItem(rtt::Team::YELLOW)),
      bluePaths(new InterfaceRobotPathItem(rtt::Team::BLUE)) {
    this->timer = new QTimer(this);
// TODO: Centralize timer
    this->addItem(this->ball);
    this->addItem(this->yellowPaths);
    this->addItem(this->bluePaths);

    QObject::connect(timer, &QTimer::timeout, this, &InterfaceFieldScene::triggerUpdate);
    QObject::connect(this, &QGraphicsScene::sceneRectChanged, this, &InterfaceFieldScene::triggerUpdate);

    timer->setInterval(20);
    timer->start();
}

void InterfaceFieldScene::triggerUpdate() {
    if (auto stateHolder = this->state.lock()) {
        auto currentFieldState = stateHolder->getState();

        if (!currentFieldState.has_value()) {
            return;
        }

        for (const auto& robot : currentFieldState->last_seen_world().yellow()) {
            doUpdateRobot(robot, true);
        }

        for (const auto& robot : currentFieldState->last_seen_world().blue()) {
            doUpdateRobot(robot, false);
        }

        for (const auto& robot : this->robots) {
            robot->updateScale(currentFieldState.field().field().field_length(), currentFieldState.field().field().field_width());
            robot->triggerUpdate(*currentFieldState);
        }

        this->ball->updateScale(currentFieldState.field().field().field_length(), currentFieldState.field().field().field_width());
        this->ball->trigger_update(*currentFieldState);

        this->yellowPaths->updateScale(currentFieldState.field().field().field_length(), currentFieldState.field().field().field_width());
        this->yellowPaths->triggerUpdate(stateHolder->getAIData(rtt::Team::YELLOW).robotPaths);
        this->bluePaths->updateScale(currentFieldState.field().field().field_length(), currentFieldState.field().field().field_width());
        this->bluePaths->triggerUpdate(stateHolder->getAIData(rtt::Team::BLUE).robotPaths);
    }
}

void InterfaceFieldScene::doUpdateRobot(const proto::WorldRobot& robot, bool isYellow) {

    auto hasRobot = std::find_if(this->robots.begin(), this->robots.end(), [&] (const auto& itm) {
                        return itm->getRobotId() == robot.id() && itm->getIsYellow() == isYellow;
                    }) != this->robots.end();

    if (!hasRobot) {
        auto newRobot = new InterfaceRobotItem((int)(robot.id()), isYellow, this->state);
        newRobot->setCacheMode(QGraphicsItem::DeviceCoordinateCache);
        this->addItem(newRobot);
        this->robots.push_back(newRobot);
    }


}