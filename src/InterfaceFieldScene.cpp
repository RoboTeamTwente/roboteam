//
// Created by Dawid Kulikowski on 01/12/2021.
//

#include "InterfaceFieldScene.h"
#include <fstream>
#include <roboteam_proto/State.pb.h>
void InterfaceFieldScene::drawBackground(QPainter *painter, const QRectF &rect) {
    if (auto availableState = this->state.lock()) {
        this->renderer.renderField(painter, availableState->getState(), rect.toRect());
    }
}
InterfaceFieldScene::InterfaceFieldScene(std::weak_ptr<InterfaceFieldStateStore> state, QWidget* parent): QGraphicsView(parent), state(state) {
    QObject::connect(&timer, &QTimer::timeout, this, &InterfaceFieldScene::triggerUpdate);

    timer.setInterval(30);
    timer.start();
}

void InterfaceFieldScene::triggerUpdate() {
    if (auto stateHolder = this->state.lock()) {
        auto currentFieldState = stateHolder->getState().last_seen_world();

        for (const auto& robot : currentFieldState.yellow()) {
            doUpdateRobot(robot, true);
        }

        for (const auto& robot : currentFieldState.blue()) {
            doUpdateRobot(robot, false);
        }
    }

    for (const auto& robot : this->robots) {
        robot->triggerUpdate();
    }
}

void InterfaceFieldScene::doUpdateRobot(proto::WorldRobot robot, bool isYellow) {
    if (!this->scene()) {
        return;
    }

    auto hasRobot = std::find_if(this->robots.begin(), this->robots.end(), [&] (const auto& itm) {
                        return itm->getRobotId() == robot.id() && itm->getIsYellow() == isYellow;
                    }) != this->robots.end();

    if (!hasRobot) {
        auto newRobot = new InterfaceRobotItem(this->state, (int)(robot.id()), isYellow);
        this->scene()->addItem(newRobot);
        this->robots.push_back(newRobot);
    }


}
void InterfaceFieldScene::resizeEvent(QResizeEvent* event) {
    if (scene()) {
        scene()->setSceneRect(viewport()->rect());
    }

    QGraphicsView::resizeEvent(event);
}
