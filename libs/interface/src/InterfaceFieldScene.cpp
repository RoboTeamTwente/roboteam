//
// Created by Dawid Kulikowski on 01/12/2021.
//

#include "InterfaceFieldScene.h"
#include <QGraphicsScene>
#include <fstream>
#include <proto/State.pb.h>
#include <proto/AIData.pb.h>

namespace rtt::Interface {
    InterfaceFieldScene::InterfaceFieldScene(std::map<rtt::Team, std::weak_ptr<MessageCache<proto::AIData>>> paths, std::weak_ptr<MessageCache<proto::State>> state, QObject* parent):
          QGraphicsScene(parent),
          renderer(state),
          state(state),
          ball(new InterfaceBallItem()),
          pathsData(paths)
    {
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
            auto currentFieldState = stateHolder->getMessage();

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
                robot->updateScale(currentFieldState->field().field().field_length(), currentFieldState->field().field().field_width());
                robot->triggerUpdate(*currentFieldState);
            }

            for (const auto& p : this->paths) {
                p->updateScale(currentFieldState->field().field().field_length(), currentFieldState->field().field().field_width());
            }

            this->doUpdatePaths(this->pathsData.at(rtt::Team::YELLOW));
            this->doUpdatePaths(this->pathsData.at(rtt::Team::BLUE));


            this->ball->updateScale(currentFieldState->field().field().field_length(), currentFieldState->field().field().field_width());
            this->ball->triggerUpdate(*currentFieldState);

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

            auto robotPath = new InterfaceRobotPathItem(isYellow ? rtt::Team::YELLOW : rtt::Team::BLUE, robot.id());
            this->paths.push_back(robotPath);
            this->addItem(robotPath);
        }


    }
    void InterfaceFieldScene::doUpdatePaths(std::weak_ptr<MessageCache<proto::AIData>> currentPaths) {
        if (auto pathsStore = currentPaths.lock()) {
            auto tpaths = pathsStore.get()->getMessage();

            if (!tpaths) return;

            for (const auto& pathSet : tpaths.value().robot_paths()) {
                auto res = std::find_if(paths.begin(), paths.end(), [&] (const auto& pos) {
                    return pos->getRobotId() == pathSet.robot_id() && pos->getTeam() == rtt::Team::YELLOW;
                });

                if (res == std::end(paths)) return;
                (*res)->triggerUpdate(pathSet);
            }
        }
    }
}

