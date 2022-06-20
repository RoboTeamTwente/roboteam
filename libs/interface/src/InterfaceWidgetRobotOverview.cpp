//
// Created by Dawid Kulikowski on 15/03/2022.
//

#include "InterfaceWidgetRobotOverview.h"
#include <QScrollArea>

#include <utility>
namespace rtt::Interface {
    InterfaceWidgetRobotOverview::InterfaceWidgetRobotOverview(std::weak_ptr<InterfaceControllerClient> ctrl, QWidget *parent): QWidget(parent), controller(std::move(ctrl)) {
        QObject::connect(this->controller.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceWidgetRobotOverview::doUpdate);
        this->setLayout(new QHBoxLayout);
        this->layout()->setAlignment(Qt::AlignmentFlag::AlignCenter);
    }

    void InterfaceWidgetRobotOverview::doUpdate() {
        if (auto ctrl = this->controller.lock()) {
            auto fieldState = ctrl->getFieldState().lock();
            auto vals = ctrl->getValues().lock();

            if (!fieldState || !vals) {
                return;
            }

            auto state = fieldState->getMessage();
            if (!state.has_value()) {
                return;
            }

            auto robots = vals->getSetting("IS_YELLOW").value() == InterfaceValue(true) ? state->last_seen_world().yellow() : state->last_seen_world().blue();

            if (this->displays.size() != robots.size()) {
                while (auto item = this->layout()->takeAt(0)) {
                    delete item;
                }
            }

            for (const auto& robot: robots) {
                if (!this->displays.contains(robot.id())) {
                    this->displays.insert_or_assign(robot.id(), new InterfaceWidgetRobotDisplay());
                    this->layout()->addWidget(this->displays.at(robot.id()));
                }

                this->displays.at(robot.id())->refreshData(robot);
            }
        }
    }
}