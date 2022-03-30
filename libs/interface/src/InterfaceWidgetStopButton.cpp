//
// Created by Dawid Kulikowski on 15/03/2022.
//

#include "InterfaceWidgetStopButton.h"

namespace rtt::Interface {
    InterfaceWidgetStopButton::InterfaceWidgetStopButton(std::weak_ptr<InterfaceControllerClient> ctrl, QWidget *parent): QPushButton(parent) {
        this->controller = std::move(ctrl);

        this->setStyleSheet("background-color: #cc0000;");
        this->setMinimumHeight(40);
        this->setText("E-STOP");

        QObject::connect(this, &QPushButton::clicked, this, &InterfaceWidgetStopButton::stopAI);
    }

    void InterfaceWidgetStopButton::stopAI(bool checked) {
        this->setEnabled(false);

        if (auto ctrl = this->controller.lock()) {
            if (auto vals = ctrl->getValues().lock()) {
                vals->setSetting("ESTOP", true);
            }
        }
    }
}
