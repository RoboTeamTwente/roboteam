//
// Created by Dawid Kulikowski on 15/03/2022.
//

#include "InterfaceWidgetRobotDisplay.h"

#include <QVBoxLayout>

namespace rtt::Interface {
    InterfaceWidgetRobotDisplay::InterfaceWidgetRobotDisplay(QWidget *parent): QWidget(parent), data(new QLabel) {
        data->setText("null\nnull\nnull");

        QVBoxLayout* layout = new QVBoxLayout;
        QLabel* placeholder = new QLabel;

        placeholder->setWordWrap(true);
        placeholder->setAlignment(Qt::AlignCenter);
        placeholder->setText("a\nb\nc\n");

        data->setWordWrap(true);
        data->setAlignment(Qt::AlignCenter);

        layout->addWidget(placeholder);
        layout->addWidget(data);

        this->setLayout(layout);
    }

    void InterfaceWidgetRobotDisplay::refreshData(const proto::WorldRobot &robot) {
        data->setText(QString::fromStdString(std::to_string(robot.id()) + "\n" + "(" + std::to_string(robot.pos().x()) + "," + std::to_string(robot.pos().x()) + ")" + "\n" + "W: " + std::to_string(robot.w())));
    }
}