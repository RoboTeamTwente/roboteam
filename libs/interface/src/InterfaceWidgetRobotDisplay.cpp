//
// Created by Dawid Kulikowski on 15/03/2022.
//

#include "InterfaceWidgetRobotDisplay.h"

#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>

namespace rtt::Interface {
    InterfaceWidgetRobotDisplay::InterfaceWidgetRobotDisplay(QWidget *parent): QFrame(parent), data(new QLabel) {
        data->setText("null\nnull\nnull");

        QVBoxLayout* layout = new QVBoxLayout;
        QLabel* placeholder = new QLabel;
        QPixmap pixmap(80, 80);
        pixmap.fill(Qt::transparent);
        QPainter painter(&pixmap);
        this->setFrameStyle(QFrame::StyledPanel | QFrame::Raised);
        this->setLineWidth(2);
        this->setMidLineWidth(0);
//        this->setFrameShadow(Shadow::Raised);

        QBrush brush({112,39,195});
        painter.setBrush(brush);

        painter.drawEllipse({40, 40}, 40, 40);

        painter.drawText(40, 40, "T");

        placeholder->setPixmap(pixmap);

//        placeholder->setWordWrap(true);
        placeholder->setAlignment(Qt::AlignCenter);
//        placeholder->setText("a\nb\nc\n");

        data->setWordWrap(true);
        data->setAlignment(Qt::AlignCenter);

        layout->addWidget(placeholder);
        layout->addWidget(data);

        this->setLayout(layout);
    }

    void InterfaceWidgetRobotDisplay::refreshData(const proto::WorldRobot &robot) {
        data->setText(QString::fromStdString(std::to_string(robot.id()) + "\n" + "(" + std::to_string(robot.pos().x()) + "," + std::to_string(robot.pos().x()) + ")" + "\n" + "W: " + std::to_string(robot.w())));
        data->setFixedWidth(150);
    }
}