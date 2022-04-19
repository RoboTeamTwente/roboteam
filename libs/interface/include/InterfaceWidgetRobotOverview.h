//
// Created by Dawid Kulikowski on 15/03/2022.
//

#ifndef RTT_INTERFACEWIDGETROBOTOVERVIEW_H
#define RTT_INTERFACEWIDGETROBOTOVERVIEW_H
#include <QWidget>
#include <QHBoxLayout>
#include <QScrollArea>
#include <MainWindow.h>
#include <QLabel>
#include <proto/WorldRobot.pb.h>
#include "InterfaceWidgetRobotDisplay.h"
#include "InterfaceControllerClient.h"

namespace rtt::Interface {
    class InterfaceWidgetRobotOverview: public QWidget {
        Q_OBJECT
    public:
        InterfaceWidgetRobotOverview(std::weak_ptr<InterfaceControllerClient>, QWidget* = nullptr);

    private:
        std::weak_ptr<InterfaceControllerClient> controller;

        std::map<int, InterfaceWidgetRobotDisplay*> displays;
        QWidget *buildWidgetForRobot(const proto::WorldRobot&);
    protected slots:
        void doUpdate();
    };
}



#endif  // RTT_INTERFACEWIDGETROBOTOVERVIEW_H
