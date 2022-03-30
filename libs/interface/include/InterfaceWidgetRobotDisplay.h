//
// Created by Dawid Kulikowski on 15/03/2022.
//

#ifndef RTT_INTERFACEWIDGETROBOTDISPLAY_H
#define RTT_INTERFACEWIDGETROBOTDISPLAY_H

#include <QWidget>
#include <QLabel>
#include <proto/WorldRobot.pb.h>

namespace rtt::Interface {
    class InterfaceWidgetRobotDisplay: public QWidget {
        Q_OBJECT
    public:
        InterfaceWidgetRobotDisplay(QWidget* parent = nullptr);
        void refreshData(const proto::WorldRobot&);
    private:
        QLabel *data;
    };
}


#endif  // RTT_INTERFACEWIDGETROBOTDISPLAY_H
