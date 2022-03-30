//
// Created by Dawid Kulikowski on 15/03/2022.
//

#ifndef RTT_INTERFACEWIDGETSTOPBUTTON_H
#define RTT_INTERFACEWIDGETSTOPBUTTON_H

#include <QObject>
#include <QPushButton>
#include "InterfaceControllerClient.h"

namespace rtt::Interface {
    class InterfaceWidgetStopButton: public QPushButton {
        Q_OBJECT

    public:
        InterfaceWidgetStopButton(std::weak_ptr<InterfaceControllerClient>, QWidget* = nullptr);

    private:
        std::weak_ptr<InterfaceControllerClient> controller;
    private slots:
        void stopAI(bool);

    };
}



#endif  // RTT_INTERFACEWIDGETSTOPBUTTON_H
