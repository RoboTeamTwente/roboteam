//
// Created by Dawid Kulikowski on 27/03/2022.
//

#ifndef RTT_INTERFACEWIDGETDEBUGDISPLAY_H
#define RTT_INTERFACEWIDGETDEBUGDISPLAY_H
#include <QWidget>
#include "MainWindow.h"
#include "InterfaceControllerClient.h"


namespace rtt::Interface {
    class InterfaceWidgetDebugDisplay: public QWidget {
    public:
        InterfaceWidgetDebugDisplay(const MainWindow*, std::weak_ptr<InterfaceControllerClient>, QWidget* = nullptr);
    private:
        std::weak_ptr<InterfaceControllerClient> ctrl;
        const MainWindow* window;

        void fullRefresh(const std::vector<std::tuple<std::string, InterfaceValue>>&);
    private slots:
        void valuesDidChange();
    };
}



#endif  // RTT_INTERFACEWIDGETDEBUGDISPLAY_H
