//
// Created by Dawid Kulikowski on 27/03/2022.
//

#ifndef RTT_INTERFACESYNCEDCHECKABLEBUTTON_H
#define RTT_INTERFACESYNCEDCHECKABLEBUTTON_H

#include <QPushButton>
#include "MainWindow.h"
#include "InterfaceControllerClient.h"

namespace rtt::Interface {
    class InterfaceSyncedCheckableButton: public QPushButton {
        Q_OBJECT
    public:
        InterfaceSyncedCheckableButton(std::weak_ptr<InterfaceControllerClient>, std::string, QWidget* = nullptr);

    protected:
        std::string identity;
        int option_idx = 0;

        std::weak_ptr<InterfaceControllerClient> ctrl;
    public slots:
        void updateValue();
    private slots:
        void didCheck(bool);
    };
}



#endif  // RTT_INTERFACESYNCEDCHECKABLEBUTTON_H
