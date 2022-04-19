//
// Created by Dawid Kulikowski on 03/10/2021.
//

#ifndef RTT_INTERFACESYNCEDCHECKBOX_H
#define RTT_INTERFACESYNCEDCHECKBOX_H
#include "roboteam_interface_utils/InterfaceDeclaration.h"
#include "MainWindow.h"
#include "InterfaceControllerClient.h"
#include <QWidget>
#include <QCheckBox>

namespace rtt::Interface {
    class InterfaceSyncedCheckbox: public QCheckBox {
        Q_OBJECT
    public:
        InterfaceSyncedCheckbox(std::weak_ptr<InterfaceControllerClient>, std::string, QWidget* = nullptr);
    protected:
        std::string identity;

        std::weak_ptr<InterfaceControllerClient> ctrl;
        void updateProps(const InterfaceDeclaration&);
    protected slots:
        void updateDeclaration();
        void updateValue();

        void notifyChangedValue(int state);

    };
}

#endif  // RTT_INTERFACESYNCEDCHECKBOX_H
