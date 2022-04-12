//
// Created by Dawid Kulikowski on 04/10/2021.
//

#ifndef RTT_INTERFACESYNCEDDROPDOWN_H
#define RTT_INTERFACESYNCEDDROPDOWN_H
#include <QComboBox>
#include "roboteam_interface_utils/InterfaceDeclaration.h"
#include "InterfaceControllerClient.h"
#include "MainWindow.h"

namespace rtt::Interface {
    class InterfaceSyncedDropdown: public QComboBox {
        Q_OBJECT
    public:
        InterfaceSyncedDropdown(std::weak_ptr<InterfaceControllerClient>, std::string, QWidget* = nullptr);
    protected:
        std::string identity;

        std::weak_ptr<InterfaceControllerClient> ctrl;
        void updateProps(const InterfaceDeclaration&);
    private slots:
        void didChangeValue(const QString&);
        void updateDeclaration();
        void updateValue();

    };
}

#endif  // RTT_INTERFACESYNCEDDROPDOWN_H
