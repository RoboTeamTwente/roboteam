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
        InterfaceSyncedDropdown(const MainWindow*, std::shared_ptr<InterfaceControllerClient>, const InterfaceDeclaration&, QWidget* = nullptr);
    protected:
        std::string identity;

        std::weak_ptr<InterfaceControllerClient> ctrl;
        void updateProps(const InterfaceDeclaration&);
    public slots:
        void didChangeValue(const QString&);
        void updateDeclaration(std::weak_ptr<InterfaceDeclarations>);
        void updateValue(std::weak_ptr<InterfaceSettings>);

    };
}

#endif  // RTT_INTERFACESYNCEDDROPDOWN_H
