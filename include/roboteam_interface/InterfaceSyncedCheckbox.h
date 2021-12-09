//
// Created by Dawid Kulikowski on 03/10/2021.
//

#ifndef RTT_INTERFACESYNCEDCHECKBOX_H
#define RTT_INTERFACESYNCEDCHECKBOX_H
#include "InterfaceDeclaration.h"
#include "MainWindow.h"
#include "InterfaceController.h"
#include <QWidget>
#include <QCheckBox>

namespace rtt::Interface {
    class InterfaceSyncedCheckbox: public QCheckBox {
        Q_OBJECT
    public:
        InterfaceSyncedCheckbox(const MainWindow*, std::shared_ptr<InterfaceController>, const InterfaceDeclaration&, QWidget* = nullptr);
    protected:
        std::string identity;

        std::weak_ptr<InterfaceController> ctrl;
        void updateProps(const InterfaceDeclaration&);
    protected slots:
        void updateDeclaration(std::weak_ptr<InterfaceDeclarations>);
        void updateValue(std::weak_ptr<InterfaceSettings>);

        void notifyChangedValue(int state);

    };
}

#endif  // RTT_INTERFACESYNCEDCHECKBOX_H
