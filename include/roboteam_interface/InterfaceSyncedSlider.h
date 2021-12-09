//
// Created by Dawid Kulikowski on 04/10/2021.
//

#ifndef RTT_INTERFACESYNCEDSLIDER_H
#define RTT_INTERFACESYNCEDSLIDER_H
#include <QSlider>
#include "InterfaceDeclaration.h"
#include "InterfaceController.h"
#include "MainWindow.h"

namespace rtt::Interface {
    class InterfaceSyncedSlider: public QSlider {
        Q_OBJECT
    public:
        InterfaceSyncedSlider(const MainWindow*, std::shared_ptr<InterfaceController>, const InterfaceDeclaration&, QWidget* = nullptr);
    private:
        std::string identity;
        int dpi;

        std::weak_ptr<InterfaceController> ctrl;

        void updateProps(const InterfaceDeclaration&);
    protected slots:
        void updateDeclaration(std::weak_ptr<InterfaceDeclarations>);
        void updateValue(std::weak_ptr<InterfaceSettings>);

        void notifyChangedValue(int value);

    };
}


#endif  // RTT_INTERFACESYNCEDSLIDER_H
