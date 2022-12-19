//
// Created by Dawid Kulikowski on 04/10/2021.
//

#ifndef RTT_INTERFACESYNCEDSLIDER_H
#define RTT_INTERFACESYNCEDSLIDER_H
#include <QSlider>
#include "roboteam_interface_utils/InterfaceDeclaration.h"
#include "InterfaceControllerClient.h"
#include "MainWindow.h"

namespace rtt::Interface {
    class InterfaceSyncedSlider: public QSlider {
        Q_OBJECT
    public:
        InterfaceSyncedSlider(std::weak_ptr<InterfaceControllerClient>, std::string, QWidget* = nullptr);
    private:
        std::string identity;
        int dpi;

        std::weak_ptr<InterfaceControllerClient> ctrl;

        void updateProps(const InterfaceDeclaration&);
    protected slots:
        void updateDeclaration();
        void updateValue();

        void notifyChangedValue(int value);

    };
}


#endif  // RTT_INTERFACESYNCEDSLIDER_H
