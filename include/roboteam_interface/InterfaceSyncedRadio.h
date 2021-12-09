//
// Created by Dawid Kulikowski on 04/10/2021.
//

#ifndef RTT_INTERFACESYNCEDRADIO_H
#define RTT_INTERFACESYNCEDRADIO_H
#include <QRadioButton>
#include <QButtonGroup>
#include "InterfaceDeclaration.h"
#include "InterfaceController.h"
#include "MainWindow.h"

namespace rtt::Interface {
    class InterfaceSyncedRadio: public QButtonGroup {
        Q_OBJECT
    public:
        InterfaceSyncedRadio(const MainWindow*, std::shared_ptr<InterfaceController>, const InterfaceDeclaration&, QWidget* = nullptr);
    private:
        std::string identity;
        int dpi;

        std::weak_ptr<InterfaceController> ctrl;

        void updateProps(const InterfaceDeclaration&);
    protected slots:
        void updateDeclaration(std::weak_ptr<InterfaceDeclarations>);
        void updateValue(std::weak_ptr<InterfaceSettings>);

        void notifyChangedValue(int id, bool enabled);

    };
}


#endif  // RTT_INTERFACESYNCEDRADIO_H
