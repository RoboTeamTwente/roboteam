//
// Created by Dawid Kulikowski on 04/10/2021.
//

#ifndef RTT_INTERFACESYNCEDTEXT_H
#define RTT_INTERFACESYNCEDTEXT_H
#include <QLineEdit>
#include "InterfaceDeclaration.h"
#include "InterfaceController.h"
#include "MainWindow.h"

namespace rtt::Interface {
    class InterfaceSyncedText: public QLineEdit {
        Q_OBJECT
    public:
        InterfaceSyncedText(const MainWindow*, std::shared_ptr<InterfaceController>, const InterfaceDeclaration&, QWidget* = nullptr);
    protected:
        std::string identity;

        std::weak_ptr<InterfaceController> ctrl;
    protected slots:
        void updateDeclaration(std::weak_ptr<InterfaceDeclarations>);
        void updateValue(std::weak_ptr<InterfaceSettings>);

        void notifyChangedValue(const QString& text);

        void updateProps(const InterfaceDeclaration& decl);

    };
}


#endif  // RTT_INTERFACESYNCEDTEXT_H
