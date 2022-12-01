//
// Created by Dawid Kulikowski on 03/10/2021.
//

#ifndef RTT_INTERFACEFIELDWIDGET_H
#define RTT_INTERFACEFIELDWIDGET_H
#include <QWidget>

namespace rtt::Interface {
    class InterfaceFieldWidget: public QWidget {
        Q_OBJECT
    public:
        InterfaceFieldWidget(QWidget* parent = nullptr);
    };
}


#endif  // RTT_INTERFACEFIELDWIDGET_H
