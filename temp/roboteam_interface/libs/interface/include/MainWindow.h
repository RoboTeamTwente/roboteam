//
// Created by Dawid Kulikowski on 19/08/2021.
//

#ifndef RTT_MAINWINDOW_H
#define RTT_MAINWINDOW_H

#include <QMainWindow>

#include "InterfaceControllerClient.h"
#include "roboteam_interface_utils/InterfaceDeclarations.h"
#include "roboteam_interface_utils/InterfaceSettings.h"
#include <InterfaceControllerClient.h>


namespace rtt::Interface {
    class MainWindow: public QMainWindow {
        Q_OBJECT
    private:
        std::shared_ptr<InterfaceControllerClient> interfaceController;
        bool initial_render = true;
        QWidget* side_panel;

    public slots:
        void conditionallyEmitDeclsChanged();
        void conditionallyEmitValuesChanged();


    public:
        explicit MainWindow(std::shared_ptr<InterfaceControllerClient>, QWidget* parent = nullptr);

    signals:
        void declarationsChanged(std::weak_ptr<InterfaceDeclarations>);
        void valuesChanged(std::weak_ptr<InterfaceSettings>);
        void updateField();

    };
}

#endif  // RTT_MAINWINDOW_H
