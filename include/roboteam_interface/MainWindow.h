//
// Created by Dawid Kulikowski on 19/08/2021.
//

#ifndef RTT_MAINWINDOW_H
#define RTT_MAINWINDOW_H

#include <QMainWindow>

#include "InterfaceController.h"
#include "InterfaceDeclarations.h"
#include "InterfaceSettings.h"


namespace rtt::Interface {
    class MainWindow: public QMainWindow {
        Q_OBJECT
    private:
        std::shared_ptr<InterfaceController> interfaceController;
        bool initial_render = true;
        QWidget* side_panel;

    public slots:
        void conditionallyEmitDeclsChanged();
        void conditionallyEmitValuesChanged();


    public:
        explicit MainWindow(std::shared_ptr<InterfaceController>, QWidget* parent = nullptr);

        void run();

    signals:
        void declarationsChanged(std::weak_ptr<InterfaceDeclarations>);
        void valuesChanged(std::weak_ptr<InterfaceSettings>);
        void updateField();

    };
}

#endif  // RTT_MAINWINDOW_H
