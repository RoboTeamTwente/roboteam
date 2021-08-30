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

    public slots:
        void conditionallyEmitDeclsChanged();
        void conditionallyEmitValuesChanged();

    public:
        explicit MainWindow(std::shared_ptr<InterfaceController> = std::make_shared<InterfaceController>(), QWidget* parent = nullptr);
        std::weak_ptr<InterfaceController> getController();

        void run();
    signals:
        void declarationsChanged(std::weak_ptr<InterfaceDeclarations>);
        void valuesChanged(std::weak_ptr<InterfaceSettings>);
        void drawRequest(); //TODO: Remote drawing
        void updateField();

    };
}

#endif  // RTT_MAINWINDOW_H
