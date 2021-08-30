//
// Created by Dawid Kulikowski on 19/08/2021.
//

#include "MainWindow.h"
#include <QTimer>
#include <thread>

namespace rtt::Interface {
    MainWindow::MainWindow(std::shared_ptr<InterfaceController> ctrl, QWidget* parent): QMainWindow(parent), interfaceController(std::move(ctrl)) {
        auto *timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(conditionallyEmitValuesChanged()));
        timer->start(500);

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(conditionallyEmitDeclsChanged()));
        timer->start(1000);
    }

    void MainWindow::conditionallyEmitDeclsChanged() {
        auto changeTracker = interfaceController->getDeclarationsChangeTracker().lock();
        auto decls = interfaceController->getDeclarations().lock();

        if (changeTracker && decls) {
            if (changeTracker->getState()) {
                emit declarationsChanged(interfaceController->getDeclarations());
            }
        }
    }

    void MainWindow::conditionallyEmitValuesChanged() {
        auto changeTracker = interfaceController->getSettingsChangeTracker().lock();
        auto vals = interfaceController->getValues().lock();

        if (changeTracker && vals) {
            if (changeTracker->getState()) {
                emit valuesChanged(interfaceController->getValues());
            }
        }
    }
    std::weak_ptr<InterfaceController> MainWindow::getController() {
        return interfaceController;
    }
}
