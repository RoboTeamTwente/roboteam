//
// Created by Dawid Kulikowski on 19/08/2021.
//

#include "MainWindow.h"

#include <InterfaceFieldView.h>

#include <QBoxLayout>
#include <QGraphicsScene>
#include <QLabel>
#include <QLayout>
#include <QSplitter>
#include <QTimer>
#include <QWidget>
#include <thread>

#include "InterfaceFieldScene.h"
#include "InterfaceFieldWidget.h"

namespace rtt::Interface {
    MainWindow::MainWindow(std::shared_ptr<InterfaceControllerClient> ctrl, QWidget* parent): QMainWindow(parent), interfaceController(ctrl) {
        setMinimumWidth(800);
        setMinimumHeight(600);


        auto *timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(conditionallyEmitValuesChanged()));
        timer->start(500);

        timer = new QTimer(this);
        connect(timer, SIGNAL(timeout()), this, SLOT(conditionallyEmitDeclsChanged()));
        timer->start(1000);
    }

    void MainWindow::conditionallyEmitDeclsChanged() {
//        auto changeTracker = interfaceController->getDeclarationsChangeTracker().lock();
//        auto decls = interfaceController->getDeclarations().lock();
//
//        if (changeTracker && decls) {
//            if (changeTracker->getState()) {
//                changeTracker.get()->clear();
//
//                if (initial_render){
//
//                    initial_render = false;
//                }
//
//                emit declarationsChanged(interfaceController->getDeclarations());
//            }
//        }
    }

    void MainWindow::conditionallyEmitValuesChanged() {
//        auto changeTracker = interfaceController->getSettingsChangeTracker().lock();
//        auto vals = interfaceController->getValues().lock();
//
//        if (changeTracker && vals) {
//            if (changeTracker->getState()) {
//                changeTracker.get()->clear();
//                emit valuesChanged(interfaceController->getValues());
//            }
//        }
    }
}
