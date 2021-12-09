//
// Created by Dawid Kulikowski on 19/08/2021.
//

#include "MainWindow.h"
#include <QTimer>
#include <QWidget>
#include <QLayout>
#include <thread>
#include <InterfaceDeclarationsRenderer.h>
#include "InterfaceFieldWidget.h"
#include <QSplitter>

namespace rtt::Interface {
    MainWindow::MainWindow(std::shared_ptr<InterfaceController> ctrl, QWidget* parent): QMainWindow(parent), interfaceController(std::move(ctrl)) {
        setMinimumWidth(800);
        setMinimumHeight(600);
        setCentralWidget(new QWidget());

        QHBoxLayout *layout = new QHBoxLayout;
        QSplitter* splitter = new QSplitter();
        QSplitter* tmp_panel = new QSplitter();
        side_panel = tmp_panel;

        tmp_panel->setOrientation(Qt::Vertical);

        splitter->addWidget(new InterfaceFieldWidget());
        splitter->addWidget(side_panel);

        layout->addWidget(splitter);
        centralWidget()->setLayout(layout);

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
                changeTracker.get()->clear();

                if (initial_render){
                    InterfaceDeclarationsRenderer r;

                    r.render(this, this->interfaceController, side_panel);

                    initial_render = false;
                }

                emit declarationsChanged(interfaceController->getDeclarations());
            }
        }
    }

    void MainWindow::conditionallyEmitValuesChanged() {
        auto changeTracker = interfaceController->getSettingsChangeTracker().lock();
        auto vals = interfaceController->getValues().lock();

        if (changeTracker && vals) {
            if (changeTracker->getState()) {
                changeTracker.get()->clear();
                emit valuesChanged(interfaceController->getValues());
            }
        }
    }
}
