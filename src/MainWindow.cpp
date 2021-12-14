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
#include "InterfaceFieldScene.h"
#include <QGraphicsScene>
#include <QSplitter>

namespace rtt::Interface {
    MainWindow::MainWindow(std::shared_ptr<InterfaceController> ctrl, QWidget* parent): QMainWindow(parent), interfaceController(ctrl) {
        setMinimumWidth(800);
        setMinimumHeight(600);
        setCentralWidget(new QWidget());

        QHBoxLayout *layout = new QHBoxLayout;
        QSplitter* splitter = new QSplitter();
        QSplitter* tmp_panel = new QSplitter();
        side_panel = tmp_panel;

        InterfaceFieldScene view(ctrl->getFieldState());
        view.setMinimumSize(1280, 720);
        view.setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        QGraphicsScene scene;
        view.setScene(&scene);
        view.setAlignment(Qt::AlignLeft | Qt::AlignTop);
        QVBoxLayout fieldLayout;

        fieldLayout.addWidget(&view);
        auto intWidget = new QWidget;
        intWidget->setMinimumSize(1280, 720);
        intWidget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
        intWidget->setLayout(&fieldLayout);


        tmp_panel->setOrientation(Qt::Vertical);

        splitter->addWidget(intWidget);
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

//                    r.render(this, this->interfaceController, side_panel);

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
