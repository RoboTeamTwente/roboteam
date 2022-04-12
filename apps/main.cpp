//
// Created by Dawid Kulikowski on 05/08/2021.
//

#include <QApplication>
#include <QtWidgets>
#include <QGraphicsScene>
#include "InterfaceFieldScene.h"
#include "InterfaceFieldView.h"
#include "MainWindow.h"
#include "InterfaceControllerClient.h"
#include <QVBoxLayout>
#include <QGridLayout>
#include <InterfaceWidgetStopButton.h>
#include <InterfaceWidgetRobotOverview.h>
#include <QScrollArea>
#include <InterfaceSyncedCheckableButton.h>
#include "InterfaceWidgetDebugDisplay.h"


int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication roboteam_interface(argc, argv);

    auto ctrl = std::make_shared<rtt::Interface::InterfaceControllerClient>();
    ctrl->run();
    ctrl->getValues().lock()->setSetting("IS_YELLOW", true);

    auto view = new InterfaceFieldView(ctrl->getFieldState());
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto scene = new InterfaceFieldScene(ctrl->getFieldState());
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    view->setScene(scene);
    view->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    auto fieldLayout = new QHBoxLayout;

    fieldLayout->addWidget(view, 4);

    auto fieldWidget = new QWidget;
    fieldWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    fieldWidget->setLayout(fieldLayout);



    rtt::Interface::MainWindow window(ctrl);
    window.setWindowState(Qt::WindowState::WindowMaximized);

    QGridLayout layout;
    QScrollArea *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(new rtt::Interface::InterfaceWidgetRobotOverview(ctrl));

    auto team_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IS_YELLOW");
    team_button->setText("Is Yellow");
    auto invariants_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IGNORE_INVARIANTS");
    invariants_button->setText("Ignore Invariants");
    auto right_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IS_RIGHT");
    right_button->setText("Play Right");
    auto ctrlLaytout = new QVBoxLayout;
    ctrlLaytout->setAlignment(Qt::AlignRight);

    ctrlLaytout->addWidget(team_button);
    ctrlLaytout->addWidget(right_button);
    ctrlLaytout->addWidget(invariants_button);



//    auto debug = new rtt::Interface::InterfaceWidgetDebugDisplay(&window, ctrl);
//    layout.addWidget(debug, 1, 2, 2, 2);
    auto leftLayout = new QVBoxLayout;
    leftLayout->addWidget(fieldWidget, 3);
    leftLayout->addLayout(ctrlLaytout, 1);
    leftLayout->addWidget(new rtt::Interface::InterfaceWidgetStopButton(ctrl), 5);
    auto topLayout = new QHBoxLayout;
    topLayout->addLayout(leftLayout, 5);
    topLayout->addWidget(scroll, 5);

    layout.addLayout(topLayout, 0, 0);


    auto intWidget = new QWidget;
    intWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    intWidget->setLayout(&layout);

    window.setCentralWidget(intWidget);
    window.show();
    window.setWindowTitle("RBTT (Interface)");


    auto retval = roboteam_interface.exec();

    ctrl->stop();

    return retval;
}