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
    ctrl->getValues().lock()->setSetting("IS_YELLOW", false);

    proto::State state;

    for (int i = 0; i < 6; i++) {
        proto::WorldRobot robot;
        robot.set_id(i);
        robot.set_angle(1.0);
        robot.set_angle(69.0);
        robot.mutable_pos()->set_x(0.0);
        robot.mutable_pos()->set_y(0.0);

        robot.mutable_vel()->set_x(0.0);
        robot.mutable_vel()->set_y(0.0);

        state.mutable_ball_camera_world()->mutable_blue()->Add(std::move(robot));
    }


    ctrl->getFieldState().lock()->setState(state);
    rtt::Interface::InterfaceDeclaration decl;
    decl.path = "AAAA_DEBUG";
    decl.description = "TEST";
    decl.isMutable = true;
    decl.defaultValue = 50.0f;
    decl.options = rtt::Interface::InterfaceSlider("AAA", 0, 100, 1, 1);
    ctrl->getDeclarations().lock()->addDeclaration(decl);

    ctrl->getValues().lock()->setSetting("AAAA_DEBUG", 50.0f);

    auto view = new InterfaceFieldView(ctrl->getFieldState());
    view->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto scene = new InterfaceFieldScene(ctrl->getFieldState());
    scene->setItemIndexMethod(QGraphicsScene::NoIndex);
    view->setScene(scene);
    view->setAlignment(Qt::AlignLeft | Qt::AlignTop);
    auto fieldLayout = new QHBoxLayout;

    fieldLayout->addWidget(view, 3);

    auto fieldWidget = new QWidget;
    fieldWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    fieldWidget->setLayout(fieldLayout);



    rtt::Interface::MainWindow window(ctrl);
    window.setWindowState(Qt::WindowState::WindowMaximized);

    QGridLayout layout;
    QScrollArea *scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setWidget(new rtt::Interface::InterfaceWidgetRobotOverview(&window, ctrl));

    auto team_button = new rtt::Interface::InterfaceSyncedCheckableButton(&window, ctrl, "IS_YELLOW");
    team_button->setText("Is Yellow");
    auto invariants_button = new rtt::Interface::InterfaceSyncedCheckableButton(&window, ctrl, "IGNORE_INVARIANTS");
    invariants_button->setText("Ignore Invariants");
    auto right_button = new rtt::Interface::InterfaceSyncedCheckableButton(&window, ctrl, "IS_RIGHT");
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
    auto topLayout = new QHBoxLayout;
    topLayout->addLayout(leftLayout, 5);
    topLayout->addWidget(scroll, 3);

    layout.addLayout(topLayout, 0, 0);


    auto intWidget = new QWidget;
    intWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    intWidget->setLayout(&layout);

    window.setCentralWidget(intWidget);
    window.show();
    window.setWindowTitle("RBTT (Interface)");

    emit window.valuesChanged(ctrl->getValues().lock());
    emit window.declarationsChanged(ctrl->getDeclarations().lock());
    emit window.valuesChanged(ctrl->getValues().lock());


    auto retval = roboteam_interface.exec();

    ctrl->stop();

    return retval;
}