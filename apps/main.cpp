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
#include "InterfaceSyncedDropdown.h"
#include <QTabWidget>
#include <InterfaceSyncedText.h>


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

    auto play_selector = new rtt::Interface::InterfaceSyncedDropdown(ctrl, "PLAY_SELECTOR");
    auto game_state_selector = new rtt::Interface::InterfaceSyncedDropdown(ctrl, "GAME_STATE_SELECTOR");

    auto dropdown_layout = new QVBoxLayout;
    dropdown_layout->addWidget(play_selector);
    dropdown_layout->addWidget(game_state_selector);

    auto team_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IS_YELLOW");
    team_button->setText("Is Yellow");
    auto invariants_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IGNORE_INVARIANTS");
    invariants_button->setText("Ignore Invariants");
    auto right_button = new rtt::Interface::InterfaceSyncedCheckableButton(ctrl, "IS_RIGHT");
    right_button->setText("Play Right");
    auto ctrlLaytout = new QVBoxLayout;
    ctrlLaytout->setAlignment(Qt::AlignCenter);

    ctrlLaytout->addWidget(team_button);
    ctrlLaytout->addWidget(right_button);
    ctrlLaytout->addWidget(invariants_button);

    auto bottom_left_layout = new QHBoxLayout;
    bottom_left_layout->addLayout(dropdown_layout, 2);
    bottom_left_layout->addLayout(ctrlLaytout, 2);
    bottom_left_layout->addWidget(new rtt::Interface::InterfaceWidgetStopButton(ctrl), 4);

    auto leftLayout = new QVBoxLayout;
    leftLayout->addWidget(fieldWidget, 3);
    leftLayout->addLayout(bottom_left_layout, 1);


    auto tabs = new QTabWidget;

    auto sim_settings = new QWidget;
    auto sim_layout = new QVBoxLayout;
    auto conn_layout = new QHBoxLayout;

    conn_layout->addWidget(new rtt::Interface::InterfaceSyncedText(ctrl, "SIM_HOST"), 4);
    auto port_text = new rtt::Interface::InterfaceSyncedText(ctrl, "SIM_PORT");
    port_text->setValidator(new QIntValidator());
    conn_layout->addWidget(port_text, 1);
    sim_layout->addWidget(new rtt::Interface::InterfaceSyncedDropdown(ctrl, "ROBOTHUB_TARGET_SELECTOR"));
    sim_layout->addLayout(conn_layout);
    sim_layout->setAlignment(Qt::AlignTop);


    sim_settings->setLayout(sim_layout);
    sim_settings->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);

    tabs->addTab(sim_settings, "Robothub");



    tabs->addTab(new rtt::Interface::InterfaceWidgetDebugDisplay(ctrl), "Debug");

    auto right_layout = new QVBoxLayout;
    right_layout->addWidget(scroll, 1);
    right_layout->addWidget(tabs, 2);


    auto topLayout = new QHBoxLayout;
    topLayout->addLayout(leftLayout, 5);
    topLayout->addLayout(right_layout, 4);

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