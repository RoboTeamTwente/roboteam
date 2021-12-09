//
// Created by Dawid Kulikowski on 05/08/2021.
//

#include <QApplication>
#include <QtWidgets>

#include "roboteam_interface/MainWindow.h"
#include "roboteam_interface/InterfaceController.h"


int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication roboteam_interface(argc, argv);
    auto ctrl = std::make_shared<rtt::Interface::InterfaceController>();
    ctrl->run();

    rtt::Interface::MainWindow window(std::move(ctrl));

    window.setWindowState(Qt::WindowState::WindowMaximized);
    window.show();
    window.setWindowTitle("Hello, world!");


    return roboteam_interface.exec();

}