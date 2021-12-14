//
// Created by Dawid Kulikowski on 05/08/2021.
//

#include <QApplication>
#include <QtWidgets>
#include <QGraphicsScene>
#include "roboteam_interface/InterfaceFieldScene.h"
#include "roboteam_interface/MainWindow.h"
#include "roboteam_interface/InterfaceController.h"
#include <QVBoxLayout>


int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication roboteam_interface(argc, argv);
    auto ctrl = std::make_shared<rtt::Interface::InterfaceController>();
    ctrl->run();

    rtt::Interface::MainWindow window(ctrl);
    window.setWindowState(Qt::WindowState::WindowMaximized);

    window.show();
    window.setWindowTitle("RBTT (Interface)");


    return roboteam_interface.exec();

}