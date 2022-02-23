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


int main(int argc, char *argv[]) {
    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QApplication roboteam_interface(argc, argv);
    auto ctrl = std::make_shared<rtt::Interface::InterfaceControllerClient>();
    ctrl->run();

    InterfaceFieldView view(ctrl->getFieldState());
    view.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    InterfaceFieldScene scene(ctrl->getFieldState());
    scene.setItemIndexMethod(QGraphicsScene::NoIndex);
    view.setScene(&scene);
    view.setAlignment(Qt::AlignLeft | Qt::AlignTop);
    auto fieldLayout = new QHBoxLayout;

    fieldLayout->addWidget(&view, 3);
    fieldLayout->addWidget(new QLabel("hiii"), 1);

    auto intWidget = new QWidget;
    intWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    intWidget->setLayout(fieldLayout);

    rtt::Interface::MainWindow window(ctrl);
    window.setWindowState(Qt::WindowState::WindowMaximized);

    window.setCentralWidget(intWidget);
    window.show();
    window.setWindowTitle("RBTT (Interface)");


    return roboteam_interface.exec();

}