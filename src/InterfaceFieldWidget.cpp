//
// Created by Dawid Kulikowski on 03/10/2021.
//

#include "InterfaceFieldWidget.h"
#include <QVBoxLayout>
#include <QLabel>

namespace rtt::Interface {
InterfaceFieldWidget::InterfaceFieldWidget(QWidget *parent) {
    this->setLayout(new QVBoxLayout());
    QLabel* label = new QLabel();

    label->setText("Waiting for field state...");
    label->setAlignment(Qt::AlignCenter);

    this->layout()->addWidget(label);
}
}