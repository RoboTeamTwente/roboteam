//
// Created by Dawid Kulikowski on 12/12/2021.
//

#include "InterfaceRobotItem.h"

#include <QGraphicsView>
#include <QStaticText>
#include <cmath>

constexpr double REAL_ROBOT_RADIUS = 0.09; // radius is about 8 cm

void InterfaceRobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->save();
    painter->setRenderHint(QPainter::RenderHint::Antialiasing);

    // Draw rotation-line of robot
    painter->setPen(QPen(Qt::red, 2));
    painter->drawLine(0, 0, radius*3, 0);

    // Draw role of robot
    QStaticText role("Halt_#");
    painter->drawStaticText(-role.size().width()/2, radius, role);

    // Draw robot
    if (!isYellow) {
        painter->setPen(Qt::blue);
        painter->setBrush(Qt::blue);
    } else {
        painter->setPen(Qt::yellow);
        painter->setBrush(Qt::yellow);
    }
    painter->drawEllipse(QPointF(0, 0), radius, radius);

    painter->restore();
}

bool InterfaceRobotItem::getIsYellow() const {
    return this->isYellow;
}

int InterfaceRobotItem::getRobotId() const {
    return this->id;
}
QRectF InterfaceRobotItem::boundingRect() const {
    return QRectF(-60, -60, 120, 120);
}

void InterfaceRobotItem::triggerUpdate() {
    if (this->scene()->views().empty()) {
        return;
    }

    auto state = this->storage.lock()->getState();

    auto searchArea = isYellow ? state.last_seen_world().yellow() : state.last_seen_world().blue();
    auto us = std::find_if(searchArea.begin(), searchArea.end(), [&] (const auto& itm) {
        return itm.id() == this->id;
    });

    if (us == searchArea.end()) {
        this->setVisible(false);
        return;
    } else {
        this->setVisible(true);
    }

    int canvasCenterX = this->scene()->views()[0]->viewport()->width()/2;
    int canvasCenterY = this->scene()->views()[0]->viewport()->height()/2;

    double x = us->pos().x() * scale + canvasCenterX;
    double y = scale * us->pos().y() + canvasCenterY;

    this->setPos(x, y);
    this->setRotation((us->angle()/M_PI) * 180);

    this->update();
}

void InterfaceRobotItem::updateScale(double fieldWidth, double fieldHeight) {
    if (fieldWidth == 0 || fieldHeight == 0) this->scale = 0.0;

    double canvasWidth = static_cast<double>(this->scene()->views()[0]->viewport()->width());
    double canvasHeight = static_cast<double>(this->scene()->views()[0]->viewport()->height());

    double widthScale = canvasWidth / (fieldWidth/1000);
    double heightScale = canvasHeight / (fieldHeight/1000);

    this->scale = std::fmin(widthScale, heightScale);
    this->radius = static_cast<int>(REAL_ROBOT_RADIUS * this->scale);
}