//
// Created by Dawid Kulikowski on 28/12/2021.
//

#include "InterfaceBallItem.h"
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>

#include <cmath>

void InterfaceBallItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->save();

    painter->setRenderHint(QPainter::RenderHint::Antialiasing);
    QColor orange{255,94,19};
    QBrush brush(orange);
    QPen pen(orange);

    painter->setPen(orange);
    painter->setBrush(brush);

    painter->drawEllipse(QPoint{5, 5}, 5, 5);

    painter->restore();
}

QRectF InterfaceBallItem::boundingRect() const {
    return QRectF(0, 0, 10, 10);
}


void InterfaceBallItem::triggerUpdate(const proto::State &state) {
    if (this->scene() == nullptr) {
        return;
    }

    if (this->scene()->views().empty()) {
        return;
    }

    if (!this->isVisible()) {
        this->setVisible(true);
    }

    double x = this->scene()->views()[0]->viewport()->width()/2 + (scale * state.last_seen_world().ball().pos().x() * 1000) - 5;
    double y = this->scene()->views()[0]->viewport()->height()/2 + (scale * state.last_seen_world().ball().pos().y() * 1000) - 5;

    this->setPos(x, y);
}

void InterfaceBallItem::updateScale(double fieldWidth, double fieldHeight) {
    if (fieldWidth == 0 || fieldHeight == 0) this->scale = 0;

    double widthScale = static_cast<double>(this->scene()->views()[0]->viewport()->width()) / fieldWidth;
    double heightScale = static_cast<double>(this->scene()->views()[0]->viewport()->height()) / fieldHeight;

    this->scale = std::fmin(widthScale, heightScale);
}