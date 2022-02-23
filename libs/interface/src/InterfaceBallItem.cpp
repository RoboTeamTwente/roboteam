//
// Created by Dawid Kulikowski on 28/12/2021.
//

#include "InterfaceBallItem.h"
#include <QPainter>
#include <QGraphicsScene>
#include <QGraphicsView>

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


void InterfaceBallItem::trigger_update(const proto::State &state) {
    if (this->scene() == nullptr) {
        return;
    }

    if (this->scene()->views().empty()) {
        return;
    }

    if (!this->isVisible()) {
        this->setVisible(true);
    }


    auto scale = getScale(state.field().field().field_length(), state.field().field().field_width(), this->scene()->views()[0]->viewport()->width(), this->scene()->views()[0]->viewport()->height());

    double x = this->scene()->views()[0]->viewport()->width()/2 + (scale * state.last_seen_world().ball().pos().x() * 1000) - 5;
    double y = this->scene()->views()[0]->viewport()->height()/2 + (scale * state.last_seen_world().ball().pos().y() * 1000) - 5;

    this->setPos(x, y);
}

double InterfaceBallItem::getScale(int field_h, int field_w, int canvas_w, int canvas_h) const {
    if (!field_h || !field_w) return 0;
    double w_scale = (double)canvas_w / (double)field_w;
    double h_scale =  (double)canvas_h / (double)field_h;

    return std::min({w_scale, h_scale});
}
