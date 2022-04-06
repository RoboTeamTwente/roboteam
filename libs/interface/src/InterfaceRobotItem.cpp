//
// Created by Dawid Kulikowski on 12/12/2021.
//

#include "InterfaceRobotItem.h"
#include <QGraphicsView>

void InterfaceRobotItem::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->save();
    painter->setRenderHint(QPainter::RenderHint::Antialiasing);
    auto cPen = painter->pen();
    auto brush = painter->brush();
    if (!isYellow) {
        cPen.setColor(Qt::blue);
        brush.setColor(Qt::blue);
    } else {
        cPen.setColor(Qt::yellow);
        brush.setColor(Qt::yellow);
    }

    painter->setBrush(brush);
    painter->setPen(cPen);
    painter->drawEllipse(QPoint{20, 20}, 20, 20);
    painter->restore();
}

bool InterfaceRobotItem::getIsYellow() const {
    return this->isYellow;
}

int InterfaceRobotItem::getRobotId() const {
    return this->id;
}
QRectF InterfaceRobotItem::boundingRect() const {
    return QRectF(0, 0, 40, 40);
}

void InterfaceRobotItem::triggerUpdate(const proto::State& state) {
    if (this->scene()->views().empty()) {
        return;
    }

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

        auto scale = getScale(state.field().field().field_length(), state.field().field().field_width(), this->scene()->views()[0]->viewport()->width(), this->scene()->views()[0]->viewport()->height());

        double x = this->scene()->views()[0]->viewport()->width()/2 + (scale * us->pos().x() * 1000) - 20;
        double y = this->scene()->views()[0]->viewport()->height()/2 + (scale * us->pos().y() * 1000) - 20;

        this->setPos(x, y);

//        std::cout << "[" + std::to_string(id) + "]" + " X: " + std::to_string(x) + " | Y: " + std::to_string(y) + " [Yellow = " <<  std::boolalpha <<  isYellow << "]" << std::endl;
}
double InterfaceRobotItem::getScale(int field_h, int field_w, int canvas_w, int canvas_h) const {
    if (!field_h || !field_w) return 0;
    double w_scale = (double)canvas_w / (double)field_w;
    double h_scale =  (double)canvas_h / (double)field_h;

    return std::min({w_scale, h_scale});
}
