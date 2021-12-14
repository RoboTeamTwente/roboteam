//
// Created by Dawid Kulikowski on 01/12/2021.
//

#include "InterfaceFieldRenderer.h"
void InterfaceFieldRenderer::renderBall(QPainter *painter, proto::State info, QRect size) {

}

void InterfaceFieldRenderer::renderField(QPainter *painter, proto::State info, QRect size) {
    auto req_w = info.field().field().field_width();
    auto req_h = info.field().field().field_length();

    double scale = get_scale(size.width(), size.height(), req_w, req_h);

    auto arcs = info.field().field().field_arcs();
    auto lines = info.field().field().field_lines();

    painter->save();

    auto dPen = painter->pen();
    dPen.setWidth(1);
    dPen.setColor(Qt::blue);
    painter->setPen(dPen);

    painter->translate(size.width()/2, size.height()/2);

    for (const auto& arc : arcs) {
        auto thickness = scale * arc.thickness();
        auto center = arc.center();
        auto radius = scale * arc.radius();

        painter->drawEllipse(QPoint{(int)(scale * center.x()), (int)(scale * center.y())}, (int)radius, (int)radius);
    }

    for (const auto& line : lines) {
        auto p1 = line.p1();
        auto p2 = line.p2();
        auto thickness = scale * line.thickness();

        painter->drawLine(scale * p1.x(), scale * p1.y(), scale * p2.x(), scale * p2.y());
    }

    painter->restore();
}

void InterfaceFieldRenderer::renderRobot(QPainter *painter, proto::State info, QRect size, bool isYellow, int id) {
}

double InterfaceFieldRenderer::get_scale(int canvas_w, int canvas_h, int field_w, int field_h) {
    if (!field_h || !field_w) return 0;
    double w_scale = (double)canvas_w / (double)field_w;
    double h_scale =  (double)canvas_h / (double)field_h;

    return std::min({w_scale, h_scale});
}


