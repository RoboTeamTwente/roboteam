#pragma once

#include <QGraphicsItem>
#include <InterfaceFieldStateStore.h>
#include "InterfaceFieldStateStore.h"
class InterfaceRobotPathItem: public QGraphicsItem {
public:
    InterfaceRobotPathItem(rtt::Team, QGraphicsItem* parent = nullptr);
    void triggerUpdate(const std::vector<rtt::RobotPath>& paths);
    void updateScale(double fieldWidth, double fieldHeight);

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

private:
    double scale;
    rtt::Team team;

    std::vector<rtt::RobotPath> paths;
};

