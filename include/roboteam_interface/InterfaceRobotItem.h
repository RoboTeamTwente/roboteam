//
// Created by Dawid Kulikowski on 12/12/2021.
//

#ifndef RTT_INTERFACEROBOTITEM_H
#define RTT_INTERFACEROBOTITEM_H

#include <QGraphicsItem>
#include "InterfaceFieldStateStore.h"
class InterfaceRobotItem: public QGraphicsItem {
private:
    int id;
    bool isYellow;
    std::weak_ptr<InterfaceFieldStateStore> world;

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

public:
    int getRobotId() const;
    bool getIsYellow() const;
    InterfaceRobotItem(std::weak_ptr<InterfaceFieldStateStore> world, int id, bool isYellow): id(id), world(world), isYellow(isYellow) {}
    void triggerUpdate();
    double getScale(int, int, int, int) const;
};


#endif  // RTT_INTERFACEROBOTITEM_H
