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

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

public:
    int getRobotId() const;
    bool getIsYellow() const;
    InterfaceRobotItem(int id, bool isYellow, QGraphicsItem* parent = nullptr): id(id), isYellow(isYellow), QGraphicsItem(parent) {}
    void triggerUpdate(const proto::State&);
    double getScale(int, int, int, int) const;
};


#endif  // RTT_INTERFACEROBOTITEM_H
