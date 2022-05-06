//
// Created by Dawid Kulikowski on 12/12/2021.
//

#ifndef RTT_INTERFACEROBOTITEM_H
#define RTT_INTERFACEROBOTITEM_H

#include <QGraphicsItem>
#include <InterfaceFieldStateStore.h>
#include "InterfaceFieldStateStore.h"
class InterfaceRobotItem: public QGraphicsItem {
private:
    int id;
    bool isYellow;
    std::weak_ptr<InterfaceFieldStateStore> storage;
    double scale;
    int radius;

protected:
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    QRectF boundingRect() const override;

public:
    int getRobotId() const;
    bool getIsYellow() const;
    InterfaceRobotItem(int id, bool isYellow, std::weak_ptr<InterfaceFieldStateStore> storage, QGraphicsItem* parent = nullptr):
          QGraphicsItem(parent),
          id(id),
          isYellow(isYellow),
          storage(storage) {}
    void triggerUpdate();
    void updateScale(double fieldWidth, double fieldHeight);
};


#endif  // RTT_INTERFACEROBOTITEM_H
