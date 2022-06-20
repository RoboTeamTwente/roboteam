//
// Created by Dawid Kulikowski on 28/12/2021.
//

#ifndef RTT_INTERFACEBALLITEM_H
#define RTT_INTERFACEBALLITEM_H
#include <QGraphicsItem>
#include <proto/State.pb.h>

namespace rtt::Interface {
    class InterfaceBallItem: public QGraphicsItem {
    private:
        double getScale(int field_h, int field_w, int canvas_w, int canvas_h) const;

        double scale;
    protected:
        void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
        QRectF boundingRect() const override;

    public:
        InterfaceBallItem(QGraphicsItem* parent = nullptr): QGraphicsItem(parent) {
            this->setPos(0,0);
            this->setVisible(false);
        }

        void updateScale(double fieldWidth, double fieldHeight);
        void triggerUpdate(const proto::State&);
    };
}



#endif  // RTT_INTERFACEBALLITEM_H
