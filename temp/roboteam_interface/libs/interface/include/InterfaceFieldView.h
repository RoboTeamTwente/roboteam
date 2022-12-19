//
// Created by Dawid Kulikowski on 28/12/2021.
//

#ifndef RTT_INTERFACEFIELDVIEW_H
#define RTT_INTERFACEFIELDVIEW_H

#include <QGraphicsItem>
#include <QGraphicsScene>
#include <QGraphicsView>

#include "InterfaceFieldRenderer.h"
#include "InterfaceFieldStateStore.h"
#include "roboteam_interface_utils/MessageCache.h"

namespace rtt::Interface {
    class InterfaceFieldView: public QGraphicsView  {
        Q_OBJECT
    public:
        InterfaceFieldView(std::weak_ptr<MessageCache<proto::State>> state, QWidget* parent = nullptr);
    protected:
        void drawBackground(QPainter *painter, const QRectF &rect) override;
        void resizeEvent(QResizeEvent *event) override;

    private:
        InterfaceFieldRenderer renderer;
        std::weak_ptr<MessageCache<proto::State>> state;
    };
}




#endif  // RTT_INTERFACEFIELDVIEW_H
