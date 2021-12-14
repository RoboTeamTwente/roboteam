//
// Created by Dawid Kulikowski on 01/12/2021.
//

#ifndef RTT_INTERFACEFIELDSCENE_H
#define RTT_INTERFACEFIELDSCENE_H
#include <QGraphicsView>
#include <QTimer>

#include "roboteam_proto/WorldRobot.pb.h"
#include "InterfaceFieldRenderer.h"
#include "InterfaceFieldStateStore.h"
#include "InterfaceRobotItem.h"

class InterfaceFieldScene: public QGraphicsView {
    Q_OBJECT
public:
    InterfaceFieldScene(std::weak_ptr<InterfaceFieldStateStore> state, QWidget* parent = nullptr);
protected:
    void drawBackground(QPainter *painter, const QRectF &rect) override;
    void resizeEvent(QResizeEvent *event) override;

protected slots:
    void triggerUpdate();

private:
    InterfaceFieldRenderer renderer;
    std::weak_ptr<InterfaceFieldStateStore> state;
    QTimer timer;

    std::vector<InterfaceRobotItem*> robots;

    void doUpdateRobot(proto::WorldRobot, bool);

};


#endif  // RTT_INTERFACEFIELDSCENE_H
