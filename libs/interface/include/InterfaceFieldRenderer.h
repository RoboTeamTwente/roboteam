//
// Created by Dawid Kulikowski on 01/12/2021.
//

#ifndef RTT_INTERFACEFIELDRENDERER_H
#define RTT_INTERFACEFIELDRENDERER_H


#include <proto/State.pb.h>
#include <InterfaceFieldStateStore.h>
#include <roboteam_utils/AIData.hpp>

#include <QPainter>

class InterfaceFieldRenderer {
public:
    InterfaceFieldRenderer(std::weak_ptr<InterfaceFieldStateStore> storage);

    void renderBall(QPainter*, QRect);
    void renderField(QPainter*, QRect);
    void renderRobot(QPainter*, QRect, bool, int);

    void updateScale(int canvasWidth, int canvasHeight, double fieldWidth, double fieldHeight);
private:
    double scale;
    std::weak_ptr<InterfaceFieldStateStore> storage;
};


#endif  // RTT_INTERFACEFIELDRENDERER_H
