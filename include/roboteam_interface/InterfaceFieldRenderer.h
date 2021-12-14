//
// Created by Dawid Kulikowski on 01/12/2021.
//

#ifndef RTT_INTERFACEFIELDRENDERER_H
#define RTT_INTERFACEFIELDRENDERER_H


#include <roboteam_proto/State.pb.h>

#include <QPainter>

class InterfaceFieldRenderer {
public:
    void renderBall(QPainter*, proto::State, QRect);
    void renderField(QPainter*, proto::State, QRect);
    void renderRobot(QPainter*, proto::State, QRect, bool, int);
private:
    double get_scale(int, int, int, int);
};


#endif  // RTT_INTERFACEFIELDRENDERER_H
