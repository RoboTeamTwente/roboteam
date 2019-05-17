//
// Created by rolf on 18-4-19.
//

#ifndef ROBOTEAM_UTILS_LINE_H
#define ROBOTEAM_UTILS_LINE_H

#include "LineBase.h"
namespace rtt {

class Line : public LineBase {
    public:
        constexpr Line() : LineBase(){};
        constexpr Line(const Vector2 &_start, const Vector2 &_end)
                :LineBase(_start, _end) { };
        double distanceToLine(const Vector2 &point) const override;
        bool isOnLine(const Vector2 &point) const override;
        Vector2 project(const Vector2 &point) const override;
        std::shared_ptr<Vector2> intersects(const Line &line) const override;
        std::shared_ptr<Vector2> intersects(const LineSegment &line) const override;
        bool doesIntersect(const Line &line) const override;
        bool doesIntersect(const LineSegment &line) const override;

};

}

#endif //ROBOTEAM_UTILS_LINE_H
