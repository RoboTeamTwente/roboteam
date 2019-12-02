//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINESEGMENT_H
#define ROBOTEAM_UTILS_LINESEGMENT_H
#include "LineBase.h"
namespace rtt {
class LineSegment : public LineBase {
    public:
    constexpr LineSegment() : LineBase(){};
    constexpr LineSegment(const Vector2 &_start, const Vector2 &_end)
            :LineBase(_start, _end) { };
    [[nodiscard]] double distanceToLine(const Vector2 &point) const override;
    [[nodiscard]] bool isOnLine(const Vector2 &point) const override;
    [[nodiscard]] Vector2 project(const Vector2 &point) const override;
    [[nodiscard]] std::shared_ptr<Vector2> intersects(const Line &line) const override;
    [[nodiscard]] std::shared_ptr<Vector2> intersects(const LineSegment &line) const override;
    [[nodiscard]] bool doesIntersect(const Line &line) const override;
    [[nodiscard]] bool doesIntersect(const LineSegment &line) const override;
    [[nodiscard]] bool nonSimpleDoesIntersect(const LineSegment &line) const;
    [[nodiscard]] std::shared_ptr<Vector2> nonSimpleIntersects(const LineSegment &line) const;
    virtual ~LineSegment() { };

    };
}
#endif //ROBOTEAM_UTILS_LINESEGMENT_H
