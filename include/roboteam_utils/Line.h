//
// Created by rolf on 18-4-19.
//

#ifndef ROBOTEAM_UTILS_LINE_H
#define ROBOTEAM_UTILS_LINE_H

#include "Vector2.h"
namespace rtt {
class Line;
class LineSegment;
class LineBase {
    public:
        constexpr LineBase()
                :start({0.0, 0.0}), end({0.0, 0.0}) { };
        constexpr LineBase(const Vector2 &_start, const Vector2 &_end)
                :start(_start), end(_end) { };
        Vector2 start;
        Vector2 end;
        double length() const;
        double length2() const;

        double slope() const;
        double intercept() const;
        Vector2 direction() const;
        std::pair<double, double> coefficients() const;

        bool isVertical() const;
        bool isParallel(const LineBase &line) const;
        bool isPoint() const;

        virtual double distanceToLine(const Vector2 &point) const = 0;
        virtual bool isOnLine(const Vector2 &point) const = 0;
        virtual Vector2 project(const Vector2 &point) const = 0;

        virtual std::shared_ptr<Vector2> intersects(const Line &line) const = 0;
        virtual std::shared_ptr<Vector2> intersects(const LineSegment &line) const = 0;
        virtual bool doesIntersect(const Line &line) const = 0;
        virtual bool doesIntersect(const LineSegment &line) const = 0;
};
class Line : public LineBase {
    public:
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
class LineSegment : public LineBase {
    public:
        constexpr LineSegment(const Vector2 &_start, const Vector2 &_end)
                :LineBase(_start, _end) { };
        double distanceToLine(const Vector2 &point) const override;
        bool isOnLine(const Vector2 &point) const override;
        Vector2 project(const Vector2 &point) const override;
        std::shared_ptr<Vector2> intersects(const Line &line) const override;
        std::shared_ptr<Vector2> intersects(const LineSegment &line) const override;
        bool doesIntersect(const Line &line) const override;
        bool doesIntersect(const LineSegment &line) const override;
        bool nonSimpleDoesIntersect(const LineSegment &line) const;
};
}

#endif //ROBOTEAM_UTILS_LINE_H
