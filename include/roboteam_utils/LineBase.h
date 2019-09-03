//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINEBASE_H
#define ROBOTEAM_UTILS_LINEBASE_H
#include "Vector2.h"
namespace rtt {
class LineSegment;
class Line;
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
}

#endif //ROBOTEAM_UTILS_LINEBASE_H
