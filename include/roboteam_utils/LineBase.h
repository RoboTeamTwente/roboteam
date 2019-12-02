//
// Created by rolf on 15-5-19.
//

#ifndef ROBOTEAM_UTILS_LINEBASE_H
#define ROBOTEAM_UTILS_LINEBASE_H

#include "Vector2.h"
#include <memory>
#include <utility>

namespace rtt {
    class LineSegment;

    class Line;

    class LineBase {
    public:
        constexpr LineBase()
                : start({0.0, 0.0}), end({0.0, 0.0}) {};

        constexpr LineBase(const Vector2 &_start, const Vector2 &_end)
                : start(_start), end(_end) {};
        Vector2 start;
        Vector2 end;

        [[nodiscard]] double length() const;

        [[nodiscard]] double length2() const;

        [[nodiscard]] double slope() const;

        [[nodiscard]] double intercept() const;

        [[nodiscard]] Vector2 direction() const;

        [[nodiscard]] std::pair<double, double> coefficients() const;

        [[nodiscard]] bool isVertical() const;

        [[nodiscard]] bool isParallel(const LineBase &line) const;

        [[nodiscard]] bool isPoint() const;

        [[nodiscard]] virtual double distanceToLine(const Vector2 &point) const = 0;

        [[nodiscard]] virtual bool isOnLine(const Vector2 &point) const = 0;

        [[nodiscard]] virtual Vector2 project(const Vector2 &point) const = 0;

        [[nodiscard]] virtual std::shared_ptr<Vector2> intersects(const Line &line) const = 0;

        [[nodiscard]] virtual std::shared_ptr<Vector2> intersects(const LineSegment &line) const = 0;

        [[nodiscard]] virtual bool doesIntersect(const Line &line) const = 0;

        [[nodiscard]] virtual bool doesIntersect(const LineSegment &line) const = 0;
    };
}

#endif //ROBOTEAM_UTILS_LINEBASE_H
