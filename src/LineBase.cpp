//
// Created by rolf on 15-5-19.
//

#include "../include/roboteam_utils/LineBase.h"

namespace rtt {

    double LineBase::length() const {
        return (end - start).length();
    }

    double LineBase::length2() const {
        return (end - start).length2();
    }

    double LineBase::slope() const {
        return (end.y - start.y) / (end.x - start.x);
    }

    bool LineBase::isVertical() const {
        return (end.x == start.x) && (end.y != start.y);
    }

    Vector2 LineBase::direction() const {
        return Vector2(end - start);
    }

    double LineBase::intercept() const {
        return start.y - this->slope() * start.x;
    }

    std::pair<double, double> LineBase::coefficients() const {
        return { slope(), intercept() };
    }

    bool LineBase::isPoint() const {
        return start == end;
    }

    bool LineBase::isParallel(const LineBase &line) const {
        // check if line is vertical, otherwise check the slope
        if (this->isVertical() || line.isVertical()) {
            return this->isVertical() && line.isVertical();
        }
        return this->slope() == line.slope();
    }

} //rtt
