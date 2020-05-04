//
// Created by haico on 22-04-20.
//

#include "HalfLine.h"

namespace rtt {
    std::optional<Vector2> HalfLine::intersect(Line line) {
        auto result = Line::intersect(start, direction, line.start, line.end);
        if (result.has_value()) {
            float t = Line::relativePosition(start, direction, result.value());
            if (t >= 0) return std::optional(result.value());
        }
        return std::nullopt;
    }
}

