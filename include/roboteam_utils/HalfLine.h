#ifndef RTT_ROBOTEAM_UTILS_SRC_HALFLINE_H_
#define RTT_ROBOTEAM_UTILS_SRC_HALFLINE_H_

#include <optional>
#include "Line.h"
#include "Vector2.h"

namespace rtt {
/**
 * The HalfLine class represents a line that is semi-infinite long, i.e. it has a start point where the line starts and a direction in which it is infinite long.
 *
 * @author Haico Dorenbos
 * @since 2020-04-22
 */
class HalfLine {
    private:
        Vector2 start; // The location where the HalfLine starts.
        Vector2 goesThrough; // An arbitrary other vector (different than start) through which the HalfLine goes.

    public:
        /**
         * Construct a new HalfLine object by using 2 different points on that HalfLine. Make sure that the given points are different.
         * @param start The location where the HalfLine starts.
         * @param goesThrough An arbitrary other vector through the HalfLine goes (make sure that it is different than the start point).
         */
        explicit HalfLine(const Vector2 &start, const Vector2 &goesThrough);

        /**
         * Compute the intersection point between this HalfLine and the given Line.
         *
         * @param line The given Line.
         * @return std::nullopt in case of no intersection. In case there is a single intersection point, return that point. In case there are multiple (infinitely many)
         * intersection points, return the intersection point closest to the origin (0,0).
         */
        std::optional<Vector2> intersect(const Line &line);

        /**
         * Projects the point onto this HalfLine, i.e. find the location on this HalfLine closest to that point.
         *
         * @param point Point that is projected on this HalfLine.
         * @return The projection point. Note that this point is located on this HalfLine, but it does NOT have to be orthogonal to the given point.
         */
        Vector2 project(const Vector2 &point) const;
};
}
#endif //RTT_ROBOTEAM_UTILS_SRC_HALFLINE_H_
