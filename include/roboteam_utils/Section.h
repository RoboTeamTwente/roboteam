#pragma once

#include "Vector2.h"

namespace rtt {
    
/**
 * \class Section
 * \brief Models a line segment
 */
struct Section {
    Vector2 a, b, center;
    double length;
    Section(double ax, double ay, double bx, double by) :
        a(ax, ay), b(bx, by), center((ax+bx)/2.0, (ay+by)/2.0),
        length(a.dist(b)) {}
    Section(Vector2 a, Vector2 b) : Section(a.x, a.y, b.x, b.y) {}
    Section() {}
    
    /**
     * \brief Gets the point of intersection between this section and another.
     * The resulting point does not have to lie on one of the two segments; if the
     * segments themselves do not intersect, the lines are extended infinitely.
     * \param other The other line segment.
     * \return The intersection of the two (possibly extended) lines.
     */
    Vector2 intersection(const Section& other) const;
    
    /**
     * \brief Checks whether a point lies on this line segment.
     * \param point The point to check.
     * \return True if the point lies on the (non-extended) line segment.
     */
    bool pointOnLine(const Vector2& point) const;
};
    
}