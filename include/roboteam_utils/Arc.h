#pragma once

#include "Vector2.h"
#include <limits>
#include <utility>
#include <cmath>
#include <optional>

namespace rtt {
    
/**
 * \brief The maximum angle value to be used in defining an Arc. This is very slightly less than 2*pi,
 * so modulo calculations will not turn it into 0. Note that std::numeric_limits<float> is used, not <double>.
 * This is because, appearantly, the fmodl function only has float precision even though it should support
 * 128-bit long doubles...
 */
constexpr double ARC_MAX = M_PI - std::numeric_limits<float>::epsilon();
    
/**
 * \class Arc
 * \brief Represents a (part of) an ellipse, which may be a circle.
 */
class Arc {
    public:
    Vector2 center;     //< The center of the ellipse 
    double length;      //< The 'radius' in the x-direction
    double width;       //< The 'radius' in the y-direction
    double angleStart;  //< The angle, with 0 being the positive x-axis, at which the arc starts, in rad
    double angleEnd;    //< The angle at which the arc ends, in rad.
    
    /**
     * \brief Constructs an Arc representing the unit circle.
     */
    Arc();
    
    /**
     * \brief Constructs an Arc which is a circle.
     */
    Arc(Vector2 center, double radius);
    
    /**
     * \brief Constructs an Arc which is part of a circle.
     */
    Arc(Vector2 center, double radius, double angleStart, double angleEnd);
    
    
    
    /**
     * \brief Constructs an Arc which is a part of an ellipse
     */
    Arc(Vector2 center, double length, double width, double angleStart, double angleEnd);
    
    /**
     * \brief Checks if this Arc is a circle.
     * \return True iff:
     *      - length == width
     *      - angleStart == 0.0
     *      - angleEnd == ARC_MAX
     */
    bool isCircle() const;
    
    /**
     * \brief Checks if this Arc is at least a part of a circle.
     * \return True iff length == width
     */
    bool isPartialCircle() const;
    
    /**
     * \brief Checks whether an angle is within the range this Arc is defined on.
     * \param angle The angle to check.
     * \return True iff angleStart <= angle <= angleEnd
     */
    bool angleWithinArc(double angle) const;
    
    /**
     * \brief Checks whether a point lies within this Arc.
     * \param point The point to check
     * \return True if the point lies within the ellipse defined by this Arc.
     */
    bool pointInArc(const Vector2& point) const;
    
    /**
     * \brief Checks whether a point lies on this Arc.
     * \param point The point to check
     * \return True if the point lies on this Arc.
     */
    bool pointOnArc(const Vector2& point) const;
    
    /**
     * \brief Gets the point(s) at which the given line segment intersects this Arc, if such  points exist.
     * \param lineStart The start of the line segment
     * \param lineEnd The end of the line segment
     * \return If there are two points of intersection, then a pair with those two points.
     *  The point with the greatest x + y value will be first. If there is only one point of
     *  intersection (incidence), then it will be stored in the first element of the pair, and
     *  the second optional will be empty. If the line segment does not intersect the arc, then
     *  both optionals will be empty.
     */
    std::pair<std::optional<Vector2>, std::optional<Vector2>>
    intersectionWithLine(Vector2 lineStart, Vector2 lineEnd) const;
    
    /**
     * \brief Gets the point on the Arc in a certain direction from the center.
     * \param angle The angle to look at, where 0.0 is the positive x-axis.
     * \return If angleWithinArc(angle), an optional containing the point in the given direction.
     * Otherwise an empty optional.
     */
    std::optional<Vector2> arcPointTowards(double angle) const;
    
    /**
     * \brief Gets the point on the Arc in a certain direction from the center, given by another point.
     * \param point The point towards with the result must be sought.
     * \return If angleWithinArc((point - center).angle()), an optional containing the point in the given direction.
     * Otherwise an empty optional.
     */
    std::optional<Vector2> arcPointTowards(Vector2 point) const;
    
    private:
    /**
     * \brief Normalizes an angle (in rad) to the range [0, 2*pi)
     */
    static double normalize(double angle);
    
    /**
     * \brief Checks whether a point lies on the Arc, and denormalizes (+center) it if so.
     * Otherwise, it returns an empty optional.
     */
    std::optional<Vector2> checkAndDenormalize(Vector2 vec) const;
};
    
}
