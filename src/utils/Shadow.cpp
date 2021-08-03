#include "HalfLine.h"
#include "Shadow.h"

namespace rtt {
std::optional<LineSegment> Shadow::shadow(const Vector2 &source, const LineSegment &obstacle, const LineSegment &project, float negligible_shadow_length) {
    if (obstacle.isOnLine(source)) {
        // If the source is on the obstacle line then the entire line is in the shadow, because every line from the source intersects with the obstacle line.
        return project.length() > negligible_shadow_length ? std::optional(project) : std::nullopt;
    } else if (project.isPoint()) {
        /* If the projection LineSegment is a point then there is no shadow on this LineSegment. This case is needed, because a point cannot be changed into an infinite line,
         * since the direction of a point is not known. */
        return std::nullopt;
    }
    std::optional<Vector2> firstIntersect = HalfLine(source, obstacle.start).intersect(Line(project));
    std::optional<Vector2> secondIntersect = HalfLine(source, obstacle.end).intersect(Line(project));
    if (!firstIntersect.has_value() && !secondIntersect.has_value()) {
        // If both lines from the sources to the start and end of the obstacle do not intersect with the infinite line expansion of this projection line then there is no shadow.
        return std::nullopt;
    }
    std::optional<LineSegment> shadow;
    if (firstIntersect.has_value() && secondIntersect.has_value()) {
        /* If they do intersect then we can compute the shadow by projecting points in the infinite expansion of the LineSegments to either endings of the LineSegment (note
         * projection does not change the intersection location if the location is already at the LineSegment). */
        shadow = LineSegment(project.project(firstIntersect.value()), project.project(secondIntersect.value()));
    } else {
        /* If there is only one intersection then the shadow is unbounded in one direction. By only checking if the ending furthest away from the intersection point is visible you
        know for all cases (both for the cases where the intersection point is outside the LineSegment and at the LineSegment) where the shadow is. Moreover you do this quite
        efficiently, because you only have to check for one LienSegment whether it intersects with the obstacle. */
        Vector2 onlyIntersection = project.project(firstIntersect.has_value() ? firstIntersect.value() : secondIntersect.value());
        double distanceIntersectionToStart = (project.start - onlyIntersection).length();
        double distanceIntersectionToEnd = (project.end - onlyIntersection).length();
        Vector2 closest = distanceIntersectionToStart < distanceIntersectionToEnd ? project.start : project.end;
        Vector2 furthest = distanceIntersectionToStart < distanceIntersectionToEnd ? project.end : project.start;
        bool furthestInShadow = LineSegment(source, furthest).doesIntersect(obstacle);
        shadow = furthestInShadow ? LineSegment(onlyIntersection, furthest) : LineSegment(onlyIntersection, closest);
    }
    return shadow->length() > negligible_shadow_length ? shadow : std::nullopt;
}
}