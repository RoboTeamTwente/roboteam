#ifndef RTT_ROBOTEAM_UTILS_SRC_SHADOW_H_
#define RTT_ROBOTEAM_UTILS_SRC_SHADOW_H_

#include "LineSegment.h"
#include "Vector2.h"

namespace rtt {
/**
 * The Shadow class computes shadows by using a point as (light) 'source', a LineSegment which is considered the 'obstacle' and a Line on which the shadow is projected which is
 * called the 'project'.
 *
 * @author Haico Dorenbos
 * @since 2020-05-27
 */
class Shadow {
   public:
    /**
     * @brief Computes the shadow caused by an 'obstacle' (LineSegment) and (light) 'source' (Vector2) on a given 'project' line (LineSegment).
     * The shadow is subset of the 'project' LineSegment. And a point p on the 'project' line is considered to be in the shadow if the line between the source and point p
     * intersects with the obstacle LineSegment (so if the source is on the obstacle LineSegment then everything is in the shadow).
     *
     * @param source The place where the (light) 'source' is located, i.d. the position from which the projected shadow is computed.
     * @param obstacle The obstacle LineSegment which blocks the 'light' from the 'source'.
     * @param project The project LineSegment on which the shadow is projected.
     * @param negligible_shadow_length If the length of the shadow is smaller than this value then there is no shadow (if not set then the negligible shadow length is 1e-6).
     * @return No LineSegment if there is no shadow or if the shadow is smaller than the negligible length. Otherwise return the LineSegment that represents the shadow on this
     * LineSegment.
     */
    [[nodiscard]] static std::optional<LineSegment> shadow(const Vector2 &source, const LineSegment &obstacle, const LineSegment &project, float negligible_shadow_length = 1e-6);
};
}  // namespace rtt
#endif  // RTT_ROBOTEAM_UTILS_SRC_SHADOW_H_
