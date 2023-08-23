//
// Created by ratoone on 20-02-20.
//

#ifndef RTT_PATHPOINTNODE_H
#define RTT_PATHPOINTNODE_H

#include <roboteam_utils/Vector2.h>

namespace rtt::ai::control {
/**
 * A node in the path tree generation. Has a position and a parent
 */
class PathPointNode {
   private:
    Vector2 position;                /**< Position of the node */
    PathPointNode *parent = nullptr; /**< Parent node of the node */

   public:
    /**
     * @brief Create a new tree node with a null parent (this will usually be the root of the tree)
     * @param position the position represented by the node
     */
    explicit PathPointNode(const Vector2 &position);

    /**
     * @brief Create a new tree node with the specified parent
     * @param position the position represented by the node
     * @param parent the parent node of the current node
     */
    PathPointNode(const Vector2 &position, PathPointNode &parent);

    /**
     * @brief Retrieves the position of the node
     * @return position of the node
     */
    [[nodiscard]] const Vector2 &getPosition() const;

    /**
     * @brief retrieves the parent of the node
     * @return parent of the node
     */
    [[nodiscard]] PathPointNode *getParent() const;
};
}  // namespace rtt::ai::control

#endif  // RTT_PATHPOINTNODE_H
