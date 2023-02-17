//
// Created by john on 1/13/20.
//

#ifndef RTT_BALL_VIEW_HPP
#define RTT_BALL_VIEW_HPP

#include "roboteam_utils/Vector2.h"
#include "world/Ball.hpp"

namespace rtt::world::view {

/**
 * @brief Ball view class, provides the Ball interface with utility functions that don't belong on a POD type
 */
class BallView {
    ball::Ball const *_ptr; /**< Pointer that points to the ball */

   public:
    /**
     * @brief Move constructor and copy assignment operator
     */
    BallView(BallView &&) noexcept;

    /**
     * @brief Checks whether the ball view has a ball
     * @return get() != nullptr;
     */
    explicit operator bool() const noexcept;

    /**
     * @brief Assign the pointer to this ball view to something else
     * @return The pointer to this ball view
     */
    BallView &operator=(BallView &&) noexcept;

    /**
     * @brief Copy constructor, internal pointer is copied over
     * @param old Old BallView to copy from
     */
    BallView(BallView const &old) = default;

    /**
     * @brief Copy assignment operator, does nothing important.
     * @param old Old ball view to copy
     * @return Returns a reference to `this`
     */
    BallView &operator=(BallView const &old) noexcept;

    /**
     * @brief Explicitly defaulted destructor, no special destruction necessary as this struct does not own the robot
     */
    ~BallView() = default;

    /**
     * @brief Constructs a BallView
     * _ptr is asserted
     * @param _ptr Pointer that this BallView should provide a view of
     */
    explicit BallView(ball::Ball const *_ptr) noexcept;

    /**
     * @brief De-reference operator that allows std::optional style de-referencing
     * Undefined behavior will occur if the contained pointer is nullptr
     * @return Returns a reference to the ball you're viewing
     */
    const ball::Ball &operator*() const noexcept;

    /**
     * @brief Gets the internally viewed pointer of the ball
     * @return Returns _ptr
     */
    [[nodiscard]] const ball::Ball *get() const noexcept;

    /**
     * @brief Member dereference operator that allows std::optional style member access
     * Undefined behavior will occur if _ptr is nullptr
     * @return Returns get()
     */
    const ball::Ball *operator->() const noexcept;
};
}  // namespace rtt::world::view

#endif  // RTT_BALL_VIEW_HPP
