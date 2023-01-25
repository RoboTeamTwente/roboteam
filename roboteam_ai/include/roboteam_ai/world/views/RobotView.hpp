//
// Created by john on 1/14/20.
//

#ifndef RTT_ROBOT_VIEW_HPP
#define RTT_ROBOT_VIEW_HPP

#include <roboteam_utils/Vector2.h>
#include <utilities/Constants.h>

#include <world/Robot.hpp>

namespace rtt::world::view {

/**
 * @brief Class that offers a read-only memory view of the Robots
 * Imagine RobotView being to Robot what string_view is to std::string,
 * except it provides more utility functions
 */
class RobotView {
    robot::Robot const *robotPtr; /**< Pointer to a robot */

   public:
    /**
     * @brief Constructs a RobotView from a Robot
     * @param _ptr Robot pointer to construct from
     * @ensures After contruction this->rbt == _ptr;
     */
    explicit RobotView(robot::Robot const *_ptr) noexcept;

    /**
     * @brief Checks whether there is a robot in this robot view
     * @return get() != null;
     */
    explicit operator bool() const noexcept;

    /**
     * @brief Gets the internal pointer to the robot (std::unique_ptr style)
     * @return Returns the internal pointer
     */
    [[nodiscard]] robot::Robot const *get() const noexcept;

    /**
     * @brief Overloads the dereference operator (*ptr) in std::unique_ptr style
     * @return Returns a constant reference to the non-owned Robot
     */
    robot::Robot const &operator*() const noexcept;

    /**
     * @brief Gives std::unique_ptr (std::unique_ptr<Robot>()->...) style access into member functions
     * @return Returns get()
     */
    robot::Robot const *operator->() const noexcept;

    /**
     * @brief Copy assignment operator
     * @return *this
     */
    RobotView &operator=(RobotView const &) = default;

    /**
     * @brief Move assignment operator
     * @return *this
     */
    RobotView &operator=(RobotView &&) = default;

    /**
     * @brief Copy constructor of the RobotView class
     */
    RobotView(RobotView const &) = default;

    /**
     * @brief Move constructor, same as copy constructor
     */
    RobotView(RobotView &&) = default;

    /**
     * @brief Default destructor of the RobotView class
     */
    ~RobotView() = default;
};

}  // namespace rtt::world::view

#endif  // RTT_ROBOT_VIEW_HPP
