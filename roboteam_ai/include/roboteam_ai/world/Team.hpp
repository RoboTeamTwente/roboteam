#ifndef RTT_TEAM_HPP
#define RTT_TEAM_HPP

namespace rtt::world {
/**
 * @brief Enumerator used for indicating the team
 */
enum Team : short {
    us,   /**< Our team */
    them, /**< Enemy team */
    both  /**< If a robot has both as team -> invalid. */
};
}  // namespace rtt::world

#endif  // RTT_TEAM_HPP
