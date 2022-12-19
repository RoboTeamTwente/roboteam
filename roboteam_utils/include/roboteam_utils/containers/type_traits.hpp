//
// Created by john on 3/4/20.
//

#ifndef RTT_TYPE_TRAITS_HPP
#define RTT_TYPE_TRAITS_HPP

#include <type_traits>

namespace rtt::type_traits {
/**
 * True case:
 * Ensures these member functions exists and are invocable
 * std::enable_if enables this overload (std::true_type), so if std::is_invocable is false (aka if T().update is not a callable, ::value == false)
 *
 * decltype() gets the type of something, so decltype(&T::update) gets the type of that function
 * std::is_nothrow_invocable_r returns true if T::update returns Enum and is passed an SI const&
 *
 * @tparam T Type for which the member has to exist
 * @tparam Enum Enum type that's passed
 * @tparam SI SkilInfo class
 */
template <typename T, typename Enum, typename SI, typename = std::enable_if<std::is_nothrow_invocable_r_v<Enum, decltype(&T::update), T*, SI const&>>,
          typename = std::enable_if<std::is_nothrow_invocable_r_v<void, decltype(&T::initialize), T*>>,
          typename = std::enable_if<std::is_nothrow_invocable_r_v<void, decltype(&T::terminate), T*>>>
struct is_valid_type : std::true_type {};

/**
 * Base false case for the above true case, incase std::enable_if is false
 * @tparam T Type
 * @tparam Enum Enum type
 * @tparam SI SkillInfo type
 */
template <typename T, typename Enum, typename SI>
struct is_valid_type<T, Enum, SI, void, void, void> : std::false_type {};

/**
 * stl-style `_v` value, allows you to is_valid_type_v<T> instead of is_valid_type<T>::value
 * (_v stands for value)
 * @tparam T Type
 * @tparam Enum Enum
 * @tparam SI Si
 * Literally: is_valid_type<T, Enum, SI>::value;
 */
template <typename T, typename Enum, typename SI>
constexpr static bool is_valid_type_v = is_valid_type<T, Enum, SI>::value;
}  // namespace rtt::type_traits

#endif  // RTT_TYPE_TRAITS_HPP
