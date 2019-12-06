//
// Created by Yuhanun Citgez on 05/12/2019.
//

#ifndef RTT_UTILS_TYPE_TRAITS_H
#define RTT_UTILS_TYPE_TRAITS_H

namespace rtt::type_traits {
    template <typename T, typename = void>
    struct is_zero_initializable : std::false_type {};

    template <typename T>
    struct is_zero_initializable<T, decltype(T(0))> : std::true_type {};

    template <typename T>
    constexpr static bool is_zero_initializable_v = is_zero_initializable<T>::value;
} // namespace rtt::type_traits

#endif // RTT_UTILS_TYPE_TRAITS_H