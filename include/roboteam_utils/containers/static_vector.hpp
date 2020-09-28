//
// Created by john on 25/9/20.
//

#ifndef RTT_STATIC_VECTOR_HPP
#define RTT_STATIC_VECTOR_HPP

#include <new>
#include <array>
#include <cstddef>
#include <stdexcept>
#include <type_traits>

namespace rtt::collections {

    /**
     * @brief Container that behaves like std::vector but is allocated on the stack with a max capacity
     * @tparam T Type to be contained
     * @tparam Cap size_t, maximum capacity of the static_vector.
     * 
     * Opposing to std::vector iterators are never invalidated unless the entire structure is dropped.
     */
    template<typename T, size_t Cap>
    class static_vector {
    public:
        /**
         * Contained type
         */
        using element_type = T;

        /**
         * Maximum size of the internal array
         */
        constexpr static size_t max_size = Cap;

        /**
         * Type of container, std::array<T, Cap>
         */
        using container_type = std::array<element_type, max_size>;

        /**
         * Iterator used for begin() and end()
         */
        using iterator = element_type *;

        /**
         * Iterator used for cbegin() and cend()
         */
        using const_iterator = const element_type*;

        /**
         * Type of the size (size_t)
         */
        using size_type = typename container_type::size_type;

    private:
        /**
         * Check whether it's default constructible, std::array requires it.
         */
        static_assert(std::is_default_constructible_v<element_type>,
                      "T has to be default constructible due to the nature of std::array.");

        /**
         * Check whether it's either copy or move constructible.
         */
        static_assert(
                std::is_nothrow_copy_constructible_v<element_type> || std::is_nothrow_move_assignable_v<element_type>,
                "T is not noexcept move or copyable.");

        /**
         * Check whether the contained type is constant, if so we cannot placement-new
         */
        static_assert(
                !std::is_const_v<element_type>, "T cannot be const.");

        /**
         * Internal array used for storage
         */
        std::array<element_type, max_size> _data;

        /**
         * Current size
         */
        size_type _size = 0;

    public:
        /**
         * Construction from initializer list.
         */
        constexpr  static_vector(std::initializer_list<T> data) {
            for (const auto& elem : data) {
                if constexpr (std::is_nothrow_move_constructible_v<element_type>) {
                    this->emplace_back(std::move(elem));
                } else {
                    this->emplace_back(elem);
                }
            }
        }

        static_vector() = default;

        /**
         * Push back an element (const&)
         */
        void push_back(const element_type &data) {
            this->emplace_back(data);
        }

        /**
         * Push back an element (copy)
         */
        void push_back(element_type&& data) {
            this->emplace_back(std::move(data));
        }

        /**
         * Emplace back an element
         * @tparam Parameters passed to ctor
         */
        template<typename... Tys>
        element_type &emplace_back(Tys &&... args) {
            if (size() >= max_size) {
                throw std::runtime_error{"Error in static_vector, attempt to append to filled static_vector."};
            }
            new(&_data[size()]) T{std::forward<Tys>(args)...};
            _size++;
            return back();
        }

        /**
         * Current size
         */
        size_type size() const noexcept {
            return _size;
        }

        /**
         * Begin iterator used for range-based for loop
         */
        constexpr iterator begin() noexcept {
            return _data.data();
        }

        constexpr const_iterator begin() const noexcept {
            return cbegin();
        }
        /**
         * End iterator used for range-based for loop
         */
        constexpr iterator end() noexcept {
            return &_data[size()];
        }

        constexpr const_iterator end() const noexcept {
            return cend();
        }
        /**
         * Begin iterator used for range-based for loop
         * `this` is const.
         */
        constexpr const_iterator cbegin() const noexcept {
            return &*_data.cbegin();
        }

        /**
         * End iterator used for range-based for loop
         * `this` is const.
         */
        constexpr const_iterator cend() const noexcept {
            return &_data[size() - 1];
        }

        /**
         * Reference to the first element
         */
        element_type &front() noexcept {
            return _data.front();
        }

        /**
         * Reference to the last element
         */
        element_type &back() noexcept {
            return _data[size() - 1];
        }

        /**
         * Pointer to the internal data
         */
        iterator data() noexcept {
            return &front();
        }

        /**
         * Reference to the first element
         */
        const element_type &front() const noexcept {
            return _data.front();
        }

        /**
         * Reference to the last element
         */
        const element_type &back() const noexcept {
            return _data[size() - 1];
        }

        /**
         * Pointer to the internal data
         */
        const_iterator data() const noexcept {
            return &front();
        }

        /**
         * operator[] defined to index the internal array
         */
        element_type &operator[](size_type index) noexcept {
            return _data[index];
        }


        /**
         * Constant version of operator[]
         */
        element_type const &operator[](size_type index) const noexcept {
            return _data[index];
        }

        /**
         * Erases an element from the array, takes an iterator.
         * 
         * To obtain a const_iterator to the element, the following is done 
         * to go from index -> elemnt
         * 
         * ~~~{cpp}
         * vec.erase(vec.begin() + index);
         * ~~~
         */
        void erase(iterator iter) {
            element_type* prev = iter++;
            if constexpr (std::is_nothrow_move_constructible_v<element_type>) {
                std::copy(iter, end(), prev);
            } else {
                std::move(iter, end(), prev);
            }
            _size--;
        }
    };
} // namespace rtt::collections

#endif // RTT_STATIC_VECTOR_HPP
