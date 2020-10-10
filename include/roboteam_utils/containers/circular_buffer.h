//
// Created by rolf on 26-09-20.
//

#ifndef RTT_CIRCULAR_BUFFER_H
#define RTT_CIRCULAR_BUFFER_H

#include <new>
#include <stdexcept>
#include <array>

namespace rtt::collections {
/**
 * @brief A standard container for Circular buffers.
 * Note this class does not have iterators because they do not comply with the standard as there would be cases where
 * first == last. This quickly becomes very ugly.
 * The front of the buffer are the oldest elements, e.g. the ones first pushed in. These are the next to be erased.
 * @tparam Tp value type
 * @tparam Size largest size the buffer can hold
 */
    template<typename Tp, std::size_t Size>
    class circular_buffer {
    public:
        using value_type = Tp;
        using pointer = Tp*;
        using const_pointer = const Tp *;
        using reference = Tp&;
        using const_reference =  const Tp &;
        using size_type = std::size_t;

    private:
        // the internal array that actually stores the values
        std::array<Tp,Size> _m_buff;

        //The virtual beginning of the circular buffer
        size_type _current_offset;

        //The current size of the circular buffer
        size_type _current_size;

    public:
        //Default constructor
        circular_buffer() : _current_offset(0), _current_size(0) {
        }

        //modifying functions
        Tp &pop_front() {
            Tp &ret = _m_buff[_current_offset++];
            _current_offset %= Size;
            --_current_size;
            return ret;
        }

        void push_back(const Tp &value) {
            emplace_back(value);
        }

        void push_back(Tp &&value) {
            emplace_back(std::move(value));
        }
        template<typename... Types>
        reference emplace_back(Types &&... args) {
            if (_current_size < max_size()) {
                ++_current_size;
            } else {
                ++_current_offset %= Size;
            }
            new(&_m_buff[(_current_offset + _current_size - 1) % Size]) Tp{std::forward<Types>(args)...};
            return back();
        }
        //Capacity functions
        [[nodiscard]] constexpr size_type size() const noexcept { return _current_size; }

        [[nodiscard]] constexpr size_type max_size() const noexcept { return Size; }

        [[nodiscard]] constexpr bool empty() const noexcept { return size() == 0; }

        [[nodiscard]] constexpr bool full() const noexcept { return size() == Size; }

        //Element access
        constexpr reference operator[](size_type n) noexcept {
            return _m_buff[(_current_offset + n) % Size];
        }

        constexpr const_reference operator[](size_type n) const noexcept {
            return _m_buff[(_current_offset + n) % Size];
        }

        constexpr reference at(size_type n) {
            if (n >= Size) {
                throw std::out_of_range("index out of range of circular buffer size!");
            }
            return _m_buff[(_current_offset + n) % Size];
        }

        //Result of expression must be lvalue so we use inline boolean as follows:
        constexpr const_reference at(size_type n) const {
            if (n >= Size) {
                throw std::out_of_range("index out of range of circular buffer size!");
            }
            return _m_buff[(_current_offset + n) % Size];
        }

        constexpr reference front() noexcept { return _m_buff[_current_offset]; }

        constexpr const_reference front() const noexcept { return _m_buff[_current_offset]; }

        constexpr reference back() noexcept { return _m_buff[(_current_offset + _current_size - 1) % Size]; }

        constexpr const_reference back() const noexcept {
            return _m_buff[(_current_offset + _current_size - 1) % Size];
        }

    };
}
#endif //RTT_CIRCULAR_BUFFER_H
