//
// Created by john on 3/4/20.
//

#ifndef RTT_STATE_MACHINE_HPP
#define RTT_STATE_MACHINE_HPP

#include <vector>
#include <memory>
#include <algorithm>

#include "type_traits.hpp"

namespace rtt::collections {
    /**
     * State machine class
     * @tparam Base Base class of all elements, all elements must derive from this
     * Must have: Enum update(SI const&), void initialize(), void terminate()
     * @tparam Enum Enum type used for status
     * @tparam SI Type of information that's passed down to update()
     */
    template <typename Base, typename Enum, typename SI>
    class state_machine {
    private:
        /**
         * Internal object vector, everything is emplace_back'd
         */
        std::vector<std::unique_ptr<Base>> _data;

        /**
         * Current index into state data, once current == size(), finished() == true
         */
        size_t current = 0;

        /**
         * Assertion that ensures that Base doesn't have a `const`
         */
        static_assert(std::is_same_v<Base, std::remove_const_t<Base>>, "The ISO standard forbids usage of a constant type in std::vector, please use a mutable type");

        /**
         * Assertions that ensures that all the member functions are present
         */
        static_assert(type_traits::is_valid_type_v<Base, Enum, SI>, "Type Base does not have the required functions (noexcept): Enum update(SI const&), void initialize(), void terminate()");

        /**
         * STL-style using's
         */
        using iterator = typename decltype(_data)::iterator;
        using const_iterator = typename decltype(_data)::const_iterator;
        using size_type = typename decltype(_data)::size_type;

    public:
        /**
         * Constructor that takes any amount of arguments, all of which must inherit from Base
         *
         * ~~~{cpp}
         * struct Base {}; // imagine proper members
         * struct Derived : public Base {};
         * struct AnotherDerived : public Derived{};
         *
         * enum EnumType{}; // again, imagine proper members
         *
         * enum SkillInfo{};
         *
         * state_machine<Base, EnumType, SkillInfo>{ Base(), Derived(), AnotherDerived() };
         * ~~~
         *
         * @tparam Tys Types of arguments
         * @param args Argument pack
         */
        template <typename...Tys>
        explicit state_machine(Tys&&... args) noexcept {
            // sizeof... gets the amount of elements in a pack
            // https://en.cppreference.com/w/cpp/language/sizeof...
            _data.reserve(sizeof...(args));
            // Fold expression
            /**
             * Fold expression, essentially:
             * for element in args:
             *  data.emplace_back(make_unique(ensure_reference_type(args))
             *
             * std::forward is a conditional cast to r-value reference, it's like std::move()
             * but it won't std::move if it wasn't already std::move'd into the constructor
             *
             * You could for example print an argument pack by doing the following
             *
             * template <typename...Tys>
             * void print_all(Tys... args) { // Copy for clarity
             *      ((std::cout << args), ...);
             * }
             *
             * Now say you'd want to add a space:
             * ((std::cout << args << " "), ...);
             *
             * Add up a pack of ints together?
             *
             * int result = (args + ...);
             */
            (_data.emplace_back(std::make_unique<Tys>(std::forward<Tys>(args))), ...);

            // Call initialize on the first element if there is an element to initialize
            if (size()) initialize();
        }

        /**
         * Gets a mutable iterator to the first element
         * @return std::vector<Base>::iterator
         */
        iterator begin() noexcept {
            return _data.begin();
        }

        /**
         * Gets a mutable iterator to 1 past the last element
         * @return std::vector<Base>::iterator
         */
        iterator end() noexcept {
            return _data.end();
        }

        /**
         * Gets an immutable iterator to the first element
         * @return std::vector<Base>::const_iterator
         */
        const_iterator begin() const noexcept {
            return _data.cbegin();
        }

        /**
         * Gets an immutable iterator to 1 past the last element
         * @return std::vector<Base>::const_iterator
         */
        const_iterator end() const noexcept {
            return _data.cend();
        }

        /**
         * Skips n elements, sets current to clamp(current + n, _data.size())
         * @param n Amount of elements to skip
         */
        void skip_n(int n) noexcept {
            current += n;
            current = std::clamp<size_t>(current, 0, _data.size());
        }

        /**
         * Checks whether al elements have been finalized
         * @return current_num() == total_count();
         */
        [[nodiscard]] bool finished() const noexcept {
            return current_num() == total_count();
        }

        /**
         * Gets current index into state machines
         * @return this->current
         */
        [[nodiscard]] size_t current_num() const noexcept {
            return current;
        }

        /**
         * Gets total amount of elements in the state machine
         * @return vector.size();
         */
        [[nodiscard]] size_type total_count() const noexcept {
            return _data.size();
        }

        /**
         * Gets the amount of elements in the vector
         * @return vector.size()
         */
        [[nodiscard]] size_type size() const noexcept {
            return _data.size();
        }

        /**
         * Updates the current element in the vector
         * if get_current() returns nullptr, this function returns failure
         * @param data Data to pass to .update(data);
         * @return .update(data)
         */
        [[nodiscard]] Enum update(SI const& data) noexcept {
            auto current_elem = get_current();
            if (!current_elem) {
                return Enum::Failure;
            }
            auto value = current_elem->update(data);
            if (value == Enum::Success) {
                terminate();
                if (!finished()) {
                    initialize();
                }
            }
            return value;
        }

        /**
         * Calls terminate() on the current Base
         * begin(0[current_num()]->terminate();
         */
        void terminate() noexcept {
            auto _current = get_current();
            if (!_current) {
                return;
            }
            _current->terminate();
            skip_n(1);
        }

        /**
         * Calls initialize on the current node.
         * begin[current_num()]->initialize();
         */
        void initialize() noexcept {
            get_current()->initialize();
        }

        /**
         * Gets a Base* to the current element
         * If curr_index exceeds the size -> nullptr is returned
         * Literally:
         *  begin()[current_num()].get();
         * @return Base* to that element
         */
        [[nodiscard]] Base* get_current() noexcept {
            auto curr_index = current_num();
            if  (curr_index >= size()) {
                return nullptr;
            }
            return begin()[curr_index].get();
        }
        /**
         * Resets the state machine
         */
        void reset() noexcept {
            skip_n(-current_num());
        }
    };
} // namespace rtt::collections

#endif //RTT_STATE_MACHINE_HPP
