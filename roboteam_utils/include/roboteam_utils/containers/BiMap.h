/* 	 ______   _______  _______  ______     _______  _______  ______   _______
 *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
 *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
 *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
 *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
 *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
 *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
 *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
 *
 * This class contains only dead code. Remove this tag if you use this code and make sure to remove this tag at other places as well that will become alive by using this code.
 * Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at another branch, because
 * other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all and should be removed).
 */

#pragma once

#include <functional>
#include <type_traits>
#include <unordered_set>
#include <utility>

namespace rtt {

/**
 * @brief BiMap, currently unused
 *
 * @tparam T First type element in set
 * @tparam U Second type element in set
 */
template <typename T, typename U>
class BiMap {
   public:
    using LeftPair = std::pair<T, U *>;
    using RightPair = std::pair<U, T *>;

    /**
     * @brief Inserts a pair into both leftSet and rightSet
     * Requires left and right to live at least as long as the owning BiMap
     * If not, undefined behavior will occur due to dangling pointers when dereferencing
     *
     * @param left l-value reference to type `T` which is inserted into both maps
     * @param right l-value reference to type `U` which is inserted into both maps
     *
     * @return Returns false if the leftSet already contains \ref `left` else true
     */
    bool insert(T &left, U &right) {
        if (containsLeft(left)) {
            return false;
        }

        leftSet.emplace(left, &right);
        rightSet.emplace(right, &left);
        return true;
    }

    /**
     * @brief Checks whether \ref `key` is already contained in leftSet
     *
     * @param key Key value to be compared
     * @return true if key is already contained
     * @return false if key is not contained
     */
    bool containsLeft(const T &key) { return leftSet.find({key, nullptr}) != leftSet.end(); }

    /**
     * @brief Checks whether \ref `key` is already contained in rightSet
     *
     * @param key Key value to be compared
     * @return true if key is already contained
     * @return false false if key is not contained
     */
    bool containsRight(const U &key) { return rightSet.find({key, nullptr}) != rightSet.end(); }

    /**
     * @brief Removes element from leftSet if it's contained
     *
     * @param key Key to be checked against set
     * @return true If key is found and element is erased
     * @return false If key is not found
     */
    bool removeByLeft(const T &key) {
        auto it = leftSet.find({key, nullptr});
        if (it == leftSet.end()) {
            return false;
        }

        rightSet.erase({*it->second, nullptr});
        leftSet.erase({key, nullptr});
        return true;
    }

    /**
     * @brief Removes element from rightSet if it's contained
     *
     * @param key Key to be checked against set
     * @return true If key is found and element is erased
     * @return false If key is not found
     */
    bool removeByRight(const U &key) {
        auto it = rightSet.find({key, nullptr});
        if (it == rightSet.end()) {
            return false;
        }
        leftSet.erase({*it->second, nullptr});
        rightSet.erase({key, nullptr});
        return true;
    }

    /**
     * @brief Looks up element in leftSet and returns second element
     *
     * @param key
     * @return U* Returns nullptr if not found, otherwise it returns the second element
     */
    U *lookupLeft(const T &key) const {
        auto it = leftSet.find({key, nullptr});
        return it == leftSet.end() ? nullptr : it->second;
    }

    /**
     * @brief Looks up element in rightSet and returns second element
     *
     * @param key
     * @return U* Returns nullptr if not found, otherwise it returns the second element
     */
    T *lookupRight(const U &key) const {
        auto it = rightSet.find({key, nullptr});
        return it == rightSet.end() ? nullptr : it->second;
    }

    /**
     * @brief Gives amount of elements in the sets, individually, not combined
     *
     * @return size_t Amount of elements
     */
    size_t size() const { return leftSet.size(); }

    /**
     * @brief Struct used for leftSet comparison
     *
     */
    typedef struct LeftComp : std::binary_function<LeftPair, LeftPair, bool> {
        /**
         * @brief Operator() definition for Comparable
         *
         * @param a lhs
         * @param b rhs
         * @return true if a.first is smaller than b.first
         * @return false if b.first >= a.first
         */
        bool operator()(const LeftPair &a, const LeftPair &b) const { return a.first < b.first; }
    } LeftComp;

    /**
     * @brief Struct used for rightSet comparison
     *
     */
    typedef struct RightComp : std::binary_function<RightPair, RightPair, bool> {
        /**
         * @brief Operator() definition for Comparable
         *
         * @param a lhs
         * @param b rhs
         * @return true if a.first is smaller than b.first
         * @return false if b.first >= a.first
         */
        bool operator()(const RightPair &a, const RightPair &b) const { return a.first < b.first; }
    } RightComp;

    /**
     * @brief Returns an iterator to the leftSet to be able to range-based for loop over it
     *
     * @return std::set<LeftPair, LeftComp>::const_iterator Const pointer to the begin of leftSet
     */
    typename std::set<LeftPair, LeftComp>::const_iterator begin() const { return leftSet.cbegin(); }

    /**
     * @brief Returns an iterator to the leftSet to be able to range-based for loop over it

     *
     * @return std::set<LeftPair, LeftComp>::const_iterator Const pointer to the end of leftSet
     */
    typename std::set<LeftPair, LeftComp>::const_iterator end() const { return leftSet.cend(); }

   private:
    /**
     * @brief Internal set used for BiMap implementation
     *
     */
    std::unordered_set<LeftPair, LeftComp> leftSet;

    /**
     * @brief Internal set used for BiMap implementation
     *
     */
    std::unordered_set<RightPair, RightComp> rightSet;
};
}  // namespace rtt