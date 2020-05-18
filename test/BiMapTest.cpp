/* 	 ______   _______  _______  ______     _______  _______  ______   _______
 *	(  __  \ (  ____ \(  ___  )(  __  \   (  ____ \(  ___  )(  __  \ (  ____ \
 *	| (  \  )| (    \/| (   ) || (  \  )  | (    \/| (   ) || (  \  )| (    \/
 *	| |   ) || (__    | (___) || |   ) |  | |      | |   | || |   ) || (__
 *	| |   | ||  __)   |  ___  || |   | |  | |      | |   | || |   | ||  __)
 *	| |   ) || (      | (   ) || |   ) |  | |      | |   | || |   ) || (
 *	| (__/  )| (____/\| )   ( || (__/  )  | (____/\| (___) || (__/  )| (____/\
 *	(______/ (_______/|/     \|(______/   (_______/(_______)(______/ (_______/
 *
 * This corresponding class that is being tested contains only dead code. Remove this tag if you use this code and make sure to remove this tag at other places as well that will
 * become alive by using this code.
 * Do not read/document/redesign/analyse/test/optimize/etc. any of this code, because it is a waste of your time! This code was not removed or placed at another branch, because
 * other software developers are very attached to this code and are afraid that this code might be used at some day (but I think it won't be used at all and should be removed).
 */

#include <gtest/gtest.h>
#include "roboteam_utils/containers/BiMap.h"
#include <string>

namespace rtt {

int ione = 1, itwo = 2, ithree = 3, ifour = 4;
std::string sone = "one", stwo = "two", sthree = "three", sfour = "four";
    
TEST(BiMapTests, insertionRemoval) {
    BiMap<int, std::string> m;
    
    ASSERT_TRUE(m.insert(ione, sone));
    ASSERT_TRUE(m.insert(itwo, stwo));
    ASSERT_TRUE(m.insert(ithree, sthree));
    ASSERT_TRUE(m.insert(ifour, sfour));
    
    ASSERT_EQ(4, m.size());
    ASSERT_EQ("one", *m.lookupLeft(1));
    ASSERT_EQ(2, *m.lookupRight("two"));
    ASSERT_EQ(nullptr, m.lookupLeft(5));
    
    ASSERT_TRUE(m.removeByLeft(ithree));
    ASSERT_TRUE(m.removeByRight(sfour));
    
    ASSERT_EQ(2, m.size());
    ASSERT_EQ(nullptr, m.lookupLeft(ithree));
    ASSERT_EQ(nullptr, m.lookupRight(sthree));
    ASSERT_EQ(nullptr, m.lookupLeft(ifour));
}

TEST(BiMapTests, duplicates) {
    BiMap<int, std::string> m;
    
    ASSERT_TRUE(m.insert(ione, sone));
    ASSERT_TRUE(m.insert(itwo, stwo));
    
    ASSERT_EQ(sone, *m.lookupLeft(ione));
    ASSERT_FALSE(m.insert(ione, sthree));
    ASSERT_EQ(sone, *m.lookupLeft(ione));
    ASSERT_EQ(2, m.size());
}

TEST(BiMapTests, iteration) {
    BiMap<int, std::string> m;
    
    m.insert(ione, sone);
    m.insert(itwo, stwo);
    m.insert(ithree, sthree);
    m.insert(ifour, sfour);
    
    int total = 0;
    for (auto it = m.begin(); it != m.end(); it++) {
        total += it->first;
    }
    ASSERT_EQ(10, total);
    
    int prod = 1;
    for (const auto& pair : m) {
        prod *= pair.first;
    }
    ASSERT_EQ(24, prod);
}

}