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

#include "roboteam_utils/Graph.h"
#include <gtest/gtest.h>
#include <vector>
#include <list>
#include <algorithm>
#include <optional>

using namespace rtt;

template<typename C, typename T>
void assert_contains(C c, T v, bool invert = false) {
    ASSERT_TRUE(invert == (std::find(c.begin(), c.end(), v) == c.end()));
}
/*
TEST(GraphTests, structure) {
    Graph<> g;
    Vertex<> v1 = g.add_vertex();
    Vertex<> v2 = g.add_vertex();
    Vertex<> v3 = g.add_vertex();
    Vertex<> v4 = g.add_vertex();
    g.connect(v1, v2, 1.0);
    g.connect(v1, v3, 4.0);
    g.connect(v2, v3, 2.0);
    assert_contains(g.neighbors(v1), Edge<>(v1, v2, 1.0));
    assert_contains(g.neighbors(v1), Edge<>(v1, v3, 4.0));
    assert_contains(g.neighbors(v2), Edge<>(v2, v3, 2.0));
    assert_contains(g.neighbors(v3), Edge<>(v1, v3, 4.0), true);
    assert_contains(g.neighbors(v3), Edge<>(v2, v3, 2.0), true);
    ASSERT_TRUE(g.neighbors(v4).empty());
    std::list<Vertex<>> path;
    path.push_back(v1);
    path.push_back(v2);
    path.push_back(v3);
    std::optional<std::list<Vertex<>>> found = g.find(v1, v3);
    ASSERT_TRUE((bool) found);
    ASSERT_EQ(*found, path);
    
}
*/

void assert_path_equals(Graph<int> g, Vertex<int> start, Vertex<int> goal, std::list<Vertex<int>>* expected) {
    std::optional<std::list<Vertex<int>>> found = g.astar(start, goal, [](const int& a, const int& b) { return b - a; } );
    
    if (expected) {
        ASSERT_TRUE((bool) found);
        ASSERT_EQ(*expected, *found);
    } else {
        ASSERT_FALSE((bool) found);
    }
}

TEST(GraphTests, astar) {
    Graph<int> g;
    Vertex<int> v1 = g.add_vertex(std::optional<int>(0));
    Vertex<int> v2 = g.add_vertex(std::optional<int>(1));
    Vertex<int> v3 = g.add_vertex(std::optional<int>(2));
    Vertex<int> v4 = g.add_vertex(std::optional<int>(1));
    Vertex<int> v5 = g.add_vertex(std::optional<int>(2));
    g.connect(v1, v4, 5.0);
    g.connect(v1, v2, 1.0);
    g.connect(v2, v3, 2.0);
    g.connect(v3, v4, 1.0);
    g.connect(v4, v5, 1.5);
    g.connect(v3, v5, 4.0);
    
    std::list<Vertex<int>> path_1_5, path_2_3, path_4_4;
    path_1_5.push_back(v1);
    path_1_5.push_back(v2);
    path_1_5.push_back(v3);
    path_1_5.push_back(v4);
    path_1_5.push_back(v5);
    path_2_3.push_back(v2);
    path_2_3.push_back(v3);
    path_4_4.push_back(v4);
    
    assert_path_equals(g, v1, v5, &path_1_5);
    assert_path_equals(g, v2, v3, &path_2_3);
    assert_path_equals(g, v4, v4, &path_4_4);
    assert_path_equals(g, v4, v1, nullptr);
    assert_path_equals(g, v5, v4, nullptr);
}