#include "roboteam_utils/Graph.h"
#include <gtest/gtest.h>
#include <vector>
#include <list>
#include <algorithm>
#include <boost/optional.hpp>

using namespace rtt;

template<typename C, typename T>
void assert_contains(C c, T v, bool invert = false) {
    ASSERT_TRUE(invert == (std::find(c.begin(), c.end(), v) == c.end()));
}

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
    assert_contains(g.neighbors(v3), Edge<>(v1, v3, 4.0));
    assert_contains(g.neighbors(v3), Edge<>(v2, v3, 2.0));
    ASSERT_TRUE(g.neighbors(v4).empty());
    std::list<Vertex<>> path;
    path.push_back(v1);
    path.push_back(v2);
    path.push_back(v3);
    boost::optional<std::list<Vertex<>>> found = g.find(v1, v3);
    ASSERT_TRUE((bool) found);
    ASSERT_EQ(*found, path);
    
}