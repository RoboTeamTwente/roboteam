#ifndef RTT_GRAPH_H_
#define RTT_GRAPH_H_

#include <boost/optional.hpp>
#include <gtest/gtest_prod.h>
#include <vector>
#include <list>
#include <stdint.h>
#include <map>
#include <string>
#include <sstream>

namespace rtt {

template<typename T = void*>
class Vertex {
public:
    uint32_t id;
    boost::optional<T> val;
    
    Vertex() {}
    Vertex(const uint32_t id, const boost::optional<T> val): id(id), val(val) {}
    bool operator<(const Vertex<T>& v) const { return id < v.id; }
    bool operator==(const Vertex& v) const { return id == v.id; }    
};

template<typename T = void*>
class Edge {
public:
    Vertex<T> in, out;
    double cost;
        
    Edge(const Vertex<T>& in, const Vertex<T>& out, const double cost) : in(in), out(out), cost(cost) {}
    bool has(const Vertex<T>& v) const { return v == in || v == out; }
    Vertex<T> other_end(const Vertex<T>& v) const { return v == in ? out : in; }
    bool operator<(const Edge<T>& e) const { return cost < e.cost; }
    bool operator==(const Edge<T>& e) const { return has(e.in) && has(e.out) && cost == e.cost; }
};
    
template <typename T = void*>
class Graph {
    
private:
    FRIEND_TEST(GraphTests, structure);
    std::vector<std::vector<Edge<T>>> adj;
    std::vector<Vertex<T>> verts;
    uint32_t size;
public:
    Graph(): size(0) {}
    Vertex<T> add_vertex(const boost::optional<T> val = boost::optional<T>()) {
        Vertex<T> v(size++, val);
        adj.push_back(std::vector<Edge<T>>());
        verts.push_back(v);
        return v;
    }
    
    void connect(const Vertex<T>& in, const Vertex<T>& out, const double cost) {
        Edge<T> e(in, out, cost);
        adj[in.id].push_back(e);
        //adj[out.id].push_back(e);
    }
    
    uint32_t get_size() const { return size; }
    
    Vertex<T> operator[](const uint32_t id) const { return verts[id]; }
    
    std::vector<Edge<T>> neighbors(const Vertex<T>& v) const { return adj[v.id]; }
    
    void reset() {size = 0; adj.clear(); verts.clear(); }
    
    boost::optional<std::list<Vertex<T>>> find(const Vertex<T>& start, const Vertex<T>& goal) const {
        std::vector<Vertex<T>> q;
        std::map<Vertex<T>, Vertex<T>> parents;
        std::map<Vertex<T>, double> costs;
        
        q.push_back(start);
        costs[start] = 0;
    
        while (!q.empty()) {
            Vertex<T> u = *std::min_element(q.begin(), q.end(), [costs](Vertex<T> a, Vertex<T> b) { return costs.at(a) < costs.at(b); });
            q.erase(std::remove(q.begin(), q.end(), u), q.end());
           
            if (u == goal) {
                std::list<Vertex<T>> path;
                path.push_front(goal);
                Vertex<T> v = goal;
                while (parents.find(v) != parents.end()) {
                    path.push_front(v = parents.at(v));
                }
                return boost::optional<std::list<Vertex<T>>>(path);
            }
        
            for (const Edge<T>& e : adj[u.id]) {
                Vertex<T> other = e.other_end(u);
                if (costs.find(other) == costs.end()) {
                    costs[other] = costs[u] + e.cost;
                    parents[other] = u;
                    q.push_back(other);
                } else {
                    double alt = costs[u] + e.cost;
                    if (alt < costs[other]) {
                        costs[other] = alt;
                        parents[other] = u;
                        q.push_back(other);
                    }   
                }
            }
        }
        return boost::optional<std::list<Vertex<T>>>();
    }
    
    std::string to_DOT() const {
        std::stringstream ss;
        ss << "digraph Graph {\n";
        for (unsigned int i = 0; i < size; i++) {
            ss << "N_" << i << ";\n";
        }
        for (const auto& from : adj) {
            for (const auto& to : from) {
                ss << "N_" << to.in.id << "->N_" << to.out.id << " [label=" << to.cost << "];\n";
            }
        }
        ss << "}";
        return ss.str();
    }
};
    
}

#endif