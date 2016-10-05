#include "roboteam_utils/Graph.h"
#include <exception>
#include <algorithm>
#include <map>
#include <list>

namespace rtt {
    

template<typename T>
Vertex<T> Graph<T>::add_vertex(const boost::optional<T> val) {
    Vertex<T> v(size++, val);
    adj.push_back(std::vector<Edge<T>>());
    verts.push_back(v);
}

template<typename T>
void Graph<T>::connect(const Vertex<T>& in, const Vertex<T>& out, const double cost) {
    Edge<T> e(in, out, cost);
    adj[in.id].push_back(e);
    adj[out.id].push_back(e);
}

template<typename T>
uint32_t Graph<T>::get_size() { return size; }

template<typename T>
Vertex<T> Graph<T>::operator[](const uint32_t id) {
    return verts[id];
}

template<typename T>
std::vector<Vertex<T>> Graph<T>::neighbors(const Vertex<T>& v) {
    return adj[v.id];
}

template<typename T>
boost::optional<std::list<Vertex<T>>> Graph<T>::find(const Vertex<T>& start, const Vertex<T>& goal) {
    std::vector<Vertex<T>> q;
    std::map<Vertex<T>, Vertex<T>> parents;
    std::map<Vertex<T>, double> costs;
        
    q.push_back(start);
    costs[start] = 0;
    
    while (!q.empty()) {
        Vertex<T> u = *min_element(q.begin(), q.end(), [costs](Vertex<T> a, Vertex<T> b) { return costs[a] < costs[b]; });
        q.erase(std::remove(q.begin(), q.end(), u), q.end());
        
        if (u == goal) {
            std::list<Vertex<T>> path;
            Vertex<T> v = goal;
            while (parents.find(v) != parents.end()) {
                path.push_front(v = parents[v]);
            }
            return boost::optional<std::list<Vertex<T>>>(path);
        }
        
        q.pop();
        for (const Edge<T>& e : adj[u.id]) {
            Vertex<T> other = e.other_end(u);
            if (costs.find(other) == costs.end()) {
                costs[other] = costs[u] + e.cost;
                parents[other] = u;
            } else {
                double alt = costs[u] + e.cost;
                if (alt < costs[other]) {
                    costs[other] = alt;
                    parents[other] = u;
                }   
            }
        }
    }
    return boost::optional<std::list<Vertex<T>>>();
}
    
}