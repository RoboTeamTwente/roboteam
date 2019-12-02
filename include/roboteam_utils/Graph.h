#ifndef RTT_GRAPH_H_
#define RTT_GRAPH_H_

#include <optional>
#include <gtest/gtest_prod.h>
#include <vector>
#include <list>
#include <stdint.h>
#include <math.h>
#include <map>
#include <string>
#include <sstream>
#include <functional>
#include "roboteam_utils/Vector2.h"

namespace rtt {

    template<typename T = void *>
    class Vertex {
    public:
        uint32_t id;
        std::optional<T> val;

        Vertex() {}

        Vertex(const uint32_t id, const std::optional<T> &val) : id{id}, val{val} {}

        bool operator<(const Vertex<T> &v) const { return id < v.id; }

        bool operator==(const Vertex &v) const { return id == v.id; }
    };

    template<typename T = void *>
    class Edge {
    public:
        Vertex<T> in, out;
        double cost;

        Edge(const Vertex<T> &in, const Vertex<T> &out, const double cost) : in(in), out(out), cost(cost) {}

        bool has(const Vertex<T> &v) const { return v == in || v == out; }

        Vertex<T> other_end(const Vertex<T> &v) const { return v == in ? out : in; }

        bool operator<(const Edge<T> &e) const { return cost < e.cost; }

        bool operator==(const Edge<T> &e) const { return has(e.in) && has(e.out) && cost == e.cost; }
    };

    template<typename T = void *>
    class Graph {

    private:
        FRIEND_TEST(GraphTests, structure);

        std::vector<std::vector<Edge<T>>> adj;
        std::vector<Vertex<T>> verts;
        uint32_t size;
    public:
        // Graph(): size(0) {} // no need, can just be default
        Graph() = default;

        Vertex<T> add_vertex(const std::optional<T> val = std::optional<T>()) {
            Vertex<T> v(size++, val);
            adj.push_back(std::vector<Edge<T>>());
            verts.push_back(v);
            return v;
        }

        void connect(const Vertex<T> &in, const Vertex<T> &out, const double cost) {
            Edge<T> e(in, out, cost);
            adj[in.id].push_back(e);
            //adj[out.id].push_back(e);
        }

        uint32_t get_size() const { return size; }

        Vertex<T> operator[](const uint32_t id) const { return verts[id]; }

        std::vector<Edge<T>> neighbors(const Vertex<T> &v) const { return adj[v.id]; }

        void reset() {
            size = 0;
            adj.clear();
            verts.clear();
        }

        std::optional<std::list<Vertex<T>>> find(const Vertex<T> &start, const Vertex<T> &goal) const {
            std::vector<Vertex<T>> q;
            std::map<Vertex<T>, Vertex<T>> parents;
            std::map<Vertex<T>, double> costs;

            q.push_back(start);
            costs[start] = 0;

            while (!q.empty()) {
                Vertex<T> u = *std::min_element(q.begin(), q.end(), [costs](Vertex<T> a, Vertex<T> b) {
                    return costs.at(a) < costs.at(b);
                });
                q.erase(std::remove(q.begin(), q.end(), u), q.end());

                if (u == goal) {
                    std::list<Vertex<T>> path;
                    path.push_front(goal);
                    Vertex<T> v = goal;
                    while (parents.find(v) != parents.end()) {
                        path.push_front(v = parents.at(v));
                    }
                    return std::optional<std::list<Vertex<T>>>(path);
                }

                for (const Edge<T> &e : adj[u.id]) {
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
            return std::nullopt;
        }

        template<typename K, typename V>
        inline V get_or_default(std::map<K, V> m, K key, V def) const {
            return m.find(key) == m.end() ? def : m.at(key);
        }

        std::optional<std::list<Vertex<T>>> astar(const Vertex<T> &start,
                                                  const Vertex<T> &goal,
                                                  std::function<double(const T &, const T &)> heuristic) const {
            std::vector<Vertex<T>> open, closed;
            open.push_back(start);
            std::map<Vertex<T>, Vertex<T>> parents;
            std::map<Vertex<T>, double> gscores, fscores;
            gscores[start] = 0.0;
            fscores[start] = heuristic(*(start.val), *(goal.val));
            while (!open.empty()) {
                Vertex<T> current = *std::min_element(open.begin(), open.end(),
                                                      [fscores](Vertex<T> a, Vertex<T> b) {
                                                          return fscores.at(a) < fscores.at(b);
                                                      });
                open.erase(std::remove(open.begin(), open.end(), current), open.end());
                if (current == goal) {
                    std::list<Vertex<T>> path;
                    path.push_front(goal);
                    Vertex<T> v = goal;
                    while (parents.find(v) != parents.end()) {
                        path.push_front(v = parents.at(v));
                    }
                    return std::optional<std::list<Vertex<T>>>(path);
                }
                closed.push_back(goal);
                for (const Edge<T> &e : adj[current.id]) {
                    Vertex<T> neighbor = e.other_end(current);
                    double tentative = get_or_default(gscores, current, 99999999.0) + e.cost;
                    if (std::find(open.begin(), open.end(), neighbor) == open.end()) {
                        open.push_back(neighbor);
                    } else if (tentative >= get_or_default(gscores, neighbor, 9999999.0)) {
                        continue;
                    }
                    parents[neighbor] = current;
                    gscores[neighbor] = tentative;
                    fscores[neighbor] = tentative + heuristic(*(neighbor.val), *(goal.val));
                }
            }
            return std::optional<std::list<Vertex<T>>>();
        }

        std::string to_DOT() const {
            std::stringstream ss;
            ss << "digraph {\n";
            for (unsigned int i = 0; i < size; i++) {
                ss << "  N" << i << ";\n";
            }
            for (const auto &from : adj) {
                for (const auto &to : from) {
                    ss << "  N" << to.in.id << " -> N" << to.out.id << " [label=\"" << to.cost << "\"];\n";
                }
            }
            ss << "}";
            return ss.str();
        }

        static std::string to_DOT(Graph<Vector2> graph) {
            std::stringstream ss;
            ss << "strict digraph {\n";
            for (unsigned int i = 0; i < graph.size; i++) {
                ss << "  N" << i << " [pos=\"" << graph[i].val->x * 10 + 300 << "," << graph[i].val->y * 10 + 200
                   << "!\"]\n";
            }
            for (const auto &from : graph.adj) {
                for (const auto &to : from) {
                    ss << "  N" << to.in.id << " -> N" << to.out.id
                       << " [label=\"" << to.cost << "\"];\n";
                }
            }
            ss << "}";
            return ss.str();
        }
    };


}

#endif