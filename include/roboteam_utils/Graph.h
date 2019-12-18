#ifndef RTT_GRAPH_H_
#define RTT_GRAPH_H_

#include <optional>
#include <gtest/gtest_prod.h>
#include <vector>
#include <list>
#include <stdint.h>
#include <math.h>
#include <unordered_map>
#include <string>
#include <sstream>
#include <functional>
#include "roboteam_utils/Vector2.h"

namespace rtt {

    /**
     * @brief Vertex class
     * 
     * @tparam * Type of the internal vertex value
     */
    template<typename T = void *>
    class Vertex {
    public:
        /**
         * @brief ID of the current vertex
         * 
         */
        uint32_t id;

        /**
         * @brief Internal optional value of the vertex
         * 
         */
        std::optional<T> val;

        /**
         * @brief Construct a new Vertex object
         * 
         */
        Vertex() = default;

        /**
         * @brief Construct a new Vertex object
         * 
         * @param id ID to give to the vertex
         * @param val Value of the vertex
         */
        Vertex(const uint32_t id, const std::optional<T> &val) : id{id}, val{val} {}

        /**
         * @brief Less than operator
         * 
         * @param v rhs vertex
         * @return true True if this->id < v.id;
         * @return false False if this->id >= v.id;
         */
        bool operator<(const Vertex<T> &v) const { return id < v.id; }

        /**
         * @brief Equals operator
         * 
         * @param v rhs vertex
         * @return true True if the id of this vertex is the same as v's id
         * @return false False if id of this vertex is not the same as v's id
         */
        bool operator==(const Vertex &v) const { return id == v.id; }
    };


    /**
     * @brief Edge class
     * Owns 2 Vertex<T>'s
     * 
     * @tparam * Type of the internal vertices
     */
    template<typename T = void *>
    class Edge {
    public:
        /**
         * @brief Vertex one
         * 
         */
        Vertex<T> in;

        /**
         * @brief Vertex two
         * 
         */
        Vertex<T> out;

        // ??
        double cost;

        /**
         * @brief Construct a new Edge object
         * 
         * @param in Vertex one
         * @param out Vertex two
         * @param cost ??
         */
        Edge(const Vertex<T> &in, const Vertex<T> &out, const double cost) : in{ in }, out{ out }, cost{ cost } {}

        /**
         * @brief Checks whether the current vertex contains v as child
         * 
         * @param v Vertex to check
         * @return true True if vertex one or two is the same vertex as v
         * @return false False if this is not the case
         */
        bool has(const Vertex<T> &v) const { return v == in || v == out; }

        /**
         * @brief Gives the other end of the vertex
         * 
         * @param v Vertex to compare against
         * @return Vertex<T> Returns out if the in == 4, otherwise in
         */
        Vertex<T> other_end(const Vertex<T> &v) const { return v == in ? out : in; }

        /**
         * @brief Checks whether the cost of this < e
         * 
         * @param e Other edge to check against
         * @return true True if cost of current < e
         * @return false False if cost of current >= e;
         */
        bool operator<(const Edge<T> &e) const { return cost < e.cost; }

        /**
         * @brief Checks whether this edge is the same as e
         * 
         * @param e Edge to check against
         * @return true If both children and the cost are the same
         * @return false If a child or the cost is not the same
         */
        bool operator==(const Edge<T> &e) const { return has(e.in) && has(e.out) && cost == e.cost; }
    };

    /**
     * @brief Graph class
     * 
     * @tparam * Type of the values in the Vertices in the graph
     */
    template<typename T = void *>
    class Graph {

    private:
        FRIEND_TEST(GraphTests, structure);

        /**
         * @brief 2D vector of edges in the graph
         * 
         */
        std::vector<std::vector<Edge<T>>> adj;

        /**
         * @brief Vertices
         * 
         */
        std::vector<Vertex<T>> verts;

        /**
         * @brief Amount of vertices in the graph
         * 
         */
        uint32_t size;
    public:
        // Graph(): size(0) {} // no need, can just be default
        /**
         * @brief Construct a new Graph object
         * 
         */
        Graph() = default;

        /**
         * @brief Adds a vertex to the graph
         * 
         * Pushes a default constructed std::vector<Edge<T>> to adj
         * Pushes a constructor Vertex<T> where the id is the old amount of elements in the graph, increments size
         * 
         * @param val Value to give to the Vertex<T>
         * @return Vertex<T>& Newly created Vertex
         */
        Vertex<T>& add_vertex(const std::optional<T>& val = std::optional<T>()) {
            adj.emplace_back();
            return verts.emplace_back(size++, val);
        }

        /**
         * @brief Creates a new Edge<T>
         * 
         * @param in The in member for the Edge
         * @param out The out member for the Edge
         * @param cost The cost of the Edge
         */
        void connect(const Vertex<T> &in, const Vertex<T> &out, const double cost) {
            adj[in.id].emplace_back(in, out, cost);
        }

        /**
         * @brief Get the amount of Vertices in the graph
         * 
         * @return uint32_t Amount of Vertices in the graph
         */
        uint32_t get_size() const { return size; }

        /**
         * @brief Gets a Vertex with a specific id
         * 
         * @param id id o get from the vertices
         * @return Vertex<T> 
         */
        Vertex<T> operator[](const uint32_t id) const { return verts[id]; }

        /**
         * @brief Gets all the neighbours for a vertex with a certain id
         * 
         * @param v Vertex to compare against
         * @return std::vector<Edge<T>> Vector which contains all the neighbouring edges
         */
        std::vector<Edge<T>> neighbors(const Vertex<T> &v) const { return adj[v.id]; }

        /**
         * @brief Resets the current datastructure
         * 
         * Ensures that size is 0, and that adj and verts are empty
         * 
         */
        void reset() {
            size = 0;
            adj.clear();
            verts.clear();
        }

        /**
         * @brief Finds a certain vertex from a begin point
         * 
         * @param start The vertex to start at
         * @param goal To-look for vertex
         * @return std::optional<std::list<Vertex<T>>>  Returns an optional list of vertices that match this vertex
         */
        std::optional<std::list<Vertex<T>>> find(const Vertex<T> &start, const Vertex<T> &goal) const {
            std::vector<Vertex<T>> q;
            std::unordered_map<Vertex<T>, Vertex<T>> parents;
            std::unordered_map<Vertex<T>, double> costs;

            q.push_back(start);
            costs[start] = 0;

            while (!q.empty()) {
                Vertex<T> const& u = *std::min_element(q.begin(), q.end(), [costs](Vertex<T> a, Vertex<T> b) {
                    return costs.at(a) < costs.at(b);
                });
                q.erase(std::remove(q.begin(), q.end(), u), q.end());

                if (u == goal) {
                    std::list<Vertex<T>> path{ };
                    path.push_front(goal);
                    Vertex<T> v = goal;
                    while (parents.find(v) != parents.end()) {
                        path.push_front(v = parents.at(v));
                    }
                    return path;
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


        /**
         * @brief Get the object with the key or return a default value
         * 
         * @tparam K Type of `key`
         * @tparam V Type of `def`
         * @param m Map to traverse through
         * @param key Key to find
         * @param def default value incase Key is not found
         * @return V Either a defalut value `def` or a copy of the m[key]
         */
        template<typename K, typename V>
        inline V get_or_default(std::unordered_map<K, V> const& m, K const& key, V const& def) const {
            return m.find(key) == m.end() ? def : m[key];
        }

        /**
         * @brief A* pathfinding algorithm
         * 
         * @param start Vertex to start at
         * @param goal Goal vertex
         * @param heuristic Heuristic calculation function
         * @return std::optional<std::list<Vertex<T>>> Optionally gets a list of vertices which are the path from start -> goal, std::nullopt if no path is available
         */
        std::optional<std::list<Vertex<T>>> astar(const Vertex<T> &start,
                                                  const Vertex<T> &goal,
                                                  std::function<double(const T &, const T &)> heuristic) const {
            std::vector<Vertex<T>> open{ start };
            std::vector<Vertex<T>> closed;
            std::unordered_map<Vertex<T>, Vertex<T>> parents;
            std::unordered_map<Vertex<T>, double> gscores{ };
            std::unordered_map<Vertex<T>, double> fscores{ };
            gscores[start] = 0.0;
            fscores[start] = heuristic(*(start.val), *(goal.val));
            while (!open.empty()) {
                Vertex<T> current = *std::min_element(open.begin(), open.end(),
                                                      [&fscores](Vertex<T> a, Vertex<T> b) {
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
                    return path;
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
            return std::nullopt;
        }

        /**
         * @brief Returns a representation of the graph
         * 
         * @return std::string Representation of the graph
         */
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

        /**
         * @brief Gets a graph representation
         * 
         * @param graph const-qualified graph which is a reference to the graph to represent as a string
         * @return std::string String representation of \ref graph
         */
        static std::string to_DOT(Graph<Vector2> const& graph) {
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