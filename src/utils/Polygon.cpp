//
// Created by rolf on 24-4-19.
//

#include <optional>
#include "Polygon.h"
#include <numeric>
#include <utility>

namespace rtt {
    ///constructor for rectangles oriented straight with respect to the x-axis.
    Polygon::Polygon(const Vector2 &lowerLeftCorner, double xlen, double ylen)
        : vertices {
            lowerLeftCorner,
            Vector2{ lowerLeftCorner.x + xlen, lowerLeftCorner.y },
            Vector2{ lowerLeftCorner.x + xlen, lowerLeftCorner.y + ylen },
            Vector2{ lowerLeftCorner.x, lowerLeftCorner.y + ylen }
        }
    {}

    Polygon::Polygon(std::vector<rtt::Vector2> vertices)
        : vertices{std::move(vertices)}
    {}

    size_t Polygon::amountOfVertices() const {
        return vertices.size();
    }

    Vector2 Polygon::operator[](size_t idx) const {
        return vertices[idx];
    }

    void Polygon::move(const Vector2 &moveBy) {

        for (auto& vertex : vertices) {
            vertex += moveBy;
        }
    }

    std::vector<LineSegment> Polygon::getBoundary() const {
        std::vector<LineSegment> boundary;
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {
            boundary.emplace_back(vertices[i], vertices[(i + 1) % n]); 
            // maybe there is a nice way to do this 'circular' access with iterators?
        }
        return boundary;
    }

    double Polygon::perimeterLength() const {
        double totalLength = 0;
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {
            totalLength += (vertices[i] - vertices[(i + 1) % n]).length();// maybe there is a nice way to do this 'circular' access with iterators?

        }
        return totalLength;
    }

    //https://stackoverflow.com/questions/471962/how-do-i-efficiently-determine-if-a-polygon-is-convex-non-convex-or-complex
    // this function only works if your polygon is already simple. If your polygon is not simple, it is highly like to not be convex
    bool Polygon::isConvex() const {
        if (amountOfVertices() < 4) 
            return true;// triangles are always convex
        bool sign = false;
        bool signSet = false;
        int n = vertices.size();
        for (int i = 0; i < n; i++) {
            Vector2 d1 = vertices[(i + 2) % n] - vertices[(i + 1) % n];
            Vector2 d2 = vertices[i] - vertices[(i + 1) % n];
            double cross = d1.cross(d2);
            // on a crossproduct of zero the points lie in one line and we can simply ignore this point's contribution to the convexity
            if (cross != 0.0) {
                if (!signSet) {
                    signSet = true;
                    sign = cross > 0;
                } else if (sign != (cross > 0)) {
                    return false;
                }
            }
        }
        return true;
    }

    //https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
    // this is black magic but if it works it works
    /// on the boundary this function does not work!! see documentation if you are interested
    bool Polygon::contains(const Vector2 &point) const {
        int c = 0;
        int n = amountOfVertices();
        for (int i = 0, j = n - 1; i < n; j = i++) {
            if (((vertices[i].y > point.y) != (vertices[j].y > point.y)) &&
                (point.x < (vertices[j].x - vertices[i].x) * (point.y - vertices[i].y) / (vertices[j].y - vertices[i].y)
                           + vertices[i].x))
                c = !c;
        }
        return c;
    }

    bool Polygon::doesIntersect(const LineSegment &line) const {
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {
            LineSegment segment(vertices[i],
                                vertices[(i + 1) %
                                         n]);// maybe there is a nice way to do this 'circular' access with iterators?
            if (line.doesIntersect(segment)) {
                return true;
            }
        }
        return false;
    }

    bool Polygon::isOnBoundary(const Vector2 &point) const {
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {
            LineSegment segment(vertices[i],
                                vertices[(i + 1) %
                                         n]);// maybe there is a nice way to do this 'circular' access with iterators?
            if (segment.isOnLine(point)) {
                return true;
            }
        }
        return false;
    }

    std::vector<Vector2> Polygon::intersections(const LineSegment &line) const {
        std::vector<Vector2> intersections;
        size_t n = vertices.size();
        for (size_t i = 0; i < n; i++) {
            LineSegment segment(vertices[i], vertices[(i + 1) % n]);
            std::vector<Vector2> line_intersections = line.multiIntersect(segment);
            intersections.insert(intersections.end(), line_intersections.begin(), line_intersections.end());
        }
        std::sort(intersections.begin(), intersections.end());
        intersections.erase(std::unique(intersections.begin(), intersections.end()), intersections.end());
        return intersections;
    }

    //https://en.wikipedia.org/wiki/Shoelace_formula
    // area is well defined only for simple polygons
    double Polygon::area() const {
        return 0.5 * abs(doubleSignedArea());
    }

    double Polygon::doubleSignedArea() const {
        int n = vertices.size();
        double sum = 0;
        for (int i = 0; i < n; i++) {
            sum += vertices[i].cross(vertices[(i + 1) % n]);
        }
        return sum;

    }

    //https://en.wikipedia.org/wiki/Centroid
    // only works for simple polygons
    Vector2 Polygon::centroid() const {
        // for triangles we can use a faster and simpler formula
        if (amountOfVertices() < 4) {
            return verticeCentroid();
        }
        double signedAreaTwice = doubleSignedArea();
        //calculation can still make sense in a geometric sense but this should probably raise a warning
        if (signedAreaTwice == 0) {
            std::cerr << "Computing the centroid for a polygon without area" << std::endl;
        }
        int n = vertices.size();
        Vector2 sum = {0, 0};
        for (int i = 0; i < n; i++) {
            sum += (vertices[i] + vertices[(i + 1) % n]) * vertices[i].cross(vertices[(i + 1) % n]);
        }
        return sum /= (3 * signedAreaTwice);
    }

    // the centroid of the set of vertices. is generally NOT the same as centroid for polygons with more than 3 sides
    Vector2 Polygon::verticeCentroid() const {
        return std::accumulate(vertices.begin(), vertices.end(), Vector2(0, 0)) /= vertices.size();
    }

    bool Polygon::isSimple() const {
        // we loop over every unique pair
        std::vector<LineSegment> lines{ };
        size_t numberIntersections = 0;
        for (auto first = vertices.begin(); first != vertices.end(); first++) {
            std::optional<LineSegment> boundarySegment;
            if (first == std::prev(vertices.end())) {
                boundarySegment = LineSegment(*first, vertices[0]);
            } else {
                boundarySegment = LineSegment(*first, *(first + 1));
            }
            for (auto const& line : lines) {
                numberIntersections += boundarySegment.value().multiIntersect(line).size();
            }
            lines.push_back(boundarySegment.value());
        }
        /* - A polygon (simple/non-simple) can never have less than the amount of vertices as intersections. Because every line start at a vertex where another line has ended.
         * - A polygon that is simple has an equal number of intersections as the amount of vertices, since it only intersects at the corners.
         * - A polygon that is not simple has more intersections, because it does not only intersects once at every corner but at least at some other place as well. */
        return numberIntersections == amountOfVertices();
    }
}//rtt
