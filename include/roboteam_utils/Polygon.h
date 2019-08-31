//
// Created by rolf on 24-4-19.
//

#ifndef ROBOTEAM_UTILS_POLYGON_H
#define ROBOTEAM_UTILS_POLYGON_H
#include "Vector2.h"
#include "LineSegment.h"
#include <iostream>
#include <vector>

namespace rtt {
class Polygon {
    public:
        std::vector<Vector2> vertices;
        Polygon(const Vector2 &lowerLeftCorner,double xlen,double ylen);
        Polygon(const std::vector<Vector2> &vertices);
        void move(const Vector2 &moveBy);
        Vector2 centroid() const;
        Vector2 verticeCentroid() const;

        Vector2 operator[](int id) const;
        int amountOfVertices() const;
        std::vector<LineSegment> getBoundary() const;
        double perimeterLength() const;
        bool isConvex() const;
        bool isSimple() const;
        bool isValid() const;
        bool contains(const Vector2& point) const;
        bool isOnBoundary(const Vector2& point) const;
        bool doesIntersect(const LineSegment& line) const;
        std::vector<Vector2> intersections(const LineSegment& line) const;
        double area() const;
        double doubleSignedArea() const;


};
}
#endif //ROBOTEAM_UTILS_POLYGON_H
