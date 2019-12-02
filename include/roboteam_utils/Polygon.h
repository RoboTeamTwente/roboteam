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

        Polygon(const Vector2 &lowerLeftCorner, double xlen, double ylen);

        Polygon(const std::vector<Vector2> &vertices);

        void move(const Vector2 &moveBy);

        [[nodiscard]] Vector2 centroid() const;

        [[nodiscard]] Vector2 verticeCentroid() const;

        [[nodiscard]] Vector2 operator[](int id) const;

        [[nodiscard]] int amountOfVertices() const;

        [[nodiscard]] std::vector<LineSegment> getBoundary() const;

        [[nodiscard]] double perimeterLength() const;

        [[nodiscard]] bool isConvex() const;

        [[nodiscard]] bool isSimple() const;

        [[nodiscard]] bool isValid() const;

        [[nodiscard]] bool contains(const Vector2 &point) const;

        [[nodiscard]] bool isOnBoundary(const Vector2 &point) const;

        [[nodiscard]] bool doesIntersect(const LineSegment &line) const;

        [[nodiscard]] std::vector<Vector2> intersections(const LineSegment &line) const;

        [[nodiscard]] double area() const;

        [[nodiscard]] double doubleSignedArea() const;


    };
}
#endif //ROBOTEAM_UTILS_POLYGON_H
