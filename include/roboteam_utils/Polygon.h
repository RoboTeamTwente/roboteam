//
// Created by rolf on 24-4-19.
//

#ifndef ROBOTEAM_UTILS_POLYGON_H
#define ROBOTEAM_UTILS_POLYGON_H
#include "Vector2.h"
#include "Line.h"
namespace rtt {
class newPolygon {
    public:
        newPolygon(std::vector<Vector2> vertices);

        std::vector<Vector2> vertices;

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


};
}
#endif //ROBOTEAM_UTILS_POLYGON_H
