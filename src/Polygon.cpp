#include "roboteam_utils/Polygon.h"
#include <exception>
#include <queue>
#include <boost/intrusive/sg_set.hpp>

namespace roboteam_utils {
    
bool Polygon::add_vertex(const Vector2& vec) {
    if (sealed || !can_add(vec)) {
        return false;
    }
    vertices.push_back(vec);
    return true;
}    

void Polygon::seal() {
    sealed = true;
    convex = calc_convex();
    simple = calc_simple();
}

bool Polygon::is_valid() {
    return vertices.size() > 2; // A polygon must at least be a triangle
}

bool Polygon::is_sealed() {
    return sealed;
}

bool Polygon::is_convex() {
    return sealed ? convex : calc_convex();
}

bool Polygon::is_simple() {
    return sealed ? simple : calc_simple();
}

bool Polygon::contains(const Vector2& vec) {
    //TODO
}

bool Polygon::intersects(const Vector2& a, const Vector2& b) {
    //TODO
}

Vector2 Polygon::centroid() {
    // See https://en.wikipedia.org/wiki/Centroid#Centroid_of_polygon
    if (!is_simple()) {
        throw std::domain_error("Cannot calculate centroid for complex polygons.");
    }
    std::vector<double> multiplicands; // used in multiple calculations below
    for (int i = 0; i < vertices.size(); i++) {
        int next_i = (i+1) % vertices.size(); //loop back at the end
        multiplicands[i] = vertices[i].x * vertices[next_i].y - vertices[next_i].x * vertices[i].y;
    }
    
    double area = 0.0;
    for (double d : multiplicands) area += d;
    area = 1.0 / (area * 3.0); // actually area /= 2, but we need 1 / (6*area) multiple times later.
    
    double cx = 0.0, cy = 0.0;
    for (int i = 0; i < vertices.size(); i++) {
        int next_i = (i+1) % vertices.size(); //loop back at the end
        cx += (vertices[i].x + vertices[next_i].x) * multiplicands[i];
        cy += (vertices[i].y + vertices[next_i].y) * multiplicands[i];
    }
    
    cx *= area;
    cy *= area;
    
    return Vector2(cx, cy);    
}

bool Polygon::calc_convex() {
    //TODO
    if (!is_valid()) {
        return false;
    }
    int sign = 0;
    for (int i = 0; i < vertices.size(); i++) {
        int next = (i+1) % vertices.size();
        int nextnext = (i+2) % vertices.size();
        double dx1 = vertices[next].x - vertices[i].x;
        double dx2 = vertices[nextnext].x - vertices[next].x;
        double dy1 = vertices[next].y - vertices[i].y;
        double dy2 = vertices[nextnext].y - vertices[next].y;
        double zcross = dx1 * dy2 - dy1 * dx2;
        int cur_sign = zcross > 0 ? 1 : zcross < 0 ? -1 : 0;
        if (i == 0 || sign == 0) {
            sign = cur_sign;
        } else if (cur_sign != 0 && sign != cur_sign) {
            return false;
        }
    }
    return true;
}

bool Polygon::ccw(int i) {
    int next = (i+1) % vertices.size();
    int nextnext = (i+2) % vertices.size();
    double dx1 = vertices[next].x - vertices[i].x;
    double dx2 = vertices[nextnext].x - vertices[next].x;
    double dy1 = vertices[next].y - vertices[i].y;
    double dy2 = vertices[nextnext].y - vertices[next].y;
    return dy1 * dx2 < dy2 * dx1;
}

bool Polygon::intersect(int i, int j) {
    int s1 = i > 0 ? i - 1 : vertices.size() - 1;
    int s2 = j > 0 ? j - 1 : vertices.size() - 1;
    
    
}

bool Polygon::calc_simple() {
    //Bentley-Ottmann algorithm
    if (!is_valid()) {
        return false;
    }
    
}

}