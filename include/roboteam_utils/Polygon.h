#ifndef POLYGON_H_
#define POLYGON_H_

#include <vector>

namespace roboteam_utils {
    
class Polygon {
    public:
    Polygon() {}
    Polygon(std::vector<Vector2> vertices, bool seal = true) : vertices(vertices), sealed(seal) {}
    
    void add_vertex(const Vector2&);
    void seal();
    bool is_sealed() const;
    
    bool is_valid() const;
    bool is_convex() const;
    bool is_simple() const;
    bool contains(const Vector2&) const;
    bool intersects(const Vector2&, const Vector2&) const;
    Vector2 centroid() const;
    
    Vector2 operator[](int idx) { return vertices[idx]; }
    
    private:
    std::vector<Vector2> vertices;
    bool sealed;
    bool convex, simple;
    
    bool calc_convex() const;
    bool calc_simple() const;
    
    bool ccw(int) const;
    bool intersect(int, int) const;
};  

}

#endif