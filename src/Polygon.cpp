//
// Created by rolf on 24-4-19.
//

#include "../include/roboteam_utils/Polygon.h"
namespace rtt{
///constructor for rectangles oriented straight with respect to the x-axis.
Polygon::Polygon(const Vector2 &lowerLeftCorner, double xlen, double ylen) {
    vertices.push_back(lowerLeftCorner);
    vertices.push_back(Vector2(lowerLeftCorner.x+xlen,lowerLeftCorner.y));
    vertices.push_back(Vector2(lowerLeftCorner.x+xlen,lowerLeftCorner.y+ylen));
    vertices.push_back(Vector2(lowerLeftCorner.x,lowerLeftCorner.y+ylen));
}
Polygon::Polygon(const std::vector<rtt::Vector2> &_vertices) {
    vertices=_vertices;
}
int Polygon::amountOfVertices() const {
    return vertices.size();
}
Vector2 Polygon::operator[](int id) const {
    return vertices[id];
}
void Polygon::move(const Vector2 &moveBy) {
    for (std::vector<Vector2>::iterator it=vertices.begin();it!=vertices.end();++it){
        *it+=moveBy;
    }
}
std::vector<LineSegment> Polygon::getBoundary() const {
    std::vector<LineSegment> boundary;
    int n=vertices.size();
    for (int i=0; i<n;i++){
        boundary.push_back(LineSegment(vertices[i],vertices[(i+1)%n])); // maybe there is a nice way to do this 'circular' access with iterators?
    }
    return boundary;
}
double Polygon::perimeterLength() const {
    double totalLength=0;
    int n=vertices.size();
    for (int i=0; i<n;i++){
        totalLength+=(vertices[i]-vertices[(i+1)%n]).length();// maybe there is a nice way to do this 'circular' access with iterators?

    }
    return totalLength;
}

//https://stackoverflow.com/questions/471962/how-do-i-efficiently-determine-if-a-polygon-is-convex-non-convex-or-complex
// this function only works if your polygon is already simple. In 90% of the cases when a polygon is not simple, it will not be convex
bool Polygon::isConvex() const {
    if (amountOfVertices()<4) return true;// triangles are always convex
    bool sign=false;
    bool signSet=false;
    int n=vertices.size();
    for (int i=0;i<n;i++){
        Vector2 d1=vertices[(i+2)%n]-vertices[(i+1)%n];
        Vector2 d2=vertices[i]-vertices[(i+1)%n];
        double cross=d1.cross(d2);
        // on a crossproduct of zero the points lie in one line and we can simply ignore this point's contribution to the convexity
        if(cross!=0.0) {
            if (! signSet) {
                signSet = true;
                sign = cross > 0;
            }
            else if (sign != (cross > 0)) {
                return false;
            }
        }
    }
    return true;
}
// there are multiple possible algorithms, see
//https://www.quora.com/What-is-the-simplest-algorithm-to-know-if-a-polygon-is-simple-or-not
// this is the 'naive' O(N^2) approach which is fine for small cases (polygons with less than say 8-10 vertices)
bool Polygon::isSimple() const {
    // we loop over every unique pair
    std::vector<LineSegment> lines;
    for (auto first=vertices.begin();first!=vertices.end();first++){
        LineSegment boundarySegment;
        if (first==std::prev(vertices.end())) {
            boundarySegment=LineSegment(*first,vertices[0]);
        }
        else{
            boundarySegment=LineSegment(*first, *(first + 1));
        }
        for (auto line :lines){
            if(boundarySegment.nonSimpleDoesIntersect(line)){
                return false;
            }
        }
        lines.push_back(boundarySegment);
    }
    return true;
}
//https://wrf.ecse.rpi.edu/Research/Short_Notes/pnpoly.html
// this is black magic but if it works it works
/// on the boundary this function does not work!! see documentation if you are interested
bool Polygon::contains(const Vector2 &point) const {
    int i, j, c = 0;
    int n=amountOfVertices();
    for (i = 0, j = n-1; i < n; j = i++) {
        if ( ((vertices[i].y>point.y) != (vertices[j].y>point.y)) &&
                (point.x < (vertices[j].x-vertices[i].x) * (point.y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) )
            c = !c;
    }
    return c;
}

bool Polygon::doesIntersect(const LineSegment &line) const {
    int n=vertices.size();
    for (int i=0; i<n;i++){
        LineSegment segment(vertices[i],vertices[(i+1)%n]);// maybe there is a nice way to do this 'circular' access with iterators?
        if (line.doesIntersect(segment)){
            return true;
        }
    }
    return false;
}
bool Polygon::isOnBoundary(const Vector2 &point) const {
    int n=vertices.size();
    for (int i=0; i<n;i++){
        LineSegment segment(vertices[i],vertices[(i+1)%n]);// maybe there is a nice way to do this 'circular' access with iterators?
        if (segment.isOnLine(point)){
            return true;
        }
    }
    return false;
}
std::vector<Vector2> Polygon::intersections(const LineSegment &line) const {
    std::vector<Vector2> intersections;
    int n=vertices.size();
    for (int i=0; i<n;i++){
        LineSegment segment(vertices[i],vertices[(i+1)%n]);// maybe there is a nice way to do this 'circular' access with iterators?
        auto intersect=line.nonSimpleIntersects(segment);
        if (intersect){
            intersections.push_back(*intersect);
        }
        // check the vertices explicitly (so we don't double count if we do line intersection)
        if (line.isOnLine(vertices[i])){
            intersections.push_back(vertices[i]);
        }
    }
    return intersections;
}
//https://en.wikipedia.org/wiki/Shoelace_formula
// area is well defined only for simple polygons
double Polygon::area() const {
    int n=vertices.size();
    double sum=0;
    for (int i=0;i<n;i++){
        sum+=vertices[i].cross(vertices[(i+1)%n]);
}    return 0.5*abs(sum);
}

//https://en.wikipedia.org/wiki/Centroid
Vector2 Polygon::centroid() const {
    Vector2 sum={0,0};
    for (auto vertice :vertices){
        sum+=vertice;
    }
    return sum.scale(1/vertices.size());
}

}//rtt