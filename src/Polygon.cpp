//
// Created by rolf on 24-4-19.
//

#include "../include/roboteam_utils/Polygon.h"
namespace rtt{
int newPolygon::amountOfVertices() const {
    return vertices.size();
}
std::vector<LineSegment> newPolygon::getBoundary() const {
    std::vector<LineSegment> boundary;
    int n=vertices.size();
    for (int i=0; i<n;i++){
        boundary.push_back(LineSegment(vertices[i],vertices[(i+1)%n])); // maybe there is a nice way to do this 'circular' access with iterators?
    }
    return boundary;
}
double newPolygon::perimeterLength() const {
    double totalLength=0;
    int n=vertices.size();
    for (int i=0; i<n;i++){
        totalLength+=(vertices[i]-vertices[(i+1)%n]).length();// maybe there is a nice way to do this 'circular' access with iterators?

    }
    return totalLength;
}

//https://stackoverflow.com/questions/471962/how-do-i-efficiently-determine-if-a-polygon-is-convex-non-convex-or-complex
// this function only works if your polygon is already simple. In 90% of the cases when a polygon is not simple, it will not be convex
bool newPolygon::isConvex() const {
    if (amountOfVertices()<4) return true;// triangles are always convex
    bool sign=false;
    int n=vertices.size();
    for (int i=0;i<n;i++){
        Vector2 d1=vertices[(i+2)%n]-vertices[(i+1)%n];
        Vector2 d2=vertices[i]-vertices[(i+1)%n];
        double cross=d1.cross(d2);
        if (i==0){
            sign=cross>0;
        }
        else if(sign!=(cross>0)){
            return false;
        }
    }
    return true;
}
// there are multiple possible algorithms, see
//https://www.quora.com/What-is-the-simplest-algorithm-to-know-if-a-polygon-is-simple-or-not
// this is the 'naive' O(N^2) approach which is fine for small cases (polygons with less than say 8-10 vertices)
bool newPolygon::isSimple() const {
    // we loop over every unique pair
    std::vector<LineSegment> lines;
    for (auto first=vertices.begin()+1; first!=vertices.end(); ++first){
        LineSegment boundarySegment;
        if (first==vertices.end()) {
            boundarySegment=LineSegment(*first,vertices[0]);
        }
        else{
            boundarySegment=LineSegment(*first, *(first - 1));
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
bool newPolygon::contains(const Vector2 &point) const {
    int i, j, c = 0;
    int n=amountOfVertices();
    for (i = 0, j = n-1; i < n; j = i++) {
        if ( ((vertices[i].y>point.y) != (vertices[j].y>point.y)) &&
                (point.x < (vertices[j].x-vertices[i].x) * (point.y-vertices[i].y) / (vertices[j].y-vertices[i].y) + vertices[i].x) )
            c = !c;
    }
    return c;
}

bool newPolygon::doesIntersect(const LineSegment &line) const {
    int n=vertices.size();
    for (int i=0; i<n;i++){
        LineSegment segment(vertices[i],vertices[(i+1)%n]);// maybe there is a nice way to do this 'circular' access with iterators?
        if (line.doesIntersect(segment)){
            return true;
        }
    }
    return false;
}
bool newPolygon::isOnBoundary(const Vector2 &point) const {
    int n=vertices.size();
    for (int i=0; i<n;i++){
        LineSegment segment(vertices[i],vertices[(i+1)%n]);// maybe there is a nice way to do this 'circular' access with iterators?
        if (segment.isOnLine(point)){
            return true;
        }
    }
    return false;
}
std::vector<Vector2> newPolygon::intersections(const LineSegment &line) const {
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
double newPolygon::area() const {
    int n=vertices.size();
    double sum=0;
    for (int i=0;i<n;i++){
        sum+=vertices[i].cross(vertices[(i+1)%n]);
    }
    return 0.5*abs(sum);
}
}//rtt