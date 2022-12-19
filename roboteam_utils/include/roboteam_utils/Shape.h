//
// Created by alexander on 29-04-22.
//

#ifndef RTT_SHAPE_H
#define RTT_SHAPE_H

#include "Vector2.h"

// TODO: add other shared functions here and use this as a base class for more shape-related classes
namespace rtt {
    class Shape {
    public:
        virtual ~Shape() {} // Necessary to prevent memory leaks: https://stackoverflow.com/questions/461203/when-to-use-virtual-destructors

        virtual bool contains(const Vector2& point) const = 0;

        virtual Vector2 project(const Vector2& point) const = 0;
    };
}


#endif  // RTT_SHAPE_H
