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
        [[nodiscard]] virtual bool contains(const Vector2& point) const = 0;

        [[nodiscard]] virtual Vector2 project(const Vector2& point) const = 0;

        virtual ~Shape() = default;
    };
}


#endif  // RTT_SHAPE_H
