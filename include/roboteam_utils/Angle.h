//
// Created by thijs on 28-2-19.
//

#ifndef ANGLE_H
#define ANGLE_H

#include "Mathematics.h"

namespace rtt {

class Vector2;

class Angle {

    public:
        constexpr Angle()
            : angle(0.0), epsilon(0.00001) { };

        constexpr Angle(const Angle &copy)
            :angle(copy.angle), epsilon(0.00001) { }

        Angle(double angle);
        Angle(const rtt::Vector2 &vec);

        double getAngle();
        void setAngle(double angle);

        double angleDiff(Angle &other);
        double angleDiff(double &other);
        double shortestAngleDiff(Angle &other);
        double shortestAngleDiff(double &other);

        rtt::Vector2 toVector2(double length = 1.0);
        bool operator==(const Angle &other) const;
        bool operator==(const double &scalar) const;

        bool operator!=(const Angle &other) const;
        bool operator!=(const double &scalar) const;

        bool operator<(const Angle &other) const;

        Angle operator+(const Angle &other) const;
        Angle operator+(const double &scalar) const;
        Angle operator-(const Angle &other) const;
        Angle operator-(const double &scalar) const;
        Angle operator+=(const Angle &other);
        Angle operator+=(const double &scalar);
        Angle operator-=(const Angle &other);
        Angle operator-=(const double &scalar);

        void operator=(const double &scalar);
        operator double() const;

    private:
        double angle;
        double epsilon;

        Angle constrain();

};

} // rtt

#endif //ANGLE_H
