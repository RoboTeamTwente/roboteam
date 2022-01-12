#include "Angle.h"

#include "Definitions.h"
#include "Vector2.h"

namespace rtt {
Angle::Angle(double angle) : angle(angle) { this->normalize(); }

Angle::Angle(const rtt::Vector2 &vec) {
    if (vec.length() < RTT_PRECISION_LIMIT) {
        angle = 0.0;
    } else {
        angle = vec.toAngle();
    }
}

Angle::operator double() const noexcept { return angle; }

Angle Angle::normalize() noexcept {
    this->angle = fmod(angle + M_PI, 2 * M_PI);
    this->angle = (angle < 0) ? angle + M_PI : angle - M_PI;
    return *this;
}

bool Angle::rotateDirection(const Angle &other) const noexcept {
    double angleDiff = other.angle - this->angle;
    bool sign = angleDiff >= 0;
    bool large = fabs(angleDiff) >= M_PI;
    return sign ^ large;
}

double Angle::shortestAngleDiff(Angle const &other) const noexcept {
    double thisDiff = fabs((*this - other).angle);
    double otherDiff = fabs((other - *this).angle);
    return std::min(thisDiff, otherDiff);
}

rtt::Vector2 Angle::toVector2(double length) const noexcept {
    rtt::Vector2 vec = rtt::Vector2(cos(this->angle), sin(this->angle));
    return vec.stretchToLength(length);
}

bool Angle::operator==(const Angle &other) const noexcept { return this->shortestAngleDiff(other) < RTT_PRECISION_LIMIT; }

bool Angle::operator!=(const Angle &other) const noexcept { return !(*this == other); }

Angle Angle::operator+(const Angle &other) const noexcept { return Angle(this->angle + other.angle); }

Angle Angle::operator-(const Angle &other) const noexcept { return Angle(this->angle - other.angle); }

Angle &Angle::operator+=(const Angle &other) noexcept {
    this->angle += other.angle;
    this->normalize();
    return *this;
}

Angle &Angle::operator-=(const Angle &other) noexcept {
    this->angle -= other.angle;
    this->normalize();
    return *this;
}

Angle &Angle::operator=(double scalar) noexcept {
    this->angle = scalar;
    this->normalize();
    return *this;
}
}  // namespace rtt