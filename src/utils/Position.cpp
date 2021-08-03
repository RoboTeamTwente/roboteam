#include "Position.h"

namespace rtt {

    Vector2 Position::location() const {
        return Vector2(x, y);
    }

    double Position::getRot() const {
        return rot;
    }

    Position Position::translate(const Vector2 &vec) const {
        return Position(x + vec.x, y + vec.y, rot);
    }

    Position Position::rotate(double rot_vel) const {
        return Position(x, y, rot + rot_vel);
    }

    Position Position::move(const Vector2 &vec, double rot_vel) const {
        return Position(x + vec.x, y + vec.y, rot + rot_vel);
    }

    bool Position::isNotNaN() const {
        return x == x && y == y && rot == rot; // NaN check
    }

    bool Position::operator==(const Position &other) const {
        return x == other.x && y == other.y && rot == other.rot;
    }

    bool Position::operator!=(const Position &other) const {
        return x != other.x || y != other.y || rot != other.rot;
    }

    Position Position::operator+(const Position &other) const {
        return Position(x + other.x, y + other.y, rot + other.rot);
    }

    Position Position::operator-(const Position &other) const {
        return Position(x - other.x, y - other.y, rot - other.rot);
    }

    Position Position::operator*(const double &other) const {
        return Position(x * other, y * other, rot * other);
    }

    Position Position::scale(double scalar) const {
        return Position(x * scalar, y * scalar, rot * scalar);
    }

    std::ostream &Position::write(std::ostream &stream) const {
        return stream << "{ x = " << x << ", y = " << y << ", rot = " << rot << " }";
    }

    std::ostream &operator<<(std::ostream &stream, const Position &pos) {
        return pos.write(stream);
    }

}
