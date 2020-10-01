#ifndef POSITION_H
#define POSITION_H

#include "Vector2.h"
#include <roboteam_proto/WorldRobot.pb.h>

namespace rtt {

    /**
     * \brief A vector of three doubles, with the semantics of (x,y) coordinates + rotation.
     */
    class Position {
    public:
        /**
         * \brief Get a Position at the origin, with a zero rotation.
         */
        constexpr Position() : x(0), y(0), rot(0) {}

        /**
         * \brief Get a Position by supplying every value.
         */
        constexpr Position(double x, double y, double rot) : x(x), y(y), rot(rot) {}

        /**
         * \brief Get a Position using a Vector2 for the location, and a double for the rotation.
         */
        constexpr Position(Vector2 loc, double rot = 0) : x(loc.x), y(loc.y), rot(rot) {}

        /**
         * \brief Get a Position by reading values from a WorldRobot message.
         */
        Position(const proto::WorldRobot &robotMsg) : Position(Vector2(robotMsg.pos()), robotMsg.angle()) {}

        /**
         * \brief Get the (x, y) coordinates of this position.
         */
        Vector2 location() const;

        /**
         * \brief Get the rotation of this position.
         */
        double getRot() const;

        /**
         * \brief Moves this position without rotating it, by the given (x, y) values.
         */
        Position translate(const Vector2 &vec) const;

        /**
         * \brief Rotate this position by the specified angle.
         */
        Position rotate(double radians) const;

        /**
         * \brief Combines translate and rotate.
         */
        Position move(const Vector2 &vec, double radians) const;

        /**
         * \brief Checks whether all components of this position are real (non-NaN).
         */
        bool isNotNaN() const;

        /**
         * \brief Checks for equality.
         */
        bool operator==(const Position &other) const;

        /**
         * \brief Checks for inequality.
         */
        bool operator!=(const Position &other) const;

        /**
         * \brief Adds another position to this one, component-wise.
         * This is basically the same as the move method.
         */
        Position operator+(const Position &other) const;

        /**
         * \brief Subtracts another position from this one, component-wise.
         */
        Position operator-(const Position &other) const;

        /**
         * \brief (Pointlessly?) Scales all components of this position by a scalar.
         */
        Position operator*(const double &other) const;

        /**
         * \brief (Pointlessly?) Scales all components of this position by a scalar.
         */
        Position scale(double scalar) const;

        std::ostream &write(std::ostream &stream) const;

        double x, y, rot;
    };

    std::ostream &operator<<(std::ostream &stream, const Position &pos);

}

#endif // POSITION_H
