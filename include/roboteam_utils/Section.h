#pragma once

#include "Vector2.h"

namespace rtt {

    /**
     * \class Section
     * \brief Models a line segment
     */
    struct Section {
        Vector2 a, b, center;
        double length;

        Section() = default;
        
        /**
         * @brief Construct a new Section object
         * 
         * @param ax Begin x
         * @param ay Begin y
         * @param bx End x
         * @param by End y
         */
        Section(double ax, double ay, double bx, double by) :
                a(ax, ay), b(bx, by), center((ax + bx) / 2.0, (ay + by) / 2.0),
                length(a.dist(b)) {}

        /**
         * @brief Construct a new Section object
         * 
         * @param a Begin  
         * @param b End
         */
        Section(Vector2 a, Vector2 b) : Section(a.x, a.y, b.x, b.y) {}


        /**
         * \brief Gets the point of intersection between this section and another.
         * The resulting point does not have to lie on one of the two segments; if the
         * segments themselves do not intersect, the lines are extended infinitely.
         * \param other The other line segment.
         * \return The intersection of the two (possibly extended) lines.
         */
        [[nodiscard]] Vector2 intersection(const Section &other) const;

        /**
         * \brief Checks whether a point lies on this line segment.
         * \param point The point to check.
         * \return True if the point lies on the (non-extended) line segment.
         */
        [[nodiscard]] bool pointOnLine(const Vector2 &point) const;

        /**
         * @brief Compares two Sections
         * 
         * @param other Other section to compare with
         * @return true True if this->a == other.a && this->b == other.b
         * @return false False if this condition is not true
         */
        bool operator==(const Section &other) const;

    };

    /**
     * @brief Writes the Section object to a stream
     * 
     * Represents in format: Section[sec.a, sec.b]
     * 
     * @param stream Strema to write to
     * @param sec Section to represents
     * @return std::ostream& Reference to \ref stream
     */
    std::ostream &operator<<(std::ostream &stream, const Section &sec);

}
