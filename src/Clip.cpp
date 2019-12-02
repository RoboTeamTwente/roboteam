#include "../include/roboteam_utils/Clip.h"
#include <stdexcept>

namespace rtt {
    namespace clip {

        int getCSCode(double x, double y) {
            int code = CSCode::INSIDE;
            if (x < X_MIN) code |= CSCode::LEFT;
            if (x > X_MAX) code |= CSCode::RIGHT;
            if (y < Y_MIN) code |= CSCode::BOTTOM;
            if (y > Y_MAX) code |= CSCode::TOP;
            return code;
        }

        bool cohenSutherlandClip(Vector2 &point0, Vector2 &point1) {
            /**
             * TODO: Make this function smaller, way too large
             */
            int code0 = getCSCode(point0.x, point0.y);
            int code1 = getCSCode(point1.x, point1.y);
            bool accept = false;
            while (true) {
                if (code0 == CSCode::INSIDE && code1 == CSCode::INSIDE) {
                    // Both points are on the field, we do not have to change anything
                    accept = true;
                    break;
                }
                if ((code0 & code1) != CSCode::INSIDE) {
                    // Both points are outside, reject the line
                    break;
                }
                // One point is outside, one is inside
                // We will now correct the outside point with respect to one edge of the field
                double m = (point1.y - point0.y) / (point1.x - point0.x);
                double x{0};
                double y{0};
                int &temp = code0 == CSCode::INSIDE ? code1 : code0; // temp = code of the outside point
                Vector2 &point = temp == code0 ? point0 : point1;
                if (temp & CSCode::TOP) {
                    x = point.x + (Y_MAX - point.y) / m;
                    y = Y_MAX;
                } else if (temp & CSCode::BOTTOM) {
                    x = point.x + (Y_MIN - point.y) / m;
                    y = Y_MIN;
                } else if (temp & CSCode::LEFT) {
                    x = X_MIN;
                    y = point.y + m * (X_MIN - point.x);
                } else if (temp & CSCode::RIGHT) {
                    x = X_MAX;
                    y = point.y + m * (X_MAX - point.x);
                } else {
                    /**
                     * Instead of this throwing an exception this should return an std::optional
                     */
                    throw std::logic_error(
                            "Illegal state in cohenSutherlandClip: point was outside, then magically was not.");
                }
                if (temp == code0) {
                    point0.x = x;
                    point0.y = y;
                    code0 = getCSCode(x, y);
                } else {
                    point1.x = x;
                    point1.y = y;
                    code1 = getCSCode(x, y);
                }
            }
            return accept;
        }

    }
}