//
// Created by rolf on 18-10-20.
//

#include "Polynomial.h"

#include <cmath>
namespace rtt {

std::optional<std::pair<double, double>> solveQuadraticPositiveA(double a, double b, double c) {
    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return std::nullopt;
    }
    discriminant = sqrt(discriminant);
    double t0 = (-b - discriminant) / (2 * a);
    double t1 = (-b + discriminant) / (2 * a);
    return std::make_pair(t0, t1);
}

}  // namespace rtt