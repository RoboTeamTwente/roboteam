//
// Created by rolf on 18-10-20.
//

#include "Polynomial.h"

#include <cmath>
namespace rtt {
    std::optional<std::pair<double, double>> solveQuadratic(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return std::nullopt;
        }
        discriminant = sqrt(discriminant);
        double t0 = (-b - discriminant) / (2 * a);
        double t1 = (-b + discriminant) / (2 * a);
        return a > 0 ? std::make_pair(t0, t1) : std::make_pair(t1, t0);
    }

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

    std::vector<double> solveQuadraticVector(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return {};
        } else if (discriminant == 0) {
            return { -b / (2 * a) };
        }
        discriminant = sqrt(discriminant);
        double t0 = (-b - discriminant) / (2 * a);
        double t1 = (-b + discriminant) / (2 * a);
        return a > 0 ? std::vector{ t0, t1 } : std::vector{ t1, t0 };
    }

    int countQuadraticSolutions(double a, double b, double c) {
        double discriminant = b * b - 4 * a * c;
        if (discriminant < 0) {
            return 0;
        } else if (discriminant == 0) {
            return 1;
        }
        return 2;
    }
}  // namespace rtt