//
// Created by rolf on 18-10-20.
//

#ifndef RTT_POLYNOMIAL_H
#define RTT_POLYNOMIAL_H
#include <optional>
#include <vector>
namespace rtt {

/**
 * Solves a quadratic equation of the form ax^2+bx+c=0
 * @return an optional containing t0,t1, where t0<=t1. It may be that t0 == t1. If the discriminant<0 the optional is empty
 */
std::optional<std::pair<double, double>> solveQuadratic(double a, double b, double c);
std::optional<std::pair<double, double>> solveQuadraticPositiveA(double a, double b, double c);
std::vector<double> solveQuadraticVector(double a, double b, double c);
int countQuadraticSolutions(double a, double b, double c);
}  // namespace rtt

#endif  // RTT_POLYNOMIAL_H
