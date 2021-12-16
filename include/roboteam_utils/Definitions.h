//
// Created by rolf on 22-01-20.
//

#ifndef RTT_DEFINITIONS_H
#define RTT_DEFINITIONS_H

/**
 * @date 22 jan. 2020
 * @author Rolf
 * This header contains compile-time definitions which we use all across our system, which are implicitly or explicitly
 * used. The idea is to in the future also add a scalar definition here.
 */

/**
 * A comparison precision limit in our system.
 * If we check in Roboteam_utils if two floats are equal then we check actually
 * if the distance between them is smaller than this given precision.
 */
constexpr static double RTT_PRECISION_LIMIT = 1e-5;

#endif  // RTT_DEFINITIONS_H
