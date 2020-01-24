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
 * If instances from Vector2 or Angle match to this degree we consider them 'equal' in length as a precaution against
 * floating point errors.
 */
constexpr static double VECTOR_PRECISION=0.00001;


#endif //RTT_DEFINITIONS_H
