//
// Created by rolf on 22-01-20.
//

#ifndef ROBOTEAM_UTILS_STATISTICS_H
#define ROBOTEAM_UTILS_STATISTICS_H

#include <vector>
#include <cmath>
#include "Vector2.h"
namespace rtt{

template <typename num>
num mean(const std::vector<num>& values){
    num sum=0;
    for (auto value : values) {
        sum+=value;
    }
    return sum /=values.size();
}

Vector2 mean(const std::vector<Vector2>&values){
    Vector2 sum(0,0);
    for (const auto& value :values) {
        sum+=value;
    }
    return sum /=values.size();
}

template<typename num>
num variance(const std::vector<num>& values){
    num average=mean(values);
    num sum = 0;
    for (auto value : values) {
        num diff= value-average;
        sum+=diff*diff;
    }
    return sum/=values.size();
}
Vector2 variance(const std::vector<Vector2>&values){
    Vector2 average=mean(values);
    Vector2 sum(0,0);
    for (const auto &value :values) {
        Vector2 diff=value-average;
        sum+=(diff*=diff);
    }
    return sum/=values.size();
}

template<typename num>
num standardDeviation(const std::vector<num>& values){
    return sqrt(variance(values));
}
Vector2 standardDeviation(const std::vector<Vector2> &values){
    Vector2 var=variance(values);
    return Vector2(sqrt(var.x),sqrt(var.y));
}


}

#endif //ROBOTEAM_UTILS_STATISTICS_H
