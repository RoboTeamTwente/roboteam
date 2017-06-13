#pragma once

#include <string>
#include <chrono>
#include <ros/param.h>
#include <iostream>

template<
    typename T
>
class SlowParam {
public:
    SlowParam(std::string paramName) : 
            paramName{paramName},
            latestValue{},
            wasParamSet{false} {
        getAndUpdateValue();
    } 

    bool isSet() {
        auto duration = std::chrono::high_resolution_clock::now() - lastCheck;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > 1000) {
            getAndUpdateValue();
        }

        return wasParamSet;
    }

    T operator()() {
        auto duration = std::chrono::high_resolution_clock::now() - lastCheck;
        if (std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() > 1000) {
            getAndUpdateValue();
        }

        return latestValue;
    }

    // Which clock: https://gamedev.stackexchange.com/questions/44374/using-stdchronosteady-clock-for-timing 
    void getAndUpdateValue() {
        wasParamSet = ros::param::get(paramName, latestValue);
        lastCheck = std::chrono::high_resolution_clock::now();
        std::cout << "Updating param: " << paramName << "!\n";
    }

private:
    std::string const paramName;

    T latestValue;
    bool wasParamSet;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastCheck;
} ;
