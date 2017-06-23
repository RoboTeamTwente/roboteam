#pragma once

#include <string>
#include <chrono>
#include <ros/param.h>
#include <iostream>


/**
 * Class to stay up-to-date on the value of a ros param in an economic manner.
 * Warning: make sure the object lives longer than the function call!
 * So make it global or a skill object member. If you create a new one every loop
 * there's no benefit from this class.
 * How to use:
 *
 * // Default value will be empty string
 * SlowParam<std::string> fieldColor("our_color");
 * // Default value will be 12345. If the param is unset this value will be set in the ros param server
 * SlowParam<int>         port("important_port", 12345);
 *
 * void updateLogic() {
 *     // To query for the param value:
 *     if (fieldColor() == "yellow") {
 *         // The object will update the param value every second  
 *         // Code...
 *         // Code...
 *         // Code...
 *     }
 *
 *     // Same goed for ints
 *     if (port() > 5) {
 *         // Code...
 *         // Code...
 *         // Code...
 *     }
 * }
 *
 * How NOT to use:
 * void updateLogic() {
 *     // DON'T DO THIS!
 *     // It will cause a param to get checked every second
 *     // It is equal to using ros::param::get!
 *     SlowParam<std::string> fieldColor("our_color");
 *     SlowParam<int>         port("important_port", 12345);
 *    
 *     if (fieldColor() == "blue" && port() == 5) {
 *         // Code...
 *         // Code...
 *         // Code...
 *     }
 * }
 */
template<
    typename T
>
class SlowParam {
public:
    SlowParam(std::string const & paramName) : 
            firstUpdate{true},
            paramName{paramName},
            latestValue{},
            defaultValue{},
            wasParamSet{false},
            lastCheck{} { } 

    SlowParam(std::string const & paramName, T const & defaultValue) :
            firstUpdate{true},
            paramName{paramName},
            latestValue{},
            defaultValue{defaultValue},
            wasParamSet{false},
            lastCheck{} { } 

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
        // std::cout << "Updating param: " << paramName << "!\n";

        // Only set the parameter to the default value if it was not set
        // If a default value was not given it will be zero initialized
        if (firstUpdate && !wasParamSet) {
            ros::param::set(paramName, defaultValue);
            latestValue = defaultValue;
        }

        firstUpdate = false;
    }

private:
    bool firstUpdate;
    std::string const paramName;
    T latestValue;
    T defaultValue;
    bool wasParamSet;
    std::chrono::time_point<std::chrono::high_resolution_clock> lastCheck;

} ;
