//
// Created by rolf on 19-08-20.
//

#ifndef RTT_RANDOM_H
#define RTT_RANDOM_H

#include <random>
class Time;
/**
 * @brief This class generates random numbers. The seed is provided by the user.
 * This is to generate deterministic but 'random' sequences. Note that for using replayability,
 * using Time::now() is pretty much always a bad idea, and that the user is better off using e.g. the time of the world
 */
class Random {
public:
    explicit Random(long seed);
    explicit Random(const Time& time);
    /**
     * @return a gaussian distributed number with gaussian parameters mean 0, standard deviation 1
     */
    double getGaussian();
    /**
     * @return a uniformly distributed random number on the [0,1] interval
     */
    double getUniform();
    std::mt19937& instance();//Can be used for custom distributions

private:
    std::mt19937 engine;

};


#endif //RTT_RANDOM_H
