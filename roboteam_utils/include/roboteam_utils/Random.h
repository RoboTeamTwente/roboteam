#pragma once
#include <iterator>
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
    std::mt19937& instance();  // Can be used for custom distributions

private:
    std::mt19937 engine;
};

/* This class will generate a simple random value, meant for testing.
 * Warning: This does not give true random values, but predictable ones. */
class SimpleRandom {
public:
    // Returns a random integer within the given range
    static int getInt(int low, int high);
    // Returns a random integer;
    static int getInt();

    // Returns a random long within the given range
    static long getLong(long low, long high);
    // Returns a random long
    static long getLong();

    // Returns a random double within the given range
    static double getDouble(double low, double high);
    // Returns a random double
    static double getDouble();

    // Returns a random boolean
    static bool getBool();

    // Returns a random element within the given range. Can be used for enums
    // by giving a vector of all possible enum values
    template <class Iterator>
    static Iterator getRandomElement(Iterator start, Iterator end) {
        auto dist = std::distance(start, end);
        if (dist == 0)
            return end;
        auto randomPosition = SimpleRandom::getLong(0, dist - 1);
        std::advance(start, randomPosition);
        return start;
    }
};
