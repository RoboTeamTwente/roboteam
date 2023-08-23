#include "Random.h"

#include <limits>

#include "Time.h"

Random::Random(long seed) : engine(seed) {}

Random::Random(const Time &time) : Random(time.asNanoSeconds()) {}

double Random::getGaussian() {
    // We could also instantiate these as class members
    std::normal_distribution<double> gauss(0.0, 1.0);
    return gauss(engine);
}

std::mt19937 &Random::instance() { return engine; }

double Random::getUniform() {
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    return uniform(engine);
}

double SimpleRandom::getDouble(double low, double high) {
    std::uniform_real_distribution<double> distribution(low, high);
    std::random_device rd;
    std::mt19937 engine(rd());
    return distribution(engine);
}

double SimpleRandom::getDouble() {
    constexpr double DOUBLE_MAX = 1000000000000.0;
    return SimpleRandom::getDouble(-DOUBLE_MAX, DOUBLE_MAX);
}

int SimpleRandom::getInt(int low, int high) {
    std::uniform_int_distribution distribution(low, high);
    std::random_device rd;
    std::mt19937 engine(rd());
    return distribution(engine);
}

int SimpleRandom::getInt() { return SimpleRandom::getInt(std::numeric_limits<int>::min(), std::numeric_limits<int>::max()); }

long SimpleRandom::getLong(long low, long high) {
    std::uniform_int_distribution<long> distribution(low, high);
    std::random_device rd;
    std::mt19937 engine(rd());
    return distribution(engine);
}

long SimpleRandom::getLong() { return SimpleRandom::getLong(std::numeric_limits<long>::min(), std::numeric_limits<long>::max()); }

bool SimpleRandom::getBool() { return static_cast<bool>(SimpleRandom::getInt(0, 1)); }
