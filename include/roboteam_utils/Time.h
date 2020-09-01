//
// Created by rolf on 19-08-20.
//

#ifndef RTT_TIME_H
#define RTT_TIME_H


#include <chrono>
#include <ctime>

/**
 * \author Rolf van der Hulst
 * \brief A class that represents a Time Duration. It measures all time points as durations from the clock's epoch (1970).
 * The system clock can be measured up to nanosecond precision.
 * \date August 19 2020
 */
class Time {
public:
    constexpr Time() : timePoint{std::chrono::nanoseconds(0)} {};
    explicit Time(std::chrono::high_resolution_clock::duration duration);
    explicit Time(double seconds);
    explicit Time(long nanoseconds) : timePoint{std::chrono::nanoseconds(nanoseconds)} {};
    static Time now();

    [[nodiscard]] Time timeSince() const;
    [[nodiscard]] Time timeTo() const;
    [[nodiscard]] long asNanoSeconds() const;
    [[nodiscard]] long asIntegerSeconds() const;
    [[nodiscard]] long asIntegerMilliSeconds() const;
    [[nodiscard]] long asMicroSeconds() const;
    [[nodiscard]] double asSeconds() const;
    [[nodiscard]] double asMilliSeconds() const;
    Time operator+(const Time &other) const;
    Time operator-(const Time &other) const;
    Time& operator-=(const Time &other);
    Time& operator+=(const Time &other);
    bool operator>(const Time &other) const;
    bool operator>=(const Time &other) const;
    bool operator<(const Time &other) const;
    bool operator<=(const Time &other) const;
    bool operator==(const Time &other) const;
    bool operator!=(const Time &other) const;

private:
    std::chrono::high_resolution_clock::duration timePoint;
};


#endif //RTT_TIME_H
