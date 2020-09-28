//
// Created by rolf on 19-08-20.
//

#include "Time.h"
#include <cmath>

Time::Time(std::chrono::nanoseconds time) : timePoint{time} {}

Time Time::operator+(const Time &other) const { return Time(timePoint + other.timePoint); }

Time Time::operator-(const Time &other) const {
    return Time(timePoint - other.timePoint);
}

Time &Time::operator-=(const Time &other) {
    timePoint -= other.timePoint;
    return *this;
}

Time &Time::operator+=(const Time &other) {
    timePoint += other.timePoint;
    return *this;
}

bool Time::operator>(const Time &other) const { return timePoint > other.timePoint; }

bool Time::operator>=(const Time &other) const { return timePoint >= other.timePoint; }

bool Time::operator<=(const Time &other) const { return timePoint <= other.timePoint; }

bool Time::operator<(const Time &other) const { return timePoint < other.timePoint; }

bool Time::operator==(const Time &other) const { return timePoint == other.timePoint; }

bool Time::operator!=(const Time &other) const { return timePoint != other.timePoint; }

double Time::asSeconds() const { return std::chrono::duration<double>(timePoint).count(); }

double Time::asMilliSeconds() const { return std::chrono::duration<double, std::milli>(timePoint).count(); }

long Time::asIntegerSeconds() const { return std::chrono::duration_cast<std::chrono::seconds>(timePoint).count(); }

long Time::asNanoSeconds() const { return std::chrono::duration<long, std::nano>(timePoint).count(); }

long Time::asMicroSeconds() const { return std::chrono::duration_cast<std::chrono::microseconds>(timePoint).count(); }

long Time::asIntegerMilliSeconds() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(timePoint).count();
}

Time Time::now() { return Time(std::chrono::system_clock::now().time_since_epoch()); }

Time Time::timeSince() const { return (now() - *this); }

Time Time::timeTo() const { return (*this - now()); }

//Unfortunately some of our input sources use doubles for times.
// We need to round to the nearsest nanosecond to prevent small fpe problems from propagating to whole nanoseconds
Time::Time(double seconds) :
        timePoint{std::chrono::nanoseconds((long) std::round(seconds * 1e9))} {
}

