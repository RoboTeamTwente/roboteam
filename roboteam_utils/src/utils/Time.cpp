//
// Created by rolf on 19-08-20.
//

#include "Time.h"

#include <cmath>
#include <iomanip>
#include <sstream>

Time::Time(std::chrono::nanoseconds time)
    : timePoint{ time } {
}

Time Time::operator+(const Time &other) const {
    return Time(timePoint + other.timePoint);
}

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

bool Time::operator>(const Time &other) const {
    return timePoint > other.timePoint;
}

bool Time::operator>=(const Time &other) const {
    return timePoint >= other.timePoint;
}

bool Time::operator<=(const Time &other) const {
    return timePoint <= other.timePoint;
}

bool Time::operator<(const Time &other) const {
    return timePoint < other.timePoint;
}

bool Time::operator==(const Time &other) const {
    return timePoint == other.timePoint;
}

bool Time::operator!=(const Time &other) const {
    return timePoint != other.timePoint;
}

double Time::asSeconds() const {
    return std::chrono::duration<double>(timePoint).count();
}

double Time::asMilliSeconds() const {
    return std::chrono::duration<double, std::milli>(timePoint).count();
}

long Time::asIntegerSeconds() const {
    return std::chrono::duration_cast<std::chrono::seconds>(timePoint).count();
}

long Time::asNanoSeconds() const {
    return std::chrono::duration<long, std::nano>(timePoint).count();
}

long Time::asMicroSeconds() const {
    return std::chrono::duration_cast<std::chrono::microseconds>(timePoint).count();
}

long Time::asIntegerMilliSeconds() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(timePoint).count();
}

Time Time::now() {
    return Time(std::chrono::system_clock::now().time_since_epoch());
}

Time Time::timeSince() const {
    return (now() - *this);
}

Time Time::timeTo() const {
    return (*this - now());
}

// Unfortunately some of our input sources use doubles for times.
//  We need to round to the nearsest nanosecond to prevent small fpe problems from propagating to whole nanoseconds
Time::Time(double seconds)
    : timePoint{ std::chrono::nanoseconds((long)std::round(seconds * 1e9)) } {
}

std::string Time::getDate(char separator) {
    using namespace std::chrono;

    // Get the current time
    auto now = system_clock::to_time_t(system_clock::now());
    auto brokenTime = *std::localtime(&now);

    // Convert time to text
    std::stringstream ss;
    ss << std::put_time(&brokenTime, "%Y") << separator
       << std::put_time(&brokenTime, "%m") << separator
       << std::put_time(&brokenTime, "%d");

    return ss.str();
}

std::string Time::getTime(char separator) {
    using namespace std::chrono;

    // Get the current time
    auto timer = system_clock::to_time_t(system_clock::now());
    auto brokenTime = *std::localtime(&timer);

    // Convert time to text
    std::stringstream ss;
    ss << std::put_time(&brokenTime, "%H") << separator
       << std::put_time(&brokenTime, "%M") << separator
       << std::put_time(&brokenTime, "%S");

    return ss.str();
}

std::string Time::getTimeWithMilliseconds(char separator) {
    using namespace std::chrono;

    // Get the current time
    auto now = system_clock::now();

    // Amount of milliseconds between full seconds
    auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

    // Get the second-precise time
    auto timer = system_clock::to_time_t(now);
    std::tm brokenTime = *std::localtime(&timer);

    // Convert these values to text
    std::stringstream ss;
    ss << std::put_time(&brokenTime, "%H") << separator     // Hours
       << std::put_time(&brokenTime, "%M") << separator     // Minutes
       << std::put_time(&brokenTime, "%S") << "."           // Seconds
       << std::setfill('0') << std::setw(3) << ms.count();  // Milliseconds

    return ss.str();
}