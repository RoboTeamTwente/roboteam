#pragma once

#include <string>
#include <stdexcept>
#include <memory>

namespace rtt {

/* This function will format your string with the given arguments following
 * the standard formatting rules, copied from here: https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
 * Unfortunately, most c++20 compilers do not support the std::format functionality *yet*,
 * hence this replacement.
 * For example, passing ("%5s", "hi") gives "hi   " */
template <typename... Args>
std::string formatString(const std::string& format, Args... args) {
    int size_s = std::snprintf(nullptr, 0, format.c_str(), args...) + 1;  // Extra space for '\0'
    if (size_s <= 0) {
        throw std::runtime_error("Error during formatting.");
    }
    auto size = static_cast<size_t>(size_s);
    auto buf = std::make_unique<char[]>(size);
    std::snprintf(buf.get(), size, format.c_str(), args...);
    return std::string(buf.get(), buf.get() + size - 1);  // We don't want the '\0' inside
}

} // namespace rtt