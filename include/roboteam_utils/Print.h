#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_

#include <iomanip>
#include <iostream>
#include <cstring>

////////////////////////////////////////////////////
// Settings for RTT logs

/*
 * Enable/disable rtt log levels entirely
 */
#define RTT_DEBUG_LOGS 1
#define RTT_ERROR_LOGS 1
#define RTT_WARNING_LOGS 1
#define RTT_INFO_LOGS 1
#define RTT_SUCCESS_LOGS 1

/*
 * Enable/disable fancy logging
 * When enabled, logs are printed in columns, have filenames, linenumbers, are clickable and optionally have colors.
 * When disabled, logs are converted to simple cout statements.
 */
#define RTT_FANCY_LOGS 1

/*
 * Enable/disable colors in fancy logs.
 * useful if colors are not supported by your terminal.
 * read more about ansi escape codes: https://en.wikipedia.org/wiki/ANSI_escape_code
 */
#define RTT_COLORED_LOGS 1

////////////////////////////////////////////////////
// Settings for RTT logs

// macro to get the filename from the full path
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#if RTT_FANCY_LOGS

#if RTT_DEBUG_LOGS
#define RTT_DEBUG(...) \
    { Printer::fancy_print("\033[35m", "DEBUG", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define RTT_DEBUG(...)
#endif

#if RTT_WARNING_LOGS
#define RTT_WARNING(...) \
    { Printer::fancy_print("\033[93;3m", "WARNING", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define RTT_WARNING(...)
#endif

#if RTT_ERROR_LOGS
#define RTT_ERROR(...) \
    { Printer::fancy_print("\033[31;1m", "ERROR", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define RTT_ERROR(...)
#endif

#if RTT_INFO_LOGS
#define RTT_INFO(...) \
    { Printer::fancy_print("\033[37m", "INFO", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define RTT_INFO(...)
#endif

#if RTT_SUCCESS_LOGS
#define RTT_SUCCESS(...) \
    { Printer::fancy_print("\033[32m", "SUCCESS", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define RTT_SUCCESS(...)
#endif

#else
#define RTT_DEBUG(...) \
    { Printer::simple_print(__VA_ARGS__); }
#define RTT_ERROR(...) \
    { Printer::simple_print(__VA_ARGS__); }
#define RTT_WARNING(...) \
    { Printer::simple_print(__VA_ARGS__); }
#define RTT_INFO(...) \
    { Printer::simple_print(__VA_ARGS__); }
#define RTT_SUCCESS(...) \
    { Printer::simple_print(__VA_ARGS__); }
#endif

class Printer {
   public:
    template <typename... Args>
    static void fancy_print(const char* color, const char* type, const std::string& file, [[maybe_unused]] const char* func, int line, Args&&... args) noexcept {
        auto fileTxt = file + ":" + std::to_string(line);
#if RTT_COLORED_LOGS
        std::cout << color;
#endif
        std::cout << std::setw(10) << std::left;
        std::cout << type << std::setw(32) << std::left;
        std::cout << fileTxt << std::left;
        ((std::cout << std::forward<Args>(args)), ...);
#if RTT_COLORED_LOGS
        std::cout << "\033[0m";  // reset the string formatting
#endif
        std::cout << std::endl;
    }

    template <typename... Args>
    static void simple_print(Args&&... args) noexcept {
        ((std::cout << std::forward<Args>(args)), ...);
        std::cout << std::endl;
    }
};

#endif  // RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
