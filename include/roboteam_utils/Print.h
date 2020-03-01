#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_

#include <iostream>
#include <iomanip>

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
#define rtt_debug(...) { Printer::print("\033[95m", "DEBUG", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define rtt_debug(...)
#endif

#if RTT_WARNING_LOGS
#define rtt_warning(...) { Printer::print("\033[93m", "WARNING", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define rtt_warning(...)
#endif

#if RTT_ERROR_LOGS
#define rtt_error(...) { Printer::print("\033[91m","ERROR", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define rtt_error(...)
#endif

#if RTT_INFO_LOGS
#define rtt_info(...) { Printer::print("\033[37m", "INFO", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define rtt_info(...)
#endif

#if RTT_SUCCESS_LOGS
#define rtt_success(...) { Printer::print("\033[92m", "SUCCESS", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#else
#define rtt_success(...)
#endif

#else
    #define rtt_debug(...) { std::cout << __VA_ARGS__ << std::endl; }
    #define rtt_error(...) { std::cerr << __VA_ARGS__ << std::endl; }
    #define rtt_warning(...) { std::cout << __VA_ARGS__ << std::endl; }
    #define rtt_info(...) { std::cout << __VA_ARGS__ << std::endl; }
    #define rtt_success(...) { std::cout << __VA_ARGS__ << std::endl; }
#endif

class Printer {
 public:
  static void print(const char * color, const char * type, const std::string& file, const char * func, int line, const std::string& txt) {
      auto fileTxt = file + ":" + std::to_string(line);
#if RTT_COLORED_LOGS
      std::cout << color;
#endif
      std::cout << std::setw(10) << std::left;
      std::cout << type << std::setw(32) << std::left;
      std::cout << fileTxt << std::setw(32) << std::left;
      std::cout << txt;
#if RTT_COLORED_LOGS
      std::cout << "\033[0m"; // reset the string formatting
#endif
      std::cout << std::endl;
  }
};

#endif //RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
