#ifndef RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
#define RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_

#include <iostream>
#include <iomanip>

class Printer {
 public:
  static void print(const std::string& color, const std::string& type, const std::string& file, const std::string& func, int line, const std::string& txt) {
      auto fileTxt =  file + ":" + std::to_string(line);

      std::cout << color << std::setw(10) << std::left;
      std::cout << type << std::setw(32) << std::left;
      std::cout << fileTxt << std::setw(32) << std::left;
      std::cout << txt;
      std::cout << "\033[0m" << std::endl;
  }
};

#ifndef DEBUG_LOGS
    #define DEBUG_LOGS 1 // set debug mode
#endif

#if DEBUG_LOGS

// macro to get the filename from the full path
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

// read more about ansi escape codes: https://en.wikipedia.org/wiki/ANSI_escape_code
#define rtt_debug(...) { Printer::print("\033[95m", "DEBUG", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_error(...) { Printer::print("\033[91m","ERROR", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_warning(...) { Printer::print("\033[93m", "WARNING", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_info(...) { Printer::print("\033[37m", "INFO", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_success(...) { Printer::print("\033[92m", "SUCCESS", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__); }
#define rtt_crash(...) {\
    Printer::print("\033[91m", "CRASH", __FILENAME__, __FUNCTION__, __LINE__, __VA_ARGS__);\
    assert(false);\
}

#else
    #define rtt_debug(...)
    #define rtt_error(...)
    #define rtt_warning(...)
    #define rtt_info(...)
    #define rtt_success(...)
#endif

#endif //RTT_ROBOTEAM_AI_SRC_UTILITIES_PRINT_H_
