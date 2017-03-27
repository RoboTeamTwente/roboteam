#pragma once

#include <cstdlib>
#include <exception>
#include <csignal>
#include <string>
#include <iostream>
#include <execinfo.h>

namespace rtt {

namespace detail { 

void fallbackTerminate() {
    std::signal(SIGABRT, SIG_DFL);
    abort();
}

}

void defaultPreExit() {
    std::cerr << "ERROR; Backtrace:\n";
    void* bt[20];
    size_t size = backtrace(bt, 20);
    char** strings = backtrace_symbols(bt, 20);
    for (size_t i = 0; i < size; i++) {
        std::cerr << "\t" << strings[i] << "\n";
    }
    free(strings);
}

// Brackets below are just in case someone wants a variable ´realfun' in their main.
    
#define REGISTER_GRACEFUL_EXIT_AFTER(fun) {\
     auto realfun = [=](int i) { fun(); exit(i); }; \
     std::signal(SIGABRT, realfun); \
     std::signal(SIGSTOP, realfun); \
     std::signal(SIGSEGV, realfun); \
     std::atexit(fun); \
     std::at_quick_exit(fun); \
     std::set_terminate(::rtt::detail::fallbackTerminate); \
  }
#define REGISTER_GRACEFUL_EXIT {\
     auto realfun = [=](int i) { exit(i); }; \
     std::signal(SIGABRT, realfun); \
     std::signal(SIGSTOP, realfun); \
     std::signal(SIGSEGV, realfun); \
     std::set_terminate(::rtt::detail::fallbackTerminate);\
  }

}