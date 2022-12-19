#include "Handler.h"

int main(int argc, char** argv) {
    auto itLog = std::find(argv, argv + argc, std::string("-log"));
    bool shouldLog = itLog != argv + argc;

    Handler handler;
    handler.start(shouldLog);
    return 0;
}