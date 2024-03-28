#include <roboteam_logging/LogFileReader.h>

#include "Handler.h"

using rtt::LogFileReader;
int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, argv + argc);
    if (args.size() != 2) {
        std::cout << "Usage: ./roboteam_observerReplayLog <logfile>" << std::endl;
        return EXIT_FAILURE;
    }
    LogFileReader reader;
    bool good = reader.open(args[1]);
    if (!good) {
        return EXIT_FAILURE;
    }
    Handler handler;
    handler.startReplay(reader);
    reader.close();
    return EXIT_SUCCESS;
}