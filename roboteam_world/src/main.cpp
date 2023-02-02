#include "Handler.h"

std::optional<std::string> findFlagValue(int argc, char** argv, std::string flag){
    // Search for flag
    auto it = std::find(argv, argv + argc, flag);
    // If flag is present in arguments
    if(it < argv + argc){
        // No value found for flag.. 
        if(argc < it - argv + 2){
            std::cout << "Warning! flag '" << flag << "' raised but no value given." << std::endl << std::endl;
            return std::nullopt;
        }
        // Return value right after flag
        return { *(it + 1) };
    }   

    // Flag not present, return "nothing"
    return std::nullopt;
}

int main(int argc, char** argv) {
    std::cout << "Usage: ./roboteam_observer --log --vision-ip <ip-address> --referee-ip <ip-address> --vision-port <port> --referee-port <port>" << std::endl;
    std::cout << "  log: log data to file" << std::endl;
    std::cout << "  vision-ip  : ip to listen for vision.  Defaults to 224.5.23.2" << std::endl;
    std::cout << "  referee-ip : ip to listen for referee. Defaults to 224.5.23.1" << std::endl;
    std::cout << "  vision-port  : port to listen for vision.  Defaults to 10006 (use 10020 for ER-Force simulator)" << std::endl;
    std::cout << "  referee-port : port to listen for referee. Defaults to 10003" << std::endl;

    std::cout << std::endl;

    std::string visionip = "224.5.23.2";
    std::string refereeip = "224.5.23.1";
    std::string visionport_str = "10006";
    std::string refereeport_str = "10003";

    // Search for log flag --log
    auto itLog = std::find(argv, argv + argc, std::string("--log"));
    bool shouldLog = itLog != argv + argc;

    std::optional<std::string> val;

    val = findFlagValue(argc, argv, "--vision-ip");
    if(val) visionip = *val;

    val = findFlagValue(argc, argv, "--referee-ip");
    if(val) refereeip = *val;

    val = findFlagValue(argc, argv, "--vision-port");
    if(val) visionport_str = *val;

    val = findFlagValue(argc, argv, "--referee-port");
    if(val) refereeport_str = *val;

    int visionport  = atoi( visionport_str.c_str() );
    int refereeport = atoi( refereeport_str.c_str() );

    Handler handler;
    handler.start(visionip, refereeip, visionport, refereeport, shouldLog);
    return 0;
}