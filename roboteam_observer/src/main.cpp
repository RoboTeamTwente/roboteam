#include "Handler.h"
#include "roboteam_utils/ArgParser.h"

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv, argv + argc);

    // Search for log flag --log
    auto itLog = std::find(args.begin(), args.end(), std::string("--log"));
    bool shouldLog = itLog != args.end();

    std::optional<std::string> val;

    std::string visionip = "224.5.23.2";
    std::string refereeip = "224.5.23.1";
    std::string visionport_str = "10006";
    std::string refereeport_str = "10003";
    std::vector<int> camera_ids;

    if (rtt::findFlagValue(args, "--help", true) || rtt::findFlagValue(args, "-h", true)) {
        std::cout << "Usage: ./roboteam_observer --log --vision-ip <ip-address> --referee-ip <ip-address> --vision-port <port> --referee-port <port>" << std::endl;
        std::cout << "  log: log data to file" << std::endl;
        std::cout << "  vision-ip  : ip to listen for vision.  Defaults to 224.5.23.2" << std::endl;
        std::cout << "  referee-ip : ip to listen for referee. Defaults to 224.5.23.1" << std::endl;
        std::cout << "  vision-port  : port to listen for vision.  Defaults to 10006 (use 10020 for ER-Force simulator)" << std::endl;
        std::cout << "  referee-port : port to listen for referee. Defaults to 10003" << std::endl;
        std::cout << "  camera-ids : comma separated list of camera ids to use. Defaults to all cameras" << std::endl;
        std::cout << "Use -h or --help to print this help message" << std::endl;

        std::cout << std::endl;
    }

    val = rtt::findFlagValue(args, "--vision-ip");
    if (val) {
        visionip = *val;
    }

    val = rtt::findFlagValue(args, "--referee-ip");
    if (val) {
        refereeip = *val;
    }

    val = rtt::findFlagValue(args, "--vision-port");
    if (val) {
        visionport_str = *val;
    }

    val = rtt::findFlagValue(args, "--referee-port");
    if (val) {
        refereeport_str = *val;
    }

    val = rtt::findFlagValue(args, "--camera-ids");
    if (val) {
        std::string camera_ids_str = *val;
        std::stringstream ss(camera_ids_str);
        std::string token;
        while (std::getline(ss, token, ',')) {
            camera_ids.push_back(std::stoi(token));
        }
    }

    int visionport = std::stoi(visionport_str);
    int refereeport = std::stoi(refereeport_str);

    if (camera_ids.empty()) {
        std::cout << "Camera ids: all\n";
    } else {
        std::cout << "Camera ids: ";
        std::copy(camera_ids.begin(), camera_ids.end(), std::ostream_iterator<int>(std::cout, " "));
        std::cout << '\n';
    }

    Handler handler;
    handler.start(visionip, refereeip, visionport, refereeport, shouldLog, camera_ids);
    return 0;
}