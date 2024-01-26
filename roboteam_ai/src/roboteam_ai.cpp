#include <roboteam_utils/Print.h>

#include <csignal>
#include <memory>
#include <sstream>
#include <utility>

#include "RobotHubMode.h"
#include "STPManager.h"
#include "gui/networking/InterfaceGateway.h"
#include "roboteam_utils/ArgParser.h"
#include "roboteam_utils/Timer.h"
#include "utilities/GameSettings.h"
#include "utilities/IOManager.h"

// Create a flag which signals to stpThread to stop
std::atomic_flag stopFlag = ATOMIC_FLAG_INIT;

void initializeExitHandler() {
    struct sigaction sa {};
    sa.sa_handler = [](int) {
        RTT_INFO("SIGINT received, stopping...")
        stopFlag.test_and_set();
    };
    sigaction(SIGINT, &sa, nullptr);
}

void runStp(std::shared_ptr<rtt::ai::gui::net::InterfaceGateway> interfaceGateway) {
    rtt::STPManager app{std::move(interfaceGateway)};
    app.start(stopFlag);
}

int main(int argc, char** argv) {
    RTT_INFO("Starting RoboTeam AI. Supported flags are: (--primary-ai | --secondary-ai), --basesation")

    const std::vector<std::string> args(argv, argv + argc);
    bool isPrimaryFlag = rtt::findFlagValue(args, "--primary-ai", true).has_value();
    bool isSecondaryFlag = rtt::findFlagValue(args, "--secondary-ai", true).has_value();

    if ((!isPrimaryFlag && !isSecondaryFlag) || (isPrimaryFlag && isSecondaryFlag)) {
        RTT_ERROR("You must provide either \"--primary-ai\" or \"--secondary-ai\" as a command line argument.");
        return 1;
    }

    RTT_INFO("\n",
             "                                           \n"
             "  ██████╗ ████████╗████████╗     █████╗ ██╗\n"
             "  ██╔══██╗╚══██╔══╝╚══██╔══╝    ██╔══██╗██║\n"
             "  ██████╔╝   ██║      ██║       ███████║██║\n"
             "  ██╔══██╗   ██║      ██║       ██╔══██║██║\n"
             "  ██║  ██║   ██║      ██║       ██║  ██║██║\n"
             "  ╚═╝  ╚═╝   ╚═╝      ╚═╝       ╚═╝  ╚═╝╚═╝\n"
             "                                           ")

    RTT_DEBUG("Debug prints enabled")

    // Set up the settings based on cmd arguments
    rtt::GameSettings::setPrimaryAI(isPrimaryFlag);

    // If primary AI, we start at being yellow on the left
    rtt::GameSettings::setLeft(rtt::GameSettings::isPrimaryAI());
    if (!rtt::GameSettings::setYellow(rtt::GameSettings::isPrimaryAI())) {
        RTT_ERROR("Could not obtain command publishing channel. Exiting...")
        return 1;
    }

    // We default to the simulator, but if the --basestation flag is given, we set the mode to basestation
    rtt::GameSettings::setRobotHubMode(rtt::findFlagValue(args, "--basestation", true).has_value() ? rtt::net::RobotHubMode::BASESTATION : rtt::net::RobotHubMode::SIMULATOR);

    const auto interfacePort = rtt::GameSettings::isPrimaryAI() ? 12676 : 12677;
    std::ostringstream ss;
    ss << "\n";
    ss << "🤖 AI Information\n";
    ss << "├── Initialized as: " << (rtt::GameSettings::isPrimaryAI() ? "PRIMARY" : "SECONDARY") << "\n";
    ss << "├── Interface port: " << interfacePort << "\n";
    ss << "├── Starting as color: " << (rtt::GameSettings::isYellow() ? "🟨 YELLOW" : "🟦 BLUE") << "\n";
    ss << "├── Starting in mode: " << rtt::net::robotHubModeToString(rtt::GameSettings::getRobotHubMode()) << "🕹️\n";
    ss << "├── Playing on side: " << (rtt::GameSettings::isLeft() ? "⬅️ LEFT" : "➡️ RIGHT") << "\n";
    ss << "└── This AI will " << (rtt::GameSettings::isPrimaryAI() ? "" : "NOT") << " broadcast settings \n";
    RTT_INFO(ss.str());

    if (!rtt::ai::io::io.init(rtt::GameSettings::isPrimaryAI())) {
        RTT_ERROR("Failed to initialize IO Manager. Exiting...")
        return 0;
    }

    RTT_DEBUG("Initialize Interface Server");
    auto interfaceGateway = std::make_shared<rtt::ai::gui::net::InterfaceGateway>(interfacePort);  /// Shared-prt because the variable is shared accross threads

    initializeExitHandler();
    std::thread stpThread(runStp, interfaceGateway);
    stpThread.join();
}
