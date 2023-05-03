#include <roboteam_utils/Print.h>

#include "STPManager.h"
#include "roboteam_utils/ArgParser.h"
#include "roboteam_utils/RobotHubMode.h"
#include "utilities/IOManager.h"
#include "utilities/Settings.h"
#include "world/World.hpp"

namespace ui = rtt::ai::interface;

ui::MainWindow* window;

void runStp(std::atomic_bool& exitApplication) {
    rtt::STPManager app{window};
    app.start(exitApplication);
}

void setDarkTheme() {
    qApp->setStyle(QStyleFactory::create("Fusion"));
    QPalette darkPalette;
    darkPalette.setColor(QPalette::Window, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::WindowText, Qt::white);
    darkPalette.setColor(QPalette::Base, QColor(25, 25, 25));
    darkPalette.setColor(QPalette::AlternateBase, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ToolTipBase, Qt::white);
    darkPalette.setColor(QPalette::ToolTipText, Qt::white);
    darkPalette.setColor(QPalette::Text, Qt::white);
    darkPalette.setColor(QPalette::Button, QColor(53, 53, 53));
    darkPalette.setColor(QPalette::ButtonText, Qt::white);
    darkPalette.setColor(QPalette::BrightText, Qt::red);
    darkPalette.setColor(QPalette::Link, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::Highlight, QColor(42, 130, 218));
    darkPalette.setColor(QPalette::HighlightedText, Qt::black);
    qApp->setPalette(darkPalette);
    qApp->setStyleSheet("QToolTip { color: #ffffff; background-color: #2a82da; border: 1px solid white; }");
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
    rtt::Settings::setPrimaryAI(isPrimaryFlag);

    // If primary AI, we start at being yellow on the left
    rtt::Settings::setLeft(rtt::Settings::isPrimaryAI());
    if (!rtt::Settings::setYellow(rtt::Settings::isPrimaryAI())) {
        RTT_ERROR("Could not obtain command publishing channel. Exiting...")
        return 1;
    }

    // We default to the simulator, but if the --basestation flag is given, we set the mode to basestation
    rtt::Settings::setRobotHubMode(
        rtt::findFlagValue(args, "--basestation", true).has_value() ? rtt::RobotHubMode::BASESTATION : rtt::RobotHubMode::SIMULATOR
    );


    RTT_INFO("AI initialized as: ", (rtt::Settings::isPrimaryAI() ? "PRIMARY" : "SECONDARY"))
    RTT_INFO("Starting as color: ", (rtt::Settings::isYellow() ? "🟨 YELLOW" : "🟦 BLUE"))
    RTT_INFO("Starting in mode: ", rtt::modeToString(rtt::Settings::getRobotHubMode()))
    RTT_INFO("Playing on side: ", (rtt::Settings::isLeft() ? "⬅️ LEFT" : "➡️ RIGHT"))
    RTT_INFO("This AI will ", rtt::Settings::isPrimaryAI() ? "" : "NOT ", "broadcast settings")

    if (!rtt::ai::io::io.init(rtt::Settings::isPrimaryAI())) {
        RTT_ERROR("Failed to initialize IO Manager. Exiting...")
        return 0;
    }

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
    // initialize the interface
    QApplication application(argc, argv);
    setDarkTheme();

    // Todo make this a not-global-static thingy
    window = new ui::MainWindow{};
    window->setWindowState(Qt::WindowMaximized);

    //Create a flag which signals to the STP thread to stop if the interface is stopped
    std::atomic_bool exitApplication = false;
    std::thread stpThread(runStp,std::ref(exitApplication));

    window->show();
    bool runQT = application.exec();
    exitApplication = true;
    stpThread.join();
    return runQT;
}
