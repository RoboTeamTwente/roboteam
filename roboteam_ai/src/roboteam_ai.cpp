#include <roboteam_utils/Print.h>

#include "STPManager.h"
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

int main(int argc, char* argv[]) {

    int id = 0;

    if (argc == 2) {
        id = *argv[1] - '0';
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

    // get the id of the ai from the init
    rtt::Settings::setId(id);

    // If primary AI, we start at being yellow on the left
    if (!rtt::Settings::setYellow(rtt::Settings::isPrimaryAI())) {
        RTT_ERROR("Could not obtain command publishing channel. Exiting...")
        return 0;
    }

    rtt::Settings::setLeft(rtt::Settings::isPrimaryAI());
    rtt::Settings::setRobotHubMode(rtt::Settings::RobotHubMode::SIMULATOR);


    RTT_INFO("AI initialized as: ", (rtt::Settings::isPrimaryAI() ? "PRIMARY" : "SECONDARY"))
    RTT_INFO("Starting as color: ", (rtt::Settings::isYellow() ? "YELLOW" : "BLUE"))
    RTT_INFO("Playing on side: ", (rtt::Settings::isLeft() ? "LEFT" : "RIGHT"))
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
