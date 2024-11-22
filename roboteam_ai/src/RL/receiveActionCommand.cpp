#include <iostream>
#include <sstream>
#include <string>

int main() {
    std::string input;

    // Read each line from stdin
    while (std::getline(std::cin, input)) {
        std::istringstream iss(input);
        int numDefender, numAttacker, numWaller;
        char comma1, comma2;

        // Parse the input in the format "numDefender,numAttacker,numWaller"
        if (iss >> numDefender >> comma1 >> numAttacker >> comma2 >> numWaller &&
            comma1 == ',' && comma2 == ',') {
            std::cout << "Received: "
                      << "numDefender=" << numDefender
                      << ", numAttacker=" << numAttacker
                      << ", numWaller=" << numWaller
                      << std::endl;
        } else {
            std::cerr << "Error: Invalid input format" << std::endl;
        }
    }

    return 0;
}
