//
// Created by Dawid Kulikowski on 05/08/2021.
//

#include "roboteam_interface/InterfaceDeclarations.h"
#include "nlohmann/json.hpp"

#include <fstream>
#include <iostream>


int main() {

    std::ifstream jsonFile("interface_declarations.json");
    nlohmann::json decls;
    jsonFile >> decls;

    rtt::Interface::InterfaceDeclarations ifaceDecls(decls);

    std::cout << ifaceDecls.getDeclarations()[0].path << std::endl;

}