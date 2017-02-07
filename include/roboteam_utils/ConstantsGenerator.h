#pragma once

#include <string>
#include <ostream>
#include "roboteam_utils/json.hpp"

namespace rtt {
    
void generate_constants(const std::string& source, const std::string& dest);   
void generate_constants(const nlohmann::json& json, std::ostream& dest_str);
    
}
