#include <vector>
#include <fstream>
#include <sstream>
#include <cstdio>

#include "roboteam_utils/ConstantsGenerator.h"

namespace rtt {

inline std::string strip(std::string s) {
    std::string str(s);
    str.erase(str.begin());
    str.erase(str.end() - 1);
    return s;
}
    
void generate_string(const std::string& key, const std::string& val, std::ostream& out, bool header) {
    if (header) {
        out << "extern const std::string " << key << ";\n\n";
        return;
    }
	out << "const std::string " << key << " = \"" << val << "\";\n\n";
}

void generate_simple_param(const std::string& key, const nlohmann::json& obj, std::ostream& out, bool header) {
    if (header) {
        out << "void get_" << key << "(" << strip(obj["type"]) << "&);\n";
        out << "void set_" << key << "(const " << strip(obj["type"]) << "&);\n";
        out << "bool has_" << key << "();\n\n";
        return;
    }
	out << "void get_" << key << "(" << strip(obj["type"]) << "& tgt) {\n";
	out << "\tros::param::get(" << obj["name"] << ", tgt);\n"; 

	out << "}\n\nvoid set_" << key << "(const " << strip(obj["type"]) << "& val) {\n";
	out << "\tros::param::set(" << obj["name"] << ", val);\n";

	out << "}\n\nbool has_" << key << "() {\n";
	out << "\treturn ros::param::has(" << obj["name"] << ");\n}\n\n";
}

void generate_enum_param(const std::string& key, const nlohmann::json& obj, std::ostream& out, bool header) {
    int size = obj["values"].size();
    if (header) {
        out << "extern const std::array<" << strip(obj["type"]) << ", " << size << "> " << key << "_valid_values;\n";
        out << "void get_" << key << "(" << strip(obj["type"]) << "& tgt, bool error_on_invalid = false);\n";
        out << "void set_" << key << "(const " << strip(obj["type"]) << "& val, bool error_on_invalid = true);\n";
        out << "bool has_" << key << "();\n\n";
        return;
    }
    
	out << "const std::array<" << strip(obj["type"]) << ", " << size << "> " << key << "_valid_values = {";
	auto it = obj["values"].begin();
	auto end = obj["values"].end();
	while (it != end) {
		out << *it;
		if (++it != end) out << ", ";
	}
	out << "};\n\n";
	
	out << "void get_" << key << "(" << strip(obj["type"]) << "& tgt, bool error_on_invalid) {\n";
	out << "\t" << strip(obj["type"]) << " val;\n\tros::param::get(" << obj["name"] << ", val);\n";
	out << "\tif(!has(" << key << "_valid_values, val)) {\n";
	out << "\t\tif (error_on_invalid) {\n\t\t\tthrow std::runtime_error(\"Param fetched with invalid value.\");\n\t\t} else {\n";
	out << "\t\t\tROS_WARN(\"Param " << key << " fetched with invalid value.\");\n\t\t}\n\t}\n";
	out << "\ttgt = val;\n}\n\n";

	out << "void set_" << key << "(" << strip(obj["type"]) << "& val, bool error_on_invalid) {\n";
	out << "\tif(!has(" << key << "_valid_values, val)) {\n";
	out << "\t\tif (error_on_invalid) {\n\t\t\tthrow new std::runtime_error(\"Tried to set invalid value to a param.\");\n\t\t} else {\n";
	out << "\t\t\tROS_WARN(\"Setting invalid value to param " << key << "\");\n\t\t}\n\t}\n";
	out << "\tros::param::set(" << obj["name"] << ", val);\n}\n\n";

	out << "bool has_" << key << "() {\n";
	out << "\treturn ros::param::has(" << obj["name"] << ");\n}\n\n";
}

void generate_constants(const nlohmann::json& json, std::ostream& dest_str, bool header) {
    
    dest_str << (header ? R"(#pragma once

#include <array>
#include <string>

// generated from String Constants

namespace rtt {

using string = std::string;

)" : R"(
#include <ros/ros.h>
#include "roboteam_utils/constants.h"

namespace rtt {
    
template<typename T, long unsigned int N>
static bool has(std::array<T, N> arr, T val) {
    for (unsigned int i = 0; i < N; i++) {
        if (arr[i] == val) {
            return true;
        }
    }
    return false;
}

)");
    
    nlohmann::json::object_t map((const nlohmann::json::object_t&)json);
    for (const auto& element : map) {
		std::string key = element.first;
		if (element.second.is_string()) {
			generate_string(key, element.second, dest_str, header);
		} else if (element.second.is_object()) {
			nlohmann::json obj = element.second;
			if (obj.find("values") != obj.end()) {
				generate_enum_param(key, obj, dest_str, header);
			} else {
				generate_simple_param(key, obj, dest_str, header);
            }
		} else {
			throw std::logic_error("Invalid JSON type");
		}
	}
    dest_str << "\n} // namespace rtt\n";
}

void generate_constants(const std::string& source, const std::string& dest, bool header) {
	std::ifstream source_str(source);
	nlohmann::json json = nlohmann::json::parse(source_str);
	std::ofstream dest_str(dest, std::ofstream::out | std::ofstream::trunc);
    generate_constants(json, dest_str, header);
}

}

int main(int argc, char* argv[]) {
    bool header = argc > 1 && std::string(argv[1]) == "header";
    nlohmann::json json = nlohmann::json::parse(std::cin);
    std::stringstream ss;
    rtt::generate_constants(json, ss, header);
    std::cout << ss.str();
}
