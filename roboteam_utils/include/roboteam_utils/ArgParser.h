#ifndef RTT_ARGPARSER_H
#define RTT_ARGPARSER_H

#include <string>

#include "roboteam_utils/Print.h"

namespace rtt {
std::optional<std::string> findFlagValue(const std::vector<std::string>& args, std::string flag, bool isBool = false) {
    // Search for flag
    auto it = std::find(args.begin(), args.end(), flag);
    // If flag is present in arguments
    if (it != args.end()) {
        // No value found for flag
        if (it == args.end() - 1) {
            // If the flag is bool we return an empty string or null optional
            if (!isBool) {
                RTT_WARNING("Warning! flag '", flag, "' raised but no value given.");
                return std::nullopt;
            }
            return {""};
        }
        // Return value right after flag
        return {*(it + 1)};
    }

    // Flag not present, return "nothing"
    return std::nullopt;
}
}  // namespace rtt

#endif  // RTT_ARGPARSER_H
