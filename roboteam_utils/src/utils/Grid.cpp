#include <roboteam_utils/Grid.h>

namespace rtt {

InvalidCellLocation::InvalidCellLocation(const std::string &message) : message(message) {}
const char *InvalidCellLocation::what() const noexcept { return this->message.c_str(); }

InvalidGridSizeException::InvalidGridSizeException(const std::string &message) : message(message) {}
const char *InvalidGridSizeException::what() const noexcept { return this->message.c_str(); }

}; // namespace rtt