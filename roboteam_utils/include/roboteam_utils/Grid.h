#pragma once

#include <roboteam_utils/Rectangle.hpp>

namespace rtt {

/*
 * A general Grid class that can contain any type of Rectangle, including another grid
 */
template <typename T>
class Grid : virtual Rectangle {
public:

    // Gets the cell at the specified location, can throw InvalidCellLocation
    virtual const T& getCell(unsigned int row, unsigned int column) const = 0;

    // A list of all cells, from left to right, top to bottom
    virtual const std::vector<T>& getCells() const = 0;

    virtual unsigned int getRowSize() const = 0;
    virtual unsigned int getColumnSize() const = 0;
};

class InvalidCellLocation : private std::exception {
public:
    explicit InvalidCellLocation(const std::string& message);
    const char* what() const noexcept override;
private:
    const std::string message;
};
class InvalidGridSizeException : private std::exception {
public:
    explicit InvalidGridSizeException(const std::string& message);
    const char* what() const noexcept override;
private:
    const std::string message;
};

} // namespace rtt