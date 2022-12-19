#pragma once

#include <roboteam_utils/FastGrid.hpp>

namespace rtt {

template <typename T>
class Grid3x3 : public virtual FastGrid<3, 3, T> {
public:
    explicit Grid3x3(const Rectangle& r) : FastGrid<3, 3, T>(r) {}

    const T& topLeftCell() const { return this->getCell(0, 0); }
    const T& topMiddleCell() const { return this->getCell(1, 0); }
    const T& topRightCell() const { return this->getCell(2, 0); }
    const T& middleLeftCell() const { return this->getCell(0, 1); }
    const T& middleMiddleCell() const { return this->getCell(1, 1); }
    const T& middleRightCell() const { return this->getCell(2, 1); }
    const T& bottomLeftCell() const { return this->getCell(0, 2); }
    const T& bottomMiddleCell() const { return this->getCell(1, 2); }
    const T& bottomRightCell() const { return this->getCell(2, 2); }
};

} // namespace rtt