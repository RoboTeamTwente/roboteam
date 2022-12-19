#include <roboteam_utils/Grid.h>
#include <roboteam_utils/FastRectangle.hpp>
#include <roboteam_utils/LazyRectangle.hpp>

namespace rtt {

template <int X, int Y, typename T>
class FastGrid : public virtual Grid<T>, public virtual FastRectangle {
public:
    explicit FastGrid(const Rectangle& r) : FastRectangle(r) {
        if (X < 1 || Y < 1) throw InvalidGridSizeException("Invalid grid size, cannot be lower than 1");

        // Populate the grid with rectangles of type T
        double cellWidth = this->width() / X;
        double cellHeight = this->height() / Y;

        for (int row = 0; row < Y; row++) {
            for (int column = 0; column < X; column++) {
                // Calculate the rectangle of the cell
                double cellTop = this->top() - row * cellHeight;
                double cellBottom = cellTop - cellHeight;
                double cellLeft = this->left() + column * cellWidth;
                double cellRight = cellLeft + cellWidth;

                // Now create the cell of the correct type and add it
                LazyRectangle cellBoundary({cellRight, cellTop}, {cellLeft, cellBottom});
                auto cell = T(cellBoundary);
                this->cells.push_back(cell);
            }
        }
    }

    const T& getCell(unsigned int x, unsigned int y) const override {
        if (y >= Y || x >= X) throw InvalidCellLocation("This cell does not exist");
        return this->cells[x + y * X];
    }
    const std::vector<T>& getCells() const override {
        return this->cells;
    }

    void addPointsToVec(std::vector<Vector2>& v) const override {
        for (const auto& cell : this->cells) {
            cell.addPointsToVec(v);
        }
    }

    unsigned int getRowSize() const override { return X; }
    unsigned int getColumnSize() const override { return Y; }

private:
    std::vector<T> cells;
};

} // namespace rtt