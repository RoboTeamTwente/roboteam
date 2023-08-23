//
// Created by jesse on 18-05-20.
//

#include "Grid.h"
namespace rtt {

const Grid DEFAULT_GRID(0, 0, 3, 3, 3, 3);  // A Default grid, starting at (0, 0) with 3x3=9 elements

Grid::Grid(double offSetX, double offSetY, double regionWidth, double regionHeight, int numPointsX, int numPointsY)
    : offSetX{offSetX}, offSetY{offSetY}, regionHeight{regionHeight}, regionWidth{regionWidth}, numPointsX{numPointsX}, numPointsY{numPointsY} {
    this->stepSizeX = regionWidth / numPointsX;
    this->stepSizeY = regionHeight / numPointsY;

    for (int i = 0; i <= numPointsX; i++) {
        std::vector<Vector2> a;
        points.emplace_back(a);
    }

    for (int i = 0; i < numPointsX; i++) {
        for (int j = 0; j < numPointsY; j++) {
            points[i].emplace_back(Vector2(offSetX + stepSizeX * i + stepSizeX / 2, offSetY + stepSizeY * j + stepSizeY / 2));
        }
    }
}

Grid::Grid() {
    this->points = DEFAULT_GRID.points;
    this->offSetX = DEFAULT_GRID.offSetX;
    this->offSetY = DEFAULT_GRID.offSetY;
    this->regionHeight = DEFAULT_GRID.regionHeight;
    this->regionWidth = DEFAULT_GRID.regionWidth;
    this->numPointsX = DEFAULT_GRID.numPointsX;
    this->numPointsY = DEFAULT_GRID.numPointsY;
    this->stepSizeX = DEFAULT_GRID.stepSizeX;
    this->stepSizeY = DEFAULT_GRID.stepSizeY;
}

/**
 * Remember this vector returns a nested list of points,
 * so you will want to loop through both vectors to get the full grid
 * @return points, the nested vector containing all Vector2 points within this grid
 */
const std::vector<std::vector<Vector2>> &Grid::getPoints() const { return points; }

double Grid::getOffSetX() const { return offSetX; }

double Grid::getOffSetY() const { return offSetY; }

double Grid::getRegionWidth() const { return regionWidth; }

double Grid::getRegionHeight() const { return regionHeight; }

int Grid::getNumPointsX() const { return numPointsX; }

int Grid::getNumPointsY() const { return numPointsY; }

double Grid::getStepSizeX() const { return stepSizeX; }

double Grid::getStepSizeY() const { return stepSizeY; }

bool Grid::operator==(const Grid &other) const {
    return this->getOffSetX() == other.getOffSetX() && this->getOffSetY() == other.getOffSetY() && this->getRegionWidth() == other.getRegionWidth() &&
           this->getRegionHeight() == other.getRegionHeight() && this->getNumPointsX() == other.getNumPointsX() && this->getNumPointsY() == other.getNumPointsY();
}

}  // namespace rtt
