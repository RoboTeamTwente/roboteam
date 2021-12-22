//
// Created by jesse on 18-05-20.
//

#include "Grid.h"
namespace rtt {
Grid::Grid(double offSetX, double offSetY, double regionWidth, double regionLength, int numPointsX, int numPointsY)
    : offSetX{offSetX}, offSetY{offSetY}, regionWidth{regionWidth}, regionLength{regionLength}, numPointsX{numPointsX}, numPointsY{numPointsY} {
    this->stepSizeX = regionLength / numPointsX;
    this->stepSizeY = regionWidth / numPointsY;

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

/**
 * Remember this vector returns a nested list of points,
 * so you will want to loop through both vectors to get the full grid
 * @return points, the nested vector containing all Vector2 points within this grid
 */
const std::vector<std::vector<Vector2>> &Grid::getPoints() const { return points; }

double Grid::getOffSetX() const { return offSetX; }

double Grid::getOffSetY() const { return offSetY; }

double Grid::getRegionWidth() const { return regionWidth; }

double Grid::getRegionLength() const { return regionLength; }

int Grid::getNumPointsX() const { return numPointsX; }

int Grid::getNumPointsY() const { return numPointsY; }

double Grid::getStepSizeX() const { return stepSizeX; }

double Grid::getStepSizeY() const { return stepSizeY; }

}  // namespace rtt
