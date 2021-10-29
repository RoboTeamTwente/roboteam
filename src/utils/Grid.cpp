//
// Created by jesse on 18-05-20.
//

#include "Grid.h"
namespace rtt {
Grid::Grid(double offSetX, double offSetY, double regionWidth, double regionHeight, int numSegmentsX, int numSegmentsY)
    : offSetX{offSetX}, offSetY{offSetY}, regionWidth{regionWidth}, regionHeight{regionHeight}, numSegmentsX{numSegmentsX}, numSegmentsY{numSegmentsY} {
    this->stepSizeX = regionWidth / numSegmentsX;
    this->stepSizeY = regionHeight / numSegmentsY;
    for (int i = 0; i <= numSegmentsX; i++) {
        std::vector<Vector2> a;
        points.emplace_back(a);
    }

    for (int i = 0; i <= numSegmentsX; i++) {
        for (int j = 0; j <= numSegmentsY; j++) {
            points[i].emplace_back(Vector2(offSetX + stepSizeX * i, offSetY + stepSizeY * j));
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

double Grid::getRegionHeight() const { return regionHeight; }

int Grid::getNumSegmentsX() const { return numSegmentsX; }

int Grid::getNumSegmentsY() const { return numSegmentsY; }

double Grid::getStepSizeX() const { return stepSizeX; }

double Grid::getStepSizeY() const { return stepSizeY; }

}  // namespace rtt
