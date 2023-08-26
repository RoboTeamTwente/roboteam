//
// Created by jesse on 18-05-20.
//

#ifndef RTT_GRID_H
#define RTT_GRID_H

#include <vector>

#include "Vector2.h"

namespace rtt {
class Grid {
   public:
    /**
     * A Grid is a 2D vector of Vector2 points. The points will be evenly spaced and centered in the given range.
     * @param offSetX the distance you want to horizontally shift the grid from 0 (where 0 is left edge)
     * @param offSetY the distance you want to vertically shift the grid from 0 (where 0 is bottom edge)
     * @param regionWidth the length of the region you want the grid to encompass (x-direction)
     * @param regionHeight the width of the region you want the grid to encompass (y-direction)
     * @param numStepsX number of segments to divide the grid into in x direction
     * @param numStepsY number of segments to divide the grid into in y direction
     */
    explicit Grid(double offSetX, double offSetY, double regionWidth, double regionHeight, int numPointsX, int numPointsY);

    // A default Grid of 3x3=9 elements, starting at (0, 0)
    Grid();

    [[nodiscard]] double getOffSetX() const;
    [[nodiscard]] double getOffSetY() const;
    [[nodiscard]] double getRegionWidth() const;
    [[nodiscard]] double getRegionHeight() const;
    [[nodiscard]] int getNumPointsX() const;
    [[nodiscard]] int getNumPointsY() const;
    [[nodiscard]] double getStepSizeX() const;
    [[nodiscard]] double getStepSizeY() const;
    [[nodiscard]] const std::vector<std::vector<Vector2>>& getPoints() const;

    bool operator==(const Grid& other) const;

   private:
    /**
     * nested vector, first index corresponds to nth x element, second to nth y element
     */
    std::vector<std::vector<Vector2>> points;
    double offSetX;
    double offSetY;
    double regionHeight;
    double regionWidth;
    int numPointsX;
    int numPointsY;
    double stepSizeX;
    double stepSizeY;
};
}  // namespace rtt

#endif  // RTT_GRID_H
