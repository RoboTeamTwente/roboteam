//
// Created by jesse on 18-05-20.
//

#ifndef RTT_GRID_H
#define RTT_GRID_H

#include "Vector2.h"
#include <vector>

namespace rtt {
class Grid {
   public:
    /**
     * A Grid is a 2D vector of Vector2 points.
     * @param offSetX the distance you want to horizontally shift the grid from 0 (where 0 is left edge)
     * @param offSetY the distance you want to vertically shift the grid from 0 (where 0 is bottom edge)
     * @param regionWidth the width of the region you want the grid to encompass
     * @param regionHeight the height of the region you want the grid to encompass
     * @param numStepsX number of segments to divide the grid into in x direction
     * @param numStepsY number of segments to divide the grid into in y direction
     */
    Grid(double offSetX, double offSetY, double regionWidth, double regionHeight, int numSegmentsX, int numSegmentsY);

    [[nodiscard]] double getOffSetX() const;
    [[nodiscard]] double getOffSetY() const;
    [[nodiscard]] double getRegionWidth() const;
    [[nodiscard]] double getRegionHeight() const;
    [[nodiscard]] int getNumSegmentsX() const;
    [[nodiscard]] int getNumSegmentsY() const;
    [[nodiscard]] double getStepSizeX() const;
    [[nodiscard]] double getStepSizeY() const;
    [[nodiscard]] const std::vector<std::vector<Vector2>> &getPoints() const;

   private:
    /**
     * nested vector, first index corresponds to nth x element, second to nth y element
     */
    std::vector<std::vector<Vector2>> points;
    double offSetX;
    double offSetY;
    double regionWidth;
    double regionHeight;
    int numSegmentsX;
    int numSegmentsY;
    double stepSizeX;
    double stepSizeY;
};
}  // namespace rtt

#endif  // RTT_GRID_H
