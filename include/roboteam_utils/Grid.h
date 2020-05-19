//
// Created by jesse on 18-05-20.
//

#ifndef RTT_GRID_H
#define RTT_GRID_H

#include "Vector2.h"

namespace rtt {
    class Grid {
    public:
        /**
         * A Grid is a 2D vector of Vector2 points.
         * @param offSetX the distance you want to horizontally shift the grid from 0
         * @param offSetY the distance you want to vertically shift the grid from 0
         * @param regionWidth the width of the region you want the grid to encompass
         * @param regionHeight the height of the region you want the grid to encompass
         * @param numStepsX number of steps to divide the grid into in x direction
         * @param numStepsY number of steps to divide the grid into in y direction
         */
        Grid(double offSetX, double offSetY, double regionWidth, double regionHeight, int numStepsX, int numStepsY);

        double getOffSetX() const;
        double getOffSetY() const;
        double getRegionWidth() const;
        double getRegionHeight() const;
        int getNumStepsX() const;
        int getNumStepsY() const;
        double getStepSizeX() const;
        double getStepSizeY() const;
        const std::vector<std::vector<Vector2>> &getPoints() const;

    private:
        /**
         * nested vector, first index corresponds to nth x element, second to nth y element
         */
        std::vector<std::vector<Vector2>> points;
        double offSetX;
        double offSetY;
        double regionWidth;
        double regionHeight;
        int numStepsX;
        int numStepsY;
        double stepSizeX;
        double stepSizeY;
    };
}

#endif //RTT_GRID_H
