//
// Created by mrlukasbos on 4-12-18.
//

#ifndef ROBOTEAM_AI_INPUT_H
#define ROBOTEAM_AI_INPUT_H

#include <roboteam_utils/Vector2.h>

#include <QtGui/QColor>
#include <iostream>
#include <mutex>
#include <tuple>

#include "Toggles.h"

namespace rtt::ai::interface {

/**
 * @brief For drawing to the interface we keep 'drawings' to draw data to the screen.
 * a drawing represents a vector of points and some specifications on how to display those.
 * e.g: form, color, size, and depth.
 */
struct Drawing {
    /**
     * @brief Enumerator for the drawing method used in interface
     */
    enum DrawingMethod { LINES_CONNECTED, DOTS, CROSSES, CIRCLES, PLUSSES, ARROWS, REAL_LIFE_DOTS, REAL_LIFE_CIRCLES };

    /**
     * @brief Constructor of the Drawing struct
     * @param visual Type of visualization, these can be toggled in the interface settings
     * @param points Vector of locations the drawing should be drawn at
     * @param color Color of the drawing
     * @param robotId ID of the robot, which the drawing corresponds to
     * @param method Type of shape we want the drawing to be
     * @param width Width of the drawing points
     * @param height Height of the drawing points
     * @param strokeWidth Thickness of the drawing
     */
    Drawing(Visual visual, std::vector<Vector2> points, QColor color, int robotId = -1, DrawingMethod method = DOTS, double width = 0.0, double height = 0.0,
            double strokeWidth = 0.0)
        : visual(visual), points(std::move(points)), color(std::move(color)), robotId(robotId), method(method), width(width), height(height), strokeWidth(strokeWidth){};

    Visual visual; /**< Type of visualization, these can be toggled in the interface settings */
    std::vector<Vector2> points; /**< Vector of locations the drawing should be drawn at */
    QColor color; /**< Color of the drawing */
    int robotId; /**< ID of the robot, which the drawing corresponds to */
    DrawingMethod method; /**< Type of shape we want the drawing to be */

    // these values are used for dots, crosses and circles
    double width = 4.0; /**< Width of the drawing points */
    double height = 4.0; /**< Height of the drawing points */
    double strokeWidth = 2.0; /**< Thickness of the drawing */
};

/**
 * @brief Input class of the interface.
 * STP should use this class for drawing in the interface
 */
class Input {
   public:
    /**
     * @brief Explicit constructor for the Input class.
     * Reserves space for the drawings vector
     */
    explicit Input();

    /**
     * @brief Destructor for the Input class.
     * Empties the drawings vector
     */
    ~Input();

    /**
     * @brief Adds a drawing that needs to be drawn to the drawing vector.
     * @param drawing Drawing that needs to be drawn
     */
    static void addDrawing(Drawing drawing);

    /**
     * @brief Clears all drawings from the drawings vector
     */
    static void clearDrawings();

    /**
     * @brief Retrieves the drawings vector
     * @return A vector containing all drawings
     */
    static const std::vector<Drawing> getDrawings();

    /**
     * @brief Retrieves the last known FPS
     * @return The last known FPS
     */
    static int getFps();

    /**
     * @brief Sets the FPS as given
     * @param fps The given FPS
     */
    static void setFps(int fps);

   private:
    static std::vector<Drawing> drawings; /**< Vector containing all drawings that need to be drawn */
    static std::mutex drawingMutex; /**< Synchronizer for the drawings */
    static std::mutex fpsMutex; /**< Synchronizer for the FPS */
    static int FPS; /**< Last known FPS */
};

}  // namespace rtt::ai::interface

#endif  // ROBOTEAM_AI_INPUT_H
