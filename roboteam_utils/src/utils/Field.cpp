#include <roboteam_utils/Field.hpp>

namespace rtt {

    bool Field::operator==(const Field& other) const {
        return this->leftPenaltyPoint == other.leftPenaltyPoint && this->rightPenaltyPoint == other.rightPenaltyPoint && this->centerCircle == other.centerCircle && this->boundaryWidth == other.boundaryWidth && this->playArea == other.playArea && this->leftPlayArea == other.leftPlayArea && this->rightPlayArea == other.rightPlayArea && this->leftDefenseArea == other.leftDefenseArea && this->rightDefenseArea == other.rightDefenseArea && this->leftGoalArea == other.leftGoalArea && this->rightGoalArea == other.rightGoalArea && this->bottomLeftGrid == other.bottomLeftGrid && this->bottomMidGrid == other.bottomMidGrid && this->bottomRightGrid == other.bottomRightGrid && this->middleLeftGrid == other.middleLeftGrid && this->middleMidGrid == other.middleMidGrid && this->middleRightGrid == other.middleRightGrid && this->topLeftGrid == other.topLeftGrid && this->topMidGrid == other.topMidGrid && this->topRightGrid == other.topRightGrid;
    }

    Field Field::createField(double fieldWidth, double fieldHeight, double defenseWidth, double defenseHeight, double goalWidth, double goalHeight, double boundaryWidth, double centerCircleRadius, const Vector2& leftPenaltyPoint, const Vector2& rightPenaltyPoint) {
        //     A---------------C----------------
        //     |               |               |
        //     E-----         _|_         G----|
        //   I-|    |        / | \        |    K--
        //   | |    |       |  O  |       |    | |
        //   |-J    |        \_|_/        |    |-L
        //     |----F          |          -----H
        //     |               |               |
        //     ----------------B---------------D

        // First, calculate points to easily convert to rectangles
        Vector2 a(-fieldWidth / 2, fieldHeight / 2);
        Vector2 d(fieldWidth / 2, -fieldHeight / 2);
        Vector2 b(0, d.y);
        Vector2 c(0, a.y);

        Vector2 e(a.x, defenseHeight / 2);
        Vector2 f(a.x + defenseWidth, -defenseHeight / 2);

        Vector2 g(d.x - defenseWidth, defenseHeight / 2);
        Vector2 h(d.x, -defenseHeight / 2);

        Vector2 i(a.x - goalWidth, goalHeight / 2);
        Vector2 j(a.x, -goalHeight / 2);

        Vector2 k(d.x + goalWidth, goalHeight / 2);
        Vector2 l(d.x, -goalHeight / 2);

        Field field = {
            .leftPenaltyPoint = leftPenaltyPoint,
            .rightPenaltyPoint = rightPenaltyPoint,
            .centerCircle = Circle(Vector2(0, 0), centerCircleRadius),
            .boundaryWidth = boundaryWidth,
            .playArea = FastRectangle(a, d),
            .leftPlayArea = FastRectangle(a, b),
            .rightPlayArea = FastRectangle(c, d),
            .leftDefenseArea = FastRectangle(e, f),
            .rightDefenseArea = FastRectangle(g, h),
            .leftGoalArea = FastRectangle(i, j),
            .rightGoalArea = FastRectangle(k, l),
        };

        // Now set the grids of the field
        auto gridWidths = field.playArea.width() / 3;
        auto gridHeights = field.playArea.height() / 3;

        auto leftX = field.playArea.left();
        auto middleX = leftX + gridWidths;
        auto rightX = leftX + 2 * gridWidths;

        auto bottomY = field.playArea.bottom();
        auto middleY = bottomY + gridHeights;
        auto topY = bottomY + 2 * gridHeights;

        const double nSegments = 3;

        field.bottomLeftGrid = Grid(leftX, bottomY, gridWidths, gridHeights, nSegments, nSegments);
        field.bottomMidGrid = Grid(middleX, bottomY, gridWidths, gridHeights, nSegments, nSegments);
        field.bottomRightGrid = Grid(rightX, bottomY, gridWidths, gridHeights, nSegments, nSegments);

        field.middleLeftGrid = Grid(leftX, middleY, gridWidths, gridHeights, nSegments, nSegments);
        field.middleMidGrid = Grid(middleX, middleY, gridWidths, gridHeights, nSegments, nSegments);
        field.middleRightGrid = Grid(rightX, middleY, gridWidths, gridHeights, nSegments, nSegments);

        field.topLeftGrid = Grid(leftX, topY, gridWidths, gridHeights, nSegments, nSegments);
        field.topMidGrid = Grid(middleX, topY, gridWidths, gridHeights, nSegments, nSegments);
        field.topRightGrid = Grid(rightX, topY, gridWidths, gridHeights, nSegments, nSegments);

        return field;
    }

}  // namespace rtt