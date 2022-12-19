#include <roboteam_utils/Field.hpp>

namespace rtt {

bool Field::operator==(const Field &other) const {
    return this->leftPenaltyPoint == other.leftPenaltyPoint
        && this->rightPenaltyPoint == other.rightPenaltyPoint
        && this->centerCircle == other.centerCircle
        && this->boundaryWidth == other.boundaryWidth
        && this->playArea == other.playArea
        && this->leftPlayArea == other.leftPlayArea
        && this->rightPlayArea == other.rightPlayArea
        && this->leftDefenseArea == other.leftDefenseArea
        && this->rightDefenseArea == other.rightDefenseArea
        && this->leftGoalArea == other.leftGoalArea
        && this->rightGoalArea == other.rightGoalArea;
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
    Vector2 a(-fieldWidth/2, fieldHeight/2);
    Vector2 d(fieldWidth/2, -fieldHeight/2);
    Vector2 b(0,    d.y);
    Vector2 c(0,    a.y);

    Vector2 e(a.x,  defenseHeight/2);
    Vector2 f(a.x + defenseWidth, -defenseHeight/2);

    Vector2 g(d.x - defenseWidth, defenseHeight/2);
    Vector2 h(d.x,  -defenseHeight/2);

    Vector2 i(a.x - goalWidth, goalHeight/2);
    Vector2 j(a.x,  -goalHeight/2);

    Vector2 k(d.x + goalWidth, goalHeight/2);
    Vector2 l(d.x,  -goalHeight/2);

    Field field = {
        .leftPenaltyPoint  = leftPenaltyPoint,
        .rightPenaltyPoint = rightPenaltyPoint,
        .centerCircle      = Circle(Vector2(0, 0), centerCircleRadius),
        .boundaryWidth     = boundaryWidth,
        .playArea          = FieldRectangle(a, d),
        .leftPlayArea      = FieldRectangle(a, b),
        .rightPlayArea     = FieldRectangle(c, d),
        .leftDefenseArea   = FieldRectangle(e, f),
        .rightDefenseArea  = FieldRectangle(g, h),
        .leftGoalArea      = FieldRectangle(i, j),
        .rightGoalArea     = FieldRectangle(k, l),
    };

    return field;
}

} // namespace rtt