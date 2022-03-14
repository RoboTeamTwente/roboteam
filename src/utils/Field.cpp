#include <Field.hpp>

namespace rtt {

Field Field::createField(double fieldWidth, double fieldHeight, double penaltyWidth, double penaltyHeight, double goalWidth, double goalHeight, double boundaryWidth, Circle centerCircle, Vector2 leftPenaltyPoint, Vector2 rightPenaltyPoint) {
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

    Vector2 e(a.x,  penaltyHeight/2);
    Vector2 f(a.x + penaltyWidth, -penaltyHeight/2);

    Vector2 g(d.x - penaltyWidth, penaltyHeight/2);
    Vector2 h(d.x,  -penaltyHeight/2);

    Vector2 i(a.x - goalWidth, goalHeight/2);
    Vector2 j(a.x,  -goalHeight/2);

    Vector2 k(d.x + goalWidth, goalHeight/2);
    Vector2 l(d.x,  -goalHeight/2);

    Field field;

    // Now set the rectangles from these points
    field.playArea          = FieldRectangle(a, d);
    field.leftPlayArea      = FieldRectangle(a, b);
    field.rightPlayArea     = FieldRectangle(c, d);
    field.leftDefenseArea   = FieldRectangle(e, f);
    field.rightDefenseArea  = FieldRectangle(g, h);
    field.leftGoalArea      = FieldRectangle(i, j);
    field.rightGoalArea     = FieldRectangle(k, l);

    // And set remaining variables
    field.boundaryWidth = boundaryWidth;
    field.centerCircle = centerCircle;
    field.leftPenaltyPoint = leftPenaltyPoint;
    field.rightPenaltyPoint = rightPenaltyPoint;

    return field;
}

} // namespace rtt