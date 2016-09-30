#include "roboteam_utils/Position.h"

namespace roboteam_utils {

Position::~Position(){}
Vector2 Position::location() {
    return Vector2(x, y);
}
double Position::getRot(){
	return rot;
}
Position Position::translate(const Vector2& vec) {
    return Position(x + vec.x, y + vec.y, rot);
}

Position Position::rotate(double rot_vel) {
    return Position(x, y, rot + rot_vel);
}

Position Position::move(const Vector2& vec, double rot_vel) {
    return Position(x + vec.x, y + vec.y, rot + rot_vel);
}

bool Position::operator==(const Position& other) {
    return x == other.x && y == other.y && rot == other.rot;
}

bool Position::operator!=(const Position& other) {
    return x != other.x || y != other.y || rot != other.rot;
}

}
