#include "roboteam_utils/Draw.h"

namespace rtt {

const bool RTT_ENABLE_DEBUG_GRAPHICS = RTT_CMAKE_ENABLE_DEBUG_GRAPHICS;

Draw::Draw() {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    debugPub = n.advertise<roboteam_msgs::DebugLine>("view_debug_lines", 1000);
    debugPubPoint = n.advertise<roboteam_msgs::DebugPoint>("view_debug_points", 1000);
}
}

void Draw::drawLine(std::string name, roboteam_utils::Vector2 start, roboteam_utils::Vector2 stop) {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    roboteam_msgs::DebugLine line;
    line.name = name;
    line.remove = false;
    roboteam_msgs::Vector2f startLine;
    startLine.x = start.x;
    startLine.y = start.y;
    roboteam_msgs::Vector2f stopLine;
    stopLine.x = stop.x + start.x;
    stopLine.y = stop.y + start.y;
    line.points.push_back(startLine);
    line.points.push_back(stopLine);
    line.color = color;
    debugPub.publish(line);
}
}

void Draw::drawLine(std::string name, std::vector<roboteam_utils::Vector2> points) {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    roboteam_msgs::DebugLine line;
    line.name = name;
    line.remove = false;

    for (auto& point : points) {
        roboteam_msgs::Vector2f pointMsg(point);
        line.points.push_back(pointMsg);
    }

    debugPub.publish(line);
}
}

void Draw::demoveLine(std::string name) {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    roboteam_msgs::DebugLine Line;
    Line.name = name;
    Line.remove = true;
    debugPub.publish(Line);
}
}

void Draw::drawPoint(std::string name, roboteam_utils::Vector2 point) {
    if (RTT_ENABLE_DEBUG_GRAPHICS) {
        roboteam_msgs::DebugPoint position;
        position.name = name;
        position.pos.x = point.x;
        position.pos.y = point.y;
        position.color = color;
        debugPubPoint.publish(position);
    }
}

void Draw::demovePoint(std::string name) {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    roboteam_msgs::DebugPoint position;
    position.name = name;
    position.remove = true;
    debugPubPoint.publish(position);
}
}

void Draw::setColor(int r, int g, int b) {
if (RTT_ENABLE_DEBUG_GRAPHICS) {
    color.r = r;
    color.g = g;
    color.b = b;
}
}

} // rtt
