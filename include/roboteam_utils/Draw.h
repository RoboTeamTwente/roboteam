#pragma once

#include "ros/ros.h"

#include <vector>

#include "roboteam_msgs/DebugPoint.h"
#include "roboteam_msgs/DebugLine.h"
#include "roboteam_msgs/Vector2f.h"
#include "roboteam_msgs/Color.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

class Draw {

public:
    Draw();
    void DrawLine(std::string name, Vector2 start, Vector2 stop);
    void DrawLine(std::string name, std::vector<Vector2> points);
    void RemoveLine(std::string name);
    void DrawPoint(std::string name, Vector2 point);
    void RemovePoint(std::string name);
    void SetColor(int r, int g, int b);

private:
    ros::NodeHandle n;
    ros::Publisher debugPub;
    ros::Publisher debugPubPoint;
    roboteam_msgs::Color color;
};

} // rtt
