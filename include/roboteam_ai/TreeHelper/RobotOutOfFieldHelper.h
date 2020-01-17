//
// Created by jessevw on 21.11.19.
//

#ifndef RTT_ROBOTOUTOFFIELDHELPER_H
#define RTT_ROBOTOUTOFFIELDHELPER_H
#include <include/roboteam_ai/bt/Node.hpp>

namespace bt {
class RobotOutOfFieldHelper {
   public:
    RobotOutOfFieldHelper();
    std::shared_ptr<Node> createRobotOutOfFieldHelper();
};

}  // namespace bt
#endif  // RTT_ROBOTOUTOFFIELDHELPER_H
