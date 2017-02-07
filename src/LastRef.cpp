#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"

#include "roboteam_utils/LastRef.h"
#include "ros/ros.h"

namespace rtt {

roboteam_msgs::RefereeData LastRef::lastRef;

roboteam_msgs::RefereeData LastRef::get() {
    return LastRef::lastRef;
}

void LastRef::set(roboteam_msgs::RefereeData refCommand) {
    LastRef::lastRef = refCommand;
}

}
