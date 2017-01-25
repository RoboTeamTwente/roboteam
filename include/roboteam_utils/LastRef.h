#pragma once

#include "roboteam_msgs/RefereeCommand.h"
#include "roboteam_msgs/RefereeData.h"
#include "roboteam_msgs/RefereeStage.h"

namespace rtt {

class LastRef {
    public:
	static roboteam_msgs::RefereeData get();
	static void set(roboteam_msgs::RefereeData refCommand);

    private:
    static roboteam_msgs::RefereeData lastRef;
};

}
