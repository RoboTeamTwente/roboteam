#include "stp/skills/TestSkill.h"

namespace rtt::ai::stp::skill {

Status TestSkill::onUpdate(const StpInfo &) noexcept { return Status::Running; }

const char *TestSkill::getName() { return "TestSkill"; }

}  // namespace rtt::ai::stp::skill
