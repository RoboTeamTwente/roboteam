#include "stp/roles/active/PassReceiver.h"

#include "stp/tactics/active/Receive.h"

namespace rtt::ai::stp::role {

PassReceiver::PassReceiver(std::string name) : Role(std::move(name)) { robotTactics = collections::state_machine<Tactic, Status, StpInfo>{tactic::Receive()}; }
}  // namespace rtt::ai::stp::role