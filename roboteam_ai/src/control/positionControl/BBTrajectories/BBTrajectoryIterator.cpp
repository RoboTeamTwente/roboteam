//
// Created by Martin Miksik on 19/02/2023.
//

#include "control/positionControl/BBTrajectories/BBTrajectoryIterator.h"

#include "control/positionControl/PositionControlUtils.h"
namespace rtt::ai::control {
//BBTrajectoryIterator::BBTrajectoryIterator(const BBTrajectory2D& trajectory, double time): trajectory(trajectory), time(time) {}
//
//StateVector& BBTrajectoryIterator::operator*() {
//    if (!item.has_value()) {
//        item = {{ trajectory.getPosition(time), trajectory.getVelocity(time) }};
//    }
//
//    return item.value();
//}
//BBTrajectoryIterator::pointer BBTrajectoryIterator::operator->() {
//    if (!item.has_value()) {
//        item = {{ trajectory.getPosition(time), trajectory.getVelocity(time) }};
//    }
//
//    return &item.value();
//}
//BBTrajectoryIterator& BBTrajectoryIterator::operator++() {
//    time += PositionControlUtils::TIME_STEP;
//    item = std::nullopt;
//    return *this;
//}
//
//bool BBTrajectoryIterator::operator==(const BBTrajectoryIterator& other) const {
//    return other.time == time;
//}
//
//bool BBTrajectoryIterator::operator!=(const BBTrajectoryIterator& other) const {
//    return !(*this == other);
//}
}