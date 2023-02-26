//
// Created by Martin Miksik on 19/02/2023.
//

#ifndef RTT_BBTRAJECTORYITERATOR_H
#define RTT_BBTRAJECTORYITERATOR_H

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "control/positionControl/StateVector.h"

namespace rtt::ai::control {

//class BBTrajectoryIterator {
//   public:
//    BBTrajectoryIterator(const BBTrajectory2D& trajectory, double time = 0);
//
//    using iteratorCategory = std::input_iterator_tag;
//    using valueType = std::pair<int, StateVector>;
//    using differenceType = std::ptrdiff_t;
//    using pointer = StateVector*;
//    using reference = StateVector&;
//
//    reference operator*();
//
//    pointer operator->();
//
//    BBTrajectoryIterator& operator++();
//    BBTrajectoryIterator operator++(int);
//    bool operator==(const BBTrajectoryIterator& other) const;
//    bool operator!=(const BBTrajectoryIterator& other) const;
//
//   private:
//    const BBTrajectory2D& trajectory;
//    std::optional<valueType> item = std::nullopt;
//    double time;
//};

}
#endif
