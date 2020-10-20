//
// Created by rolf on 19-10-20.
//

#include "Observer.h"

//TODO: actually implement
proto::State Observer::getState(Time time) const {
    return proto::State();
}

void
Observer::process(std::vector<proto::SSL_WrapperPacket> visionPackets, std::vector<proto::SSL_Referee> refereePackets) {

}
