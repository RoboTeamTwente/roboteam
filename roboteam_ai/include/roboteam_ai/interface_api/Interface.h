//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACE_H
#define RTT_INTERFACE_H

#include <span>

#include "proto/NewInterface.pb.h"
#include "roboteam_utils/Vector2.h"

//using namespace rtt::ai::interface;

namespace rtt::ai::new_interface {

class Interface {
   public:
    static void draw(std::basic_string<char> label, proto::Drawing::Color color, proto::Drawing::Method method, std::span<Vector2> points, int retainForTicks = 1);
    static void reportNumber(std::basic_string<char> label, double value, std::basic_string<char> unit = "");

    static void consumeBuffers(std::function<void(proto::MessageEnvelope&, proto::MessageEnvelope&)> consumer);

   private:
    static proto::MessageEnvelope drawingsBuffer;
    static proto::MessageEnvelope metricsBuffer;
};

}

#endif  // RTT_INTERFACE_H
