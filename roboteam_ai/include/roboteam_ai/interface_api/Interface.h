//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACE_H
#define RTT_INTERFACE_H

#include <span>

#include "proto/NewInterface.pb.h"
#include "roboteam_utils/Vector2.h"

namespace rtt::ai::new_interface {

class Interface {
   public:
    struct DrawArgs {
        std::basic_string<char> label;
        proto::Drawing::Color color = proto::Drawing::WHITE;
        proto::Drawing::Method method = proto::Drawing::DOTS;
        proto::Drawing::Category category = proto::Drawing::DEBUG;
        int forRobotId = -1;
        int retainForTicks = 10;
    };

    static void draw(const DrawArgs& args, std::span<Vector2> points);
    static void reportNumber(std::basic_string<char> label, double value, std::basic_string<char> unit = "");

    static void consumeVisualizations(std::function<void(const proto::MsgToInterface::VisualizationBuffer& visualizations)> consumer);

   private:
    static proto::Drawing* initDrawing(const DrawArgs& args);
    static proto::MsgToInterface::VisualizationBuffer* visualizations;
    static google::protobuf::Arena arena;
};

}

#endif  // RTT_INTERFACE_H
