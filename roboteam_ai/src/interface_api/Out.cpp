//
// Created by Martin Miksik on 25/04/2023.
//

#include "interface_api/Out.h"
#include "proto/NewInterface.pb.h"
#include "utilities/GameSettings.h"

namespace rtt::ai::new_interface {

google::protobuf::Arena Out::arena;
proto::MsgToInterface::VisualizationBuffer* Out::visualizations = google::protobuf::Arena::CreateMessage<proto::MsgToInterface::VisualizationBuffer>(&arena);

void Out::draw(const DrawArgs& args, std::span<Vector2> points) {
    const double orientation = GameSettings::isLeft() ? -1 : 1;
    auto drawing = initDrawing(args);
    for (auto& point : points) {
        auto protoPoint = drawing->add_points();
        protoPoint->set_x(orientation * point.x);
        protoPoint->set_y(orientation * point.y);
    }
}

void Out::decimal(std::basic_string<char> label, double value, std::basic_string<char> unit) {
    auto metric = visualizations->add_metrics();
    metric->set_label(label);
    metric->mutable_decimal()->set_value(value);
    metric->mutable_decimal()->set_unit(unit);
}

void Out::bounded(std::basic_string<char> label, double value, double min, double max, std::basic_string<char> unit) {
    auto metric = visualizations->add_metrics();
    metric->set_label(label);
    metric->mutable_bounded_value()->set_value(value);
    metric->mutable_bounded_value()->set_min(min);
    metric->mutable_bounded_value()->set_max(max);
    metric->mutable_bounded_value()->set_unit(unit);
}

proto::Drawing* Out::initDrawing(const DrawArgs& args) {
    const auto drawing = visualizations->add_drawings();
    drawing->set_label(args.label);
    drawing->set_color(args.color);
    drawing->set_method(args.method);
    drawing->set_retain_for_ticks(args.retainForTicks);
    drawing->set_category(args.category);
    drawing->set_for_robot_id(args.forRobotId);
    drawing->set_size(args.size);
    drawing->set_thickness(args.thickness);
    return drawing;
}

void Out::consumeVisualizations(std::function<void(const proto::MsgToInterface::VisualizationBuffer&)> consumer){
    consumer(*visualizations);
    visualizations->mutable_drawings()->Clear();
    visualizations->mutable_metrics()->Clear();
}

}  // namespace rtt::ai::new_interface