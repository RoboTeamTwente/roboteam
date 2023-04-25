//
// Created by Martin Miksik on 25/04/2023.
//

#include "interface_api/Interface.h"

#include "utilities/Settings.h"

namespace rtt::ai::new_interface {

proto::MessageEnvelope Interface::drawingsBuffer;
proto::MessageEnvelope Interface::metricsBuffer;

void Interface::draw(std::basic_string<char> label, proto::Drawing::Color color, proto::Drawing::Method method, std::span<Vector2> points, int retainForTicks) {
    const double orientation = SETTINGS.isLeft() ? -1 : 1;
    const auto drawing = drawingsBuffer.mutable_drawingbuffer()->add_buffer();
    drawing->set_label(label);
    drawing->set_color(color);
    drawing->set_method(method);
    drawing->set_retainforticks(retainForTicks);
    for (auto& point : points) {
        auto protoPoint = drawing->add_points();
        protoPoint->set_x(orientation * point.x);
        protoPoint->set_y(orientation * point.y);
    }
}

void Interface::reportNumber(std::basic_string<char> label, double value, std::basic_string<char> unit) {
    auto metric = metricsBuffer.mutable_metricbuffer()->add_buffer();
    metric->set_label(label);
    metric->mutable_decimal()->set_value(value);
    metric->mutable_decimal()->set_unit(unit);
}

void Interface::consumeBuffers(std::function<void(proto::MessageEnvelope&, proto::MessageEnvelope&)> consumer) {
    consumer(drawingsBuffer, metricsBuffer);
    drawingsBuffer.mutable_drawingbuffer()->clear_buffer();
    metricsBuffer.mutable_metricbuffer()->clear_buffer();
}

}  // namespace rtt::ai::new_interface