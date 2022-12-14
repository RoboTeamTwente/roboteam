//
// Created by Dawid Kulikowski on 08/08/2021.
//
#include "InterfaceValue.h"
#include "roboteam_utils/Print.h"

namespace rtt::Interface {

InterfaceValue::InterfaceValue(const proto::UiValue& val) {
    switch (val.value_case()) {
        case proto::UiValue::ValueCase::kBoolValue:
            this->variant = val.bool_value();
            break;
        case proto::UiValue::ValueCase::kFloatValue:
            this->variant = val.float_value();
            break;
        case proto::UiValue::ValueCase::kIntegerValue:
            this->variant = val.integer_value();
            break;
        case proto::UiValue::ValueCase::kTextValue:
            this->variant = val.text_value();
            break;
        case proto::UiValue::ValueCase::VALUE_NOT_SET:
            RTT_ERROR("[Interface] Interface value not set (corrupted proto message)");
            std::terminate();
            break;
        default:
            RTT_ERROR("[Interface] Interface value unknown (corrupted proto message)");
            std::terminate();
            break;
    }
}
proto::UiValue InterfaceValue::toProto() const {
    proto::UiValue value;

    if (auto* intVal = std::get_if<int64_t>(&this->variant)) {
        value.set_integer_value(*intVal);
    } else if (auto* boolVal = std::get_if<bool>(&this->variant)) {
        value.set_bool_value(*boolVal);
    } else if (auto* floatVal = std::get_if<float>(&this->variant)) {
        value.set_float_value(*floatVal);
    } else if (auto* stringVal = std::get_if<std::string>(&this->variant)) {
        value.set_text_value(*stringVal);
    } else {
        throw std::logic_error{"Invalid state of InterfaceValue during serialization!"};
    }

    return value;
}
}