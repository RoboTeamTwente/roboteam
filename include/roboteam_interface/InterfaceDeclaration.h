//
// Created by Dawid Kulikowski on 10/08/2021.
//

#ifndef RTT_INTERFACEDECLARATION_H
#define RTT_INTERFACEDECLARATION_H
#include <variant>
#include "InterfaceValue.h"
#include <roboteam_proto/UiOptions.pb.h>
#include <nlohmann/json.hpp>

namespace rtt::Interface {
struct InterfaceSlider {
    std::string text;
    float min;
    float max;
    float interval;
    int dpi;

    InterfaceSlider() = default;
    InterfaceSlider(const proto::Slider&);
    InterfaceSlider(const std::string, const float, const float, const float, const int);

    proto::Slider toProto() const {
        proto::Slider slider;

        slider.set_text(this->text);
        slider.set_min(this->min);
        slider.set_max(this->max);
        slider.set_interval(this->interval);
        slider.set_dpi(this->dpi);

        return slider;
    }

    bool operator==(const InterfaceSlider& lhs) const {
        return lhs.text == this->text &&
        lhs.min == this->min &&
        lhs.max == this->max &&
        lhs.interval == this->interval &&
            lhs.dpi == this->dpi;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceSlider, text, min, max, interval, dpi);
};

struct InterfaceCheckbox {
    std::string text;

    InterfaceCheckbox() = default;
    InterfaceCheckbox(const proto::Checkbox&);
    InterfaceCheckbox(const std::string);

    proto::Checkbox toProto() const {
        proto::Checkbox checkbox;

        checkbox.set_text(this->text);

        return checkbox;
    }

    bool operator==(const InterfaceCheckbox& lhs) const {
        return lhs.text == this->text;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceCheckbox, text);
};

struct InterfaceDropdown {
    std::string text;
    std::vector<std::string> options;

    InterfaceDropdown() = default;
    InterfaceDropdown(const proto::Dropdown&);
    InterfaceDropdown(const std::string, const std::vector<std::string>);

    proto::Dropdown toProto() const {
        proto::Dropdown dropdown;

        dropdown.set_text(this->text);
        dropdown.mutable_options()->Add(options.begin(), options.end());

        return dropdown;
    }

    bool operator==(const InterfaceDropdown& lhs) const {
        return lhs.text == this->text &&
        lhs.options == this->options;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceDropdown, text, options);

};

struct InterfaceRadio {
    std::vector<std::string> options;

    InterfaceRadio() = default;
    InterfaceRadio(const proto::RadioButton&);
    InterfaceRadio(const std::vector<std::string>);

    proto::RadioButton toProto() const {
        proto::RadioButton radio;

        radio.mutable_options()->Add(options.begin(), options.end());
        return radio;
    }

    bool operator==(const InterfaceRadio& lhs) const {
        return lhs.options == this->options;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceRadio, options);
};

struct InterfaceText {
    std::string text;

    InterfaceText() = default;
    InterfaceText(const proto::TextField&);
    InterfaceText(const std::string);

    proto::TextField toProto() const {
        proto:: TextField txtField;

        txtField.set_text(text);

        return txtField;
    }

    bool operator==(const InterfaceText& lhs) const {
        return lhs.text == this->text;
    }

    NLOHMANN_DEFINE_TYPE_INTRUSIVE(InterfaceText, text);
};

typedef std::variant<std::monostate, InterfaceSlider, InterfaceCheckbox, InterfaceDropdown, InterfaceRadio, InterfaceText> InterfaceOptions;

struct InterfaceDeclaration {
   public:
    std::string path;
    std::string description;

    bool isMutable;

    InterfaceValue defaultValue;

    InterfaceOptions options;

    proto::UiOptionDeclaration toProto() const;


    InterfaceDeclaration() = default;
    InterfaceDeclaration(const proto::UiOptionDeclaration&);
    InterfaceDeclaration(const std::string, const std::string, const bool, const InterfaceValue, const InterfaceOptions);

    bool operator==(const InterfaceDeclaration& lhs) const {
        return lhs.path == this->path &&
        lhs.description == this->description &&
        lhs.isMutable == this->isMutable &&
        lhs.options == this->options;
    }
};

void from_json(const nlohmann::json& json, InterfaceDeclaration& declaration);
void to_json(nlohmann::json& j, const InterfaceDeclaration& declaration);

}  // namespace rbtt::Interface

#endif  // RTT_INTERFACEDECLARATION_H
