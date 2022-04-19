//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "InterfaceDeclaration.h"
#include "InterfaceValue.h"
#include <roboteam_utils/Print.h>
#include <exception>

namespace rtt::Interface {

InterfaceDeclaration::InterfaceDeclaration(const proto::UiOptionDeclaration& decl) : defaultValue(InterfaceValue(decl.default_())) {
    this->path = decl.path();
    this->description = decl.description();
    this->isMutable = decl.is_mutable();

    switch (decl.ui_elements_case()) {
        case proto::UiOptionDeclaration::UiElementsCase::kCheckbox:
            this->options = decl.checkbox();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kDropdown:
            this->options = decl.dropdown();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kRadiobutton:
            this->options = decl.radiobutton();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kSlider:
            this->options = decl.slider();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::kTextfield:
            this->options = decl.textfield();
            break;
        case proto::UiOptionDeclaration::UiElementsCase::UI_ELEMENTS_NOT_SET:
        default:
            throw std::logic_error{"Unknown variant type when initialising InterfaceDeclaration!"};
    }
}

InterfaceDeclaration::InterfaceDeclaration(const std::string path, const std::string description, const bool isMutable, const InterfaceValue defaultValue,
                                           const InterfaceOptions options)
    : defaultValue(defaultValue) {
    this->path = path;
    this->description = description;
    this->isMutable = isMutable;
    this->options = options;
}
proto::UiOptionDeclaration InterfaceDeclaration::toProto() const {

    proto::UiOptionDeclaration protoDecl;

    protoDecl.set_path(this->path);
    protoDecl.set_description(this->description);
    protoDecl.set_is_mutable(this->isMutable);
    protoDecl.mutable_default_()->CopyFrom(this->defaultValue.toProto());

    if (const auto* slider = std::get_if<Interface::InterfaceSlider>(&this->options)) {
        protoDecl.mutable_slider()->CopyFrom(slider->toProto());
    } else if (const auto* checkbox = std::get_if<InterfaceCheckbox>(&this->options)) {
        protoDecl.mutable_checkbox()->CopyFrom(checkbox->toProto());
    } else if (const auto* dropdown = std::get_if<InterfaceDropdown>(&this->options)) {
        protoDecl.mutable_dropdown()->CopyFrom(dropdown->toProto());
    } else if (const auto* radio = std::get_if<InterfaceRadio>(&this->options)) {
        protoDecl.mutable_radiobutton()->CopyFrom(radio->toProto());
    } else if (const auto* text = std::get_if<InterfaceText>(&this->options)) {
        protoDecl.mutable_textfield()->CopyFrom(text->toProto());
    } else {
        throw std::logic_error{"Variant was in an invalid state when serialising InterfaceDeclaration to JSON!"};
    }

    return protoDecl;
}

InterfaceText::InterfaceText(const proto::TextField& protoTextField) { this->text = protoTextField.text(); }

InterfaceText::InterfaceText(const std::string text) { this->text = text; }

InterfaceRadio::InterfaceRadio(const proto::RadioButton& protoRadio) {
    // protobuf owns this message, we should copy it
    std::copy(protoRadio.options().begin(), protoRadio.options().end(), std::back_inserter(this->options));
}

InterfaceRadio::InterfaceRadio(const std::vector<std::string> options) { this->options = options; }

InterfaceDropdown::InterfaceDropdown(const proto::Dropdown& protoDropdown) {
    this->text = protoDropdown.text();

    std::copy(protoDropdown.options().begin(), protoDropdown.options().end(), std::back_inserter(this->options));
}

InterfaceDropdown::InterfaceDropdown(const std::string text, const std::vector<std::string> options) {
    this->text = text;
    this->options = options;
}

InterfaceCheckbox::InterfaceCheckbox(const proto::Checkbox& protoCheckbox) { this->text = protoCheckbox.text(); }

InterfaceCheckbox::InterfaceCheckbox(const std::string text) { this->text = text; }

Interface::InterfaceSlider::InterfaceSlider(const proto::Slider& protoSlider) {
    this->text = protoSlider.text();
    this->min = protoSlider.min();
    this->max = protoSlider.max();
    this->interval = protoSlider.interval();
    this->dpi = protoSlider.dpi();
}

Interface::InterfaceSlider::InterfaceSlider(const std::string text, const float min, const float max, const float interval, const int dpi) {
    this->text = text;
    this->min = min;
    this->max = max;
    this->interval = interval;
    this->dpi = dpi;
}
}
