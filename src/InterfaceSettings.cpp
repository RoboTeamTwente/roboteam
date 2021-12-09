//
// Created by Dawid Kulikowski on 10/08/2021.
//

#include "InterfaceSettings.h"
#include "roboteam_utils/Print.h"

namespace rtt::Interface {

std::optional<InterfaceValue> InterfaceSettings::getSetting(const std::string name) const noexcept {
    std::scoped_lock lck(this->mtx);

    if (!this->values.contains(name)) {
        return std::nullopt;
    }

    return this->values.at(name);
}

void InterfaceSettings::setSetting(const std::string name, const InterfaceValue newValue) noexcept {
    std::scoped_lock lck(this->mtx);

    this->values.insert_or_assign(name, newValue);

    if (auto iptr = this->stateHandler.lock()) {
        iptr->stateDidChange();
    }
}

void InterfaceSettings::handleData(const proto::UiValues& uiValues, std::weak_ptr<InterfaceDeclarations> declsptr, InterfaceSettingsPrecedence precedence) noexcept {
    std::scoped_lock lck(this->mtx);

    auto decls = declsptr.lock();

    if (uiValues.ui_values().empty()) {
        return;
    }

    this->values.clear();

    for (const auto& entry : uiValues.ui_values()) {
        const auto declForVal = decls->getDeclaration(entry.first);

        if (!declForVal.has_value()) continue;
        if (precedence == InterfaceSettingsPrecedence::AI && declForVal->isMutable) continue; // Prevent interface from changing AI-only uiValues
        if (precedence == InterfaceSettingsPrecedence::IFACE && !declForVal->isMutable) continue; // Prevent AI from changing interface-only uiValues

        this->values.insert_or_assign(entry.first, entry.second);
    }

    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }

}
proto::UiValues InterfaceSettings::toProto() const noexcept {
    std::scoped_lock lck(this->mtx);

    proto::UiValues vals;

    for (const auto& [key, value] : this->values) {
        vals.mutable_ui_values()->insert({key, value.toProto()});
    }

    return vals;
}

[[maybe_unused]] void InterfaceSettings::removeSetting(const std::string name) noexcept {
    std::scoped_lock lck(this->mtx);

    this->values.erase(name);

    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }
}
}