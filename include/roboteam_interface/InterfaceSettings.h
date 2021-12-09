//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACESETTINGS_H
#define RTT_INTERFACESETTINGS_H
#include <utility>

#include "roboteam_proto/UiOptions.pb.h"
#include "InterfaceDeclarations.h"
#include "InterfaceValue.h"
#include "InterfaceStateHandler.h"

namespace rtt::Interface {
enum class InterfaceSettingsPrecedence { AI, IFACE };

class InterfaceSettings {
   private:
    std::map<std::string, InterfaceValue> values;

    mutable std::mutex mtx;

    std::weak_ptr<InterfaceStateHandler> stateHandler;


   public:
       InterfaceSettings() = delete;
       ~InterfaceSettings() = default;

       InterfaceSettings(InterfaceSettings&&) = delete;
       InterfaceSettings& operator=(InterfaceSettings&&) = delete;

       InterfaceSettings(const InterfaceSettings&) = delete;
       InterfaceSettings& operator=(const InterfaceSettings&) = delete;

    explicit InterfaceSettings(std::weak_ptr<InterfaceStateHandler> sts): stateHandler(std::move(sts)) {};

    std::optional<InterfaceValue> getSetting(std::string name) const noexcept;
    void setSetting(std::string name, InterfaceValue newValue) noexcept;
    [[maybe_unused]] void removeSetting(std::string name) noexcept;

    void handleData(const proto::UiValues&, std::weak_ptr<InterfaceDeclarations>, InterfaceSettingsPrecedence = InterfaceSettingsPrecedence::AI) noexcept;

    proto::UiValues toProto() const noexcept;
};
}  // namespace rbtt::Interface

#endif  // RTT_INTERFACESETTINGS_H
