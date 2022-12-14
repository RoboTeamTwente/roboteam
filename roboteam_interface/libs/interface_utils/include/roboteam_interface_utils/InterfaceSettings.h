//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACESETTINGS_H
#define RTT_INTERFACESETTINGS_H
#include <utility>

#include "proto/UiOptions.pb.h"
#include "InterfaceDeclarations.h"
#include "InterfaceValue.h"

namespace rtt::Interface {

enum class InterfaceSettingsPrecedence { AI, IFACE };

class InterfaceSettings {
   private:
    std::map<std::string, InterfaceValue> values;

    mutable std::mutex mtx;

    std::atomic_bool did_change;

   public:
       InterfaceSettings() = default;
       ~InterfaceSettings() = default;

       // TODO: This may be possible, albeit unnecessary
       InterfaceSettings(InterfaceSettings&&) = delete;
       InterfaceSettings& operator=(InterfaceSettings&&) = delete;

       InterfaceSettings(const InterfaceSettings&) = delete;
       InterfaceSettings& operator=(const InterfaceSettings&) = delete;

    std::optional<InterfaceValue> getSetting(std::string name) const noexcept;
    std::vector<std::tuple<std::string, InterfaceValue>> getSettingsWithSuffix(const std::string suffix) const noexcept;
    void setSetting(std::string name, InterfaceValue newValue) noexcept;
    [[maybe_unused]] void removeSetting(std::string name) noexcept;

    void handleData(const proto::UiValues&, std::weak_ptr<InterfaceDeclarations>, InterfaceSettingsPrecedence = InterfaceSettingsPrecedence::AI) noexcept;

    proto::UiValues toProto() const noexcept;

    void populateWithDefaults(std::weak_ptr<InterfaceDeclarations>) noexcept;

    bool getDidChange();
};
}  // namespace rbtt::Interface

#endif  // RTT_INTERFACESETTINGS_H
