#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <basestation/Basestation.hpp>
#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

namespace rtt::robothub::basestation {

enum class TeamColorUsage { ONLY_YELLOW, ONLY_BLUE, YELLOW_AND_BLUE, NEITHER_YELLOW_NOR_BLUE };

class BasestationCollection {
   public:
    BasestationCollection();
    ~BasestationCollection();

    void updateBasestationList(const std::vector<libusb_device*>& pluggedBasestationDevices);

    bool sendMessageToBasestation(const BasestationMessage& message, utils::TeamColor teamColor) const;

    void setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback);

    void printCollection() const;

   private:
    std::vector<std::shared_ptr<Basestation>> basestations;
    std::shared_ptr<Basestation> blueBasestation;
    std::shared_ptr<Basestation> yellowBasestation;

    std::chrono::time_point<std::chrono::steady_clock> lastUseOfYellowBasestation;
    std::chrono::time_point<std::chrono::steady_clock> lastUseOfBlueBasestation;
    TeamColorUsage currentTeamColorUsage;
    TeamColorUsage getTeamColorUsage();
    void updateTeamColorUsage(utils::TeamColor usedBasestationColor);

    bool shouldUpdateBasestationSelection;
    std::thread basestationSelectionUpdaterThread;
    void updateBasestationSelection();
    std::vector<std::shared_ptr<Basestation>> getUnselectedBasestations() const;
    bool trySelectBasestationOfColor(utils::TeamColor color);
    void selectBasestationAtColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color);
    void unselectBasestationAtColor(utils::TeamColor color);

    std::mutex messageCallbackMutex;  // Guards the messageFromBasestationCallback
    void onMessageFromBasestation(const BasestationMessage& message, utils::TeamColor color);
    std::function<void(const BasestationMessage&, utils::TeamColor color)> messageFromBasestationCallback;

    static bool wirelessChannelMatchesTeamColor(WirelessChannel channel, utils::TeamColor color);
    static WirelessChannel getWirelessChannelCorrespondingTeamColor(utils::TeamColor color);
    static bool basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices);
    static bool deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations);
};

}  // namespace rtt::robothub::basestation