#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <basestation/Basestation.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <map>

namespace rtt::robothub::basestation {

enum class WirelessChannel : unsigned int { YELLOW_CHANNEL = 0, BLUE_CHANNEL = 1, UNKNOWN = 2 };

class BasestationCollection {
   public:
    BasestationCollection();
    ~BasestationCollection();

    void updateBasestationList(const std::vector<libusb_device*>& pluggedBasestationDevices);

    bool sendMessageToBasestation(BasestationMessage& message, utils::TeamColor teamColor);

    void setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback);

    void printCollection() const;

   private:
    // Collection and selection of basestations
    std::vector<std::shared_ptr<Basestation>> basestations;
    std::shared_ptr<Basestation> blueBasestation;
    std::shared_ptr<Basestation> yellowBasestation;

    // Keeps track of which basestation has which wireless channel
    std::map<uint8_t, WirelessChannel> basestationIdToWirelessChannel;
    WirelessChannel getWirelessChannelOfBasestation(uint8_t serialId) const;

    // Updating the basestation selection
    void askChannelOfBasestationsWithUnknownChannel();
    bool shouldUpdateBasestationSelection;
    std::thread basestationSelectionUpdaterThread;
    void updateBasestationSelection();
    std::vector<std::shared_ptr<Basestation>> getUnselectedBasestations() const;

    bool trySelectBasestationOfColor(utils::TeamColor color);
    void selectBasestationOfColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color);
    void unselectBasestationOfColor(utils::TeamColor color);

    std::mutex messageCallbackMutex;  // Guards the messageFromBasestationCallback
    void onMessageFromBasestation(const BasestationMessage& message, uint8_t serialID);
    
    void onMessageFromBasestation(const BasestationMessage& message, utils::TeamColor color);
    std::function<void(const BasestationMessage&, utils::TeamColor color)> messageFromBasestationCallback;

    static WirelessChannel getWirelessChannelCorrespondingTeamColor(utils::TeamColor color);
    static utils::TeamColor getTeamColorCorrespondingWirelessChannel(WirelessChannel channel);
    static bool basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices);
    static bool deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations);

    // Converts a WirelessChannel to a value that can be used in REM
    static bool wirelessChannelToREMChannel(WirelessChannel channel);
    // Converts a wireless channel value from REM to a WirelessChannel
    static WirelessChannel remChannelToWirelessChannel(bool remChannel);
    static std::string wirelessChannelToString(WirelessChannel channel);
};

}  // namespace rtt::robothub::basestation