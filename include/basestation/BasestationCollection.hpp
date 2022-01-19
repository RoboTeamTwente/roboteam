#pragma once

#include <libusb-1.0/libusb.h>
#include <utilities.h>

#include <basestation/Basestation.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace rtt::robothub::basestation {

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

    // Updating the basestation selection
    bool shouldUpdateBasestationSelection;
    std::thread basestationSelectionUpdaterThread;
    void updateBasestationSelection();
    std::vector<std::shared_ptr<Basestation>> getUnselectedBasestations() const;

    bool trySelectBasestationOfColor(utils::TeamColor color);
    void selectBasestationOfColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color);
    void unselectBasestationOfColor(utils::TeamColor color);

    std::mutex messageCallbackMutex;  // Guards the messageFromBasestationCallback
    void onMessageFromBasestation(const BasestationMessage& message, utils::TeamColor color);
    std::function<void(const BasestationMessage&, utils::TeamColor color)> messageFromBasestationCallback;

    static WirelessChannel getWirelessChannelCorrespondingTeamColor(utils::TeamColor color);
    static bool basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices);
    static bool deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations);
};

}  // namespace rtt::robothub::basestation