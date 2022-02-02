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

// An enum that represents on which channel a basestation emits messages with robots
enum class WirelessChannel : unsigned int { YELLOW_CHANNEL = 0, BLUE_CHANNEL = 1, UNKNOWN = 2 };

/* This class will take any collection of basestations, and will pick two basestations that
   can send messages to the blue and the yellow robots. It does this by asking the basestations
   what channel they currently use, and if necessary, request them to change it. */
class BasestationCollection {
   public:
    BasestationCollection();
    ~BasestationCollection();

    // This function makes sure the collection is up to date. Call this function frequently
    void updateBasestationCollection(const std::vector<libusb_device*>& pluggedBasestationDevices);

    // Sends a message to the basestation of the given team. Returns success
    bool sendMessageToBasestation(BasestationMessage& message, utils::TeamColor teamColor);

    // Set a callback function to receive all messages from the two basestations of the teams
    void setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback);

    // Prints the current status of the collection directly to the console
    void printCollection() const;

   private:
    // Collection and selection of basestations
    std::vector<std::shared_ptr<Basestation>> basestations; // All basestations
    std::shared_ptr<Basestation> yellowBasestation; // Basestation selected for yellow team
    std::shared_ptr<Basestation> blueBasestation; // Basestation selected for blue team
    // TODO: Make basestations and yellow and blue basestation thread safe

    // Keeps track of which basestation has which wireless channel
    std::map<uint8_t, WirelessChannel> basestationIdToWirelessChannel;
    WirelessChannel getWirelessChannelOfBasestation(uint8_t serialId) const;

    // Updating the basestation selection
    bool shouldUpdateBasestationSelection;
    std::thread basestationSelectionUpdaterThread;
    void updateBasestationSelection();

    void askChannelOfBasestationsWithUnknownChannel();
    // Gets list of basestations that could be selected as the blue or yellow basestation
    std::vector<std::shared_ptr<Basestation>> getSelectableBasestations() const;
    // Sends a channel change request to the given basestation to change to the given channel
    bool sendChannelChangeRequest(std::shared_ptr<Basestation> basestation, WirelessChannel newChannel);

    // Will try to select for the given colors, returns the amount of basestations it selected
    int selectBasestations(bool selectYellowBasestation, bool selectBlueBasestation);
    bool unselectIncorrectlySelectedBasestations(); // Returns whether it unselected basestations

    std::mutex messageCallbackMutex; // Guards the messageFromBasestationCallback
    void onMessageFromBasestation(const BasestationMessage& message, uint8_t serialID);
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