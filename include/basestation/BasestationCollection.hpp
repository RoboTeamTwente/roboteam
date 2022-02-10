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

// Represents which combination of basestations is wanted
enum class WantedBasestations { ONLY_YELLOW, ONLY_BLUE, YELLOW_AND_BLUE, NEITHER_YELLOW_NOR_BLUE };

typedef struct BasestationCollectionStatus {
   WantedBasestations wantedBasestations;
   bool hasYellowBasestation;
   bool hasBlueBasestation;
   int amountOfBasestations;
} BasestationCollectionStatus;

/* This class will take any collection of basestations, and will pick two basestations that
   can send messages to the blue and the yellow robots. It does this by asking the basestations
   what channel they currently use, and if necessary, request them to change it. */
class BasestationCollection {
   public:
    BasestationCollection();
    ~BasestationCollection();

    // This function makes sure the collection is up to date. Call this function frequently
    void updateBasestationCollection(const std::vector<libusb_device*>& pluggedBasestationDevices);

    // Sends a message to the basestation of the given team. Returns bytes sent, -1 if error
    int sendMessageToBasestation(BasestationMessage& message, utils::TeamColor teamColor);

    // Set a callback function to receive all messages from the two basestations of the teams
    void setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback);

    // Prints the current status of the collection directly to the console
    const BasestationCollectionStatus getStatus() const;

   private:
    // Collection and selection of basestations
    mutable std::mutex basestationsMutex; // Guards the basestations vector
    std::vector<std::shared_ptr<Basestation>> basestations; // All basestations
    std::vector<std::shared_ptr<Basestation>> getAllBasestations() const;

    mutable std::mutex basestationSelectionMutex; // Guards both selected basestations
    std::shared_ptr<Basestation> yellowBasestation; // Basestation selected for yellow team
    std::shared_ptr<Basestation> blueBasestation; // Basestation selected for blue team

    std::shared_ptr<Basestation> getSelectedBasestation(utils::TeamColor colorOfBasestation) const;
    void setSelectedBasestation(std::shared_ptr<Basestation> newBasestation, utils::TeamColor color);

    // Keeps track of which basestation has which wireless channel
    std::map<const BasestationIdentifier, WirelessChannel> basestationIdToChannel;
    mutable std::mutex basestationIdToChannelMutex; // Guards the basestationIdToChannel map
    
    WirelessChannel getChannelOfBasestation(const BasestationIdentifier& basestationId) const;
    void setChannelOfBasestation(const BasestationIdentifier& basestationId, WirelessChannel newChannel);
    void removeBasestationIdToChannelEntry(const BasestationIdentifier& basestationId);

    // Keeps track of which basestations are actually used
    std::chrono::time_point<std::chrono::steady_clock> lastRequestForYellowBasestation;
    std::chrono::time_point<std::chrono::steady_clock> lastRequestForBlueBasestation;
    // This will update the lastRequestFor variables to the current time
    void updateWantedBasestations(utils::TeamColor lastRequestedBasestation);
    // Will return which basestations are being used, or requested if not selected yet
    WantedBasestations getWantedBasestations() const;

    // Updating the basestation selection
    bool shouldUpdateBasestationSelection;
    std::thread basestationSelectionUpdaterThread;
    void updateBasestationSelection();
    void removeOldBasestations(const std::vector<libusb_device*>& pluggedBasestationDevices);
    void addNewBasestations(const std::vector<libusb_device*>& pluggedBasestationDevices);

    void askChannelOfBasestationsWithUnknownChannel() const;
    // Gets list of basestations that could be selected as the blue or yellow basestation
    std::vector<std::shared_ptr<Basestation>> getSelectableBasestations() const;
    // Sends a channel change request to the given basestation to change to the given channel
    bool sendChannelChangeRequest(const std::shared_ptr<Basestation>& basestation, WirelessChannel newChannel);

    int unselectUnwantedBasestations(); // Unselect basestations that are not used
    int unselectIncorrectlySelectedBasestations(); // Unselect basestations that have the wrong channel
    int selectWantedBasestations(); // Select basestations that we need
    int selectBasestations(bool selectYellowBasestation, bool selectBlueBasestation); // Will try to select the given basestations

    std::mutex messageCallbackMutex; // Guards the messageFromBasestationCallback
    void onMessageFromBasestation(const BasestationMessage& message, const BasestationIdentifier& basestationId);
    std::function<void(const BasestationMessage&, utils::TeamColor color)> messageFromBasestationCallback;

    static WirelessChannel getWirelessChannelCorrespondingTeamColor(utils::TeamColor color);
    static utils::TeamColor getTeamColorCorrespondingWirelessChannel(WirelessChannel channel);
    static bool basestationIsInDeviceList(const std::shared_ptr<Basestation>& basestation, const std::vector<libusb_device*>& devices);
    static bool deviceIsInBasestationList(libusb_device* const device, const std::vector<std::shared_ptr<Basestation>>& basestations);

    // Converts a WirelessChannel to a value that can be used in REM
    static bool wirelessChannelToREMChannel(WirelessChannel channel);
    // Converts a wireless channel value from REM to a WirelessChannel
    static WirelessChannel remChannelToWirelessChannel(bool remChannel);
    static std::string wirelessChannelToString(WirelessChannel channel);
};

}  // namespace rtt::robothub::basestation