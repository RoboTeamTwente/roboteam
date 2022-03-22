#include <REM_BasestationConfiguration.h>
#include <REM_BasestationGetConfiguration.h>
#include <REM_BasestationSetConfiguration.h>
#include <roboteam_utils/Print.h>

#include <basestation/BasestationCollection.hpp>
#include <cstring>

namespace rtt::robothub::basestation {

constexpr int TIME_UNTILL_BASESTATION_IS_UNWANTED_S = 1;        // 1 second with no interaction
constexpr int BASESTATION_SELECTION_UPDATE_FREQUENCY_MS = 420;  // Why 420? No reason at all...

BasestationCollection::BasestationCollection() {
    this->shouldUpdateBasestationSelection = true;
    this->basestationSelectionUpdaterThread = std::thread(&BasestationCollection::updateBasestationSelection, this);
}
BasestationCollection::~BasestationCollection() {
    this->shouldUpdateBasestationSelection = false;
    if (this->basestationSelectionUpdaterThread.joinable()) {
        this->basestationSelectionUpdaterThread.join();
    }
}

void BasestationCollection::updateBasestationCollection(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    this->removeOldBasestations(pluggedBasestationDevices);
    this->addNewBasestations(pluggedBasestationDevices);
}

void BasestationCollection::removeOldBasestations(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    auto iterator = this->basestations.begin();
    while (iterator != this->basestations.end()) {
        auto basestation = *iterator;

        if (!basestationIsInDeviceList(basestation, pluggedBasestationDevices)) {
            // This basestation is not plugged in anymore -> remove it

            // Remove basestation from the selection if it was in there
            auto selectedYellow = this->getSelectedBasestation(rtt::Team::YELLOW);
            if (basestation->operator==(selectedYellow)) this->setSelectedBasestation(nullptr, rtt::Team::YELLOW);

            auto selectedBlue = this->getSelectedBasestation(rtt::Team::BLUE);
            if (basestation->operator==(selectedBlue)) this->setSelectedBasestation(nullptr, rtt::Team::BLUE);

            // Remove its serial id from wireless channel map
            this->removeBasestationIdToChannelEntry(basestation->getIdentifier());

            // Fist, obtain the basestations mutex, because we will edit the vector, which might be read by other threads
            std::scoped_lock<std::mutex> lock(this->basestationsMutex);
            // Remove basestation from list
            iterator = this->basestations.erase(iterator);
        } else {
            ++iterator;
        }
    }
}

void BasestationCollection::addNewBasestations(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    auto callbackForNewBasestations = [&](const BasestationMessage& message, BasestationIdentifier basestationId) { this->onMessageFromBasestation(message, basestationId); };

    // Add plugged in basestations that are not in the list yet
    for (libusb_device* const pluggedBasestationDevices : pluggedBasestationDevices) {
        if (!deviceIsInBasestationList(pluggedBasestationDevices, this->basestations)) {
            // This basestation is plugged in but not in the list -> add it
            try {
                auto newBasestation = std::make_shared<Basestation>(pluggedBasestationDevices);
                newBasestation->setIncomingMessageCallback(callbackForNewBasestations);

                // Lock the basestations list so we can safely add this basestation
                std::scoped_lock<std::mutex> lock(this->basestationsMutex);
                this->basestations.push_back(newBasestation);
            } catch (FailedToOpenDeviceException e) {
                RTT_ERROR(e.what())
                RTT_INFO("Did you edit your PC's user permissions?")
            }
        }
    }
}

int BasestationCollection::sendMessageToBasestation(BasestationMessage& message, rtt::Team teamColor) {
    const auto& basestation = teamColor == rtt::Team::YELLOW ? this->getSelectedBasestation(rtt::Team::YELLOW) : this->getSelectedBasestation(rtt::Team::BLUE);

    int bytesSent = -1;
    if (basestation != nullptr) {
        bytesSent = basestation->sendMessageToBasestation(message);
    }

    // Update our basestations usage
    this->updateWantedBasestations(teamColor);

    return bytesSent;
}

void BasestationCollection::setIncomingMessageCallback(std::function<void(const BasestationMessage&, rtt::Team)> callback) {
    this->messageFromBasestationCallback = callback;
}

const BasestationCollectionStatus BasestationCollection::getStatus() const {
    const BasestationCollectionStatus status{.wantedBasestations = this->getWantedBasestations(),
                                             .hasYellowBasestation = this->getSelectedBasestation(rtt::Team::YELLOW) != nullptr,
                                             .hasBlueBasestation = this->getSelectedBasestation(rtt::Team::BLUE) != nullptr,
                                             .amountOfBasestations = (int)basestations.size()};

    return status;
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getAllBasestations() const {
    // First, lock the original basestations list
    std::scoped_lock<std::mutex> lock(this->basestationsMutex);

    std::vector<std::shared_ptr<Basestation>> copyVector;
    for (const auto& basestation : this->basestations) {
        copyVector.push_back(basestation);
    }

    return copyVector;
}

std::shared_ptr<Basestation> BasestationCollection::getSelectedBasestation(rtt::Team colorOfBasestation) const {
    std::shared_ptr<Basestation> basestation;

    std::scoped_lock<std::mutex> lock(this->basestationSelectionMutex);

    switch (colorOfBasestation) {
        case rtt::Team::BLUE:
            basestation = this->blueBasestation;
            break;
        case rtt::Team::YELLOW:
            basestation = this->yellowBasestation;
            break;
    }

    return basestation;
}

void BasestationCollection::setSelectedBasestation(std::shared_ptr<Basestation> newBasestation, rtt::Team color) {
    std::scoped_lock<std::mutex> lock(this->basestationSelectionMutex);
    switch (color) {
        case rtt::Team::BLUE:
            this->blueBasestation = newBasestation;
            break;
        case rtt::Team::YELLOW:
            this->yellowBasestation = newBasestation;
            break;
    }
}

WirelessChannel BasestationCollection::getChannelOfBasestation(const BasestationIdentifier& basestationId) const {
    WirelessChannel channel = WirelessChannel::UNKNOWN;

    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);

    auto iterator = this->basestationIdToChannel.find(basestationId);
    if (iterator != this->basestationIdToChannel.end()) {
        // If the iterator found a key in the map, the channel is the second value
        channel = iterator->second;
    }

    return channel;
}

void BasestationCollection::setChannelOfBasestation(const BasestationIdentifier& basestationId, WirelessChannel newChannel) {
    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);
    this->basestationIdToChannel[basestationId] = newChannel;
}

void BasestationCollection::removeBasestationIdToChannelEntry(const BasestationIdentifier& basestationId) {
    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);
    this->basestationIdToChannel.erase(basestationId);
}

WantedBasestations BasestationCollection::getWantedBasestations() const {
    auto now = std::chrono::steady_clock::now();

    // Calculate how long ago the basestations were used
    auto timeAfterLastYellowUsage = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastRequestForYellowBasestation).count();
    auto timeAfterLastBlueUsage = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastRequestForBlueBasestation).count();

    // If they were used recently enough, we say we still want them
    bool wantsYellowBasestation = timeAfterLastYellowUsage <= TIME_UNTILL_BASESTATION_IS_UNWANTED_S;
    bool wantsBlueBasestation = timeAfterLastBlueUsage <= TIME_UNTILL_BASESTATION_IS_UNWANTED_S;

    WantedBasestations wantedBasestations;

    if (wantsYellowBasestation && wantsBlueBasestation) {
        wantedBasestations = WantedBasestations::YELLOW_AND_BLUE;
    } else if (wantsYellowBasestation) {
        wantedBasestations = WantedBasestations::ONLY_YELLOW;
    } else if (wantsBlueBasestation) {
        wantedBasestations = WantedBasestations::ONLY_BLUE;
    } else {
        wantedBasestations = WantedBasestations::NEITHER_YELLOW_NOR_BLUE;
    }

    return wantedBasestations;
}

void BasestationCollection::updateWantedBasestations(rtt::Team requestedBasestationColor) {
    auto now = std::chrono::steady_clock::now();

    switch (requestedBasestationColor) {
        case rtt::Team::YELLOW:
            this->lastRequestForYellowBasestation = now;
            break;
        case rtt::Team::BLUE:
            this->lastRequestForBlueBasestation = now;
            break;
    }
}

void BasestationCollection::updateBasestationSelection() {
    while (this->shouldUpdateBasestationSelection) {
        int wrongBasestations = this->unselectIncorrectlySelectedBasestations();
        if (wrongBasestations > 0) {
            RTT_WARNING("Selected basestation(s) had incorrect channel(s)")
        }

        this->askChannelOfBasestationsWithUnknownChannel();

        this->unselectUnwantedBasestations();

        this->selectWantedBasestations();

        std::this_thread::sleep_for(std::chrono::milliseconds(BASESTATION_SELECTION_UPDATE_FREQUENCY_MS));
    }
}

void BasestationCollection::askChannelOfBasestationsWithUnknownChannel() const {
    // Create the channel request message
    REM_BasestationGetConfiguration getConfigurationMessage;
    getConfigurationMessage.header = PACKET_TYPE_REM_BASESTATION_GET_CONFIGURATION;

    REM_BasestationGetConfigurationPayload getConfigurationPayload;
    encodeREM_BasestationGetConfiguration(&getConfigurationPayload, &getConfigurationMessage);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_REM_BASESTATION_GET_CONFIGURATION;
    std::memcpy(&message.payloadBuffer, &getConfigurationPayload.payload, message.payloadSize);

    // Send it to every basestation with unknown channel
    for (const auto& basestation : this->getAllBasestations()) {
        WirelessChannel channel = this->getChannelOfBasestation(basestation->getIdentifier());
        if (channel == WirelessChannel::UNKNOWN) {
            basestation->sendMessageToBasestation(message);
        }
    }
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getSelectableBasestations() const {
    const auto selectedYellowCopy = this->getSelectedBasestation(rtt::Team::YELLOW);
    const auto selectedBlueCopy = this->getSelectedBasestation(rtt::Team::BLUE);

    std::vector<std::shared_ptr<Basestation>> selectableBasestations;

    for (const auto& basestation : this->getAllBasestations()) {
        if (!basestation->operator==(selectedYellowCopy) && !basestation->operator==(selectedBlueCopy) &&
            this->getChannelOfBasestation(basestation->getIdentifier()) != WirelessChannel::UNKNOWN) {
            // This basestation is neither selected as blue nor yellow basestation, and has a known channel
            selectableBasestations.push_back(basestation);
        }
    }

    return std::move(selectableBasestations);
}

bool BasestationCollection::sendChannelChangeRequest(const std::shared_ptr<Basestation>& basestation, WirelessChannel newChannel) {
    REM_BasestationSetConfiguration setConfigurationCommand;
    setConfigurationCommand.header = PACKET_TYPE_REM_BASESTATION_SET_CONFIGURATION;
    setConfigurationCommand.remVersion = LOCAL_REM_VERSION;
    setConfigurationCommand.channel = BasestationCollection::wirelessChannelToREMChannel(newChannel);

    REM_BasestationSetConfigurationPayload setConfigurationPayload;
    encodeREM_BasestationSetConfiguration(&setConfigurationPayload, &setConfigurationCommand);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_REM_BASESTATION_SET_CONFIGURATION;
    std::memcpy(&message.payloadBuffer, &setConfigurationPayload.payload, message.payloadSize);

    bool sentSuccesfully = basestation->sendMessageToBasestation(message);

    // Set the wireless channel of this basestation to UNKNOWN, as at this point, its channel is uncertain
    this->setChannelOfBasestation(basestation->getIdentifier(), WirelessChannel::UNKNOWN);

    return sentSuccesfully;
}

int BasestationCollection::unselectUnwantedBasestations() {
    bool hasYellowBasestation = this->getSelectedBasestation(rtt::Team::YELLOW) != nullptr;
    bool hasBlueBasestation = this->getSelectedBasestation(rtt::Team::BLUE) != nullptr;

    int unselectedBasestations = 0;

    switch (this->getWantedBasestations()) {
        case WantedBasestations::NEITHER_YELLOW_NOR_BLUE: {
            if (hasYellowBasestation) {
                this->setSelectedBasestation(nullptr, rtt::Team::YELLOW);
                unselectedBasestations++;
            }
            if (hasBlueBasestation) {
                this->setSelectedBasestation(nullptr, rtt::Team::BLUE);
                unselectedBasestations++;
            }
            break;
        }
        case WantedBasestations::ONLY_YELLOW: {
            if (hasBlueBasestation) {
                this->setSelectedBasestation(nullptr, rtt::Team::BLUE);
                unselectedBasestations++;
            }
            break;
        }
        case WantedBasestations::ONLY_BLUE: {
            if (hasYellowBasestation) {
                this->setSelectedBasestation(nullptr, rtt::Team::YELLOW);
                unselectedBasestations++;
            }
            break;
        }
    }

    return unselectedBasestations;
}

int BasestationCollection::selectWantedBasestations() {
    bool needsToSelectYellowBasestation = false;
    bool needsToSelectBlueBasestation = false;

    switch (this->getWantedBasestations()) {
        case WantedBasestations::YELLOW_AND_BLUE:
            needsToSelectYellowBasestation = this->getSelectedBasestation(rtt::Team::YELLOW) == nullptr;
            needsToSelectBlueBasestation = this->getSelectedBasestation(rtt::Team::BLUE) == nullptr;
            break;
        case WantedBasestations::ONLY_YELLOW:
            needsToSelectYellowBasestation = this->getSelectedBasestation(rtt::Team::YELLOW) == nullptr;
            break;
        case WantedBasestations::ONLY_BLUE:
            needsToSelectBlueBasestation = this->getSelectedBasestation(rtt::Team::BLUE) == nullptr;
            break;
    }

    int selectedBasestations = this->selectBasestations(needsToSelectYellowBasestation, needsToSelectBlueBasestation);
    return selectedBasestations;
}

int BasestationCollection::selectBasestations(bool needYellowBasestation, bool needBlueBasestation) {
    // Fist get basestations that aren't already selected as the blue or yellow basestation
    auto selectableBasestations = this->getSelectableBasestations();

    // Then get a list of all unselected basestations in combination with their channel
    std::vector<std::shared_ptr<Basestation>> yellowBasestations;
    std::vector<std::shared_ptr<Basestation>> blueBasestations;

    for (const auto& basestation : selectableBasestations) {
        auto basestationsChannel = this->getChannelOfBasestation(basestation->getIdentifier());
        switch (basestationsChannel) {
            case WirelessChannel::YELLOW_CHANNEL: {
                yellowBasestations.push_back(basestation);
                break;
            }
            case WirelessChannel::BLUE_CHANNEL: {
                blueBasestations.push_back(basestation);
                break;
            }
        }
    }

    int numberOfSelectedBasestations = 0;
    bool hasSelectedYellowBasestation = false;
    bool hasSelectedBlueBasestation = false;

    const auto selectedYellowCopy = this->getSelectedBasestation(rtt::Team::YELLOW);
    // Now try to select a basestation which already has the channel we want
    if (needYellowBasestation && !yellowBasestations.empty()) {
        this->setSelectedBasestation(yellowBasestations.front(), rtt::Team::YELLOW);
        hasSelectedYellowBasestation = true;
        numberOfSelectedBasestations++;

        // Also remove this basestation from the unselected yellow basestations list
        yellowBasestations.erase(yellowBasestations.begin());
    }
    if (needBlueBasestation && !blueBasestations.empty()) {
        this->setSelectedBasestation(blueBasestations.front(), rtt::Team::BLUE);
        hasSelectedBlueBasestation = true;
        numberOfSelectedBasestations++;

        // Also remove this basestation from the unselected blue basestations list
        blueBasestations.erase(blueBasestations.begin());
    }

    // If we failed, we will request a basestation with different channel to change its channel
    if (needYellowBasestation && !hasSelectedYellowBasestation && !blueBasestations.empty()) {
        // Ask blue basestation to change to yellow
        this->sendChannelChangeRequest(blueBasestations.front(), WirelessChannel::YELLOW_CHANNEL);
    }
    if (needBlueBasestation && !hasSelectedBlueBasestation && !yellowBasestations.empty()) {
        // Ask yellow basestation to change to blue
        this->sendChannelChangeRequest(yellowBasestations.front(), WirelessChannel::BLUE_CHANNEL);
    }

    return numberOfSelectedBasestations;
}

int BasestationCollection::unselectIncorrectlySelectedBasestations() {
    int unselectedBasestations = 0;

    const auto selectedYellowCopy = this->getSelectedBasestation(rtt::Team::YELLOW);
    const auto selectedBlueCopy = this->getSelectedBasestation(rtt::Team::BLUE);

    if (selectedYellowCopy != nullptr && this->getChannelOfBasestation(selectedYellowCopy->getIdentifier()) != WirelessChannel::YELLOW_CHANNEL) {
        // The yellow basestation is not actually yellow, so unselect it
        this->setSelectedBasestation(nullptr, rtt::Team::YELLOW);
        unselectedBasestations++;
    }
    if (selectedBlueCopy != nullptr && this->getChannelOfBasestation(selectedBlueCopy->getIdentifier()) != WirelessChannel::BLUE_CHANNEL) {
        // The blue basestation is not actually blue, so unselect it
        this->setSelectedBasestation(nullptr, rtt::Team::BLUE);
        unselectedBasestations++;
    }
    return unselectedBasestations;
}

void BasestationCollection::onMessageFromBasestation(const BasestationMessage& message, const BasestationIdentifier& basestationId) {
    // If this message contains what channel the basestation has, parse it and update our map
    if (message.payloadBuffer[0] == PACKET_TYPE_REM_BASESTATION_CONFIGURATION) {
        REM_BasestationConfigurationPayload configurationPayload;
        std::memcpy(&configurationPayload.payload, message.payloadBuffer, PACKET_SIZE_REM_BASESTATION_CONFIGURATION);

        REM_BasestationConfiguration basestationConfiguration;
        decodeREM_BasestationConfiguration(&basestationConfiguration, &configurationPayload);

        WirelessChannel usedChannel = BasestationCollection::remChannelToWirelessChannel(basestationConfiguration.channel);
        this->setChannelOfBasestation(basestationId, usedChannel);
    }

    // This function can be called by multiple basestations simultaneously, so protect it with a mutex
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    // And forward the message if this serialID belongs to a selected basestation
    if (this->messageFromBasestationCallback != nullptr) {
        const auto selectedYellowCopy = this->getSelectedBasestation(rtt::Team::YELLOW);
        const auto selectedBlueCopy = this->getSelectedBasestation(rtt::Team::BLUE);

        if (selectedYellowCopy != nullptr && selectedYellowCopy->operator==(basestationId)) {
            this->messageFromBasestationCallback(message, rtt::Team::YELLOW);
        } else if (selectedBlueCopy != nullptr && selectedBlueCopy->operator==(basestationId)) {
            this->messageFromBasestationCallback(message, rtt::Team::BLUE);
        }
    }
}

WirelessChannel BasestationCollection::getWirelessChannelCorrespondingTeamColor(rtt::Team color) {
    WirelessChannel matchingWirelessChannel;

    switch (color) {
        case rtt::Team::BLUE:
            matchingWirelessChannel = WirelessChannel::BLUE_CHANNEL;
            break;
        case rtt::Team::YELLOW:
            matchingWirelessChannel = WirelessChannel::YELLOW_CHANNEL;
            break;
        default:
            matchingWirelessChannel = WirelessChannel::UNKNOWN;
            break;
    }

    return matchingWirelessChannel;
}

rtt::Team BasestationCollection::getTeamColorCorrespondingWirelessChannel(WirelessChannel channel) {
    switch (channel) {
        case WirelessChannel::YELLOW_CHANNEL:
            return rtt::Team::YELLOW;
        case WirelessChannel::BLUE_CHANNEL:
            return rtt::Team::BLUE;
        default:
            return rtt::Team::YELLOW;
    }
}

bool BasestationCollection::basestationIsInDeviceList(const std::shared_ptr<Basestation>& basestation, const std::vector<libusb_device*>& devices) {
    bool basestationIsInList = false;

    for (const auto& device : devices) {
        if (*basestation == device) {
            basestationIsInList = true;
            break;
        }
    }

    return basestationIsInList;
}

bool BasestationCollection::deviceIsInBasestationList(libusb_device* const device, const std::vector<std::shared_ptr<Basestation>>& basestations) {
    bool deviceIsInList = false;

    for (const auto& basestation : basestations) {
        if (*basestation == device) {
            deviceIsInList = true;
            break;
        }
    }

    return deviceIsInList;
}

bool BasestationCollection::wirelessChannelToREMChannel(WirelessChannel channel) {
    bool remChannel;
    switch (channel) {
        case WirelessChannel::BLUE_CHANNEL:
            remChannel = true;
            break;
        case WirelessChannel::YELLOW_CHANNEL:
            remChannel = false;
            break;
        default:
            remChannel = false;
            break;
    }

    return remChannel;
}

WirelessChannel BasestationCollection::remChannelToWirelessChannel(bool channel) { return channel ? WirelessChannel::BLUE_CHANNEL : WirelessChannel::YELLOW_CHANNEL; }

std::string BasestationCollection::wirelessChannelToString(WirelessChannel channel) {
    switch (channel) {
        case WirelessChannel::BLUE_CHANNEL:
            return "blue channel";
        case WirelessChannel::YELLOW_CHANNEL:
            return "yellow channel";
        default:
            return "unknown channel";
    }
}

}  // namespace rtt::robothub::basestation
