#include <BasestationConfiguration.h>     // REM packet
#include <BasestationSetConfiguration.h>  // REM packet
#include <BasestationGetConfiguration.h>  // REM packet

#include <basestation/BasestationCollection.hpp>
#include <iostream>
#include <cstring>

namespace rtt::robothub::basestation {

constexpr int TIME_UNTILL_BASESTATION_IS_UNSELECTED_S = 3;  // 3 seconds with no interaction
constexpr int BASESTATION_SELECTION_UPDATE_FREQUENCY_MS = 500;

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
            auto selectedYellow = this->getSelectedBasestation(utils::TeamColor::YELLOW);
            if (basestation->operator==(selectedYellow)) this->setSelectedBasestation(nullptr, utils::TeamColor::YELLOW);
            
            auto selectedBlue = this->getSelectedBasestation(utils::TeamColor::BLUE);
            if (basestation->operator==(selectedBlue)) this->setSelectedBasestation(nullptr, utils::TeamColor::BLUE);

            // Remove its serial id from wireless channel map
            this->removeBasestationIdToChannelEntry(basestation->getSerialID());

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
    auto callbackForNewBasestations = [&](const BasestationMessage& message, uint8_t serialID) {
        this->onMessageFromBasestation(message, serialID);
    };

    // Add plugged in basestations that are not in the list yet
    for (libusb_device* pluggedBasestationDevices : pluggedBasestationDevices) {
        if (!deviceIsInBasestationList(pluggedBasestationDevices, this->basestations)) {
            // This basestation is plugged in but not in the list -> add it
            try {
                auto newBasestation = std::make_shared<Basestation>(pluggedBasestationDevices);
                newBasestation->setIncomingMessageCallback(callbackForNewBasestations);

                // Lock the basestations list so we can safely add this basestation
                std::scoped_lock<std::mutex> lock(this->basestationsMutex);
                this->basestations.push_back(newBasestation);
            } catch (FailedToOpenDeviceException e) {
                std::cout << "Error: " << e.what() << std::endl;
                std::cout << "Did you edit your PC's user permissions?" << std::endl;
            }
        }
    }
}

bool BasestationCollection::sendMessageToBasestation(BasestationMessage& message, utils::TeamColor teamColor) {
    const auto& basestation = teamColor == utils::TeamColor::YELLOW ? this->getSelectedBasestation(utils::TeamColor::YELLOW) : this->getSelectedBasestation(utils::TeamColor::BLUE);

    bool sentMessage = false;
    if (basestation != nullptr) {
        sentMessage = basestation->sendMessageToBasestation(message);
    }

    return sentMessage;
}

void BasestationCollection::setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback) {
    this->messageFromBasestationCallback = callback;
}

void BasestationCollection::printCollection() {
    const std::string yes = "yes    ";
    const std::string no =  "no     ";
    const std::string yellow =  "yellow ";
    const std::string blue =    "blue   ";
    const std::string unknown = "unknown";

    const auto selectedYellowCopy = this->getSelectedBasestation(utils::TeamColor::YELLOW);

    std::string yellowFilled = selectedYellowCopy != nullptr ? yes : no;
    std::string yellowChannel = unknown;
    if (selectedYellowCopy != nullptr) {
        WirelessChannel channel = this->getChannelOfBasestation(selectedYellowCopy->getSerialID());
        if (channel == WirelessChannel::BLUE_CHANNEL) {
            yellowChannel = blue;
        } else if (channel == WirelessChannel::YELLOW_CHANNEL) {
            yellowChannel = yellow;
        }
    }

    const auto selectedBlueCopy = this->getSelectedBasestation(utils::TeamColor::BLUE);

    std::string blueFilled = selectedBlueCopy != nullptr ? yes : no;
    std::string blueChannel = unknown;
    if (selectedBlueCopy != nullptr) {
        WirelessChannel channel = this->getChannelOfBasestation(selectedBlueCopy->getSerialID());
        if (channel == WirelessChannel::BLUE_CHANNEL) {
            blueChannel = blue;
        } else if (channel == WirelessChannel::YELLOW_CHANNEL) {
            blueChannel = yellow;
        }
    }

    auto selectableBasestations = this->getSelectableBasestations();
    int remaining = selectableBasestations.size();

    std::cout << "==== Yellow slot ====== Blue slot =====" << std::endl
              << "| filled:  " << yellowFilled << " | filled:  " << blueFilled << " |" << std::endl
              << "| channel: " << yellowChannel << " | channel: " << blueChannel << " |" << std::endl
              << "|----- Remaining: " << remaining << ", Waiting: ? ------|" << std::endl;

    int i = 0;
    for (const auto& basestation : this->getSelectableBasestations()) {
        auto channel = this->getChannelOfBasestation(basestation->getSerialID());
        std::cout << "|- " << i << ": " << BasestationCollection::wirelessChannelToString(channel) << std::endl;
    }
    std::cout << std::endl;
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getAllBasestations() {
    // First, lock the original basestations list
    std::scoped_lock<std::mutex> lock(this->basestationsMutex);

    std::vector<std::shared_ptr<Basestation>> copyVector;
    for (const auto& basestation : this->basestations) {
        copyVector.push_back(basestation);
    }

    return copyVector;
}

std::shared_ptr<Basestation> BasestationCollection::getSelectedBasestation(utils::TeamColor colorOfBasestation) {
    std::shared_ptr<Basestation> basestation;

    std::scoped_lock<std::mutex> lock(this->basestationSelectionMutex);

    switch (colorOfBasestation) {
        case utils::TeamColor::BLUE:
            basestation = this->blueBasestation;
            break;
        case utils::TeamColor::YELLOW:
            basestation = this->yellowBasestation;
            break;
    }

    return basestation;
}

void BasestationCollection::setSelectedBasestation(std::shared_ptr<Basestation> newBasestation, utils::TeamColor color) {
    std::scoped_lock<std::mutex> lock(this->basestationSelectionMutex);
    switch (color) {
        case utils::TeamColor::BLUE:
            this->blueBasestation = newBasestation;
            break;
        case utils::TeamColor::YELLOW:
            this->yellowBasestation = newBasestation;
            break;
    }
}

WirelessChannel BasestationCollection::getChannelOfBasestation(uint8_t serialID) {
    WirelessChannel channel = WirelessChannel::UNKNOWN;

    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);

    auto iterator = this->basestationIdToChannel.find(serialID);
    if (iterator != this->basestationIdToChannel.end()) {
        // If the iterator found a key in the map, the channel is the second value
        channel = iterator->second;
    }

    return channel;
}

void BasestationCollection::setChannelOfBasestation(uint8_t serialId, WirelessChannel newChannel) {
    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);
    this->basestationIdToChannel[serialId] = newChannel;
}

void BasestationCollection::removeBasestationIdToChannelEntry(uint8_t serialId) {
    // Lock the map for thread safety
    std::scoped_lock<std::mutex> lock(this->basestationIdToChannelMutex);
    this->basestationIdToChannel.erase(serialId);
}

void BasestationCollection::updateBasestationSelection() {
    while (this->shouldUpdateBasestationSelection) {
        
        bool hadWrongSelection = this->unselectIncorrectlySelectedBasestations();
        if (hadWrongSelection) {
            std::cout << "Warning: Incorrect basestations were selected!" << std::endl;
        }

        this->askChannelOfBasestationsWithUnknownChannel();

        bool needsToSelectYellowBasestation = this->getSelectedBasestation(utils::TeamColor::YELLOW) == nullptr;
        bool needsToSelectBlueBasestation = this->getSelectedBasestation(utils::TeamColor::BLUE) == nullptr;

        this->selectBasestations(needsToSelectYellowBasestation, needsToSelectBlueBasestation);

        std::this_thread::sleep_for(std::chrono::milliseconds(BASESTATION_SELECTION_UPDATE_FREQUENCY_MS));
    }
}

void BasestationCollection::askChannelOfBasestationsWithUnknownChannel() {
    // Create the channel request message
    BasestationGetConfiguration getConfigurationMessage;
    getConfigurationMessage.header = PACKET_TYPE_BASESTATION_GET_CONFIGURATION;

    BasestationGetConfigurationPayload getConfigurationPayload;
    encodeBasestationGetConfiguration(&getConfigurationPayload, &getConfigurationMessage);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_BASESTATION_GET_CONFIGURATION;
    std::memcpy(&message.payloadBuffer, &getConfigurationPayload.payload, message.payloadSize);
    
    // Send it to every basestation with unknown channel
    for (auto basestation : this->getAllBasestations()) {
        WirelessChannel channel = this->getChannelOfBasestation(basestation->getSerialID());
        if (channel == WirelessChannel::UNKNOWN) {
            basestation->sendMessageToBasestation(message);
        }
    }
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getSelectableBasestations() {
    const auto selectedYellowCopy = this->getSelectedBasestation(utils::TeamColor::YELLOW);
    const auto selectedBlueCopy = this->getSelectedBasestation(utils::TeamColor::BLUE);
    
    std::vector<std::shared_ptr<Basestation>> selectableBasestations;

    for (const auto& basestation : this->getAllBasestations()) {

        if (!basestation->operator==(selectedYellowCopy)
         && !basestation->operator==(selectedBlueCopy)
         && this->getChannelOfBasestation(basestation->getSerialID()) != WirelessChannel::UNKNOWN) {
            // This basestation is neither selected as blue nor yellow basestation, and has a known channel
            selectableBasestations.push_back(basestation);
        }
    }

    return std::move(selectableBasestations);
}

bool BasestationCollection::sendChannelChangeRequest(std::shared_ptr<Basestation> basestation, WirelessChannel newChannel) {

    BasestationSetConfiguration setConfigurationCommand;
    setConfigurationCommand.header = PACKET_TYPE_BASESTATION_SET_CONFIGURATION;
    setConfigurationCommand.remVersion = LOCAL_REM_VERSION;
    setConfigurationCommand.channel = BasestationCollection::wirelessChannelToREMChannel(newChannel);

    BasestationSetConfigurationPayload setConfigurationPayload;
    encodeBasestationSetConfiguration(&setConfigurationPayload, &setConfigurationCommand);

    BasestationMessage message;
    message.payloadSize = PACKET_SIZE_BASESTATION_SET_CONFIGURATION;
    std::memcpy(&message.payloadBuffer, &setConfigurationPayload.payload, message.payloadSize);

    bool sentSuccesfully = basestation->sendMessageToBasestation(message);

    // Set the wireless channel of this basestation to UNKNOWN, as at this point, its channel is uncertain
    this->setChannelOfBasestation(basestation->getSerialID(), WirelessChannel::UNKNOWN);
    
    return sentSuccesfully;
}

int BasestationCollection::selectBasestations(bool needYellowBasestation, bool needBlueBasestation) {
    // Fist get basestations that aren't already selected as the blue or yellow basestation
    auto selectableBasestations = this->getSelectableBasestations();

    // Then get a list of all unselected basestations in combination with their channel
    std::vector<std::shared_ptr<Basestation>> yellowBasestations;
    std::vector<std::shared_ptr<Basestation>> blueBasestations;

    for (const auto& basestation : selectableBasestations) {
        auto basestationsChannel = this->getChannelOfBasestation(basestation->getSerialID());
        switch (basestationsChannel) {
            case WirelessChannel::YELLOW_CHANNEL: {
                yellowBasestations.push_back(basestation);
                break;
            } case WirelessChannel::BLUE_CHANNEL: {
                blueBasestations.push_back(basestation);
                break;
            }
        }
    }

    int numberOfSelectedBasestations = 0;
    bool hasSelectedYellowBasestation = false;
    bool hasSelectedBlueBasestation = false;

    const auto selectedYellowCopy = this->getSelectedBasestation(utils::TeamColor::YELLOW);
    // Now try to select a basestation which already has the channel we want
    if (needYellowBasestation && !yellowBasestations.empty()) {
        this->setSelectedBasestation(yellowBasestations.front(), utils::TeamColor::YELLOW);
        hasSelectedYellowBasestation = true;
        numberOfSelectedBasestations++;

        // Also remove this basestation from the unselected yellow basestations list
        yellowBasestations.erase(yellowBasestations.begin());
    }
    if (needBlueBasestation && !blueBasestations.empty()) {
        this->setSelectedBasestation(blueBasestations.front(), utils::TeamColor::BLUE);
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

bool BasestationCollection::unselectIncorrectlySelectedBasestations() {
    bool unselectedBasestations = false;
    
    const auto selectedYellowCopy = this->getSelectedBasestation(utils::TeamColor::YELLOW);
    const auto selectedBlueCopy = this->getSelectedBasestation(utils::TeamColor::BLUE);

    if (selectedYellowCopy != nullptr && this->getChannelOfBasestation(selectedYellowCopy->getSerialID()) != WirelessChannel::YELLOW_CHANNEL) {
        // The yellow basestation is not actually yellow, so unselect it
        this->setSelectedBasestation(nullptr, utils::TeamColor::YELLOW);
        unselectedBasestations = true;
    }
    if (selectedBlueCopy != nullptr && this->getChannelOfBasestation(selectedBlueCopy->getSerialID()) != WirelessChannel::BLUE_CHANNEL) {
        // The blue basestation is not actually blue, so unselect it
        this->setSelectedBasestation(nullptr, utils::TeamColor::BLUE);
        unselectedBasestations = true;
    }
    return unselectedBasestations;
}

void BasestationCollection::onMessageFromBasestation(const BasestationMessage& message, uint8_t serialID) {
    // If this message contains what channel the basestation has, parse it and update our map
    if (message.payloadBuffer[0] == PACKET_TYPE_BASESTATION_CONFIGURATION) {
        BasestationConfigurationPayload configurationPayload;
        std::memcpy(&configurationPayload.payload, message.payloadBuffer, PACKET_SIZE_BASESTATION_CONFIGURATION);

        BasestationConfiguration basestationConfiguration;
        decodeBasestationConfiguration(&basestationConfiguration, &configurationPayload);

        WirelessChannel usedChannel = BasestationCollection::remChannelToWirelessChannel(basestationConfiguration.channel);
        this->setChannelOfBasestation(serialID, usedChannel);
    }

    // This function can be called by multiple basestations simultaneously, so protect it with a mutex
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    // And forward the message if this serialID belongs to a selected basestation
    if (this->messageFromBasestationCallback != nullptr) {
        const auto selectedYellowCopy = this->getSelectedBasestation(utils::TeamColor::YELLOW);
        const auto selectedBlueCopy = this->getSelectedBasestation(utils::TeamColor::BLUE);
        
        if (selectedYellowCopy->operator==(serialID)) {
            this->messageFromBasestationCallback(message, utils::TeamColor::YELLOW);
        } else if (selectedBlueCopy->operator==(serialID)) {
            this->messageFromBasestationCallback(message, utils::TeamColor::BLUE);
        }
    }
}

WirelessChannel BasestationCollection::getWirelessChannelCorrespondingTeamColor(utils::TeamColor color) {
    WirelessChannel matchingWirelessChannel;

    switch (color) {
        case utils::TeamColor::BLUE:
            matchingWirelessChannel = WirelessChannel::BLUE_CHANNEL;
            break;
        case utils::TeamColor::YELLOW:
            matchingWirelessChannel = WirelessChannel::YELLOW_CHANNEL;
            break;
        default:
            matchingWirelessChannel = WirelessChannel::UNKNOWN;
            break;
    }

    return matchingWirelessChannel;
}

utils::TeamColor BasestationCollection::getTeamColorCorrespondingWirelessChannel(WirelessChannel channel) {
    switch (channel) {
        case WirelessChannel::YELLOW_CHANNEL:
            return utils::TeamColor::YELLOW;
        case WirelessChannel::BLUE_CHANNEL:
            return utils::TeamColor::BLUE;
        default:
            return utils::TeamColor::YELLOW;
    }
}

bool BasestationCollection::basestationIsInDeviceList(std::shared_ptr<Basestation> basestation, const std::vector<libusb_device*>& devices) {
    bool basestationIsInList = false;

    for (auto device : devices) {
        if (*basestation == device) {
            basestationIsInList = true;
            break;
        }
    }

    return basestationIsInList;
}

bool BasestationCollection::deviceIsInBasestationList(libusb_device* device, const std::vector<std::shared_ptr<Basestation>>& basestations) {
    bool deviceIsInList = false;

    for (auto basestation : basestations) {
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
