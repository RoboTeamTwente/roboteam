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
            // Remove basestation from team selection
            if (basestation->operator==(this->blueBasestation)) this->blueBasestation = nullptr;
            if (basestation->operator==(this->yellowBasestation)) this->yellowBasestation = nullptr;

            // Remove serial id from wireless channel map
            this->basestationIdToWirelessChannel.erase(basestation->getSerialID());

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

                this->basestations.push_back(newBasestation);
            } catch (FailedToOpenDeviceException e) {
                std::cout << "Error: " << e.what() << std::endl;
                std::cout << "Did you edit your PC's user permissions?" << std::endl;
            }
        }
    }
}

bool BasestationCollection::sendMessageToBasestation(BasestationMessage& message, utils::TeamColor teamColor) {
    const auto& basestation = teamColor == utils::TeamColor::YELLOW ? this->yellowBasestation : this->blueBasestation;

    bool sentMessage = false;
    if (basestation != nullptr) {
        sentMessage = basestation->sendMessageToBasestation(message);
    }

    return sentMessage;
}

void BasestationCollection::setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback) {
    this->messageFromBasestationCallback = callback;
}

void BasestationCollection::printCollection() const {
    const std::string yes = "yes    ";
    const std::string no =  "no     ";
    const std::string yellow =  "yellow ";
    const std::string blue =    "blue   ";
    const std::string unknown = "unknown";

    std::string yellowFilled = this->yellowBasestation != nullptr ? yes : no;
    std::string yellowChannel = unknown;
    if (this->yellowBasestation != nullptr) {
        WirelessChannel channel = this->getWirelessChannelOfBasestation(yellowBasestation->getSerialID());
        if (channel == WirelessChannel::BLUE_CHANNEL) {
            yellowChannel = blue;
        } else if (channel == WirelessChannel::YELLOW_CHANNEL) {
            yellowChannel = yellow;
        }
    }

    std::string blueFilled = this->blueBasestation != nullptr ? yes : no;
    std::string blueChannel = unknown;
    if (this->blueBasestation != nullptr) {
        WirelessChannel channel = this->getWirelessChannelOfBasestation(blueBasestation->getSerialID());
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
        auto channel = this->getWirelessChannelOfBasestation(basestation->getSerialID());
        std::cout << "|- " << i << ": " << BasestationCollection::wirelessChannelToString(channel) << std::endl;
    }
    std::cout << std::endl;
}

WirelessChannel BasestationCollection::getWirelessChannelOfBasestation(uint8_t serialID) const {
    WirelessChannel channel = WirelessChannel::UNKNOWN;

    auto iterator = this->basestationIdToWirelessChannel.find(serialID);
    if (iterator != this->basestationIdToWirelessChannel.end()) {
        // If the iterator found a key in the map, the channel is the second value
        channel = iterator->second;
    }

    return channel;
}

void BasestationCollection::updateBasestationSelection() {
    while (this->shouldUpdateBasestationSelection) {
        
        bool hadWrongSelection = this->unselectIncorrectlySelectedBasestations();
        if (hadWrongSelection) {
            std::cout << "Warning: Incorrect basestations were selected!" << std::endl;
        }

        this->askChannelOfBasestationsWithUnknownChannel();

        bool needsToSelectYellowBasestation = this->yellowBasestation == nullptr;
        bool needsToSelectBlueBasestation = this->blueBasestation == nullptr;

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
    for (auto basestation : this->basestations) {
        WirelessChannel channel = this->getWirelessChannelOfBasestation(basestation->getSerialID());
        if (channel == WirelessChannel::UNKNOWN) {
            basestation->sendMessageToBasestation(message);
        }
    }
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getSelectableBasestations() const {
    std::vector<std::shared_ptr<Basestation>> selectableBasestations;

    for (const auto& basestation : this->basestations) {

        if (!basestation->operator==(this->blueBasestation)
         && !basestation->operator==(this->yellowBasestation)
         && this->getWirelessChannelOfBasestation(basestation->getSerialID()) != WirelessChannel::UNKNOWN) {
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
    this->basestationIdToWirelessChannel[basestation->getSerialID()] = WirelessChannel::UNKNOWN;
    
    return sentSuccesfully;
}

int BasestationCollection::selectBasestations(bool needYellowBasestation, bool needBlueBasestation) {
    // Fist get basestations that aren't already selected as the blue or yellow basestation
    auto selectableBasestations = this->getSelectableBasestations();
    // TODO: Protect basestations vector because of unsafe behaviour

    // Then get a list of all unselected basestations in combination with their channel
    std::vector<std::shared_ptr<Basestation>> yellowBasestations;
    std::vector<std::shared_ptr<Basestation>> blueBasestations;

    for (const auto& basestation : selectableBasestations) {
        auto basestationsChannel = this->getWirelessChannelOfBasestation(basestation->getSerialID());
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

    // Now try to select a basestation which already has the channel we want
    if (needYellowBasestation && !yellowBasestations.empty()) {
        this->yellowBasestation = yellowBasestations.front();
        hasSelectedYellowBasestation = true;
        numberOfSelectedBasestations++;

        // Also remove this basestation from the unselected yellow basestations list
        yellowBasestations.erase(yellowBasestations.begin());
    }
    if (needBlueBasestation && !blueBasestations.empty()) {
        this->blueBasestation = blueBasestations.front();
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
    
    if (this->yellowBasestation != nullptr && this->getWirelessChannelOfBasestation(this->yellowBasestation->getSerialID()) != WirelessChannel::YELLOW_CHANNEL) {
        // The yellow basestation is not actually yellow, so unselect it
        this->yellowBasestation = nullptr;
        unselectedBasestations = true;
    }
    if (this->blueBasestation != nullptr && this->getWirelessChannelOfBasestation(this->blueBasestation->getSerialID()) != WirelessChannel::BLUE_CHANNEL) {
        // The blue basestation is not actually blue, so unselect it
        this->blueBasestation = nullptr;
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
        this->basestationIdToWirelessChannel[serialID] = usedChannel;
    }

    // This function can be called by multiple basestations simultaneously, so protect it with a mutex
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    // And forward the message if this serialID belongs to a selected basestation
    if (this->messageFromBasestationCallback != nullptr) {
        if (this->yellowBasestation->operator==(serialID)) {
            this->messageFromBasestationCallback(message, utils::TeamColor::YELLOW);
        } else if (this->blueBasestation->operator==(serialID)) {
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
