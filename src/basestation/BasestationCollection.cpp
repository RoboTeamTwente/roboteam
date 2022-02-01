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
    this->blueBasestation = nullptr;
    this->yellowBasestation = nullptr;

    this->shouldUpdateBasestationSelection = true;
    this->basestationSelectionUpdaterThread = std::thread(&BasestationCollection::updateBasestationSelection, this);
}
BasestationCollection::~BasestationCollection() {
    this->shouldUpdateBasestationSelection = false;
    if (this->basestationSelectionUpdaterThread.joinable()) {
        this->basestationSelectionUpdaterThread.join();
    }

    this->unselectBasestationOfColor(utils::TeamColor::BLUE);
    this->unselectBasestationOfColor(utils::TeamColor::YELLOW);

    this->basestations.clear();
}

// Updates current basestations list by comparing it with the given plugged-in basestation devices list
void BasestationCollection::updateBasestationList(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    // Remove basestations that are not plugged in anymore
    auto iterator = this->basestations.begin();
    while (iterator != this->basestations.end()) {
        auto basestation = *iterator;

        if (!basestationIsInDeviceList(basestation, pluggedBasestationDevices)) {
            // This basestation is not plugged in anymore -> remove it
            // Remove basestation from team selection
            if (basestation == this->blueBasestation) this->unselectBasestationOfColor(utils::TeamColor::BLUE);
            if (basestation == this->yellowBasestation) this->unselectBasestationOfColor(utils::TeamColor::YELLOW);

            // Remove serial id from wireless channel map
            this->basestationIdToWirelessChannel.erase(basestation->getSerialID());

            // Remove basestation from list
            iterator = this->basestations.erase(iterator);

        } else {
            ++iterator;
        }
    }

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

    auto unselectedBasestations = this->getUnselectedBasestations();
    int remaining = unselectedBasestations.size();

    std::cout << "==== Yellow slot ====== Blue slot =====" << std::endl
              << "| filled:  " << yellowFilled << " | filled:  " << blueFilled << " |" << std::endl
              << "| channel: " << yellowChannel << " | channel: " << blueChannel << " |" << std::endl
              << "|----- Remaining: " << remaining << ", Waiting: ? ------|" << std::endl;

    int i = 0;
    for (const auto& basestation : this->getUnselectedBasestations()) {
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

void BasestationCollection::askChannelOfBasestationsWithUnknownChannel() {
    // Create the message we want to send
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

void BasestationCollection::updateBasestationSelection() {
    while (this->shouldUpdateBasestationSelection) {
    
        if (this->yellowBasestation == nullptr) {
            this->trySelectBasestationOfColor(utils::TeamColor::YELLOW);
        }
        if (this->blueBasestation == nullptr) {
            this->trySelectBasestationOfColor(utils::TeamColor::BLUE);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(BASESTATION_SELECTION_UPDATE_FREQUENCY_MS));
    }
}

std::vector<std::shared_ptr<Basestation>> BasestationCollection::getUnselectedBasestations() const {
    std::vector<std::shared_ptr<Basestation>> unselectedBasestations;

    for (const auto& basestation : this->basestations) {

        if (!basestation->operator==(this->blueBasestation)
         && !basestation->operator==(this->yellowBasestation)) {
            // This basestation is neither selected as blue nor yellow basestation
            unselectedBasestations.push_back(basestation);
        }
    }

    return unselectedBasestations;
}

bool BasestationCollection::trySelectBasestationOfColor(utils::TeamColor color) {
    // Fist get basestations that aren't already selected as the blue or yellow basestation
    auto unselectedBasestations = this->getUnselectedBasestations();

    bool selectedBasestation = false;
    for (const auto& basestation : unselectedBasestations) {
        WirelessChannel channel = this->getWirelessChannelOfBasestation(basestation->getSerialID());
        
        if (channel == getWirelessChannelCorrespondingTeamColor(color)) {
            this->selectBasestationOfColor(basestation, color);
            selectedBasestation = true;
            break;
        }
    }

    return selectedBasestation;
}

void BasestationCollection::onMessageFromBasestation(const BasestationMessage& message, uint8_t serialID) {
    // This function can be called by multiple basestations simultaneously, so protect it with a mutex
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    // If this message contains what channel the basestation has, parse it and update our map
    if (message.payloadBuffer[0] == PACKET_TYPE_BASESTATION_CONFIGURATION) {
        BasestationConfigurationPayload configurationPayload;
        std::memcpy(&configurationPayload.payload, message.payloadBuffer, PACKET_SIZE_BASESTATION_CONFIGURATION);

        BasestationConfiguration basestationConfiguration;
        decodeBasestationConfiguration(&basestationConfiguration, &configurationPayload);

        WirelessChannel usedChannel = BasestationCollection::remChannelToWirelessChannel(basestationConfiguration.channel);
        this->basestationIdToWirelessChannel[serialID] = usedChannel;
    }

    // And forward the message if this serialID belongs to a selected basestation
    if (this->messageFromBasestationCallback != nullptr) {
        if (this->yellowBasestation->operator==(serialID)) {
            this->messageFromBasestationCallback(message, utils::TeamColor::YELLOW);
        } else if (this->blueBasestation->operator==(serialID)) {
            this->messageFromBasestationCallback(message, utils::TeamColor::BLUE);
        }
    }
}

void BasestationCollection::selectBasestationOfColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color) {
    switch (color) {
        case utils::TeamColor::BLUE:
            this->blueBasestation = basestation;
            break;
        case utils::TeamColor::YELLOW:
            this->yellowBasestation = basestation;
            break;
    }
}

void BasestationCollection::unselectBasestationOfColor(utils::TeamColor color) {
    switch (color) {
        case utils::TeamColor::BLUE:
            this->blueBasestation = nullptr;
            break;
        case utils::TeamColor::YELLOW:
            this->yellowBasestation = nullptr;
            break;
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
