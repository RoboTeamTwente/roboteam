#include <BasestationConfiguration.h>     // REM packet
#include <BasestationSetConfiguration.h>  // REM packet

#include <basestation/BasestationCollection.hpp>
#include <iostream>

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
            iterator = this->basestations.erase(iterator);
            // Remove basestation from team selection
            if (basestation == this->blueBasestation) this->unselectBasestationOfColor(utils::TeamColor::BLUE);
            if (basestation == this->yellowBasestation) this->unselectBasestationOfColor(utils::TeamColor::YELLOW);
        } else {
            ++iterator;
        }
    }

    // Add plugged in basestations that are not in the list yet
    for (libusb_device* pluggedBasestationDevices : pluggedBasestationDevices) {
        if (!deviceIsInBasestationList(pluggedBasestationDevices, this->basestations)) {
            // This basestation is plugged in but not in the list -> add it
            try {
                auto newBasestation = std::make_shared<Basestation>(pluggedBasestationDevices);
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
    const std::string no = "no     ";
    const std::string yellow = "yellow ";
    const std::string blue = "blue   ";
    const std::string unknown = "unknown";

    std::string yellowFilled = this->yellowBasestation != nullptr ? yes : no;
    std::string yellowChannel = unknown;
    if (this->yellowBasestation != nullptr) {
        if (this->yellowBasestation->getChannel() == WirelessChannel::BLUE_CHANNEL) {
            yellowChannel = blue;
        } else if (this->yellowBasestation->getChannel() == WirelessChannel::YELLOW_CHANNEL) {
            yellowChannel = yellow;
        }
    }

    std::string blueFilled = this->blueBasestation != nullptr ? yes : no;
    std::string blueChannel = unknown;
    if (this->blueBasestation != nullptr) {
        if (this->blueBasestation->getChannel() == WirelessChannel::BLUE_CHANNEL) {
            blueChannel = blue;
        } else if (this->blueBasestation->getChannel() == WirelessChannel::YELLOW_CHANNEL) {
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
        std::cout << "|- " << i << ": " << Basestation::wirelessChannelToString(basestation->getChannel()) << std::endl;
    }
    std::cout << std::endl;
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
        if (basestation != this->blueBasestation && basestation != this->yellowBasestation) {
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
        if (basestation->getChannel() == getWirelessChannelCorrespondingTeamColor(color)) {
            this->selectBasestationOfColor(basestation, color);
            selectedBasestation = true;
            break;
        }
    }

    return selectedBasestation;
}

void BasestationCollection::onMessageFromBasestation(const BasestationMessage& message, utils::TeamColor color) {
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    if (this->messageFromBasestationCallback != nullptr) {
        this->messageFromBasestationCallback(message, color);
    }
}

void BasestationCollection::selectBasestationOfColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color) {
    this->unselectBasestationOfColor(color);

    auto callback = [&](const BasestationMessage& message) { this->onMessageFromBasestation(message, color); };

    switch (color) {
        case utils::TeamColor::BLUE:
            this->blueBasestation = basestation;
            this->blueBasestation->setIncomingMessageCallback(callback);
            break;
        case utils::TeamColor::YELLOW:
            this->yellowBasestation = basestation;
            this->yellowBasestation->setIncomingMessageCallback(callback);
            break;
    }
}

void BasestationCollection::unselectBasestationOfColor(utils::TeamColor color) {
    switch (color) {
        case utils::TeamColor::BLUE: {
            if (this->blueBasestation != nullptr) {
                this->blueBasestation->setIncomingMessageCallback(nullptr);
                this->blueBasestation = nullptr;
            }
            break;
        }
        case utils::TeamColor::YELLOW: {
            if (this->yellowBasestation != nullptr) {
                this->yellowBasestation->setIncomingMessageCallback(nullptr);
                this->yellowBasestation = nullptr;
            }
            break;
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

}  // namespace rtt::robothub::basestation
