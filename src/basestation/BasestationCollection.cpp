#include <BasestationConfiguration.h>

#include <algorithm>
#include <basestation/BasestationCollection.hpp>
#include <chrono>
#include <iostream>
#include <string>

namespace rtt::robothub::basestation {

constexpr int TIME_UNTILL_BASESTATION_IS_UNSELECTED_S = 3;  // 3 seconds with no interaction
constexpr int BASESTATION_SELECTION_UPDATE_FREQUENCY_MS = 500;

BasestationCollection::BasestationCollection() {
    this->blueBasestation = nullptr;
    this->yellowBasestation = nullptr;

    this->currentTeamColorUsage = TeamColorUsage::NEITHER_YELLOW_NOR_BLUE;
    // Set the last use of basestations to a few seconds before, so this class does not think they were used
    this->lastUseOfBlueBasestation = std::chrono::steady_clock::now() - std::chrono::steady_clock::duration(std::chrono::seconds(TIME_UNTILL_BASESTATION_IS_UNSELECTED_S));
    this->lastUseOfYellowBasestation = std::chrono::steady_clock::now() - std::chrono::steady_clock::duration(std::chrono::seconds(TIME_UNTILL_BASESTATION_IS_UNSELECTED_S));

    this->shouldUpdateBasestationSelection = true;
    this->basestationSelectionUpdaterThread = std::thread(&BasestationCollection::updateBasestationSelection, this);
}
BasestationCollection::~BasestationCollection() {
    this->shouldUpdateBasestationSelection = false;
    if (this->basestationSelectionUpdaterThread.joinable()) {
        this->basestationSelectionUpdaterThread.join();
    }

    this->unselectBasestationAtColor(utils::TeamColor::BLUE);
    this->unselectBasestationAtColor(utils::TeamColor::YELLOW);

    this->basestations.clear();
}

void BasestationCollection::updateBasestationList(const std::vector<libusb_device*>& pluggedBasestationDevices) {
    // Remove basestations that are not plugged in anymore
    auto iterator = this->basestations.begin();
    while (iterator != this->basestations.end()) {
        auto basestation = *iterator;
        if (!basestationIsInDeviceList(basestation, pluggedBasestationDevices)) {
            // This basestation is not plugged in anymore -> remove it
            iterator = this->basestations.erase(iterator);

            // Remove basestation from team selection
            if (basestation == this->blueBasestation) this->blueBasestation = nullptr;
            if (basestation == this->yellowBasestation) this->yellowBasestation = nullptr;

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
                std::cout << "Did you edit your user permissions?" << std::endl;
            }
        }
    }
}

bool BasestationCollection::sendMessageToBasestation(const BasestationMessage& message, utils::TeamColor teamColor) const {
    bool messageSent;
    switch (teamColor) {
        case utils::TeamColor::BLUE:
            messageSent = this->blueBasestation->sendMessageToBasestation(message);
            break;
        case utils::TeamColor::YELLOW:
            messageSent = this->yellowBasestation->sendMessageToBasestation(message);
            break;
        default:
            messageSent = false;
    }

    return messageSent;
}

void BasestationCollection::setIncomingMessageCallback(std::function<void(const BasestationMessage&, utils::TeamColor)> callback) {
    this->messageFromBasestationCallback = callback;
}

void BasestationCollection::printCollection() const {
    const std::string yes = "yes";
    const std::string no = "no ";
    const std::string yellow = "yellow ";
    const std::string blue = "blue   ";
    const std::string unknown = "unknown";

    std::string yellowFilled = this->yellowBasestation != nullptr ? yes : no;
    std::string yellowChannel = unknown;
    if (this->yellowBasestation != nullptr) {
        if (this->yellowBasestation->getCurrentUsedChannel() == WirelessChannel::BLUE_CHANNEL) {
            yellowChannel = blue;
        } else if (this->yellowBasestation->getCurrentUsedChannel() == WirelessChannel::YELLOW_CHANNEL) {
            yellowChannel = yellow;
        }
    }

    std::string blueFilled = this->blueBasestation != nullptr ? yes : no;
    std::string blueChannel = unknown;
    if (this->blueBasestation != nullptr) {
        if (this->blueBasestation->getCurrentUsedChannel() == WirelessChannel::BLUE_CHANNEL) {
            blueChannel = blue;
        } else if (this->blueBasestation->getCurrentUsedChannel() == WirelessChannel::YELLOW_CHANNEL) {
            blueChannel = yellow;
        }
    }

    int remaining = this->getUnselectedBasestations().size();

    std::cout << "=== Yellow slot ====== Blue slot ====" << std::endl
              << "| filled:  " << yellowFilled << " | filled:  " << blueFilled << " |" << std::endl
              << "| channel: " << yellowChannel << " | channel: " << blueChannel << " |" << std::endl
              << "|----------- Remaining: " << remaining << " ----------|" << std::endl;
}

TeamColorUsage BasestationCollection::getTeamColorUsage() {
    auto now = std::chrono::steady_clock::now();

    // Calculate how long ago the basestations were used
    auto timeAfterLastBlueUsage = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastUseOfBlueBasestation).count();
    auto timeAfterLastYellowUsage = std::chrono::duration_cast<std::chrono::seconds>(now - this->lastUseOfYellowBasestation).count();

    bool recentlyUsedBlueBasestation = timeAfterLastBlueUsage <= TIME_UNTILL_BASESTATION_IS_UNSELECTED_S;
    bool recentlyUsedYellowBasestation = timeAfterLastYellowUsage <= TIME_UNTILL_BASESTATION_IS_UNSELECTED_S;

    TeamColorUsage usage = TeamColorUsage::NEITHER_YELLOW_NOR_BLUE;

    if (recentlyUsedBlueBasestation && recentlyUsedYellowBasestation) {
        usage = TeamColorUsage::YELLOW_AND_BLUE;
    } else if (!recentlyUsedBlueBasestation && recentlyUsedYellowBasestation) {
        usage = TeamColorUsage::ONLY_YELLOW;
    } else if (recentlyUsedBlueBasestation && !recentlyUsedYellowBasestation) {
        usage = TeamColorUsage::ONLY_BLUE;
    }

    return usage;
}

void BasestationCollection::updateTeamColorUsage(utils::TeamColor usedBasestationColor) {
    auto now = std::chrono::steady_clock::now();

    switch (usedBasestationColor) {
        case utils::TeamColor::BLUE: {
            this->lastUseOfBlueBasestation = now;
            break;
        }
        case utils::TeamColor::YELLOW: {
            this->lastUseOfYellowBasestation = now;
            break;
        }
    }

    this->currentTeamColorUsage = this->getTeamColorUsage();
}

void BasestationCollection::updateBasestationSelection() {
    while (this->shouldUpdateBasestationSelection) {
        bool hasBlueBasestation = this->blueBasestation != nullptr;
        bool hasYellowBasestation = this->yellowBasestation != nullptr;

        switch (this->currentTeamColorUsage) {
            case TeamColorUsage::NEITHER_YELLOW_NOR_BLUE: {
                this->blueBasestation = nullptr;
                this->yellowBasestation = nullptr;
                break;
            }
            case TeamColorUsage::ONLY_BLUE: {
                this->yellowBasestation = nullptr;
                if (!hasBlueBasestation) {
                    this->trySelectBasestationOfColor(utils::TeamColor::BLUE);
                }
                break;
            }
            case TeamColorUsage::ONLY_YELLOW: {
                this->blueBasestation = nullptr;
                if (!hasYellowBasestation) {
                    this->trySelectBasestationOfColor(utils::TeamColor::YELLOW);
                }
                break;
            }
            case TeamColorUsage::YELLOW_AND_BLUE: {
                if (!hasBlueBasestation) {
                    this->trySelectBasestationOfColor(utils::TeamColor::BLUE);
                }
                if (!hasYellowBasestation) {
                    this->trySelectBasestationOfColor(utils::TeamColor::YELLOW);
                }
                break;
            }
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

    // Find a basestation that uses the channel of the color we need, and is not waiting on a frequency change
    auto basestationWithRightChannel = std::find_if(unselectedBasestations.begin(), unselectedBasestations.end(), [color](const std::shared_ptr<Basestation>& basestation) {
        return wirelessChannelMatchesTeamColor(basestation->getCurrentUsedChannel(), color) &&
               !basestation->isWaitingForChannelChange();  // If we earlier on requested this one to change, do not use this one, as its frequency might change later
    });

    if (basestationWithRightChannel != std::end(unselectedBasestations)) {
        // There is already a basestation with the frequency that matches the wanted team color, so just use that one
        this->selectBasestationAtColor(*basestationWithRightChannel, color);
        return true;
    } else {
        // Request a basestation to change its frequency, so next iteration we might be able to find and select it
        // TODO: Prefer to ask a basestation that does not have a known frequency yet

        auto basestationToAskFrequencyChange = std::find_if(unselectedBasestations.begin(), unselectedBasestations.end(), [](const std::shared_ptr<Basestation>& basestation) {
            return !basestation->isWaitingForChannelChange();  // Pick a basestation that is not already waiting for a new channel
        });

        if (basestationToAskFrequencyChange != std::end(unselectedBasestations)) {
            // We found a basestation we can ask a frequency change!
            return (*basestationToAskFrequencyChange)->requestChannelChange(BasestationCollection::getWirelessChannelCorrespondingTeamColor(color));
        } else {
            // There is no unselected basestation that is not waiting for a channel change, so we cannot ask one :(
        }
    }

    return false;
}

void BasestationCollection::onMessageFromBasestation(const BasestationMessage& message, utils::TeamColor color) {
    std::lock_guard<std::mutex> lock(this->messageCallbackMutex);

    if (this->messageFromBasestationCallback != nullptr) {
        this->messageFromBasestationCallback(message, color);
    }
}

void BasestationCollection::selectBasestationAtColor(std::shared_ptr<Basestation> basestation, utils::TeamColor color) {
    this->unselectBasestationAtColor(color);

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

void BasestationCollection::unselectBasestationAtColor(utils::TeamColor color) {
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
    WirelessChannel matchingWirelessChannel = WirelessChannel::UNKNOWN;

    switch (color) {
        case utils::TeamColor::BLUE:
            matchingWirelessChannel = WirelessChannel::BLUE_CHANNEL;
            break;
        case utils::TeamColor::YELLOW:
            matchingWirelessChannel = WirelessChannel::YELLOW_CHANNEL;
            break;
    }

    return matchingWirelessChannel;
}

bool BasestationCollection::wirelessChannelMatchesTeamColor(WirelessChannel channel, utils::TeamColor color) {
    return (channel == WirelessChannel::BLUE_CHANNEL && color == utils::TeamColor::BLUE) || (channel == WirelessChannel::YELLOW_CHANNEL && color == utils::TeamColor::YELLOW);
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