//
// Created by Dawid Kulikowski on 15/06/2022.
//

#ifndef RTT_MESSAGECACHE_H
#define RTT_MESSAGECACHE_H
#include <proto/State.pb.h>

#include <mutex>
#include <optional>
#include <string>

namespace rtt::Interface {
    template <typename C>
    class MessageCache {
    private:
        mutable std::mutex mtx;

        std::optional<C> cachedState;
        std::optional<std::string> state;

    public:
        void setMessage(C) noexcept;
        void setMessage(std::string) noexcept;
        std::optional<C> getMessage();
    };

    template <typename C>
    void MessageCache<C>::setMessage(C msg) noexcept {
        std::scoped_lock lck(mtx);
        this->state = std::nullopt;
        this->cachedState = msg;
    }

    template <typename C>
    void MessageCache<C>::setMessage(std::string msg) noexcept {
        std::scoped_lock lck(mtx);

        this->state = msg;
    }

    template <typename C>
    std::optional<C> MessageCache<C>::getMessage() {
        std::scoped_lock lck(mtx);

        if (this->state.has_value()) {
            if (this->cachedState.has_value()) {
                this->cachedState->ParseFromString(this->state.value());
            } else {
                C a;
                a.ParseFromString(this->state.value());
                this->cachedState = std::move(a);
            }

            this->state = std::nullopt;
        }

        return this->cachedState;
    }
}

#endif  // RTT_MESSAGECACHE_H
