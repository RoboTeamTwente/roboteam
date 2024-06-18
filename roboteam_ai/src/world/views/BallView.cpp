#include "world/views/BallView.hpp"

namespace rtt::world::view {
BallView::BallView(const rtt::world::ball::Ball *const _ptr) noexcept : _ptr{_ptr} {}

const ball::Ball *BallView::get() const noexcept { return _ptr; }

const ball::Ball &BallView::operator*() const noexcept { return *get(); }

const ball::Ball *BallView::operator->() const noexcept { return get(); }

BallView &BallView::operator=(const BallView &old) noexcept {
    if (this != &old) {
        _ptr = old._ptr;
    }
    return *this;
}

BallView &BallView::operator=(BallView &&other) noexcept {
    if (this != &other) {
        _ptr = other._ptr;
        other._ptr = nullptr;
    }
    return *this;
}

BallView::BallView(BallView &&other) noexcept : _ptr{other._ptr} { other._ptr = nullptr; }

BallView::operator bool() const noexcept { return get() != nullptr; }

}  // namespace rtt::world::view