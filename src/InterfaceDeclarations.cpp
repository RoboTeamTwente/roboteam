//
// Created by Dawid Kulikowski on 10/08/2021.
//
#include "InterfaceDeclarations.h"
#include <algorithm>
#include <vector>

namespace rtt::Interface {

void InterfaceDeclarations::addDeclaration(const InterfaceDeclaration decl) noexcept {
    std::scoped_lock lck(this->mtx);

    this->decls.insert_or_assign(decl.path, decl);

    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }
}

void InterfaceDeclarations::removeDeclaration(const std::string path) noexcept {
    std::scoped_lock lck(this->mtx);

    this->decls.erase(path);

    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }
}

std::map<std::string, InterfaceDeclaration> InterfaceDeclarations::getDeclarations() const noexcept {
    std::scoped_lock lck(this->mtx);

    return this->decls;
}

std::optional<InterfaceDeclaration> InterfaceDeclarations::getDeclaration(const std::string path) const noexcept {
    std::scoped_lock lck(this->mtx);

    return this->decls.at(path);
}

void InterfaceDeclarations::handleData(const proto::UiOptionDeclarations& recvDecls) noexcept {
    std::scoped_lock lck(this->mtx);

    this->decls.clear();

    for (auto singleDecl: recvDecls.options()) {
        this->decls.insert_or_assign(singleDecl.path(), singleDecl);
    }

    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }
}
proto::UiOptionDeclarations InterfaceDeclarations::toProto() const noexcept {
    std::scoped_lock lck(this->mtx);

    proto::UiOptionDeclarations protoDecls;

    for (const auto& entry : this->decls) {
        protoDecls.mutable_options()->Add(entry.second.toProto());
    }

    return protoDecls;
}
InterfaceDeclarations::InterfaceDeclarations(const nlohmann::json& json, std::weak_ptr<InterfaceStateHandler> sts): stateHandler(sts) {
    std::vector<InterfaceDeclaration> tmpDecl;
    json.get_to(tmpDecl);

    for (const auto& val : tmpDecl) {
        this->decls.insert_or_assign(val.path, val);
    }


    if (auto state = this->stateHandler.lock()) {
        state->stateDidChange();
    }
}
}