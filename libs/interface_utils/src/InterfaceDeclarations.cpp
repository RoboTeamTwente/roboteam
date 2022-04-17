//
// Created by Dawid Kulikowski on 10/08/2021.
//
#include "InterfaceDeclarations.h"
#include <algorithm>
#include <vector>

namespace rtt::Interface {

void InterfaceDeclarations::addDeclaration(const InterfaceDeclaration decl) noexcept {
    std::scoped_lock lck(this->mtx);

    bool did_change = !this->decls.contains(decl.path) || this->decls.at(decl.path) != decl;

    this->decls.insert_or_assign(decl.path, decl);

}

void InterfaceDeclarations::removeDeclaration(const std::string path) noexcept {
    std::scoped_lock lck(this->mtx);
    this->decls.erase(path);

}

std::map<std::string, InterfaceDeclaration> InterfaceDeclarations::getDeclarations() const noexcept {
    std::scoped_lock lck(this->mtx);

    return this->decls;
}

std::optional<InterfaceDeclaration> InterfaceDeclarations::getDeclaration(const std::string path) const noexcept {
    std::scoped_lock lck(this->mtx);

    if (!this->decls.contains(path)) return std::nullopt;

    return this->decls.at(path);
}

std::vector<std::string> InterfaceDeclarations::getWithSuffix(const std::string& suffix) const noexcept {
    std::vector<std::string> res;

    for (const auto& [key, value] : this->decls) {
        if (key.ends_with(suffix)) {
            res.push_back(key);
        }
    }

    return res;
}

void InterfaceDeclarations::handleData(const proto::UiOptionDeclarations& recvDecls) noexcept {
    std::scoped_lock lck(this->mtx);

    this->decls.clear();

    for (auto singleDecl: recvDecls.options()) {
        this->decls.insert_or_assign(singleDecl.path(), singleDecl);
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
}