//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACEDECLARATIONS_H
#define RTT_INTERFACEDECLARATIONS_H

#include <roboteam_proto/UiOptions.pb.h>

#include <nlohmann/json.hpp>
#include <utility>

#include "InterfaceDeclaration.h"
#include "InterfaceStateHandler.h"

namespace rtt::Interface {
class InterfaceDeclarations {
   private:
       // Duplicate path for ease of use
       std::map<std::string, InterfaceDeclaration> decls;

       mutable std::mutex mtx;

       std::weak_ptr<InterfaceStateHandler> stateHandler;

   public:
       InterfaceDeclarations() = delete;
       ~InterfaceDeclarations() = default;

       InterfaceDeclarations(InterfaceDeclarations&&) = delete;
       InterfaceDeclarations& operator=(InterfaceDeclarations&&) = delete;


       InterfaceDeclarations(const InterfaceDeclarations&) = delete;
       InterfaceDeclarations& operator=(const InterfaceDeclarations&) = delete;


       InterfaceDeclarations(const nlohmann::json&, std::weak_ptr<InterfaceStateHandler>);
       explicit InterfaceDeclarations(std::weak_ptr<InterfaceStateHandler> sts): stateHandler(std::move(sts)) {};
       explicit InterfaceDeclarations(std::map<std::string, InterfaceDeclaration> vec, std::weak_ptr<InterfaceStateHandler> sts = {}) : decls(std::move(vec)), stateHandler(std::move(sts)) {};
       explicit InterfaceDeclarations(const std::vector<InterfaceDeclaration>& vec, std::weak_ptr<InterfaceStateHandler> sts = {}): stateHandler(std::move(sts)) {
           for (const auto& val : vec) {
               this->decls.insert_or_assign(val.path, val);
           }
       }

       [[maybe_unused]] void addDeclaration(InterfaceDeclaration) noexcept;
       [[maybe_unused]] void removeDeclaration(std::string) noexcept;

       std::map<std::string, InterfaceDeclaration> getDeclarations() const noexcept;
       std::optional<InterfaceDeclaration> getDeclaration(std::string) const noexcept;

       void handleData(const proto::UiOptionDeclarations&) noexcept;

       proto::UiOptionDeclarations toProto() const noexcept;
};

}
#endif  // RTT_INTERFACEDECLARATIONS_H
