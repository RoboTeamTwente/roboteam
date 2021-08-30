//
// Created by Dawid Kulikowski on 09/08/2021.
//

#ifndef RTT_INTERFACEDECLARATIONS_H
#define RTT_INTERFACEDECLARATIONS_H

#include <roboteam_proto/UiOptions.pb.h>

#include <nlohmann/json.hpp>

#include "InterfaceDeclaration.h"
#include "InterfaceStateHandler.h"

namespace rtt::Interface {
class InterfaceDeclarations {
   private:
       std::vector<InterfaceDeclaration> decls = {};

       std::vector<InterfaceDeclaration> delta = {};

       mutable std::mutex mtx;

       std::weak_ptr<InterfaceStateHandler> stateHandler;

       void _unsafeAddDeclaration(InterfaceDeclaration);
       void _unsafeRemoveDeclaration(std::string);

       std::optional<InterfaceDeclaration> _unsafeGetDeclaration(std::string) const;

   public:
       InterfaceDeclarations(const nlohmann::json&);
       InterfaceDeclarations(std::weak_ptr<InterfaceStateHandler> sts): stateHandler(sts) {}

       explicit InterfaceDeclarations(const std::vector<InterfaceDeclaration> vec, std::weak_ptr<InterfaceStateHandler> sts) : decls(vec), stateHandler(sts) {};

       void addDeclaration(InterfaceDeclaration);
       void removeDeclaration(std::string);

       void addDeclarationToDelta(InterfaceDeclaration);
       std::vector<InterfaceDeclaration> getDeclarationDelta();
       bool doesHaveDelta();
       void mergeDeclarationDelta();

       std::vector<InterfaceDeclaration> getDeclarations() const;
       std::optional<InterfaceDeclaration> getDeclaration(std::string) const;

       void handleData(const proto::UiOptionDeclarations&);

       proto::UiOptionDeclarations toProto() const;
};

}
#endif  // RTT_INTERFACEDECLARATIONS_H
