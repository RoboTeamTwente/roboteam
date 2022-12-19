//
// Created by Dawid Kulikowski on 03/10/2021.
//

#include "InterfaceSyncedCheckbox.h"
#include <variant>
#include "InterfaceControllerClient.h"

#include "roboteam_interface_utils/InterfaceDeclaration.h"

namespace rtt::Interface {
    InterfaceSyncedCheckbox::InterfaceSyncedCheckbox(std::weak_ptr<InterfaceControllerClient> ctrl, std::string ident, QWidget *parent): ctrl(std::move(ctrl)), QCheckBox(parent), identity(ident) {
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedCheckbox::updateDeclaration);
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedCheckbox::updateValue);

        QObject::connect(this, &InterfaceSyncedCheckbox::stateChanged, this, &InterfaceSyncedCheckbox::notifyChangedValue);
    }

    void InterfaceSyncedCheckbox::updateProps(const InterfaceDeclaration& decl) {
        if (!decl.isMutable) this->setCheckable(false);
    }

    void InterfaceSyncedCheckbox::updateValue() {
        auto cptr = ctrl.lock();
        auto sptr = cptr.get()->getValues();

        if (auto settings = sptr.lock()) {

            auto newValue = settings->getSetting(this->identity);
            if (!newValue.has_value()) return;

            try {
                auto booleanValue = std::get<bool>(newValue.value().variant);

                this->setChecked(booleanValue);
            }
            catch (const std::bad_variant_access& ex) {
                std::cout << ex.what() << '\n';
            }

        }
    }

    void InterfaceSyncedCheckbox::updateDeclaration() {
        auto cptr = ctrl.lock();
        auto decls = cptr.get()->getDeclarations().lock();

        if (!decls) return;

        auto self = decls.get()->getDeclaration(this->identity);

        if (!self.has_value()) return;

        this->updateProps(self.value());
    }

    void InterfaceSyncedCheckbox::notifyChangedValue(int state) {
        auto controller = this->ctrl.lock();
        if (!controller) return;

        auto values = controller->getValues().lock();
        if (!values) return;

        controller->markForUpdate();

        switch (state) {
            case Qt::CheckState::Unchecked:
                values->setSetting(this->identity, InterfaceValue(false));
                break;
            case Qt::CheckState::Checked:
                values->setSetting(this->identity, InterfaceValue(true));
                break;
            default:
                break;
        }
    }
}