//
// Created by Dawid Kulikowski on 27/03/2022.
//

#include "InterfaceSyncedCheckableButton.h"

namespace rtt::Interface {

    InterfaceSyncedCheckableButton::InterfaceSyncedCheckableButton(std::weak_ptr<InterfaceControllerClient> ctrl, std::string ident, QWidget* parent): QPushButton(parent), ctrl(std::move(ctrl)), identity(ident) {
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedCheckableButton::updateValue);
        QObject::connect(this, &QPushButton::toggled, this, &InterfaceSyncedCheckableButton::didCheck);

        this->setCheckable(true);
        this->setFlat(true);
        this->setStyleSheet("QPushButton:checked { background-color: green; border: none;}");
    }

    void InterfaceSyncedCheckableButton::updateValue() {
        auto cptr = ctrl.lock();

        if (auto vals = cptr.get()->getValues().lock()) {
            if(vals->getSetting(this->identity) == InterfaceValue(true)) {
                this->setChecked(true);
            } else {
                this->setChecked(false);
            }
        }

        if (auto decls = cptr->getDeclarations().lock()) {
            auto self = decls->getDeclaration(this->identity);

            if (!self.has_value()) return;

            this->setEnabled(self->isMutable);
        }
    }

    void InterfaceSyncedCheckableButton::didCheck(bool checked) {
        if(auto ctrl = this->ctrl.lock()) {
            ctrl->markForUpdate();
            if (auto vals = ctrl->getValues().lock()) {
                vals->setSetting(this->identity, checked);
            }
        }
    }
}