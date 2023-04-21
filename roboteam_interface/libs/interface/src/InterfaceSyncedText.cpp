//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedText.h"

namespace rtt::Interface {
    InterfaceSyncedText::InterfaceSyncedText(std::weak_ptr<InterfaceControllerClient> ctrlptr, std::string ident, QWidget *parent): ctrl(std::move(ctrlptr)), QLineEdit(parent), identity(ident) {
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedText::updateDeclaration);
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedText::updateValue);

        QObject::connect(this, &InterfaceSyncedText::textChanged, this, &InterfaceSyncedText::notifyChangedValue);

    }

    void InterfaceSyncedText::updateValue() {
        auto vals = this->ctrl.lock().get()->getValues().lock();

        if (!vals) return;

        auto thisVal = vals->getSetting(this->identity);

        if (!thisVal.has_value()) return;

        try {
            auto textValue = std::get<std::string>(thisVal.value().variant);
            this->setText(QString::fromStdString(textValue));
        } catch (std::bad_variant_access e) {
            std::cout << e.what() << std::endl;
        }
    }

    void InterfaceSyncedText::updateDeclaration() {
        return;
    }

    void InterfaceSyncedText::notifyChangedValue(const QString &text) {
        auto interfaceCtrl = this->ctrl.lock();
        if (!interfaceCtrl) return;
        interfaceCtrl->markForUpdate();

        auto valueCtrl = interfaceCtrl->getValues().lock();
        if (!valueCtrl) return;

        valueCtrl->setSetting(this->identity, InterfaceValue(this->text().toStdString()));
    }

    void InterfaceSyncedText::updateProps(const InterfaceDeclaration& decl) {
        try {
            this->setReadOnly(!decl.isMutable);
        } catch (std::bad_variant_access e) {
            std::cout << e.what() << std::endl;
        }
    }
}
