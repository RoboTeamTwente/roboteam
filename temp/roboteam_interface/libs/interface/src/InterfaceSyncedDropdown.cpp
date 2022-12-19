//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedDropdown.h"

namespace rtt::Interface {
    InterfaceSyncedDropdown::InterfaceSyncedDropdown(std::weak_ptr<InterfaceControllerClient> ctrlptr, std::string ident, QWidget* parent): QComboBox(parent), identity(ident), ctrl(ctrlptr) {
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedDropdown::updateDeclaration);
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedDropdown::updateValue);

        QObject::connect(this, &InterfaceSyncedDropdown::currentTextChanged, this, &InterfaceSyncedDropdown::didChangeValue);
    }

    void InterfaceSyncedDropdown::didChangeValue(const QString& newText) {
        auto iface = this->ctrl.lock();
        if (!iface) return;
        iface->markForUpdate();

        auto sptr = iface.get()->getValues().lock();
        if (!sptr) return;

        sptr.get()->setSetting(this->identity, InterfaceValue(this->currentText().toStdString()));
    }

    void InterfaceSyncedDropdown::updateProps(const InterfaceDeclaration& decl) {
        this->clear();

        if (!decl.isMutable) this->setEnabled(false);

        try {
            auto dropdown = std::get<InterfaceDropdown>(decl.options);

            for (const auto& option: dropdown.options) {
                this->addItem(QString::fromStdString(option));
            }
        } catch (const std::bad_variant_access& ex) {
            std::cout << ex.what() << '\n';
        }
    }
    void InterfaceSyncedDropdown::updateDeclaration() {
        auto cptr = ctrl.lock();
        auto declptr = cptr.get()->getDeclarations();
        auto allDecls = declptr.lock();

        if (!allDecls) return;

        std::vector<std::string> current_options;

        for (int i = 0; i < this->count(); i++) {
            current_options.push_back(this->itemText(i).toStdString());
        }

        auto us = allDecls->getDeclaration(this->identity);
        if (!us) return;

        if (current_options == std::get<InterfaceDropdown>(us.value().options).options) {
            return;
        }

        auto self = allDecls->getDeclaration(this->identity);
        if (!self.has_value()) return;

        this->updateProps(self.value());
    }

    void InterfaceSyncedDropdown::updateValue() {
        auto cptr = ctrl.lock();
        auto sptr = cptr.get()->getValues();
        auto allSettings = sptr.lock();
        if (!allSettings) return;

        auto thisSettings = allSettings.get()->getSetting(this->identity);

        if (!thisSettings.has_value()) return;

        try {
            auto newText = std::get<std::string>(thisSettings.value().variant);
            this->setCurrentText(QString::fromStdString(newText));
        } catch (const std::bad_variant_access& ex) {
            std::cout << ex.what() << '\n';
        }
    }
}
