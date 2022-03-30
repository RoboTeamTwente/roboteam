//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedDropdown.h"

namespace rtt::Interface {
    InterfaceSyncedDropdown::InterfaceSyncedDropdown(const MainWindow* window, std::weak_ptr<InterfaceControllerClient> ctrlptr, std::string ident, QWidget* parent): QComboBox(parent), identity(ident), ctrl(ctrlptr) {
        QObject::connect(window, &MainWindow::declarationsChanged, this, &InterfaceSyncedDropdown::updateDeclaration);
        QObject::connect(window, &MainWindow::valuesChanged, this, &InterfaceSyncedDropdown::updateValue);

        QObject::connect(this, &InterfaceSyncedDropdown::currentTextChanged, this, &InterfaceSyncedDropdown::didChangeValue);
    }

    void InterfaceSyncedDropdown::didChangeValue(const QString& newText) {
        auto iface = this->ctrl.lock();
        if (!iface) return;

        auto sptr = iface.get()->getValues().lock();
        if (!sptr) return;

        sptr.get()->setSetting(this->identity, InterfaceValue((int64_t)this->currentIndex()));
    }

    void InterfaceSyncedDropdown::updateProps(const InterfaceDeclaration& decl) {
        for (int i = 0; i < this->count(); i++) {
            this->removeItem(i);
        }

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
    void InterfaceSyncedDropdown::updateDeclaration(std::weak_ptr<InterfaceDeclarations> declptr) {
        auto allDecls = declptr.lock();

        if (!allDecls) return;

        auto self = allDecls.get()->getDeclaration(this->identity);
        if (!self.has_value()) return;

        this->updateProps(self.value());
    }

    void InterfaceSyncedDropdown::updateValue(std::weak_ptr<InterfaceSettings> sptr) {
        auto allSettings = sptr.lock();
        if (!allSettings) return;

        auto thisSettings = allSettings.get()->getSetting(this->identity);

        if (!thisSettings.has_value()) return;

        try {
            auto newIndex = std::get<int64_t>(thisSettings.value().variant);
            this->setCurrentIndex(newIndex);
        } catch (const std::bad_variant_access& ex) {
            std::cout << ex.what() << '\n';
        }
    }
}
