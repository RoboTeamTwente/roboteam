//
// Created by Dawid Kulikowski on 03/10/2021.
//

#include "InterfaceSyncedCheckbox.h"
#include <variant>

#include "InterfaceDeclaration.h"

namespace rtt::Interface {
    InterfaceSyncedCheckbox::InterfaceSyncedCheckbox(const MainWindow* window, std::shared_ptr<InterfaceController> ctrl, const InterfaceDeclaration &decl, QWidget *parent): QCheckBox(parent){
        QObject::connect(window, &MainWindow::declarationsChanged, this, &InterfaceSyncedCheckbox::updateDeclaration);
        QObject::connect(window, &MainWindow::valuesChanged, this, &InterfaceSyncedCheckbox::updateValue);

        QObject::connect(this, &InterfaceSyncedCheckbox::stateChanged, this, &InterfaceSyncedCheckbox::notifyChangedValue);
        this->identity = decl.path;

        this->ctrl = ctrl;

        this->updateProps(decl);
    }

    void InterfaceSyncedCheckbox::updateProps(const InterfaceDeclaration& decl) {
        if (!decl.isMutable) this->setCheckable(false);
    }

    void InterfaceSyncedCheckbox::updateValue(std::weak_ptr<InterfaceSettings> valuesPtr) {
        if (auto settings = valuesPtr.lock()) {

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

    void InterfaceSyncedCheckbox::updateDeclaration(std::weak_ptr<InterfaceDeclarations> declPtr) {
        auto decls = declPtr.lock();
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