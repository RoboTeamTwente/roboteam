//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedSlider.h"

namespace rtt::Interface {
    InterfaceSyncedSlider::InterfaceSyncedSlider(std::weak_ptr<InterfaceControllerClient> ctrl, std::string ident, QWidget *parent): ctrl(std::move(ctrl)), QSlider(parent), identity(ident){
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedSlider::updateDeclaration);
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceSyncedSlider::updateValue);

        QObject::connect(this, &InterfaceSyncedSlider::valueChanged, this, &InterfaceSyncedSlider::notifyChangedValue);

        this->setTracking(false);
        this->setOrientation(Qt::Horizontal);
    }

    void InterfaceSyncedSlider::updateValue() {
        auto ctrlValid = this->ctrl.lock();
        auto valuesPtr = ctrlValid.get()->getValues();
        if (auto settings = valuesPtr.lock()) {

            auto newValue = settings->getSetting(this->identity);
            if (!newValue.has_value()) return;

            try {
                auto floatValue = std::get<float>(newValue.value().variant);

                this->setValue(floatValue);
            }
            catch (const std::bad_variant_access& ex) {
                std::cout << ex.what() << '\n';
            }

        }
    }

    void InterfaceSyncedSlider::updateDeclaration() {
        auto ctrlValid = this->ctrl.lock();
        auto allDecls = ctrlValid.get()->getDeclarations().lock();

        if (!allDecls) return;

        auto thisDecl = allDecls->getDeclaration(this->identity);
        if (!thisDecl.has_value()) return;

        this->updateProps(thisDecl.value());
    }

    void InterfaceSyncedSlider::updateProps(const InterfaceDeclaration& gdecl) {
        if (!gdecl.isMutable) this->setEnabled(false);

        try {
            auto decl = std::get<InterfaceSlider>(gdecl.options);

            this->dpi = (decl.dpi % 10) == 0 ? decl.dpi : 1;

            this->setMaximum(decl.max * dpi);
            this->setMinimum(decl.min * dpi);
            this->setSingleStep(decl.interval);
        }
        catch (const std::bad_variant_access& ex) {
            std::cout << ex.what() << '\n';
        }

    }

    void InterfaceSyncedSlider::notifyChangedValue(int value) {
        auto controller = this->ctrl.lock();
        if (!controller) return;
        controller->markForUpdate();

        auto values = controller->getValues().lock();
        if (!values) return;

        float fval = (float)value * (float)dpi;
        values->setSetting(this->identity, InterfaceValue(fval));
    }
}