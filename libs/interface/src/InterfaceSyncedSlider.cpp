//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedSlider.h"

namespace rtt::Interface {
    InterfaceSyncedSlider::InterfaceSyncedSlider(const MainWindow* window, std::weak_ptr<InterfaceControllerClient> ctrl, std::string ident, QWidget *parent): QSlider(parent), identity(ident){
        QObject::connect(window, &MainWindow::declarationsChanged, this, &InterfaceSyncedSlider::updateDeclaration);
        QObject::connect(window, &MainWindow::valuesChanged, this, &InterfaceSyncedSlider::updateValue);

        QObject::connect(this, &InterfaceSyncedSlider::valueChanged, this, &InterfaceSyncedSlider::notifyChangedValue);

        this->setTracking(false);
        this->setOrientation(Qt::Horizontal);

        this->ctrl = ctrl;
    }

    void InterfaceSyncedSlider::updateValue(std::weak_ptr<InterfaceSettings> valuesPtr) {
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

    void InterfaceSyncedSlider::updateDeclaration(std::weak_ptr<InterfaceDeclarations> declPtr) {
        auto allDecls = declPtr.lock();

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

        auto values = controller->getValues().lock();
        if (!values) return;

        float fval = (float)value * (float)dpi;
        values->setSetting(this->identity, InterfaceValue(fval));
    }
}