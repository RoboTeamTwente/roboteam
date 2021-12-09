//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedText.h"

namespace rtt::Interface {
    InterfaceSyncedText::InterfaceSyncedText(const MainWindow *window, std::shared_ptr<InterfaceController> ctrlptr, const InterfaceDeclaration &decl, QWidget *parent): QLineEdit(parent) {
        QObject::connect(window, &MainWindow::declarationsChanged, this, &InterfaceSyncedText::updateDeclaration);
        QObject::connect(window, &MainWindow::valuesChanged, this, &InterfaceSyncedText::updateValue);

        QObject::connect(this, &InterfaceSyncedText::textChanged, this, &InterfaceSyncedText::notifyChangedValue);

        this->identity = decl.path;
        this->ctrl = ctrlptr;

        updateProps(decl);
    }

    void InterfaceSyncedText::updateValue(std::weak_ptr<InterfaceSettings> settings) {
        auto vals = settings.lock();

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

    void InterfaceSyncedText::updateDeclaration(std::weak_ptr<InterfaceDeclarations> declPtr) {
        return;
    }

    void InterfaceSyncedText::notifyChangedValue(const QString &text) {
        auto interfaceCtrl = this->ctrl.lock();
        if (!interfaceCtrl) return;

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
