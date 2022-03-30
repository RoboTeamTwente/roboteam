//
// Created by Dawid Kulikowski on 04/10/2021.
//

#include "InterfaceSyncedRadio.h"

namespace rtt::Interface {
InterfaceSyncedRadio::InterfaceSyncedRadio(const MainWindow * window, std::weak_ptr<InterfaceControllerClient> ctrlptr, std::string ident, QWidget *parent): QButtonGroup(parent), identity(ident) {
    QObject::connect(window, &MainWindow::declarationsChanged, this, &InterfaceSyncedRadio::updateDeclaration);
    QObject::connect(window, &MainWindow::valuesChanged, this, &InterfaceSyncedRadio::updateValue);

    QObject::connect(this, &InterfaceSyncedRadio::idToggled, this, &InterfaceSyncedRadio::notifyChangedValue);

    this->setExclusive(true);

    this->ctrl = ctrlptr;
}

void InterfaceSyncedRadio::updateProps(const InterfaceDeclaration &decl) {
    for (const auto& button :this->buttons()) {
        this->removeButton(button);
    }

    try {
        auto radio = std::get<InterfaceRadio>(decl.options);
        for (int i = 0; i < radio.options.size(); i++) {
            QRadioButton* tmpButton = new QRadioButton(QString::fromStdString(radio.options.at(i)));
            if (!decl.isMutable) {
                tmpButton->setCheckable(false);
            }

            this->addButton(tmpButton, i);
        }

        this->button(std::get<int64_t>(decl.defaultValue.variant))->setChecked(true);
    } catch (const std::bad_variant_access& ex) {
        std::cout << ex.what() << '\n';
    }


}
void InterfaceSyncedRadio::updateDeclaration(std::weak_ptr<InterfaceDeclarations> declptr) {
    auto alldecl = declptr.lock();
    if (!alldecl) return;

    auto decl = alldecl.get()->getDeclaration(this->identity);

    if (!decl.has_value()) return;

    this->updateProps(decl.value());
}

void InterfaceSyncedRadio::updateValue(std::weak_ptr<InterfaceSettings> settings) {
    auto values = settings.lock();
    if (!values) return;
    this->updateDeclaration(ctrl.lock()->getDeclarations());
    auto newVal = values.get()->getSetting(this->identity);
    if (!newVal.has_value()) return;

    try {
        auto newButtonSelect = std::get<int64_t>(newVal.value().variant);
        auto current = this->checkedId();
        if (this->checkedButton()) {
            this->checkedButton()->setChecked(false);
        }
        this->button(newButtonSelect)->setChecked(true);

    } catch (const std::bad_variant_access& ex) {
        std::cout << ex.what() << '\n';
    }



}
void InterfaceSyncedRadio::notifyChangedValue(int id, bool enabled) {
    if (!enabled) return;

    auto controller = this->ctrl.lock();
    if (!controller) return;

    auto allVals = controller.get()->getValues().lock();
    if (!allVals) return;

    allVals->setSetting(this->identity, InterfaceValue((int64_t)id));
}

}