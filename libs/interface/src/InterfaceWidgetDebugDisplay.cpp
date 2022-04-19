//
// Created by Dawid Kulikowski on 27/03/2022.
//

#include "InterfaceWidgetDebugDisplay.h"
#include <QLayout>
#include <QApplication>
#include <QLabel>
#include <utility>
#include "InterfaceSyncedSlider.h"
#include "InterfaceSyncedCheckbox.h"
#include "InterfaceSyncedDropdown.h"
#include "InterfaceSyncedRadio.h"
#include "InterfaceSyncedText.h"

namespace rtt::Interface {

    InterfaceWidgetDebugDisplay::InterfaceWidgetDebugDisplay(std::weak_ptr<InterfaceControllerClient> ctrl, QWidget *parent): QWidget(parent), ctrl(std::move(ctrl)), window(window) {
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceWidgetDebugDisplay::valuesDidChange);
        QObject::connect(this->ctrl.lock().get(), &InterfaceControllerClient::refresh_trigger, this, &InterfaceWidgetDebugDisplay::valuesDidChange);
        this->setLayout(new QVBoxLayout);
        this->layout()->setAlignment(Qt::AlignTop);

    }

    void InterfaceWidgetDebugDisplay::valuesDidChange() {
        if (auto ctrl = this->ctrl.lock()) {
            if (auto decls = ctrl->getDeclarations().lock()) {
                auto debug_values = decls->getWithSuffix("_DEBUG");
                auto current_count = this->layout()->count();
                if (debug_values.size() * 2 != current_count) { // Take into consideration the label
                    this->fullRefresh(debug_values);
                }
            }
        }
    }

    void InterfaceWidgetDebugDisplay::fullRefresh(const std::vector<std::string>& items) {
        auto ctrl = this->ctrl.lock();
        if (!ctrl) return;
        auto decls = ctrl->getDeclarations().lock();
        if (!decls) return;

        while (auto item = this->layout()->takeAt(0)) {
            if (item->widget()) {
                this->layout()->removeWidget(item->widget());
                delete item->widget();
            }
            delete item;
        }

        for (const auto& key : items) {
            auto this_decl = decls->getDeclaration(key);
            this->layout()->addWidget(new QLabel(QString::fromStdString(this_decl->path)));
            if (const auto* slider = std::get_if<Interface::InterfaceSlider>(&this_decl->options)) {
                this->layout()->addWidget(new InterfaceSyncedSlider(this->ctrl, key));
            } else if (const auto* checkbox = std::get_if<InterfaceCheckbox>(&this_decl->options)) {
                this->layout()->addWidget(new InterfaceSyncedCheckbox(this->ctrl, key));
            } else if (const auto* dropdown = std::get_if<InterfaceDropdown>(&this_decl->options)) {
                this->layout()->addWidget(new InterfaceSyncedDropdown(this->ctrl, key));
            } else if (const auto* radio = std::get_if<InterfaceRadio>(&this_decl->options)) {
                auto tmpLayout = new QHBoxLayout;
                auto tmpWidget = new QWidget;
                auto radioButtons = new InterfaceSyncedRadio(this->ctrl, key);
                for (auto button : radioButtons->buttons()) {
                    tmpLayout->addWidget(button);
                }
                tmpWidget->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::MinimumExpanding);
                tmpWidget->setMinimumSize(40, 20);
                tmpWidget->setLayout(tmpLayout);
                this->layout()->addWidget(tmpWidget);
            } else if (const auto* text = std::get_if<InterfaceText>(&this_decl->options)) {
                this->layout()->addWidget(new InterfaceSyncedText(this->ctrl, key));
            } else {
                throw std::logic_error{"Variant was in an invalid state when serialising InterfaceDeclaration to JSON!"};
            }
        }
    }
}