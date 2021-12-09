//
// Created by Dawid Kulikowski on 03/10/2021.
//

#include "InterfaceDeclarationsRenderer.h"


#include <QLabel>
#include <QLayout>
#include <QSplitter>
#include <QString>
#include <sstream>
#include <string>

#include "InterfaceSyncedCheckbox.h"
#include "InterfaceSyncedSlider.h"
#include "InterfaceSyncedDropdown.h"
#include "InterfaceSyncedText.h"
#include "InterfaceSyncedRadio.h"

namespace rtt::Interface {
    std::vector<std::string> InterfaceDeclarationsRenderer::split_path(const std::string path) const noexcept {
        std::vector<std::string> components = {};

        std::stringstream ss(path);
        std::string item;
        while(std::getline(ss, item, '/')) {
            if (item.empty()) continue;
            components.push_back(item);
        }

        return components;
    }

    InterfaceDeclarationTree<std::string, InterfaceDeclaration> InterfaceDeclarationsRenderer::convert_to_tree_type(const std::map<std::string, InterfaceDeclaration> & decls) const noexcept {
        auto result = std::make_shared<InterfaceDeclarationTree<std::string, InterfaceDeclaration>>();

        for (const auto [key, value]: decls) {
            const auto path = this->split_path(key);

            std::weak_ptr<InterfaceDeclarationTree<std::string, InterfaceDeclaration>> currentLevel = result;
            for (const auto& level : path) {
                currentLevel = currentLevel.lock()->insert_or_add(level);
            }

            currentLevel.lock()->set_data(value);
        }

        return *result.get();
    }
    void InterfaceDeclarationsRenderer::render(const MainWindow* window, std::weak_ptr<InterfaceController> ctrl, QWidget* layout) {
        if (auto controller = ctrl.lock()) {
            auto declController = controller->getDeclarations().lock();
            if (!declController) return;

            auto ret = this->convert_to_tree_type(declController.get()->getDeclarations());

            recursive_append_widget(window, ctrl, std::make_shared<InterfaceDeclarationTree<std::string, InterfaceDeclaration>>(ret), layout, 0, "/");
        }
    }

    void InterfaceDeclarationsRenderer::recursive_append_widget(const MainWindow* window, std::weak_ptr<InterfaceController> ctrl, std::shared_ptr<InterfaceDeclarationTree<std::string, InterfaceDeclaration>> decls, QWidget* widget, uint8_t current_depth, std::string branch) const {
        QWidget* next_widget = widget;

        switch (current_depth) {
            case 1:
                // TODO: Support toplevel elements
                next_widget = append_tab_view(window, branch, widget);
                break;
            case 2:
                next_widget = append_tab(window, branch, widget);
                break;
            default:
                break;
        }

        if (decls->get_keys().empty() && decls->get_data().has_value() && (current_depth < 1 || current_depth > 2)) {
            append_final_widget(window, ctrl, decls->get_data().value(), widget);
            return;
        }

        for (const auto& key : decls->get_keys()) {
            auto next_decls = decls->get(key);

            recursive_append_widget(window, ctrl, decls->get(key).value().lock(), next_widget, current_depth + 1, key);
        }
    }

    QWidget* InterfaceDeclarationsRenderer::append_tab_view(const MainWindow* window, const std::string& table_name, QWidget* parent) const {
        QTabWidget* tab_widget = new QTabWidget();
        QLabel* label = new QLabel();
        label->setText(QString::fromStdString(table_name));

        QWidget* table_component = new QWidget();
        table_component->setLayout(new QVBoxLayout());
        table_component->layout()->addWidget(label);
        table_component->layout()->addWidget(tab_widget);

        ((QSplitter*)parent)->addWidget(table_component);

        return tab_widget;
    }

    QWidget* InterfaceDeclarationsRenderer::append_tab(const MainWindow* window, const std::string& tab_name, QWidget* parent) const {
        QTabWidget* table = (QTabWidget*)(parent);

        QWidget* new_widget = new QWidget();
        new_widget->setLayout(new QVBoxLayout());

        table->addTab(new_widget, QString::fromStdString(tab_name));

        return new_widget;
    }
    void InterfaceDeclarationsRenderer::append_final_widget(const MainWindow* window, std::weak_ptr<InterfaceController> ctrl, const InterfaceDeclaration& decl, QWidget* parent) const {
        QWidget* end = nullptr;
        if (auto sharedController = ctrl.lock()) {
            if (const auto* slider = std::get_if<Interface::InterfaceSlider>(&decl.options)) {
                end = new InterfaceSyncedSlider(window, sharedController, decl);
            } else if (const auto* checkbox = std::get_if<InterfaceCheckbox>(&decl.options)) {
                end = new InterfaceSyncedCheckbox(window, sharedController, decl);
            } else if (const auto* dropdown = std::get_if<InterfaceDropdown>(&decl.options)) {
                end = new InterfaceSyncedDropdown(window, sharedController, decl);
            } else if (const auto* radio = std::get_if<InterfaceRadio>(&decl.options)) {
                end = new QWidget();
                new InterfaceSyncedRadio(window, sharedController, decl, end);
            } else if (const auto* text = std::get_if<InterfaceText>(&decl.options)) {
                end = new InterfaceSyncedText(window, sharedController, decl);
            } else {
                throw std::logic_error{"Variant was in an invalid state when serialising InterfaceDeclaration to JSON!"};
            }
        }

        if (end) {
            parent->layout()->addWidget(end);
        }
    }
}