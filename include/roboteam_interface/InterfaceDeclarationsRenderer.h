//
// Created by Dawid Kulikowski on 03/10/2021.
//

#ifndef RTT_INTERFACEDECLARATIONSRENDERER_H
#define RTT_INTERFACEDECLARATIONSRENDERER_H

#include <QWidget>
#include <optional>
#include "InterfaceDeclarations.h"
#include "MainWindow.h"

namespace rtt::Interface {
    template<typename Iter, typename Stor>
    class InterfaceDeclarationTree {
    private:
        std::optional<Stor> data = std::nullopt;
        std::map<Iter, std::shared_ptr<InterfaceDeclarationTree<Iter, Stor>>> tree = {};
    public:
        InterfaceDeclarationTree() {};
        InterfaceDeclarationTree(Stor pdata): data(pdata) {};

        void join(Iter at, InterfaceDeclarationTree<Iter, Stor> next) noexcept {
            tree.insert_or_assign(at, next);
        }

        std::weak_ptr<InterfaceDeclarationTree<Iter, Stor>> insert_or_add(Iter at) noexcept {
            if (has_key(at)) {
                return tree.at(at);
            } else {
                return add_empty(at);
            }
        }

        std::weak_ptr<InterfaceDeclarationTree<Iter, Stor>> insert(Iter at, Stor next) noexcept {
            add_empty(at);

            tree.at(at).set_data(next);

            return tree.at(at);
        }

        std::weak_ptr<InterfaceDeclarationTree<Iter, Stor>> add_empty(Iter at) noexcept {
            tree.insert_or_assign(at, std::make_shared<InterfaceDeclarationTree>());
            return tree.at(at);
        }

        std::optional<Stor> get_data() const noexcept {
            return data;
        }

        void set_data(Stor dat) noexcept {
            this->data = dat;
        }
        bool has_key(Iter key) const noexcept {
            return tree.contains(key);
        }

        std::vector<Iter> get_keys() const {
            std::vector<std::string> keys;
            std::for_each(tree.begin(), tree.end(), [&](const auto& elem) {keys.push_back(elem.first);});

            return keys;
        }

        std::optional<std::weak_ptr<InterfaceDeclarationTree<Iter, Stor>>> get(Iter key) const noexcept {
            if (has_key(key)) {
                return std::make_optional(tree.at(key));
            }

            return std::nullopt;
        }
    };

    class InterfaceDeclarationsRenderer {
    public:
        void render(const MainWindow*, const std::weak_ptr<InterfaceController>,  QWidget*);
    private:
        std::vector<std::string> split_path(const std::string path) const noexcept;
        InterfaceDeclarationTree<std::string, InterfaceDeclaration> convert_to_tree_type(const std::map<std::string, InterfaceDeclaration>&) const noexcept;

        QWidget* append_tab(const MainWindow*, const std::string&, QWidget*) const;
        QWidget* append_tab_view(const MainWindow*, const std::string&, QWidget*) const;
        void append_final_widget(const MainWindow*, std::weak_ptr<InterfaceController>, const InterfaceDeclaration&, QWidget*) const;


        void recursive_append_widget(const MainWindow*, std::weak_ptr<InterfaceController>, std::shared_ptr<InterfaceDeclarationTree<std::string, InterfaceDeclaration>>, QWidget*, uint8_t, std::string branch) const;
    };
}


#endif  // RTT_INTERFACEDECLARATIONSRENDERER_H
