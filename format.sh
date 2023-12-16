find -L . -not -path "./roboteam_autoref/*" -not -path "./roboteam/roboteam_robothub/roboteam_embedded_messages/*" | grep -E "^./roboteam.*?\.(c|cpp|h|hpp)$" --color=never | xargs clang-format -i -style="{BasedOnStyle : Google, IndentWidth : 4, ColumnLimit : 180}"