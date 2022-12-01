#!/bin/bash

cp ./formatting/pre-commit ./.git/hooks/pre-commit
chmod 755 ./.git/hooks/pre-commit

for submodule in $(grep path .gitmodules | sed 's/.*= //') 
do
    cp formatting/pre-commit ./.git/modules/$submodule/hooks/pre-commit
    chmod 755 ./.git/modules/$submodule/hooks/pre-commit
done

cp ./formatting/roboteam_clang-format.file ~/.config/roboteam_clang-format.file

if hash clang-format 2>/dev/null; then 
    echo "clang-format has been properly set up";
else
    echo "The files have been set up, however clang-format was not found"
    echo "Install through:"
    echo "  # apt install clang-format"
    echo "  $ yay -S clang-format"
    echo "  $ brew install clang-format"
fi
