{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs?rev=c57e0a6ef5afc353293d169cfc618b75d31da98b";
    flake-utils.url = "github:numtide/flake-utils?rev=5aed5285a952e0b949eb3ba02c12fa4fcfef535f";
  };
  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = nixpkgs.legacyPackages.${system};
      in
      {
        devShells.default = pkgs.mkShell {
          buildInputs = with pkgs;[
            nixpkgs-fmt
            conan
            gcc10
            clang-tools
            ninja
            cmake
            valgrind
            ccache
            cppcheck
            lcov
          ];
        };
      }
    );
}
