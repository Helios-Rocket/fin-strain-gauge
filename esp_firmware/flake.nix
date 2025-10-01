{
  description = "Rust flake for teensy dev";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    systems.url = "github:nix-systems/default";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
    # rust-esp = {
    #   url = "path:./rust-esp";
    #   inputs.nixpkgs.follows = "nixpkgs";
    # };
  };

  outputs = { nixpkgs, systems, fenix, ... }:
    let
      overlays = [ fenix.overlays.default ];
      eachSystem = fn: nixpkgs.lib.genAttrs (import systems) (system: fn system (import nixpkgs { inherit system overlays; }));
    in {
      devShells = eachSystem (system: pkgs: {
        default = (pkgs.buildFHSEnv {
          name = "rust-teensy";
          targetPkgs = pkgs: with pkgs; [
            # (pkgs.fenix.combine (with pkgs.fenix; [
            #   stable.defaultToolchain
            #   stable.llvm-tools
            # ]))
            rust-analyzer
            rustup
            cargo-generate
            cargo-binutils
            just
            just-lsp
            espup
            esp-generate
            cargo-espflash
            probe-rs
            gcc
            libz
          ];
          runScript = "bash";
          profile = ''
            . ~/export-esp.sh
          '';
        }).env;
        # default = pkgs.mkShell {
        #   name = "rust-esp";
        #   buildInputs = with pkgs; [
        #     rust-esp.packages.${system}.rust-esp
        #     rust-esp.packages.${system}.xtensa-esp-elf
        #     rust-esp.packages.${system}.xtensa-esp32-elf-clang
        #     cargo-espflash
        #     esp-generate
        #   ];
        #   LIBCLANG_PATH = "${rust-esp.packages.${system}.xtensa-esp32-elf-clang}/lib";
        # };
      });
    };
}
