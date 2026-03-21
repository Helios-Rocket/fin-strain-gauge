{
  description = "Rust flake for embedded dev";

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
        esp = (pkgs.buildFHSEnv {
          name = "rust-esp";
          targetPkgs = pkgs: with pkgs; [
            # (pkgs.fenix.combine (with pkgs.fenix; [
            #   stable.defaultToolchain
            #   stable.llvm-tools
            #   targets.thumbv7em-none-eabihf.latest.rust-std
            # ]))
            rust-analyzer
            rustup
            cargo-generate
            cargo-binutils
            just
            just-lsp
            espup
            esp-generate
            espflash
            cargo-expand
            gcc
            libz
            gdb
          ];
          runScript = "bash";
          profile = ''
            . ~/export-esp.sh
          '';
        }).env;
        stm = pkgs.mkShell {
          name = "rust-stm";
          buildInputs = with pkgs; [
            (pkgs.fenix.combine (with pkgs.fenix; [
              stable.defaultToolchain
              targets.thumbv7em-none-eabihf.latest.rust-std
            ]))
            probe-rs-tools
            flip-link # embedded stack protection
            rust-analyzer
          ];
        };
      });
    };
}
