{
  description = "Rust flake for teensy dev";

  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs?ref=nixos-unstable";
    systems.url = "github:nix-systems/default";
    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
    };
  };

  outputs = { nixpkgs, systems, fenix, ... }:
    let
      overlays = [ fenix.overlays.default ];
      eachSystem = fn: nixpkgs.lib.genAttrs (import systems) (system: fn (import nixpkgs { inherit system overlays; }));
    in {
      devShells = eachSystem (pkgs: {
        default = pkgs.mkShell {
          name = "rust-teensy";
          buildInputs = with pkgs; [
            (pkgs.fenix.combine (with pkgs.fenix; [
              stable.defaultToolchain
              stable.llvm-tools
              targets.thumbv7em-none-eabihf.stable.rust-std
            ]))
            rust-analyzer
            cargo-generate
            cargo-binutils
            just
            just-lsp
            teensy-loader-cli
            minicom
          ];
        };
      });
    };
}
