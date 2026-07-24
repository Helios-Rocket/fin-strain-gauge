{
  description = "Rust flake for embedded dev";

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
      eachSystem = fn: nixpkgs.lib.genAttrs (import systems) (system: fn system (import nixpkgs { inherit system overlays; }));
    in {
      devShells = eachSystem (system: pkgs:
        let
          rust-esp = pkgs.callPackage ./nix/rust-esp.nix { };
          rust-src-esp = pkgs.callPackage ./nix/rust-src-esp.nix { };
          gcc-esp = pkgs.callPackage ./nix/gcc-esp.nix { };
        in {
        default = pkgs.mkShell {
          name = "embedded-rust";
          buildInputs = [
            (pkgs.fenix.combine [
              rust-esp
              rust-src-esp
            ])
            pkgs.probe-rs-tools
            pkgs.flip-link # embedded stack protection
            pkgs.rust-analyzer
            gcc-esp
          ];
          shellHook = ''
            export LD_LIBRARY_PATH="${pkgs.lib.makeLibraryPath [ pkgs.stdenv.cc.cc.lib ]}"
          '';
        };
      });
    };
}
