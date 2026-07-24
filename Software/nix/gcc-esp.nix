{ pkgs
, stdenv
, fetchurl
}:
let
  platform = {
    x86_64-linux = "x86_64-linux-gnu";
    aarch64-linux = "aarch64-linux-gnu";
    aarch64-darwin = "aarch64-apple-darwin";
  }.${stdenv.hostPlatform.system} or (throw "unsupported system ${stdenv.hostPlatform.system}");
in
stdenv.mkDerivation rec {
  pname = "gcc-esp";
  version = "16.1.0_20260609";

  src = fetchurl {
    url = "https://github.com/espressif/crosstool-NG/releases/download/esp-${version}/xtensa-esp-elf-${version}-${platform}.tar.gz";
    sha256 = {
      x86_64-linux = "sha256:680473ea7f6cd4d30f138a80f401fa8171c1572a76ced9a9453dec774a7b10ec";
      aarch64-linux = "sha256:c80bf180c4d2e6d76613599d266d1e70da49bf0369aaf3fb97286a7319168ded";
      aarch64-darwin = "sha256:648fa624e84ffa2d9e06c9a17dbc7338fbb643a64f9635acb608d763f1ea8013";
    }.${stdenv.hostPlatform.system};
  };

  nativeBuildInputs = with pkgs; [
    autoPatchelfHook
    gcc
    stdenv.cc.cc
    pkg-config
  ];

  outputs = [ "out" ];

  installPhase = ''
    mkdir -p $out
    cp -r ./* $out/
  '';
}
