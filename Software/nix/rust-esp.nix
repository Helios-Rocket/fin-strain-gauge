{ lib
, stdenv
, fetchurl
, zlib
}:
let
  platform = {
    x86_64-linux = "x86_64-unknown-linux-gnu";
    aarch64-linux = "aarch64-unknown-linux-gnu";
    aarch64-darwin = "aarch64-apple-darwin";
  }.${stdenv.hostPlatform.system} or (throw "unsupported system ${stdenv.hostPlatform.system}");
  rpath = "${zlib}/lib:$out/lib";
in
stdenv.mkDerivation rec {
  pname = "rust-esp";
  version = "1.97.0.0";

  src = fetchurl {
    url = "https://github.com/esp-rs/rust-build/releases/download/v${version}/rust-${version}-${platform}.tar.xz";
    sha256 = {
      aarch64-linux = "sha256:dc6efa8906849608497c2ffb3d6a9466588835a79b5e8886c229de133640945c";
      x86_64-linux = "sha256:a99bfee69221e9ff6d86388f6811ee688cd405e6a0400a3cd1784e8d463e9d99";
      aarch64-darwin = "sha256:430fcbf54967e99d16debe48926a1df558ab8b67af3c060a658d25ce752bd790";
    }.${stdenv.hostPlatform.system};
  };

  installPhase = ''
    patchShebangs install.sh
    CFG_DISABLE_LDCONFIG=1 ./install.sh --prefix=$out

    rm $out/lib/rustlib/{components,install.log,manifest-*,rust-installer-version,uninstall.sh} || true

    ${lib.optionalString stdenv.isLinux ''
      if [ -d $out/bin ]; then
        for file in $(find $out/bin -type f); do
          if isELF "$file"; then
            patchelf \
              --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
              --set-rpath ${rpath} \
              "$file" || true
          fi
        done
      fi

      if [ -d $out/lib ]; then
        for file in $(find $out/lib -type f); do
          if isELF "$file"; then
            patchelf --set-rpath ${rpath} "$file" || true
          fi
        done
      fi

      if [ -d $out/libexec ]; then
        for file in $(find $out/libexec -type f); do
          if isELF "$file"; then
            patchelf \
              --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
              --set-rpath ${rpath} \
              "$file" || true
          fi
        done
      fi

      for file in $(find $out/lib/rustlib/*/bin -type f); do
        if isELF "$file"; then
          patchelf \
            --set-interpreter ${stdenv.cc.bintools.dynamicLinker} \
            --set-rpath ${stdenv.cc.cc.lib}/lib:${rpath} \
            "$file" || true
        fi
      done
    ''}
  '';
  dontStrip = true;
}
