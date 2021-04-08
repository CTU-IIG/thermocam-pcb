{ stdenv, autoPatchelfHook, requireFile, libjpeg, udev, libcurl, qt4 }:
stdenv.mkDerivation rec {
  pname = "wic_sdk";
  version = "1.2.1";
  src = requireFile rec {
    name = "wic-sdk-${version}_Ubuntu16.06-x86_64_installer.run";
    sha256 = "97df553ad47f84e1803d5eafaf50d39f5e0d896751941b67e1a541fb0bc8c763";
    message = ''
        Unfortunately, we cannot download file ${name} automatically.
        Please add it to the Nix store manually using:
          nix-store --add-fixed sha256 ${name}
      '';
  };
  outputs = [ "out" "ebus_sdk" ];
  nativeBuildInputs = [ autoPatchelfHook ];
  buildInputs = [ libjpeg stdenv.cc.cc.lib udev libcurl qt4 ];

  buildCommand = ''
    sh $src --target tmp --noexec
    cd tmp

    mkdir -p $out
    tar -xf install_files.tar -C $out

    ar x eBUS_SDK_x86_64-5.1.10-4642.deb
    mkdir ebus_sdk
    tar -xf data.tar.gz -C ebus_sdk
    mv ebus_sdk/opt/pleora/ebus_sdk/Ubuntu-x86_64 $ebus_sdk/

    addAutoPatchelfSearchPath $ebus_sdk/lib
    addAutoPatchelfSearchPath $ebus_sdk/lib/genicam/bin/Linux64_x64
    autoPatchelf $out
    autoPatchelf $ebus_sdk
  '';
}
