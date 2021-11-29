{ stdenv, autoPatchelfHook, requireFile, libjpeg, udev, libcurl, qt4, unzip }:
stdenv.mkDerivation rec {
  pname = "wic_sdk";
  version = "2.0";
  src = requireFile rec {
    name = "WIC_SDK_Linux.zip";
    sha256 = "0ygkcg94rdmflhy5ggim24wlgzv0q5342ycdxd0gza48zsfjj2i9";
    message = ''
        Unfortunately, we cannot download file ${name} automatically.
        Please download it from https://software.workswell.eu/wic_sdk/Linux/
        and add it to the Nix store manually using:
          nix-store --add-fixed sha256 ${name}
      '';
  };
  outputs = [ "out" "ebus_sdk" ];
  nativeBuildInputs = [ autoPatchelfHook unzip ];
  buildInputs = [ libjpeg stdenv.cc.cc.lib udev libcurl qt4 ];

  buildCommand = ''
    unzip -q $src  # unzips to wicsdk2-x64-linux/

    # Move eBUS SDK out of the WIC SDK directory
    mv wicsdk2-x64-linux/eBUS_SDK_Ubuntu-x86_64-5.1.10-4642.deb .

    # Install WIC SDK
    mkdir -p $out
    mv wicsdk2-x64-linux/* $out

    # Install eBUS SDK
    ar x eBUS_SDK_Ubuntu-x86_64-5.1.10-4642.deb
    mkdir ebus_sdk
    tar -xf data.tar.gz -C ebus_sdk
    mv ebus_sdk/opt/pleora/ebus_sdk/Ubuntu-x86_64 $ebus_sdk/

    addAutoPatchelfSearchPath $ebus_sdk/lib
    addAutoPatchelfSearchPath $ebus_sdk/lib/genicam/bin/Linux64_x64
    autoPatchelf $out
    autoPatchelf $ebus_sdk
  '';
}
