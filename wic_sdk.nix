{ pkgs ? import <nixpkgs> {}
}:
with pkgs;
stdenv.mkDerivation rec {
  pname = "wic_sdk";
  version = "1.1.0";
  src = ./WIC_SDK-Linux_Ubuntu_16.04_64b-1.1.0.run;
  outputs = [ "out" "ebus_sdk" ];
  buildCommand = ''
    $src --target tmp --noexec
    cd tmp

    mkdir -p $out
    tar -xf install_files.tar -C $out

    ar x eBUS_SDK_x86_64.deb
    mkdir ebus_sdk
    tar -xf data.tar.gz -C ebus_sdk
    mv ebus_sdk/opt/pleora/ebus_sdk/Ubuntu-x86_64 $ebus_sdk/
  '';
}
