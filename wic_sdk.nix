{ pkgs ? import <nixpkgs> {}
}:
with pkgs;
stdenv.mkDerivation rec {
  pname = "wic_sdk";
  version = "1.1.0";
  src = requireFile rec {
    name = "WIC_SDK-Linux_Ubuntu_16.04_64b-1.1.0.run";
    sha256 = "b97103de50ad81e3e664bd6b9f6090751f729e812630821e380a251c3108c2cc";
    message = ''
        Unfortunately, we cannot download file ${name} automatically.
        Please add it to the Nix store manually using:
          nix-store --add-fixed sha256 ${name}
      '';
  };
  outputs = [ "out" "ebus_sdk" ];
  buildCommand = ''
    sh $src --target tmp --noexec
    cd tmp

    mkdir -p $out
    tar -xf install_files.tar -C $out

    ar x eBUS_SDK_x86_64.deb
    mkdir ebus_sdk
    tar -xf data.tar.gz -C ebus_sdk
    mv ebus_sdk/opt/pleora/ebus_sdk/Ubuntu-x86_64 $ebus_sdk/
  '';
}
