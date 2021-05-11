{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
, debug ? false # enable with `nix-build --arg debug true`
, with_wic ? true
}:
with pkgs;
let
  libjpeg = callPackage ./libjpeg.nix {};
  libcurl = callPackage ./libcurl-gnutls-stub.nix {};
  wic_sdk = callPackage ./wic_sdk.nix { inherit libjpeg libcurl; };
  ebus_sdk = wic_sdk.ebus_sdk;
  opencv = callPackage ./opencv.nix {};
in
callPackage ./thermocam.nix {
  inherit wic_sdk ebus_sdk debug opencv with_wic;
}
