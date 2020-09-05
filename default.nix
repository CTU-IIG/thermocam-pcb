{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
, wic_sdk ? /opt/workswell/wic_sdk
, ebus_sdk ? /opt/pleora/ebus_sdk/Ubuntu-x86_64
}:
with pkgs;
callPackage ./thermocam.nix {
  wic_sdk = wic_sdk;
  ebus_sdk = ebus_sdk;
  opencv = opencv2.override { enableGtk2 = true; };
}
