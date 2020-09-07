{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
, wic_sdk ? import ./wic_sdk.nix { pkgs = pkgs; }
, ebus_sdk ? wic_sdk.ebus_sdk
}:
with pkgs;
callPackage ./thermocam.nix {
  wic_sdk = wic_sdk;
  ebus_sdk = ebus_sdk;
  opencv = opencv2.override { enableGtk2 = true; };
}
