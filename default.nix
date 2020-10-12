{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
, wic_sdk ? import ./wic_sdk.nix { pkgs = pkgs; }
, ebus_sdk ? wic_sdk.ebus_sdk
}:
with pkgs;
callPackage ./thermocam.nix {
  wic_sdk = wic_sdk;
  ebus_sdk = ebus_sdk;
  opencv = (opencv4.override { enableGtk3 = true; }).overrideAttrs(oldAttrs: rec {
    version = "d5dce632544f333306b2bf105d57a59c52054955";
    src = fetchFromGitHub {
      owner  = "opencv";
      repo   = "opencv";
      rev    = version;
      sha256 = "1z4kgy9nvg0x50rivzf0farr878vjl2s8n6hkxzkziq0xqlw2l3x";
    };
  });
}
