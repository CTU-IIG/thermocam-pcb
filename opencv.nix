{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
}:
with pkgs;
opencv4.override { enableGtk3 = true; enableFfmpeg = true; }
