{ sources ? import ./nix/sources.nix
, pkgs ? import sources.nixpkgs { }
}:
with pkgs;
(opencv4.override { enableGtk3 = true; enableFfmpeg = true; }).overrideAttrs(oldAttrs: rec {
    version = "d986cc4861b978415fc20c3a0dc6f16ff9d0bcdf";
    src = fetchFromGitHub {
      owner  = "opencv";
      repo   = "opencv";
      rev    = version;
      sha256 = "1yaw4gz4zn4bcjhyv8knl490zvmy2pylnkz3pmvmc700pgwgy79a";
    };
})
