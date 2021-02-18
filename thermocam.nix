{ stdenv, lib, opencv, meson, ninja, pkg-config, boost, wic_sdk
, ebus_sdk, usbutils, nix-gitignore, systemd
}:
stdenv.mkDerivation {
  name = "thermocam-pcb";
  src =  if builtins.pathExists ./.git then
    builtins.fetchGit { url = ./.; }
  else
    nix-gitignore.gitignoreSource [] ./.;
  nativeBuildInputs = [ meson ninja pkg-config ];
  buildInputs = [
    boost
    ebus_sdk
    opencv
    wic_sdk
    # for usbreset in systemd .service file
    usbutils
    systemd
  ];
  mesonFlags = [ "-Dwic_home=${wic_sdk}" "-Debus_home=${ebus_sdk}" ];

  # Meson is no longer able to pick up Boost automatically.
  # https://github.com/NixOS/nixpkgs/issues/86131
  BOOST_INCLUDEDIR = "${stdenv.lib.getDev boost}/include";
  BOOST_LIBRARYDIR = "${stdenv.lib.getLib boost}/lib";
}
