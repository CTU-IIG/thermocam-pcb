{ stdenv
, lib
, opencv
, meson
, ninja
, pkg-config
, boost
, zlib
, with_wic ? true, wic_sdk, ebus_sdk, tbb
, usbutils
, nix-gitignore
, systemd
, nlohmann_json
, debug ? false
}:
stdenv.mkDerivation {
  name = "thermocam-pcb";
  src =
    if builtins.pathExists ./.git then
      builtins.fetchGit { url = ./.; }
    else
      nix-gitignore.gitignoreSource [ ] ./.;
  nativeBuildInputs = [ meson ninja pkg-config ];
  buildInputs = [
    boost
    opencv
    # for usbreset in systemd .service file
    usbutils
    systemd
    nlohmann_json
    zlib.dev
  ] ++ lib.optionals with_wic [
    ebus_sdk
    wic_sdk
    tbb
  ];
  preConfigure = ''
    patchShebangs file2cpp
  '';
  mesonFlags = lib.optionals with_wic [
    "-Dwic_home=${wic_sdk}"
    "-Debus_home=${ebus_sdk}"
  ] ++ lib.lists.optional debug [
    "--buildtype=debugoptimized"
    "-Db_sanitize=address,undefined"
  ];
  dontStrip = debug;

  # Meson is no longer able to pick up Boost automatically.
  # https://github.com/NixOS/nixpkgs/issues/86131
  BOOST_INCLUDEDIR = "${lib.getDev boost}/include";
  BOOST_LIBRARYDIR = "${lib.getLib boost}/lib";

  shellHook = lib.optionalString with_wic ''
    export PUREGEV_ROOT="${ebus_sdk}";
    export GENICAM_ROOT=$PUREGEV_ROOT/lib/genicam
    export GENICAM_ROOT_V2_4=$GENICAM_ROOT
    export GENICAM_LOG_CONFIG=$GENICAM_ROOT/log/config/DefaultLogging.properties
    export GENICAM_LOG_CONFIG_V2_4=$GENICAM_LOG_CONFIG
    export GENICAM_CACHE_V2_4=$HOME/.config/Pleora/genicam_cache_v2_4
    export GENICAM_CACHE=$GENICAM_CACHE_V2_4
    export GENICAM_LIB_DIR=$GENICAM_ROOT/bin/Linux64_x64
    mkdir -p "$GENICAM_CACHE" || :
    export GENICAM_ROOT_V3_0=$GENICAM_ROOT
  '';
}
