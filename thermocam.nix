{ stdenv, lib, makeWrapper, opencv, meson, ninja, pkg-config, boost, wic_sdk
, ebus_sdk, udev, writeText, usbutils, nix-gitignore }:
let
  # libPvGenICam.so from ebus_sdk links to libcurl-gnutls.so.4 and
  # expect it to be compiled with versioned symbols. Nix does not use
  # versioned symbols in this library so we cannot use nix-provided
  # library. We also don't want to change curl derivation to use
  # versioned symbols because nix fetch* functions depend on curl and
  # the change causes rebuild of almost everything. Since we do not
  # need curl-related functionality, we replace this library with a
  # stub of empty functions.
  libcurl-gnutls-stub = stdenv.mkDerivation {
    name = "libcurl-gnutls-stub";
    src = writeText "libcurl-gnutls.c" ''
      #define STUB(x) \
      	void x() {} \
      	__asm__(".symver " #x "," #x "@CURL_GNUTLS_3")

      STUB(curl_easy_init);
      STUB(curl_easy_strerror);
      STUB(curl_easy_perform);
      STUB(curl_easy_setopt);
      STUB(curl_easy_cleanup);
          '';
    vers = writeText "libcurl-gnutls.vers" "CURL_GNUTLS_3 {};";
    buildCommand = ''
      mkdir -p $out/lib
      gcc -fPIC -shared -Wl,--version-script=$vers -Wl,--soname='libcurl-gnutls.so.4' -o $out/lib/libcurl-gnutls.so.4 $src
    '';

  };
  ldLibraryPath = lib.strings.makeLibraryPath [ udev libcurl-gnutls-stub ];
in stdenv.mkDerivation {
  name = "thermocam-pcb";
  src =  if builtins.pathExists ./.git then
    builtins.fetchGit { url = ./.; }
  else
    nix-gitignore.gitignoreSource [] ./.;
  nativeBuildInputs = [ meson ninja pkg-config makeWrapper ];
  buildInputs = [
    boost
    ebus_sdk
    opencv
    wic_sdk
    # Needed by ebus_sdk
    libcurl-gnutls-stub
    udev
    # for usbreset in systemd .service file
    usbutils
  ];
  mesonFlags = [ "-Dwic_home=${wic_sdk}" "-Debus_home=${ebus_sdk}" ];

  LD_LIBRARY_PATH = ldLibraryPath;

  postFixup = ''
    wrapProgram $out/bin/thermocam-pcb --prefix LD_LIBRARY_PATH : "${ldLibraryPath}"
  '';

  # Meson is no longer able to pick up Boost automatically.
  # https://github.com/NixOS/nixpkgs/issues/86131
  BOOST_INCLUDEDIR = "${stdenv.lib.getDev boost}/include";
  BOOST_LIBRARYDIR = "${stdenv.lib.getLib boost}/lib";
}
