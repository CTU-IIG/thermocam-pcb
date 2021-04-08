{ stdenv, writeText }:
  # libPvGenICam.so from ebus_sdk links to libcurl-gnutls.so.4 and
  # expect it to be compiled with versioned symbols. Nix does not use
  # versioned symbols in this library so we cannot use nix-provided
  # library. We also don't want to change curl derivation to use
  # versioned symbols because nix fetch* functions depend on curl and
  # the change causes rebuild of almost everything. Since we do not
  # need curl-related functionality, we replace this library with a
  # stub of empty functions.
  stdenv.mkDerivation {
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

  }
