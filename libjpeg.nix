{ stdenv, lib, fetchurl, static ? false }:

with lib;

stdenv.mkDerivation {
  name = "libjpeg-8d";

  src = fetchurl {
    url = http://www.ijg.org/files/jpegsrc.v8d.tar.gz;
    sha256 = "1cz0dy05mgxqdgjf52p54yxpyy95rgl30cnazdrfmw7hfca9n0h0";
  };

  configureFlags = optional static "--enable-static --disable-shared";

  outputs = [ "bin" "dev" "out" "man" ];

  meta = {
    homepage = http://www.ijg.org/;
    description = "A library that implements the JPEG image file format";
    license = lib.licenses.free;
    platforms = lib.platforms.unix;
  };
}
