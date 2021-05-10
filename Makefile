all: build/build.ninja
	ninja -C build >&2

clean:
	ninja -C build $@

build/build.ninja:
	meson setup build $(mesonFlags) # mesonFlags is set by nix-shell/lorri

.PHONY: thermocam-pcb.includes

# Generate .includes file for QtCreator based on Nix variables (from nix-shell or lorri)
thermocam-pcb.includes:
	echo . > $@
	echo build/ >> $@
	echo $$NIX_CFLAGS_COMPILE | \
	  grep -Eo '(-I|-isystem) ?([^ ]*)' | \
	  sed -Ee 's/^(-I|-isystem) *//' | \
	  sed -e '/opencv/ s|.*|\0/opencv4|' | sort -u >> $@
