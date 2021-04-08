all: build/build.ninja
	ninja -C build >&2

clean:
	ninja -C build $@

build/build.ninja:
	meson setup build $(mesonFlags) # mesonFlags is set by nix-shell/lorri
