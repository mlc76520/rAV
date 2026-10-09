#!/bin/sh
# Build rav-audio_<version>_arm64.deb: compiles rav_unified.cpp (bcm2835
# backend, Raspberry Pi 3) and packs it with packaging/root and the
# maintainer scripts in packaging/debian.
#   usage: packaging/build-deb.sh [version] [output dir]
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
SRC=$HERE/../rav_unified.cpp
VERSION=${1:-1.0.0}
OUT=$(cd "${2:-$HERE}" && pwd)
STAGE=$(mktemp -d)
SHLIBS_DIR=$(mktemp -d)
trap 'rm -rf "$STAGE" "$SHLIBS_DIR"' EXIT

cp -a "$HERE/root/." "$STAGE/"
mkdir -p "$STAGE/usr/bin" "$STAGE/DEBIAN"
# Normalized modes, whatever the builder's umask
find "$STAGE" -type d -exec chmod 755 {} +
find "$STAGE" -type f -exec chmod 644 {} +

echo "Compiling rav-visualizer..."
g++ -o "$STAGE/usr/bin/rav-visualizer" "$SRC" \
    -O3 -march=native -std=c++17 -DUSE_BCM2835 -Wall -Wextra \
    -I/usr/include/freetype2 \
    $(pkg-config --cflags --libs dbus-1 libpipewire-0.3) \
    -fstack-protector-strong -D_FORTIFY_SOURCE=2 -fPIE -pie -Wl,-z,relro,-z,now \
    -lbcm2835 -lpthread -lasound -lfftw3f -lm -lfreetype -lmpdclient
strip --strip-unneeded "$STAGE/usr/bin/rav-visualizer"
chmod 755 "$STAGE/usr/bin/rav-visualizer"
install -D -m 644 "$HERE/../trixel-square.ttf" "$STAGE/usr/share/rav-audio/trixel-square.ttf"

# dpkg-shlibdeps wants a debian/control in its working directory
mkdir -p "$SHLIBS_DIR/debian"
printf 'Source: rav-audio\n\nPackage: rav-audio\nArchitecture: arm64\n' > "$SHLIBS_DIR/debian/control"
SHLIBS=$(cd "$SHLIBS_DIR" && dpkg-shlibdeps -O "$STAGE/usr/bin/rav-visualizer" 2>/dev/null \
         | sed -n 's/^shlibs:Depends=//p')

SIZE=$(du -sk --exclude=DEBIAN "$STAGE" | cut -f1)
sed -e "s/@VERSION@/$VERSION/" -e "s/@SIZE@/$SIZE/" -e "s/@SHLIBS@/$SHLIBS/" \
    "$HERE/debian/control.in" > "$STAGE/DEBIAN/control"
install -m 755 "$HERE/debian/postinst" "$HERE/debian/prerm" "$HERE/debian/postrm" "$STAGE/DEBIAN/"
# Everything shipped under /etc is a conffile: local edits survive upgrades
(cd "$STAGE" && find etc -type f | sed 's|^|/|' | sort) > "$STAGE/DEBIAN/conffiles"

DEB="$OUT/rav-audio_${VERSION}_arm64.deb"
dpkg-deb --root-owner-group --build "$STAGE" "$DEB"
echo "Built $DEB"
