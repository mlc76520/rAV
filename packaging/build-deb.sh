#!/bin/sh
# Build rav-audio_<version>_arm64.deb: compiles the Rust visualizer in
# rust/ (one binary for the SSD1309 and SSD1322 panels, any Pi) and packs
# it with packaging/root and the maintainer scripts in packaging/debian.
#   usage: packaging/build-deb.sh [version] [output dir]
# Needs: cargo/rustc (Debian 13's 1.85 is enough), libclang-dev,
# libpipewire-0.3-dev, libfreetype-dev, pkg-config.
set -e

HERE=$(cd "$(dirname "$0")" && pwd)
CRATE=$HERE/../rust
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

# Under 2 GB of RAM (Pi 3) a parallel build gets rustc OOM-killed during
# the LTO link.
JOBS=$(awk '/MemTotal/ { print ($2 < 2000000) ? 1 : 0 }' /proc/meminfo)
echo "Compiling rav-visualizer ($([ "$JOBS" = 1 ] && echo "1 job, low memory" || echo "all cores"))..."
(cd "$CRATE" && if [ "$JOBS" = 1 ]; then cargo build --release -j 1; else cargo build --release; fi)
install -m 755 "$CRATE/target/release/rav-visualizer" "$STAGE/usr/bin/rav-visualizer"
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
