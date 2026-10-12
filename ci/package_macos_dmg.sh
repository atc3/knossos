#!/bin/bash
# Build a self-contained KNOSSOS.app from a source checkout and wrap it in a DMG.
#
#   ci/package_macos_dmg.sh <source dir> <work dir> <dmg name, e.g. KNOSSOS-v5.1-fork1>
#
# For Apple Silicon Homebrew (/opt/homebrew). Upstream's ci/build_macos.sh is the same
# recipe for Intel CI; this one bundles everything recursively instead of by a fixed list,
# because Python's extension modules pull in a chain of Homebrew libraries.
#
# The result is signed ad hoc, not with a Developer ID, and not notarized: on another Mac,
# Gatekeeper blocks the first launch until it is allowed in System Settings ▸ Privacy &
# Security, or the quarantine flag is removed. See the release notes for the steps.
set -euo pipefail
SRC=$(cd "$1" && pwd)
WORK=$2
NAME=$3
QT=${QT_PREFIX:-/opt/homebrew/opt/qt@5}
DEPS=${KNOSSOS_DEPS:-$HOME/knossos-deps}
PYROOT=${PYTHON_ROOT:-/opt/homebrew/opt/python@3.12}
PYVER=3.12

BUILD=$WORK/build
STAGE=$WORK/stage
APP=$STAGE/KNOSSOS.app
mkdir -p "$WORK"

echo "== build"
cmake -G Ninja -S "$SRC" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS=-Wno-deprecated-declarations \
      -DCMAKE_PREFIX_PATH="$QT;$DEPS" -DPython_FIND_STRATEGY=LOCATION -DPython_ROOT_DIR="$PYROOT" \
      -DKNOSSOS_CODESIGN_IDENTITY= >/dev/null
cmake --build "$BUILD"

echo "== stage"
rm -rf "$STAGE"
mkdir -p "$STAGE"
cp -R "$BUILD/knossos.app" "$APP"
rm -rf "$APP/Contents/Frameworks"   # the build tree's are symlinks into $DEPS

echo "== Qt"
"$QT/bin/macdeployqt" "$APP" -libpath="$DEPS/lib" -always-overwrite >/dev/null

echo "== Python $PYVER"
# The interpreter finds its standard library relative to its own framework, so the framework
# goes in whole — not just the dylib macdeployqt may have copied.
PYFW_SRC=$(cd "$PYROOT/Frameworks/Python.framework" && pwd -P)
PYFW=$APP/Contents/Frameworks/Python.framework
rm -rf "$PYFW"
mkdir -p "$PYFW/Versions"
ditto "$PYFW_SRC/Versions/$PYVER" "$PYFW/Versions/$PYVER"
ln -s "$PYVER" "$PYFW/Versions/Current"
for link in Python Resources Headers; do ln -s "Versions/Current/$link" "$PYFW/$link"; done
PYLIB=$PYFW/Versions/$PYVER/lib/python$PYVER
# what an embedded interpreter never uses: tests, Homebrew's site-packages, the static lib,
# the command-line launchers and their app, bytecode caches
rm -rf "$PYLIB/test" "$PYLIB/idlelib/idle_test" "$PYLIB/config-$PYVER-darwin" \
       "$PYFW/Versions/$PYVER/bin" "$PYFW/Versions/$PYVER/Resources/Python.app" "$PYFW/Versions/$PYVER/share"
rm -rf "$PYLIB/site-packages" && mkdir -p "$PYLIB/site-packages"
find "$PYFW" -name __pycache__ -type d -prune -exec rm -rf {} +
PYBIN=$PYFW/Versions/$PYVER/Python
chmod u+w "$PYBIN"
install_name_tool -id "@executable_path/../Frameworks/Python.framework/Versions/$PYVER/Python" "$PYBIN"
OLD=$(otool -L "$APP/Contents/MacOS/knossos" | awk '/Python.framework/{print $1; exit}')
install_name_tool -change "$OLD" "@executable_path/../Frameworks/Python.framework/Versions/$PYVER/Python" "$APP/Contents/MacOS/knossos"

echo "== everything else, recursively, then sign"
BUNDLE_LIBPATH="$DEPS/lib:$(brew --prefix)/lib" python3 "$SRC/ci/macos_bundle_deps.py" "$APP"

echo "== dmg"
rm -f "$WORK/$NAME.dmg"
ln -s /Applications "$STAGE/Applications"
hdiutil create -volname "KNOSSOS" -srcfolder "$STAGE" -ov -format UDZO "$WORK/$NAME.dmg" >/dev/null
rm "$STAGE/Applications"
echo "== done: $WORK/$NAME.dmg ($(du -h "$WORK/$NAME.dmg" | cut -f1))"
