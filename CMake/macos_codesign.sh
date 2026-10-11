#!/bin/sh
# Post-build: sign the app bundle with a fixed local certificate, if there is one.
#
# Without this the linker signs ad hoc, the app's identity is a hash of the binary, and
# macOS asks for folder access again after every rebuild. With a fixed certificate the
# identity is "bundle id + certificate", which a rebuild does not change. Create the
# certificate once with ci/macos_make_signing_cert.sh. No certificate, no change: the
# build stays ad hoc exactly as before, so this never breaks a build.
#
#   macos_codesign.sh <identity name> <path to .app>
IDENTITY="$1"
BUNDLE="$2"
if [ -z "$IDENTITY" ] || ! security find-certificate -c "$IDENTITY" >/dev/null 2>&1; then
    exit 0
fi
if codesign --force --sign "$IDENTITY" "$BUNDLE" >/dev/null 2>&1; then
    echo "-- signed $(basename "$BUNDLE") with \"$IDENTITY\""
else
    # a signing failure must not fail the build; the app still runs, it just keeps asking
    echo "-- warning: could not sign $(basename "$BUNDLE") with \"$IDENTITY\"; left ad hoc"
fi
exit 0
