#!/bin/sh
# Create a self-signed code-signing certificate for local KNOSSOS builds, once.
#
# Why: the build is otherwise signed ad hoc, which makes the app's identity a hash of the
# binary. macOS keys folder permissions (Documents, Downloads, Desktop, cloud folders,
# removable volumes) to that identity, so every rebuild looks like a new app and asks again.
# Signed with a fixed certificate, the identity becomes "identifier org.mpin.KNOSSOS + this
# certificate", which survives rebuilds — grant a folder once and it stays granted.
#
# The certificate never leaves this Mac and need not come from Apple; it only has to stay
# the same. CMake signs each build with it automatically once it exists (see
# CMakeLists.txt, KNOSSOS_CODESIGN_IDENTITY).
#
#   ci/macos_make_signing_cert.sh                 # into the login keychain
#   ci/macos_make_signing_cert.sh <keychain-path> # into another keychain, e.g. for testing
#
# Undo: delete "KNOSSOS Local Signing" in Keychain Access (both the certificate and its key).
set -eu
NAME="KNOSSOS Local Signing"
KEYCHAIN="${1:-$HOME/Library/Keychains/login.keychain-db}"

if security find-identity -p codesigning "$KEYCHAIN" 2>/dev/null | grep -q "\"$NAME\""; then
    echo "\"$NAME\" already exists in $KEYCHAIN — nothing to do."
    exit 0
fi

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT
cat > "$WORK/cert.cnf" <<CNF
[req]
distinguished_name = dn
x509_extensions = ext
prompt = no
[dn]
CN = $NAME
[ext]
basicConstraints = critical, CA:false
keyUsage = critical, digitalSignature
extendedKeyUsage = critical, codeSigning
CNF
# ten years: long enough that it never has to be thought about again
openssl req -x509 -newkey rsa:2048 -nodes -days 3650 -config "$WORK/cert.cnf" \
        -keyout "$WORK/key.pem" -out "$WORK/cert.pem" 2>/dev/null
# -legacy: macOS's `security import` cannot read the PKCS#12 ciphers OpenSSL 3 defaults to
PASS="knossos-$$"
openssl pkcs12 -export -legacy -inkey "$WORK/key.pem" -in "$WORK/cert.pem" -name "$NAME" \
        -out "$WORK/identity.p12" -passout "pass:$PASS" 2>/dev/null \
    || openssl pkcs12 -export -inkey "$WORK/key.pem" -in "$WORK/cert.pem" -name "$NAME" \
        -out "$WORK/identity.p12" -passout "pass:$PASS"
# -T /usr/bin/codesign lets codesign use the key without a keychain prompt on every build
security import "$WORK/identity.p12" -k "$KEYCHAIN" -P "$PASS" -T /usr/bin/codesign >/dev/null
echo "Created \"$NAME\" in $KEYCHAIN."
echo "Rebuild KNOSSOS; macOS will ask for folder access once more, and then not again."
