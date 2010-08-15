#!/bin/sh

set -e

cd $(dirname "$0")

echo "Nuking old files";
rm -f dsp.zip
rm -f dsp-signed.zip

echo "Update APK & .SO"
mkdir -p dsp/system/lib
mkdir -p dsp/system/app
cp -v ~/cyanogenmod/out/target/product/hero/system/lib/libaudioflinger.so dsp/system/lib/
cp -v ~/android_packages_apps_DSPManager.git/bin/DSPManager.apk dsp/system/app/

echo "Build zip";
cd dsp/
zip -r ../dsp.zip .
cd ..

echo "Sign zip"
java -jar ~/cyanogenmod/out/host/linux-x86/framework/signapk.jar \
 ~/cyanogenmod/build/target/product/security/testkey.x509.pem \
 ~/cyanogenmod/build/target/product/security/testkey.pk8 \
 dsp.zip dsp-signed.zip
