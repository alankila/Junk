#!/bin/sh

set -e

cd $(dirname "$0")

adb remount
# boot failure if used during boot.
mkdir -p dsp/system/lib
cp -v ~/cyanogenmod/out/target/product/hero/system/lib/libaudioflinger.so dsp/system/lib/
adb push dsp/system/lib/libaudioflinger.so /system/lib/
#adb push ~/Työpöytä/DSPManager.apk /system/app/
adb shell kill -9 '$(pidof mediaserver)'
