#!/bin/sh

set -e

cd $(dirname "$0")

adb remount
mkdir -p dsp/system/lib
cp -v ~/git/cyanogenmod/out/target/product/hero/system/lib/libaudioflinger.so dsp/system/lib/
cp -v ~/git/cyanogenmod/out/target/product/hero/system/app/DSPManager.apk dsp/system/app/
adb push dsp/system/lib/libaudioflinger.so /system/lib/
adb push dsp/system/app/DSPManager.apk /system/app/
adb shell kill -9 '$(pidof mediaserver)'
