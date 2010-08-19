#!/bin/sh

set -e

cd $(dirname "$0")

adb remount
mkdir -p dsp/system/lib
cp -v ~/git/cyanogenmod/out/target/product/hero/system/lib/libaudioflinger.so dsp/system/lib/
adb push dsp/system/lib/libaudioflinger.so /system/lib/
adb shell kill -9 '$(pidof mediaserver)'
