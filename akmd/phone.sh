#!/bin/bash

set -e

cd $(dirname $0)
ndk-build
adb remount
adb push akmd /system/bin/akmd
adb push libs/armeabi/akmd.free /system/bin/akmd.free
adb shell killall -TERM akmd akmd.free
