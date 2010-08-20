#!/bin/sh

adb push akmd.sh /system/bin/akmd
adb push libs/armeabi/akmd.free /system/bin/akmd.free
adb shell killall -TERM akmd.free
