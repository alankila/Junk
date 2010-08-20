#!/bin/sh

adb push akmd /system/bin/akmd
adb push libs/armeabi/akmd.free /system/bin/akmd.free
adb shell killall -TERM akmd.free
