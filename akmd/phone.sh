#!/bin/sh

adb push akmd.sh /system/bin/akmd
adb push akmd /system/bin/akmd.free
adb shell killall -TERM akmd.free
