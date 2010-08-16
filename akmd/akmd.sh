#!/system/bin/sh

# 1st: analog offsets 0 .. 255
# 2nd: analog gain 0 .. 15
# 3rd: digital correction -128 .. 127
# 4th: 0C reference value 0 .. 255

exec akmd.free \
       6    -2    -3 \
       4     3     4 \
       4     0     0 \
     112
