#!/system/bin/sh

# 1st: analog offsets 0 .. 255
# 2nd: analog gain 0 .. 15
# 4th: 0C reference value 0 .. 255

exec akmd.free  \
       7    -2    -4 \
       4     3     1 \
     112
