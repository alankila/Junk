#!/system/bin/sh

# 1st: analog offsets 0 .. 255
# 2nd: analog gain 0 .. 15
# 3rd: 0C reference value 0 .. 255

exec akmd.free \
      10     0     0 \
       4     4     5 \
     112
