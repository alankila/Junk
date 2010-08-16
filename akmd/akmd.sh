#!/system/bin/sh

# 1st: analog offsets 0 .. 255
# 2nd: analog gain 0 .. 15
# 3rd: digital offsets -2048 .. 2048
# 4th: 0C reference value 0 .. 255

exec akmd.free  \
       6   128     0 \
       4     3     1 \
     160  -320  -800 \
     112
