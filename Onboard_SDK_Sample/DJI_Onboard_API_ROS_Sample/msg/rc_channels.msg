# RC Map
#
#  mode:
#
# +8000 <--->  0  <---> -8000
#  API  <---> ATT <--->  POS
#
# CH3(throttle) +10000            CH1(pitch) +10000
#             ^                            ^
#             |                            |                  / -10000
#  CH2(yaw)   |                CH0(roll)   |                 /
# -10000 <-----------> +10000 -10000 <-----------> +10000   H(gear_up)
#             |                            |                 \
#             |                            |                  \ -4545
#             V                            V
#          -10000                       -10000
#
Header header
int32 ts
float32 roll
float32 pitch
float32 yaw
float32 throttle
float32 mode
float32 gear_up
