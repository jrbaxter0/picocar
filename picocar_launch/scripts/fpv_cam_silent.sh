#! /bin/bash

# hide all warnings regarding deprecated pixel format
# will also hide any important wornings, so keep that in mind
roslaunch picocar_launch fpv_cam.launch 2>/dev/null
