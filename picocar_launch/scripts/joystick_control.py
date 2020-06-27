#!/usr/bin/env python

# Copyright (c) 2020, James Baxter
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32

THROTTLE_INDEX = 1
THROTTLE_SCALE = 1.0

STEERING_INDEX = 3
STEERING_SCALE = -1.0

throttle_pub = None
steering_pub = None

def joy_callback(joy_msg):
    throttle_msg = Float32(data=joy_msg.axes[THROTTLE_INDEX]*THROTTLE_SCALE)
    throttle_pub.publish(throttle_msg)

    steering_msg = Float32(data=joy_msg.axes[STEERING_INDEX]*STEERING_SCALE)
    steering_pub.publish(steering_msg)

def joystick_control_node():
    rospy.init_node('joystick_control_node', anonymous=True)

    global THROTTLE_INDEX, THROTTLE_SCALE, STEERING_INDEX, STEERING_SCALE, throttle_pub, steering_pub

    THROTTLE_INDEX = rospy.get_param('~throttle/index', THROTTLE_INDEX)
    THROTTLE_SCALE = rospy.get_param('~throttle/scale', THROTTLE_SCALE)

    STEERING_INDEX = rospy.get_param('~steering/index', STEERING_INDEX)
    STEERING_SCALE = rospy.get_param('~steering/scale', STEERING_SCALE)

    throttle_pub = rospy.Publisher('control_effort/throttle', Float32, queue_size=0)
    steering_pub = rospy.Publisher('control_effort/steering', Float32, queue_size=0)

    rospy.Subscriber('joy', Joy, joy_callback)

    rospy.spin()

if __name__ == '__main__':
    joystick_control_node()
