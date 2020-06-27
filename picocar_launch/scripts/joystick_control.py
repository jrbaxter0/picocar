#!/usr/bin/env python

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
