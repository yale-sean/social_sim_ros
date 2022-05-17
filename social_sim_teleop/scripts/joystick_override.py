#!/usr/bin/env python3

# Copyright (c) 2020, Members of Yale Interactive Machines Group, Yale University,
# Nathan Tsoi
# All rights reserved.
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import rospy
import numpy as np
import math
import tf
import tf2_ros
from tf import transformations
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy


class JoystickOverride:
    def __init__(self):
        self.override = None
        self.override_delta_t = rospy.Duration(1)
        rospy.init_node('joystick_override')
        rospy.Subscriber("/planner_cmd_vel", Twist, self.planner_cb)
        rospy.Subscriber("/joystick_cmd_vel", Twist, self.joystick_cb)
        rospy.Subscriber("/joy", Joy, self.joy_cb)
        self.pub = rospy.Publisher("/mobile_base_controller/cmd_vel", Twist, queue_size=10)

    def planner_cb(self, twist):
        if self.js_override():
            return
        self.pub.publish(twist)

    def joystick_cb(self, twist):
        if not self.js_override():
            return
        self.pub.publish(twist)

    def joy_cb(self, joy):
        if joy.buttons[4]:
            self.override = rospy.get_rostime()
    
    def js_override(self):
        if self.override is not None and rospy.get_rostime() - self.override  < self.override_delta_t:
            return True
        return False

if __name__ == '__main__':
    JoystickOverride()
    rospy.spin()
