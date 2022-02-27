#!/usr/bin/env python3

# Copyright (c) 2021, Members of Yale Interactive Machines Group, Yale University,
# Nathan Tsoi
# All rights reserved.
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import datetime

import rospy
import numpy as np

from rosgraph_msgs.msg import Clock

class DebugTime(object):
    def __init__(self):
        self.i = 0
        self.total_delta_t = 0
        rospy.init_node("debug_time")
        rospy.Subscriber("/clock", Clock, self.clock_callback, queue_size=10)
        self.start_clock = None
        self.start_ts = None
        print("starting")
        rospy.spin()


    def clock_callback(self, msg):
        self.i += 1
        if self.start_ts is None:
            self.start_clock = datetime.datetime.now()
            self.start_ts = msg.clock
        else:
            systemtime = (datetime.datetime.now() - self.start_clock).total_seconds()
            simtime = (msg.clock - self.start_ts).to_sec()
            deltatime = simtime-systemtime
            self.total_delta_t += deltatime
            if self.i % 500 == 0:
                #print(f" Delta time: {deltatime}")
                print(f" Delta time: {self.total_delta_t}")
                #print(f"System time: {systemtime}")
                #print(f"   Sim time: {simtime}")
            #print(f"   Sim time: {datetime.timedelta((msg.clock - self.start_ts).to_sec())}")


if __name__ == "__main__":
    DebugTime()