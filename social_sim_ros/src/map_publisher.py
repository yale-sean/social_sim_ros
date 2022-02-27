#!/usr/bin/env python3

import sys
import rospy

from social_sim_ros.msg import SceneInfo

import psutil
import subprocess
import shlex
import signal

import re

ENV_TO_MAP = {
        # lab scenes
        'lab': 'lab',
        'labstudy': 'labstudy',
        'agentcontrollab': 'lab',
        'labscenegraph' : 'lab',
        # warehouse scenes
        'smallwarehouse': 'warehouse_small',
        'agentcontrolsmallwarehouse': 'warehouse_small',
        'smallwarehousescenegraph':'warehouse_small',
        # outdoor scenes
        'outdoor': 'outdoor',
        'agentcontroloutdoor': 'outdoor',
        'outdoorscenegraph' : 'outdoor',
        'hotel' : 'hotel',
        'university' : 'university',
        'eth' : 'ETH',
        'zara' : 'zara',

}

def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()

class MapPublisher():
    ''' Publish the correct map for the current scene '''

    def __init__(self):

        rospy.init_node("map_publisher")

        self.map = None
        self.pid = None

        rospy.Subscriber("/social_sim/scene_info", SceneInfo, self.info_callback)

        rospy.on_shutdown(self.stop_map)

        rospy.loginfo("Ready to publish map")

        rospy.spin()

    def info_callback(self, info):
        env = info.environment.lower()
        env = re.sub('robotcontrol', '', env)
        env = re.sub('agentcontrol', '', env)
        env = re.sub('scene$', '', env)
        if env not in ENV_TO_MAP:
            rospy.logerr("Could not find {} for {} in {}".format(env, info.environment, ENV_TO_MAP))
        _map = ENV_TO_MAP[env]

        if _map != self.map:
            self.stop_map()
            self.publish_map(_map)

    def ros_command(self, _map):
        cmd = "roslaunch --wait social_sim_ros map_server.launch scene:={}".format(_map)
        return shlex.split(cmd)

    def publish_map(self, _map):
        process = subprocess.Popen(self.ros_command(_map))
        self.pid = process.pid
        rospy.loginfo("Started publishing map {}".format(self.map))
        self.map = _map

    def stop_map(self):
        if self.pid is None:
            return
        signal_process_and_children(self.pid, signal.SIGINT, wait=True)
        self.pid = None
        rospy.loginfo("Stopped publishing map")



if __name__ == "__main__":
    try:
        MapPublisher()
    except rospy.ROSInterruptException:
        pass
