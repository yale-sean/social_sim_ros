#!/usr/bin/env python3
# based on: https://gist.github.com/vbschettino/13c1d0102fa184811b31397cb9a4e85d

"""Node to record a rosbag with start/stop/pause control through service calls.
Example call:
    rosrun utilities rosbag_controlled_recording.py _rosbag_command:="rosbag record -o /home/foo/test_bag /bar_topic"
Then start/pause/resume/stop can be controlled through:
    rosservice call /rosbag_controlled_recording/start
    rosservice call /rosbag_controlled_recording/stop
If this node is killed recording is also stopped. If recording was paused, it is momentarily resumed before stopping.
"""

import psutil
import subprocess
import shlex
import signal

import rospy
from social_sim_ros.msg import SceneInfo, TrialInfo

import time
import json
import os
import yaml

topics = [
    # transforms
    '/tf',
    # control
    '/social_sim/trigger',
    '/social_sim/cmd_vel',
    '/mobile_base_controller/cmd_vel', # needed for navstack recording
    # planning
    '/move_base/GlobalPlanner/plan',
    # simulation 
    '/social_sim/scene_info',
    '/social_sim/metrics',
    '/social_sim/agent_positions',
    '/social_sim/group_positions',
    # images
    '/agent_rgb/camera_info',
    '/agent_rgb/compressed',
    '/robot_firstperson_rgb/camera_info',
    '/robot_firstperson_rgb/compressed',
    '/robot_thirdperson_rgb/camera_info',
    '/robot_thirdperson_rgb/compressed',
    '/robot_overhead_rgb/camera_info',
    '/robot_overhead_rgb/compressed',
    '/center_depth/camera_info',
    '/center_depth/compressed',
    # laser from the depth image
    '/scan',
    # raycast laser
    '/laser_raycast',
    # classification
    '/social_sim/situations/rule_based/cross_path',
    '/social_sim/situations/rule_based/down_path',
    '/social_sim/situations/rule_based/empty',
    '/social_sim/situations/rule_based/join_group',
    '/social_sim/situations/rule_based/leave_group',
    # voice
    '/speech_recognition/vosk_result',
    # TODO: dagger
    ## attention
    #'/lifecycle_learner/attention_l',
    #'/lifecycle_learner/attention_cmd_vel',
    #'/lifecycle_learner/social_situation'
]

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


def format_to_columns(input_list, cols):
    """Adapted from https://stackoverflow.com/questions/171662/formatting-a-list-of-text-into-columns"""
    max_width = max(map(len, input_list))
    justify_list = map(lambda x: x.ljust(max_width + 4), input_list)
    lines = (''.join(justify_list[i:i + cols]) for i in range(0, len(justify_list), cols))
    return '\n'.join(lines)


class RosbagControlledRecorder(object):
    """Record a rosbag with service calls to control start, stop"""

    def __init__(self, interactive=True, only_scene_info=False):
        self.interactive = interactive
        self.only_scene_info = only_scene_info
        self.scene_info = None
        self.metrics = None
        self.last_start_time = 0
        recording_prefix = rospy.get_param("~prefix", "")
        recording_prefix = os.path.expanduser(recording_prefix)
        if recording_prefix:
            self.prefix = f"{recording_prefix}/"
        else:
            self.prefix = f"{os.environ.get('HOME')}/sim_ws/data/"
        self.recording = False
        self.process_pid = None
        self.run_details = ""

    def start_recording(self):
        if self.recording:
            rospy.logwarn("Cannot start recording, recording has already started - nothing to be done")
            return
        # get sceneinfo data before recording
        if self.scene_info is None:
            rospy.logwarn("Cannot start recording, no sceneinfo")
            return
        process = subprocess.Popen(self.rosbag_command())
        self.process_pid = process.pid
        self.recording = True
        rospy.loginfo("Started recording rosbag")

    def stop_recording(self):
        if self.process_pid is not None:
            signal_process_and_children(self.process_pid, signal.SIGINT, wait=True)
            self.process_pid = None
            rospy.loginfo("Stopped recording rosbag")
        self.recording = False

    def scene_callback(self, msg):
        self.scene_info = msg
        #rospy.loginfo(f"Got scene info: {msg}")

    def trial_callback(self, msg):
        if self.scene_info is None:
            return

        self.metrics = msg
        #rospy.loginfo(f"Got trial info: {msg}")

        msg_trial_start_sec = self.metrics.trial_start.to_sec()

        if self.last_start_time == msg_trial_start_sec:
            return

        if not self.recording:
            self.start_recording()
            self.last_start_time = msg_trial_start_sec
        else:
            self.stop_recording()


    def msg2json(self, msg):
        y = yaml.load(str(msg))
        return json.dumps(y,indent=4)

    def rosbag_command(self):
        base_fname = "{}{}-{}-{}-{}-{}".format(
            self.prefix,
            time.time(),
            'interactive' if self.interactive else 'noninteractive',
            self.scene_info.environment,
            self.scene_info.scenario_name,
            self.scene_info.num_people
        )
        with open(base_fname+".json", "w") as f:
            f.write(self.msg2json(self.scene_info))
        #cmd = "rosbag record -a -O {}".format(base_fname+".bag")
        cmd = "rosbag record -O {} {}".format(base_fname+".bag", ' '.join(topics))
        rospy.loginfo("STARTING: " + cmd)
        return shlex.split(cmd)

if __name__ == '__main__':
    rospy.init_node('rosbag_controlled_recording')

    # Get parameters
    interactive  = rospy.get_param('~interactive', True)
    # in case you have a situation without scene_is_running, set this to true
    only_scene_info  = rospy.get_param('~only_scene_info', False)

    # Start recorder object
    recorder = RosbagControlledRecorder(interactive, only_scene_info)

    scene_topic = "/social_sim/scene_info"
    trial_topic = "/social_sim/metrics"

    rospy.Subscriber(scene_topic, SceneInfo, recorder.scene_callback)
    rospy.Subscriber(trial_topic, TrialInfo, recorder.trial_callback)

    # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
    rospy.on_shutdown(recorder.stop_recording)

    rospy.loginfo(f"Ready to record, waiting for {scene_topic} and {trial_topic}")

    while not rospy.is_shutdown():
        rospy.sleep(1.0)
