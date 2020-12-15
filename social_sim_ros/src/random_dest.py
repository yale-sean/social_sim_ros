#!/usr/bin/env python
'''
run social sim random destination navigation
'''
import actionlib
import rospy
from rospy_message_converter import message_converter
import tf

from geometry_msgs.msg import PoseArray, Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from social_sim_ros.msg import TrialStart, TrialInfo
from std_msgs.msg import Bool

import csv
import errno
import json
import os
from random import randint

import numpy as np

# Message serialization helpers
def msg_json_to_dict(json_message):
    return json.loads(json_message)

def msg_dict_to_ros(message_type, dict_message, strict_mode=True):
    return message_converter.convert_dictionary_to_ros_message(message_type, dict_message, strict_mode=strict_mode)

def msg_json_to_ros(message_type, json_message, strict_mode=True):
    return msg_dict_to_ros(message_type, msg_json_to_dict(json_message), strict_mode=strict_mode)

def msg_ros_to_dict(message):
    return message_converter.convert_ros_message_to_dictionary(message)

def msg_ros_to_json(message):
    return json.dumps(msg_ros_to_dict(message))

class SocialSimRandomDest(object):
    POSITION_MODES = ['rand', 'once']
    def __init__(self):
        rospy.init_node("social_sim_random_dest")

        self.listener = tf.TransformListener()

        self.goal_epsilon = rospy.get_param('~goal_epsilon', 0.5)

        # call to restart the trial runner
        self.reset_state()

        # get the action server
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(5.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Got action server")
        rospy.loginfo("Waiting for positions")

        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        rospy.spin()

    def reset_state(self):
        self.positions = None
        self.current_goal_idx = None
        self.last_msg = ""

    def positions_callback(self, positions_msg):
        if self.positions is None:
            rospy.loginfo("Got positions")
            self.positions = positions_msg.poses
            self.should_move_to_new_dest()

    def should_move_to_new_dest(self, robot_position=None):
        # cant't run without positions
        if self.positions is None:
            return
        if self.current_goal_idx == None:
            self.publish_new_dest()
            return
        if robot_position is not None:
            goal = self.positions[self.current_goal_idx].position
            pos = [robot_position.pose.position.x, robot_position.pose.position.y, robot_position.pose.position.z]
            dist = np.linalg.norm(np.array([goal.x, goal.y, goal.z]) - np.array(pos))
            msg = "robot is {:.2f}m from the goal".format(dist)
            if msg != self.last_msg:
                rospy.loginfo(msg)
                self.last_msg = msg
            if dist < self.goal_epsilon:
                self.publish_new_dest()

    def pick_position(self):
        # random choice
        rospy.loginfo("Randomly choosing 1 of {} available positions".format(len(self.positions)))
        self.current_goal_idx = randint(0, len(self.positions) - 1)
        return self.positions[self.current_goal_idx]

    def active_cb(self):
        rospy.loginfo("Active")

    def feedback_cb(self, feedback):
        self.should_move_to_new_dest(feedback.base_position)
        #rospy.loginfo("Feedback (position): {}".format(feedback))

    def done_cb(self, status, result):
        if status == 2:
            rospy.loginfo("Canceled")

        if status == 3:
            rospy.loginfo("Reached") 
            self.publish_new_dest()

        if status == 4:
            rospy.loginfo("Aborted")
            rospy.sleep(10.0)
            self.publish_new_dest()

        if status == 5:
            rospy.loginfo("Rejected")
            rospy.sleep(10.0)
            self.publish_new_dest()
            return

        if status == 8:
            rospy.loginfo("Canceled before starting")
            rospy.sleep(10.0)
            self.publish_new_dest()

        rospy.loginfo("Done")

    def publish_new_dest(self):
        # send goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.pick_position()
        self.client.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        rospy.loginfo("Sent goal: {}".format(goal))
        rospy.sleep(5.0)


if __name__ == "__main__":
    try:
        SocialSimRandomDest()
    except rospy.ROSInterruptException:
        pass
