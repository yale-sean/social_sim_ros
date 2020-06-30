#!/usr/bin/env python
'''
run social sim trials
'''
import rospy
import actionlib
import tf
from social_sim_ros.msg import TrialStart, PoseArray, TrialInfo
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionGoal
from std_msgs.msg import Bool
from random import randint
import csv

class SocialSimRunner(object):
    def __init__(self):
        rospy.init_node("social_sim_runner")
        self.prev_info_stamp = 0
        self.info_stamp = 1
        self.current_trial = 0
        self.is_trialing = False
        self.trial_ready = False
        self.info_ready = False
        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        self.start_pub = rospy.Publisher("/social_sim/start_trial", TrialStart, queue_size=10)
        self.status_sub = rospy.Subscriber("/social_sim/is_running", Bool, self.status_callback, queue_size=10)
        self.info_sub = rospy.Subscriber("/social_sim/last_info", TrialInfo, self.info_callback, queue_size=10)

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        self.num_trials = rospy.get_param('~num_trials', 10)
        print("Number of Trials: {}".format(self.num_trials))
        self.num_peds = rospy.get_param('~num_peds', 10)
        print("Number of Pedestrians: {}".format(self.num_peds))
        self.time_limit = rospy.get_param('~time_limit_sec', 90)
        print("Time limit (sec): {}".format(self.time_limit))
        self.trial_name = rospy.get_param('~trial_name')
        print("Trial name: {}".format(self.trial_name))

        file = open(self.trial_name + '.csv', 'w')
        fnames = ['timestamp', 'dist_to_target', 'dist_to_ped',
                  'num_collisions', 'run_complete', 'time_elapsed']
        self.writer = csv.DictWriter(file, fieldnames=fnames)
        self.writer.writeheader()

        print("Waiting for a /social_sim/is_running message")
        print("Please start Unity")

        rospy.spin()

    def positions_callback(self, positions_msg):
        positions = positions_msg.positions
        n = len(positions)
        spawn_pos_idx = randint(0, n - 1)
        self.spawn_pos = positions[spawn_pos_idx]
        del positions[spawn_pos_idx]
        target_pos_idx = randint(0, n - 2)
        self.target_pos = positions[target_pos_idx]
        self.trial_ready = True

    def status_callback(self, trial_status_msg):
        self.is_trialing = trial_status_msg.data
        if self.is_trialing is False \
        and self.info_stamp != self.prev_info_stamp \
        and self.trial_ready is True:
            if self.info_ready:
                self.record_csv()
            self.run_trial()
            self.move_client.send_goal(self.goal)
            self.prev_info_stamp = self.info_stamp
        if self.trial_ready is True:
            self.start_pub.publish(self.trial_start_msg)
            #self.goal_pub.publish(self.goal_msg)
            


    def info_callback(self, trial_info_msg):
        self.trial_info = trial_info_msg
        self.info_stamp = self.trial_info.header.stamp.secs
        self.info_ready = True

    def run_trial(self):
        print("Running Trial {}".format(self.current_trial))
        if self.current_trial >= self.num_trials:
            return
        self.trial_start_msg = TrialStart()
        self.trial_start_msg.header.stamp = rospy.Time.now()
        self.trial_start_msg.spawn = self.spawn_pos
        self.trial_start_msg.target = self.target_pos
        self.trial_start_msg.num_peds = self.num_peds
        self.trial_start_msg.time_limit = self.time_limit
        self.goal = MoveBaseGoal()
        self.goal.target_pose.header.frame_id = "map"
        self.goal.target_pose.header.stamp = rospy.Time.now()
        self.goal.target_pose.pose = self.target_pos

        self.current_trial += 1

    def record_csv(self):
        self.writer.writerow({
                'timestamp': self.trial_info.header.stamp,
                'dist_to_target': self.trial_info.dist_to_target,
                'dist_to_ped': self.trial_info.dist_to_ped,
                'num_collisions': self.trial_info.num_collisions,
                'run_complete': self.trial_info.run_complete,
                'time_elapsed': self.trial_info.time_elapsed
            })


if __name__ == "__main__":
    try:
        node = SocialSimRunner()
    except rospy.ROSInterruptException:
        pass
