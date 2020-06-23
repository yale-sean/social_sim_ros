#!/usr/bin/env python
'''
run social sim trials
'''
import rospy
import tf
from social_sim_ros.msg import TrialStart, PoseArray, TrialInfo
from move_base_msgs.msg import MoveBaseGoal
from std_msgs.msg import Bool
from random import randint
import csv

class SocialSimRunner(object):
    def __init__(self):
        rospy.init_node("social_sim_runner")
        self.current_trial = 0;
        self.is_trialing = False
        self.trial_ready = False
        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        self.start_pub = rospy.Publisher("/social_sim/start_trial", TrialStart, queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base/goal", MoveBaseGoal, queue_size=10)
        self.status_sub = rospy.Subscriber("/social_sim/is_running", Bool, self.status_callback, queue_size=10)
        self.info_sub = rospy.Subscriber("/social_sim/last_info", TrialInfo, self.info_callback, queue_size=10)



        self.num_trials = rospy.get_param('~num_trials')
        print(self.num_trials)
        self.num_peds = rospy.get_param('~num_peds')
        print(self.num_peds)
        self.time_limit = rospy.get_param('~time_limit')
        print(self.time_limit)
        self.trial_name = rospy.get_param('~trial_name')
        print(self.trial_name)

        self.csvfile = csv.open(trial_name + '.csv', 'wb')

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
        prev_status = self.is_trialing
        self.is_trialing = trial_status_msg.data
        if prev_status is True and self.is_trialing is False:
            self.record_csv()
            self.run_trial()


    def info_callback(self, trial_info_msg):
        self.trial_info = trial_info_msg

    def run_trial(self):
        if self.current_trial >= self.num_trials:
            return
        trial_start_msg = TrialStart()
        trial_start_msg.spawn = self.spawn_pos
        trial_start_msg.target = self.target_pos
        trial_start_msg.num_peds = self.num_peds
        trial_start_msg.time_limit = self.time_limit
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose.pose = self.target_pos
        self.current_trial += 1
        self.start_pub.publish(trial_start_msg)
        self.goal_pub.publish(goal_msg)

    def record_csv():
        writer = csv.writer(self.csvfile)
        writer.writerow(self.trial_info)



if __name__ == "__main__":
    try:
        node = SocialSimRunner()
        node.run_trial()
    except rospy.ROSInterruptException:
        pass
