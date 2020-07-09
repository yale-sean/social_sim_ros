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
import csv, os

class SocialSimRunner(object):
    def __init__(self):
        rospy.init_node("social_sim_runner")

        # call to restart the trial runner
        self.reset_state()

        self.output_folder = rospy.get_param('~output_folder', 'experiments')
        print("Output folder: {}".format(self.output_folder))
        self.num_trials = rospy.get_param('~num_trials', 10)
        print("Number of Trials: {}".format(self.num_trials))
        self.num_peds = rospy.get_param('~num_peds', 10)
        print("Number of Pedestrians: {}".format(self.num_peds))
        self.time_limit = rospy.get_param('~time_limit_sec', 90)
        print("Time limit (sec): {}".format(self.time_limit))
        self.trial_name = rospy.get_param('~trial_name')
        print("Trial name: {}".format(self.trial_name))

        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        self.start_pub = rospy.Publisher("/social_sim/start_trial", TrialStart, queue_size=10)
        self.status_sub = rospy.Subscriber("/social_sim/is_running", Bool, self.status_callback, queue_size=10)
        self.info_sub = rospy.Subscriber("/social_sim/last_info", TrialInfo, self.info_callback, queue_size=10)

        self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_client.wait_for_server()

        print("Waiting for a /social_sim/is_running message")
        print("Please start Unity")

        rospy.spin()

    def reset_state(self):
        self.rows_to_write = []
        self.prev_info_stamp = 0
        self.info_stamp = 1
        self.current_trial = 0
        self.trial_ready = False

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
        if not self.trial_ready:
            return
        # boolean message indicates if unity is running
        is_trialing = trial_status_msg.data
        if not is_trialing and self.info_stamp != self.prev_info_stamp:
            self.run_trial()

    def info_callback(self, trial_info_msg):
        self.record_row(trial_info_msg)

    def run_trial(self):
        print("Running Trial {}".format(self.current_trial))
        if self.current_trial >= self.num_trials:
            print("Trials complete")
            self.record_csv()
            # TODO: reset to start state
            # self.is_trialing = False
            print("Exiting")
            rospy.signal_shutdown("Trials Complete")
            return

        # trial is starting, publish message
        trial_start_msg = TrialStart()
        trial_start_msg.header.stamp = rospy.Time.now()
        trial_start_msg.spawn = self.spawn_pos
        trial_start_msg.target = self.target_pos
        trial_start_msg.num_peds = self.num_peds
        trial_start_msg.time_limit = self.time_limit
        self.start_pub.publish(trial_start_msg)

        # send goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.target_pos
        self.move_client.send_goal(goal)

        # record previous stamp
        self.prev_info_stamp = self.info_stamp
        self.current_trial += 1

    def record_row(self, msg):
        self.info_stamp = msg.header.stamp.secs
        row = {
            'timestamp': msg.header.stamp,
            'dist_to_target': msg.dist_to_target,
            'dist_to_ped': msg.dist_to_ped,
            'num_collisions': msg.num_collisions,
            'run_complete': msg.run_complete,
            'time_elapsed': msg.time_elapsed
        }
        self.rows_to_write.append(row)

    def record_csv(self):
        if len(self.rows_to_write) <= 0:
            print("ERROR: No rows to write")
            return
        fieldnames = self.rows_to_write[0].keys()
        csv_path = os.path.join(self.output_folder, '{}_{}.csv'.format(self.trail_name, self.rows_to_write[0]['timestamp']))
        with open(csv_path, 'w') as f:
            self.writer = csv.DictWriter(file)
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows_to_write:
                writer.writerow(row)
        print("Write csv {}".format(csv_path))

if __name__ == "__main__":
    try:
        node = SocialSimRunner()
    except rospy.ROSInterruptException:
        pass
