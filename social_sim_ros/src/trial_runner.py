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

        # to avoid starting the next trial too soon
        self.debounce_seconds = rospy.Duration.from_sec(1.0)

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
        self.teleop = rospy.get_param('~teleop', False)
        print("Teleop mode: {}".format(self.teleop))
        self.trial_name = rospy.get_param('~trial_name')
        print("Trial name: {}".format(self.trial_name))

        if not os.path.exists(self.output_folder):
            os.makedirs(self.output_folder)

        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        self.start_pub = rospy.Publisher("/social_sim/start_trial", TrialStart, queue_size=10)
        self.status_sub = rospy.Subscriber("/social_sim/is_running", Bool, self.status_callback, queue_size=10)
        self.info_sub = rospy.Subscriber("/social_sim/last_info", TrialInfo, self.info_callback, queue_size=10)

        if not self.teleop:
            print("Waiting for the move_base action server")
            print("  enable _teleop:=True to skip this")
            self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.move_client.wait_for_server()

        print("Waiting for a /social_sim/spawn_positions, /social_sim/is_running message")
        print("Please start Unity")

        rospy.spin()

    def reset_state(self):
        self.last_info_msg_time = None
        self.last_status_msg_state = None
        self.last_status_msg_time = rospy.Time.now()
        self.is_trialing = None
        self.positions = None
        self.rows_to_write = []
        self.current_trial = 0

    def positions_callback(self, positions_msg):
        if self.positions is None: 
            self.positions = positions_msg.positions

    def status_callback(self, trial_status_msg):
        '''
            The status message indicates if a trial is currently being run or not
        '''
        now = rospy.Time.now()
        if now - self.last_status_msg_time < self.debounce_seconds:
            return
        self.last_status_msg_time = rospy.Time.now()
        # Check for a change since the last message, must last longer than the debouce time
        self.is_trialing = trial_status_msg.data
        if self.last_status_msg_state == self.is_trialing:
            return
        self.last_status_msg_state = self.is_trialing
        self.should_run_trial()

    def info_callback(self, trial_info_msg):
        '''
            The info callback returns the current state of the trial
        '''
        # Wait for positions before we record any info
        if self.positions == None:
            return
        # Debounce
        if self.last_info_msg_time == trial_info_msg.header.stamp:
            return
        self.last_info_msg_time = trial_info_msg.header.stamp
        self.record_row(trial_info_msg)
        # run a new trial, if ready
        self.should_run_trial()

    def should_run_trial(self):
        '''
            Decides if a new trial should be run, executed after both the status and info callbacks

            A new trial is run when:

                We have positions and the current trial is 0, indicating this node has just been started

                OR

                Unity is not running a trial (is_trialing == False)
        '''
        # don't run trials without positions
        if self.positions is None:
            return
        # startup case, start a new trial no matter what state unity is in
        if self.current_trial == 0:
            return self.run_trial()
        # if we are not running a trial, run a new one
        if self.is_trialing == False:
            return self.run_trial()

    def pick_positions(self):
        n = len(self.positions)
        spawn_pos_idx = randint(0, n - 1)
        self.spawn_pos = self.positions[spawn_pos_idx]
        del self.positions[spawn_pos_idx]
        target_pos_idx = randint(0, n - 2)
        self.target_pos = self.positions[target_pos_idx]

    
    def check_complete(self):
        ''' Called before every trial run
            Checks if we have completed the requested trials
        '''
        if self.current_trial < self.num_trials:
            return
        print("Trials complete")
        # Save our data
        self.record_csv()
        rospy.signal_shutdown("Trials Complete")
        print("Exiting")
        exit()

    def run_trial(self):
        self.check_complete()
        self.current_trial += 1
        print("Running Trial {}".format(self.current_trial))

        self.pick_positions()

        # trial is starting, publish message
        trial_start_msg = TrialStart()
        trial_start_msg.header.stamp = rospy.Time.now()
        trial_start_msg.trial_name = self.trial_name
        trial_start_msg.trial_number = self.current_trial
        trial_start_msg.spawn = self.spawn_pos
        trial_start_msg.target = self.target_pos
        trial_start_msg.num_peds = self.num_peds
        trial_start_msg.time_limit = self.time_limit
        self.start_pub.publish(trial_start_msg)

        # send goal
        if not self.teleop:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.pose = self.target_pos
            self.move_client.send_goal(goal)

    def record_row(self, msg):
        self.info_stamp = msg.header.stamp.secs
        row = {
            'timestamp': msg.header.stamp,
            'trial_name': msg.trial_name,
            'trial_number': msg.trial_number,
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
        csv_path = os.path.join(self.output_folder, '{}_{}.csv'.format(self.trial_name, self.rows_to_write[0]['timestamp']))
        with open(csv_path, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows_to_write:
                writer.writerow(row)
        print("Write csv {}".format(csv_path))

if __name__ == "__main__":
    try:
        node = SocialSimRunner()
    except rospy.ROSInterruptException:
        pass
