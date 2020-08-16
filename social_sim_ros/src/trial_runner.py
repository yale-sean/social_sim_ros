#!/usr/bin/env python
'''
run social sim trials
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
import logging
from random import randint

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

class SocialSimRunner(object):
    POSITION_MODES = ['rand', 'once']
    def __init__(self):
        rospy.init_node("social_sim_runner")

        # to avoid starting the next trial too soon
        self.debounce_seconds = rospy.Duration.from_sec(1.0)

        self.output_folder = rospy.get_param('~output_folder', 'experiments')
        logging.info("Output folder: {}".format(self.output_folder))
        self.num_trials = rospy.get_param('~num_trials', 10)
        logging.info("Number of Trials: {}".format(self.num_trials))
        self.num_peds = rospy.get_param('~num_peds', 10)
        logging.info("Number of Pedestrians: {}".format(self.num_peds))
        self.time_limit = rospy.get_param('~time_limit_sec', 90)
        logging.info("Time limit (sec): {}".format(self.time_limit))
        self.teleop = rospy.get_param('~teleop', False)
        self.position_mode = rospy.get_param('~position_mode', 'rand')
        if self.position_mode not in self.POSITION_MODES:
            msg = "Position mode must be one of {}".format(self.POSITION_MODES)
            logging.error(msg)
            rospy.signal_shutdown(msg)
        logging.info("Position mode: {}".format(self.position_mode))
        logging.info("Teleop mode: {}".format(self.teleop))
        self.trial_name = rospy.get_param('~trial_name')
        if not self.trial_name:
            msg = "_trial_name cannot be empty. Please provide a unique trial name to run a new trial or an existing trial name to run more episodes for this trial."
            logging.error(msg)
            rospy.signal_shutdown(msg)
        logging.info("Trial name: {}".format(self.trial_name))

        self.positions_sub = rospy.Subscriber("/social_sim/spawn_positions", PoseArray, self.positions_callback, queue_size=10)
        self.start_pub = rospy.Publisher("/social_sim/start_trial", TrialStart, queue_size=10)
        self.status_sub = rospy.Subscriber("/social_sim/is_running", Bool, self.status_callback, queue_size=10)
        self.info_sub = rospy.Subscriber("/social_sim/last_info", TrialInfo, self.info_callback, queue_size=10)

        # call to restart the trial runner
        self.reset_state()

        if not self.teleop:
            logging.info("Waiting for the move_base action server. Enable _teleop:=True to skip this")
            self.move_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
            self.move_client.wait_for_server()

        logging.warn("Waiting for a /social_sim/spawn_positions, /social_sim/is_running message")
        logging.warn("Please (re)start Unity")

        # NOTE: trial configuration occurs after the positions callback

        rospy.spin()


    def reset_state(self):
        # completely reset the trial runner
        self.last_info_msg_time = None
        self.last_status_msg_state = None
        self.last_status_msg_time = rospy.Time.now()
        self.is_trialing = None
        self.positions = None
        self.rows_to_write = []
        self.current_trial = 0
        # is set to True if repeating a trial, adding episodes
        self.repeat = False
        # set from the positions message
        self.timestamp = None
        # set from rosparams, the location where outputs are written/read (subsequent runs of a trial)
        self.output_path = os.path.join(self.output_folder, self.trial_name)
        try:
            os.makedirs(self.output_path)
        except OSError as e:
            if e.errno != errno.EEXIST:
                raise
        # current trial spawn and goal position
        self.spawn_pos = None
        self.target_pos = None
        # configure logging
        self.log_path = os.path.join(self.output_path, 'trial.log')
        logger = logging.getLogger()
        logger.setLevel(logging.INFO)
        fh = logging.FileHandler(self.log_path)
        fh.setLevel(logging.INFO)
        ch = logging.StreamHandler()
        ch.setLevel(logging.INFO)
        fm = logging.Formatter("%(asctime)s [%(levelname)s] %(message)s")
        fh.setFormatter(fm)
        ch.setFormatter(fm)
        logger.addHandler(ch)
        logger.addHandler(fh)
        logging.info("Configured logging to output to: {} and terminal".format(self.log_path))

    def positions_callback(self, positions_msg):
        if self.positions is None:
            self.positions = positions_msg.poses
            self.configure_trial(positions_msg.header.stamp)

    def persist_positions(self):
        with open(self.positions_path, 'w') as f:
            f.write(json.dumps(self.all_positions))

    def configure_trial(self, timestamp):
        ''' if the trial exists and there is a run config, repeat this
            otherwise, generate one according to the params
        '''
        self.timestamp = timestamp
        self.positions_path = os.path.join(self.output_path, 'positions.json')
        all_pos = [msg_ros_to_dict(p) for p in self.positions]
        # Make sure existing positions match the current simulator's available positions
        if os.path.exists(self.positions_path):
            logging.info("loading previous positions: {}".format(self.positions_path))
            with open(self.positions_path, 'r') as f:
                try:
                    self.all_positions = json.loads(f.read())
                except ValueError as e:
                    logging.error(e)
                    msg = "{} is probably empty or corrupted, try a new trial name or (at your own risk) delete the experiment folder: {}".format(self.positions_path, self.output_path)
                    logging.warn(msg)
                    rospy.signal_shutdown(msg)
                if self.all_positions['all'] != all_pos:
                    msg = "Incorrect configuration, trying to run more episodes in existing trial, but positions do not match:\nexisting: {}\n from Unity: {}".format(self.all_positions, all_pos)
                    logging.error(msg)
                    rospy.signal_shutdown(msg)
                self.spawn_positions = self.all_positions['spawn']
                self.target_positions = self.all_positions['target']
                self.people_positions = self.all_positions['people']
            self.repeat = True
        else:
            self.all_positions = {
                'all': all_pos,
                'spawn': {},
                'target': {},
                'people': {}
            }
            self.persist_positions()
            return

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
        '''
            if the trial is random position
        '''
        # if we're repeating a trial, use the replay params
        if self.repeat:
            trial_key = str(self.current_trial)
            if trial_key not in self.spawn_positions or trial_key not in self.target_positions:
                msg = "could not find trial {} in the existing positions, exiting".format(trial_key)
                logging.warn(msg)
                rospy.signal_shutdown(msg)
            spawn_pos = msg_dict_to_ros('geometry_msgs/Pose', self.spawn_positions[trial_key])
            target_pos = msg_dict_to_ros('geometry_msgs/Pose', self.target_positions[trial_key])
            people_poses = [msg_dict_to_ros('geometry_msgs/Pose', p) for p in self.people_positions[trial_key]]
            return spawn_pos, target_pos, people_poses
        # otherwise, random position for each trial
        if self.position_mode == 'once' and self.spawn_pos:
            # keep the current (first) spawn / target pos
            return self.spawn_pos, self.target_pos, self.people_poses
        # random choice
        print("Randomly choosing 1 of {} available positions".format(len(self.positions)))
        n = len(self.positions) - 1
        spawn_pos_idx = randint(0, n)
        n -= 1
        spawn_pos = self.positions[spawn_pos_idx]
        target_pos_idx = randint(0, n)
        n -= 1
        target_pos = self.positions[target_pos_idx]
        people_poses = []
        for i in range(self.num_peds):
            # re-use spawn positions, we we need to
            if n < 0:
                n = len(self.positions) - 1
            idx = randint(0, n)
            n -= 1
            people_poses.append(self.positions[idx])
        return spawn_pos, target_pos, people_poses

    def check_complete(self):
        ''' Called before every trial run
            Checks if we have completed the requested trials
        '''
        if self.current_trial < self.num_trials:
            return
        logging.info("Trials complete")
        # Save our data
        logging.info("Exiting")
        rospy.signal_shutdown("Trials Complete")

    def run_trial(self):
        self.check_complete()
        self.current_trial += 1
        logging.info("Running Trial {}".format(self.current_trial))

        # populates spawn
        self.spawn_pos, self.target_pos, self.people_poses = self.pick_positions()
        stamp = rospy.Time.now()
        people = PoseArray()
        people.header.stamp = stamp
        for pose in self.people_poses:
            people.poses.append(pose)

        # persist the positions
        self.all_positions['spawn'][self.current_trial] = msg_ros_to_dict(self.spawn_pos)
        self.all_positions['target'][self.current_trial] = msg_ros_to_dict(self.target_pos)
        self.all_positions['people'][self.current_trial] = [msg_ros_to_dict(pose) for pose in self.people_poses]
        self.persist_positions()

        # trial is starting, publish message
        trial_start_msg = TrialStart()
        trial_start_msg.header.stamp = stamp
        trial_start_msg.trial_name = self.trial_name
        trial_start_msg.trial_number = self.current_trial
        trial_start_msg.spawn = self.spawn_pos
        trial_start_msg.target = self.target_pos
        trial_start_msg.people = people
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
            #'trial_number': msg.trial_number,
            'trial_number' : self.current_trial + 1,
            'dist_to_target': msg.dist_to_target,
            'dist_to_ped': msg.dist_to_ped,
            'num_collisions': msg.num_collisions,
            'run_complete': msg.run_complete,
            'time_elapsed': msg.time_elapsed
        }
        self.rows_to_write.append(row)
        # update on disk each time a row is updated
        self.record_csv()

    def record_csv(self):
        if len(self.rows_to_write) <= 0:
            logging.error("ERROR: No rows to write")
            return
        fieldnames = self.rows_to_write[0].keys()
        csv_path = os.path.join(self.output_folder, '{}_{}.csv'.format(self.trial_name, self.rows_to_write[0]['timestamp']))
        with open(csv_path, 'w') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.rows_to_write:
                writer.writerow(row)
        logging.info("Write csv {}".format(csv_path))


if __name__ == "__main__":
    try:
        node = SocialSimRunner()
    except rospy.ROSInterruptException:
        pass
