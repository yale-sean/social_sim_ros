#!/usr/bin/env python
'''
run social sim trials
'''
import rospy
import tf

# TODO
#import social_sim_ros.msg import TrialStatus, TrialStart

class SocialSimRunner(object):
    def __init__(self):
        rospy.init_node("social_sim_runner")
        self.current_trial = 0;
        self.is_trialing = False
        # TODO
        #self.trial_status_sub = rospy.Subscriber("/social_sim/trial_status", TrialStatus, self.status_callback, queue_size=10)
        #self.trial_starter_pub = rospy.Publisher("/social_sim/trial_status", TrialStart, queue_size=10)


        self.num_trials = rospy.get_param('~num_trials')
        print(self.num_trials)

    def status_callback(self, trial_status_msg):
        # record results to csv
        self.is_trialing = trial_status_msg.running;
        if not self.is_trialing:
            self.run_trial()

    def run_trial(self):
        trialStart = TrialStart()
        self.trial_starter_pub.publish(trialStart)



if __name__ == "__main__":
    try:
        node = SocialSimRunner()
        node.run_trial()
    except rospy.ROSInterruptException:
        pass
