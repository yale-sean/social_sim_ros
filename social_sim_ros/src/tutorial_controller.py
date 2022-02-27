#!/usr/bin/env python3  
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped

goal_pose = None
goal_msg = None
get_pos_once = False
def goal_callback(msg):
    global goal_pose, goal_msg
    goal_pose = msg.pose.position
    goal_msg = msg


if __name__ == '__main__':
    rospy.init_node('simple_controller')
    listener = tf.TransformListener()

    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=1)

    rate = rospy.Rate(10.0)

    angle = 0
    while not rospy.is_shutdown():
        if not goal_pose:
            continue
        try:
            #(goal_trans,goal_rot) = listener.lookupTransform(goal.header.frame_id, '/map', rospy.Time(0))
            (robot_trans,robot_rot) = listener.lookupTransform('/base_link', goal_msg.header.frame_id, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        # print("goal: ", goal_pose)
        # print("robot: ", robot_trans, robot_rot)

        if not get_pos_once:
            robot_to_goal = [goal_pose.x - robot_trans[0], goal_pose.y - robot_trans[1]]
            print("robot to goal: ", robot_to_goal)
            angle = math.atan2(robot_to_goal[1], robot_to_goal[0])
            # get_pos_once = True
            
        orientation = tf.transformations.euler_from_quaternion(robot_rot)[2]
        # if orientation < 0:
        #     orientation += math.pi * 2

        print("angle: ", angle * 180/math.pi )
        print("robot orientation: ", orientation * 180/math.pi)
        angular = (orientation - angle) * 0.1 #-0.5 * angle
        linear = 0



        cmd = Twist()
        cmd.linear.x = 0
        cmd.angular.z = angular

        print("cmd_vel: ", cmd)
        cmd_vel_publisher.publish(cmd)

        rate.sleep()