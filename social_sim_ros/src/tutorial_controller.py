#!/usr/bin/env python3
# Walkthrough of this code: https://sean.interactive-machines.come/tutorials/controller
import rospy
import math
import tf
from geometry_msgs.msg import Twist, PoseStamped

goal_pose = None
goal_msg = None

def goal_callback(msg):
    global goal_pose, goal_msg
    goal_pose = msg.pose.position
    goal_msg = msg


if __name__ == '__main__':
    rospy.init_node('simple_controller')
    rospy.Subscriber("/move_base_simple/goal", PoseStamped, goal_callback)

    listener = tf.TransformListener()

    cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist,queue_size=1)

    rate = rospy.Rate(10.0)

    angle = 0
    while not rospy.is_shutdown():
        if not goal_pose:
            continue
        try:
            (robot_trans,robot_rot) = listener.lookupTransform(goal_msg.header.frame_id, '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        robot_to_goal = [goal_pose.x - robot_trans[0], goal_pose.y - robot_trans[1]]
        # print("robot to goal: ", robot_to_goal)
        angle = math.atan2(robot_to_goal[1], robot_to_goal[0])

        orientation = tf.transformations.euler_from_quaternion(robot_rot)[2]

        # print("angle: ", angle * 180/math.pi )
        # print("robot orientation: ", orientation * 180/math.pi)

        # Simple p controller
        p_ang = 0.5
        p_lin = 0.2
        robot_min_spd = 0.1
        robot_max_spd = 0.5
        angular = ((((angle - orientation) + math.pi) % (2 * math.pi)) - math.pi) * p_ang
        linear = math.sqrt(robot_to_goal[0]**2 + robot_to_goal[1]**2) * p_lin
        linear = max(robot_min_spd, min(robot_max_spd, linear))

        cmd = Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        # print("cmd_vel: ", cmd)
        cmd_vel_publisher.publish(cmd)

        rate.sleep()
