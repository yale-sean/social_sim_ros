#!/usr/bin/env python
'''
Nathan Tsoi (2020)
copy the /odom topic message from Unity to the tf tree
'''
import rospy
import tf
from geometry_msgs.msg import PoseStamped


def callback(msg):
    #print(msg)
    br = tf.TransformBroadcaster()
    p = msg.pose.position
    o = msg.pose.orientation
    br.sendTransform((p.x, p.y, p.z),
                     (o.x, o.y, o.z, o.w),
                     msg.header.stamp,
                     "base_link",
                     "odom")

if __name__ == '__main__':
    rospy.init_node('odom_tf_broadcaster')
    rospy.Subscriber("/odom", PoseStamped, callback)
    rospy.spin()
