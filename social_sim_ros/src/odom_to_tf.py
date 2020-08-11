#!/usr/bin/env python
'''
Nathan Tsoi (2020)
copy the messages /map_to_odom and /odom_to_robot from Unity to the tf tree
'''
import rospy
import tf
from geometry_msgs.msg import PoseStamped


def map_to_odom_callback(msg):
    br = tf.TransformBroadcaster()
    p = msg.pose.position
    o = msg.pose.orientation
    br.sendTransform((p.x, p.y, p.z),
                     (o.x, o.y, o.z, o.w),
                     msg.header.stamp,
                     "odom",
                     "map")

def odom_to_robot_callback(msg):
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

    publish_map_frame = rospy.get_param('~publish_map_frame', False)
    print("Publish Map Frame: {}".format(publish_map_frame))

    if publish_map_frame:
        rospy.Subscriber("/map_to_odom", PoseStamped, map_to_odom_callback)
    rospy.Subscriber("/odom_to_robot", PoseStamped, odom_to_robot_callback)
    rospy.spin()
