#!/usr/bin/env python3

# Copyright (c) 2020, Members of Yale Interactive Machines Group, Yale University,
# Nathan Tsoi
# Juncheng Tang
# All rights reserved.
# This source code is licensed under the BSD-style license found in the
# LICENSE file in the root directory of this source tree.

import rospy
import numpy as np
import cv2
import message_filters
from std_msgs.msg import Header
from sensor_msgs.msg import CompressedImage, Image, CameraInfo, RegionOfInterest
from cv_bridge import CvBridge, CvBridgeError
from social_sim_ros.msg import RealDepthImage

MAX_RANGE=20
MIN_RANGE=0.1

def camera_info(stamp, frame_id):
    '''
    header: 
      seq: 2037
      stamp: 
        secs: 1629214336
        nsecs: 826989889
      frame_id: "kuri_rgb_link"
    height: 480
    width: 640
    distortion_model: "plumb_bob"
    D: []
    K: [415.6921691894531, 0.0, 320.0, 0.0, 415.6921691894531, 240.0, 0.0, 0.0, 1.0]
    R: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    P: [415.6921691894531, 0.0, 320.0, 0.0, 0.0, 415.6921691894531, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    binning_x: 0
    binning_y: 0
    roi: 
      x_offset: 0
      y_offset: 0
      height: 0
      width: 0
      do_rectify: False
    '''
    ci = CameraInfo()
    ci.header = Header()
    ci.header.stamp = stamp
    ci.header.frame_id = frame_id
    ci.height = 480
    ci.width = 640
    ci.distortion_model = "plumb_bob"
    ci.D = []
    ci.K = [415.6921691894531, 0.0, 320.0, 0.0, 415.6921691894531, 240.0, 0.0, 0.0, 1.0]
    ci.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    ci.P = [415.6921691894531, 0.0, 320.0, 0.0, 0.0, 415.6921691894531, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
    ci.binning_x = 0
    ci.binning_y = 0
    ci.roi = RegionOfInterest()
    ci.roi.x_offset = 0
    ci.roi.y_offset = 0
    ci.roi.height = 0
    ci.roi.width = 0
    ci.roi.do_rectify = False
    return ci


def image_callback(compressed_image):
    frame_id='center_depth_link'
    ci = camera_info(compressed_image.header.stamp, frame_id)
    arr = np.frombuffer(compressed_image.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    # NOTE: this might help if you're having trouble w/ the conversion: https://www.h-schmidt.net/FloatConverter/IEEE754.html
    # always keep the sign positive, ignoring the last bit from the shader
    img[:,:,3] = np.bitwise_and(img[:,:,3], np.full((img.shape[0],img.shape[1]), 0x01111111, np.uint8))
    # something is wrong w/ the exponent... but this fixes it!
    img32 = (img.view(dtype=np.single) * 1e39).astype(np.single)
    # RANGE LIMITS MUST MATCH CAMERA CLIPPING PLANES
    img32[img32 > MAX_RANGE] = np.Inf
    img32[img32 < MIN_RANGE] = np.NaN

    # debugging
    #print("img", img[int(img.shape[0]/2),int(img.shape[1]/2)])
    #print("img32", img32[int(img32.shape[0]/2),int(img32.shape[1]/2)])

    info_pub.publish(ci)
    try:
      image_message = bridge.cv2_to_imgmsg(img32, "32FC1")
      image_message.header.frame_id = frame_id
      image_message.header.stamp = compressed_image.header.stamp
      image_pub.publish(image_message)
    except CvBridgeError as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    rospy.init_node('depth_sync_publisher')

    rospy.Subscriber('/center_depth/compressed', CompressedImage, image_callback)

    bridge = CvBridge()
    info_pub = rospy.Publisher('/center_depth_sync/camera_info', CameraInfo, queue_size=10)
    image_pub = rospy.Publisher('/center_depth_sync/image_raw', Image, queue_size=10)


    #ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.5, allow_headerless=True)
    #ts.registerCallback(callback)

    print("ready")
    rospy.spin()
