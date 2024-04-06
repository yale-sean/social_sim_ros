#!/usr/bin/env python3

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService, UnityService
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import PoseStamped, PoseArray, Twist
from social_sim_ros.msg import SceneInfo, TrialInfo, TrialStart, AgentArray
from std_msgs.msg import String, Bool, Float64, Float32MultiArray, Float32
from nav_msgs.msg import Odometry, Path

from sensor_msgs.msg import CompressedImage, Image, CameraInfo, LaserScan
# from robotics_demo.msg import PosRot, UnityColor
# from robotics_demo.srv import PositionService, ObjectPoseService

from tcp_server_tf import RosTFBroadcaster


def main():
    ros_node_name = 'tcp_server'
    rospy.init_node(ros_node_name, anonymous=True)

    buffer_size = rospy.get_param("/TCP_BUFFER_SIZE", 1024)
    connections = rospy.get_param("/TCP_CONNECTIONS", 10)

    ip = rospy.get_param('~ip', '0.0.0.0')
    port = rospy.get_param('~port', 10000)
    tcp_server = TcpServer(ros_node_name, buffer_size, connections, tcp_ip=ip, tcp_port=port)

    tcp_server.start({
        # simulated clock
        '/clock': RosPublisher('/clock', Clock, queue_size=10),

        # trial info
        '/social_sim/scene_info' : RosPublisher('/social_sim/scene_info', SceneInfo,queue_size=10),
        '/social_sim/metrics' : RosPublisher('/social_sim/metrics', TrialInfo, queue_size=10),
        '/social_sim/agents' : RosPublisher('/social_sim/agents', AgentArray, queue_size=10),
        '/social_sim/agent_positions' : RosPublisher('/social_sim/agent_positions', PoseArray, queue_size=10),
        '/social_sim/group_positions' : RosPublisher('/social_sim/group_positions', PoseArray, queue_size=10),

        # social sim control and status
        '/social_sim/control/task/new': RosSubscriber('/social_sim/control/task/new', Bool, tcp_server),

        # implemented by InputPublisher
        '/social_sim/trigger' : RosPublisher('/social_sim/trigger', Bool, queue_size=10),
        '/social_sim/cmd_vel' : RosPublisher('/social_sim/cmd_vel', Twist, queue_size=10),

        # implemented by SituationRuleBased
        '/social_sim/situations/rule_based/cross_path' : RosPublisher('/social_sim/situations/rule_based/cross_path', Float32, queue_size=10),
        '/social_sim/situations/rule_based/down_path' : RosPublisher('/social_sim/situations/rule_based/down_path', Float32, queue_size=10),
        '/social_sim/situations/rule_based/empty' : RosPublisher('/social_sim/situations/rule_based/empty', Float32, queue_size=10),
        '/social_sim/situations/rule_based/join_group' : RosPublisher('/social_sim/situations/rule_based/join_group', Float32, queue_size=10),
        '/social_sim/situations/rule_based/leave_group' : RosPublisher('/social_sim/situations/rule_based/leave_group', Float32, queue_size=10),
        
        # TF
        # implemented by WorldTransformPublisher
        '/map_to_odom' : RosTFBroadcaster('odom', 'map'),
        '/map_to_base_link' : RosTFBroadcaster('base_link', 'map'),
        # implemented by RelativeTransformPublisher
        '/base_link_to_firstperson_rgb_link' : RosTFBroadcaster('firstperson_rgb_link', 'base_link'),
        '/base_link_to_thirdperson_rgb_link' : RosTFBroadcaster('thirdperson_rgb_link', 'base_link'),
        '/base_link_to_overhead_rgb_link' : RosTFBroadcaster('overhead_rgb_link', 'base_link'),
        '/base_link_to_center_depth_link' : RosTFBroadcaster('center_depth_link', 'base_link'),
        '/base_link_to_center_depth_optical_frame' : RosTFBroadcaster('center_depth_optical_frame', 'base_link'),
        '/base_link_to_laser' : RosTFBroadcaster('laser', 'base_link'),
        '/base_link_to_front_right_wheel_link' : RosTFBroadcaster('front_right_wheel_link', 'base_link'),
        '/base_link_to_front_left_wheel_link' : RosTFBroadcaster('front_left_wheel_link', 'base_link'),

        # odom
        '/robot_odom' : RosPublisher('/robot_odom', Odometry),

        # implemented in CreateMap
        '/short_map/compressed' : RosPublisher('short_map/compressed', CompressedImage),
        '/tall_map/compressed' : RosPublisher('tall_map/compressed', CompressedImage),
        
        # images implemented in classs that derive from BaseCameraPublisher
        '/center_depth/camera_info' : RosPublisher('/center_depth/camera_info', CameraInfo),
        '/center_depth/compressed' : RosPublisher('/center_depth/compressed', CompressedImage),
        '/agent_rgb/camera_info' : RosPublisher('/agent_rgb/camera_info', CameraInfo),
        '/agent_rgb/compressed' : RosPublisher('/agent_rgb/compressed', CompressedImage),
        '/robot_firstperson_rgb/camera_info' : RosPublisher('/robot_firstperson_rgb/camera_info', CameraInfo),
        '/robot_firstperson_rgb/compressed': RosPublisher('/robot_firstperson_rgb/compressed', CompressedImage),
        '/robot_thirdperson_rgb/camera_info' : RosPublisher('/robot_thirdperson_rgb/camera_info', CameraInfo),
        '/robot_thirdperson_rgb/compressed' : RosPublisher('/robot_thirdperson_rgb/compressed', CompressedImage),
        '/robot_overhead_rgb/camera_info' : RosPublisher('/robot_overhead_rgb/camera_info', CameraInfo),
        '/robot_overhead_rgb/compressed': RosPublisher('/robot_overhead_rgb/compressed', CompressedImage),

        # player cameras
        '/player_firstperson_rgb/camera_info' : RosPublisher('/player_firstperson_rgb/camera_info', CameraInfo),
        '/player_firstperson_rgb/compressed': RosPublisher('/player_firstperson_rgb/compressed', CompressedImage),

        '/laser_raycast' : RosPublisher('/laser_raycast', LaserScan, queue_size=10),

        # low level motor control from ROS (TODO)
        '/wheel_left_joint_cmd' : RosSubscriber('/wheel_left_joint_cmd', Float64, tcp_server),
        '/wheel_right_joint_cmd' : RosSubscriber('/wheel_right_joint_cmd', Float64, tcp_server),

        # goal
        '/move_base_simple/goal' : RosPublisher('/move_base_simple/goal', PoseStamped),
        '/move_base/GlobalPlanner/plan' : RosSubscriber('/move_base/GlobalPlanner/plan', Path, tcp_server),
        '/mobile_base_controller/cmd_vel' : RosSubscriber('/mobile_base_controller/cmd_vel', Twist, tcp_server),

        # learner
        '/lifecycle_learner/local_plan' : RosSubscriber('/lifecycle_learner/local_plan', Path, tcp_server),
        '/lifecycle_learner/global_plan' : RosSubscriber('/lifecycle_learner/global_plan', Path, tcp_server),
        '/lifecycle_learner/attention_l' : RosSubscriber('/lifecycle_learner/attention_l', Float32MultiArray, tcp_server),
        '/lifecycle_learner/attention_cmd_vel' : RosSubscriber('/lifecycle_learner/attention_cmd_vel', Float32MultiArray, tcp_server),
        '/lifecycle_learner/social_situation' : RosSubscriber('/lifecycle_learner/social_situation', String, tcp_server),
        '/lifecycle_learner/ss_debug_vector': RosSubscriber('/lifecycle_learner/ss_debug_vector', Float32MultiArray, tcp_server),

        '/mobile_base_controller/cmd_vel': RosSubscriber('/mobile_base_controller/cmd_vel', Twist, tcp_server),
    })

    rospy.spin()


if __name__ == "__main__":
    main()
