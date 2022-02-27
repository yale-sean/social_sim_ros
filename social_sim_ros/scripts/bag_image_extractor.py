#!/usr/bin/env python3


from calendar import c
from collections import defaultdict
from email.policy import default
import os
import cv2
import glob
import errno
import re
import json
import math
import time
import argparse
import numpy as np

# ros related imports 
import rosbag
# if no rosbag -> source /opt/ros/melodic/setup.sh 
import subprocess
import rospy

from cv_bridge import CvBridge
from geometry_msgs.msg import PoseArray, PoseStamped, TransformStamped, Twist, Pose, Point
#from social_sim_ros.msg import RealDepthImage
from sensor_msgs.msg import CompressedImage


bridge = CvBridge()
MAX_RANGE=20
MIN_RANGE=0.1

class ImageChain:
        """
        Class for storing consecutive RGB images (and their timestamps),
        as well as the 'dominating' social situation for the chain.
        The dominating social situation refers to the social situation with which we 
        would label a video clip made from the ImageChain.

        As an example, we could have an image chain 'x' with x.ss = 'down_path'
        and an image chain 'y' with y.ss = 'empty'. We could concatenate 'y' to 'x'
        if 'y' follows consecutively after 'x', and the chain 'x+y' would have (x+y).ss = 'down_path'.
        We might want to do this to pad some context to the clip for 'x', or if 'y' separates 'x'
        from an ImageChain that is also 'down_path'.
        
        If y.ss = 'cross_path', however, we would not want to combine 'x' and 'y' because
        the resulting ImageChain would have an ambiguous social situation type.
        """

        def __init__(self, social_situation: str, imgs: list):
            """NOTE: Must be initialized with at least one image in imgs"""
            self.ss = social_situation

            assert(len(imgs) > 0)
            self.imgs = imgs

            # print(f"IMGS IN INIT {imgs}")

            # Allow our chains to be at most x seconds long
            self.MAX_CHAIN_RUNTIME = 15
            
        def add_image(self, ts, img):
            self.imgs.append([ts, img])
            # print(f"IMGS IN AADD {self.imgs}")

        def join_with(self, other):
            """
            Joins this chain with 'other', adding that chains data to this chains internal fields.
            
            Returns True on success, False on failure.
            """

            # Verify conditions for joining chains
            if(self.ss != other.ss):
                # If the chains are not the same social situation, one must be 'empty'
                if not (self.ss == 'empty' or other.ss == 'empty'):
                    # print("FAIL", self.ss, other.ss)
                    return False

            if not self.mergeable or not other.mergeable:
                return False

            # Add images from 'other' to 'self'; first check the relative chain-ordering via timestamp
            if(self.start_time < other.start_time):
                # Check that adding these chains together won't make the resulting chain too long
                if((other.end_time-self.start_time)/1e9 > self.MAX_CHAIN_RUNTIME):
                    assert((other.end_time-self.start_time)/1e9 > 0)
                    # print("FAIL", (other.end_time-self.start_time)/1e9)
                    return False

                self.imgs = self.imgs + other.imgs
            else:
                # Check that adding these chains together won't make the resulting chain too long
                if((self.end_time-other.start_time)/1e9 > self.MAX_CHAIN_RUNTIME):
                    assert((self.end_time-other.start_time)/1e9 > 0)
                    # print("FAIL", (self.end_time-other.start_time)/1e9)
                    return False

                self.imgs = other.imgs + self.imgs
                self.start_time = other.start_time

            # Transfer the dominating social situation, if necessary
            if(self.ss == 'empty'):
                self.ss = other.ss

            return True

        @property
        def length(self):
            return len(self.imgs)

        @property
        def runtime(self):
            return (self.end_time - self.start_time) / 1e9

        @property
        def start_time(self):
            """Returns starting time of chain, in nanoseconds"""
            return self.imgs[0][0]

        @property
        def end_time(self):
            """Returns ending time of chain, in nanoseconds"""
            return self.imgs[-1][0]

        @property
        def mergeable(self):
            """
            For chains that are not of social situation 'empty', they can
            only be merged with other chains if they contain at least x
            number of seconds. This ensures after consolidation that chains
            labeled with a certain social situation contain adequate number of seconds
            of that social situation.
            """
            # if(self.ss == 'cross_path'):
            #     return (self.runtime > 0.9)

            return (self.runtime > 0.8)

def mkdirp(path):
    """Creates directory path 

    Args:
        path (str): path 
    """
    try:
        os.makedirs(path)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def read_raw_depth(msg):
    # TODO: maybe the bridge would work better?
    #img = bridge.compressed_imgmsg_to_cv2(msg)
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    img[:,:,3] = np.bitwise_and(img[:,:,3], np.full((img.shape[0],img.shape[1]), 0x01111111, np.uint8))
    # something is wrong w/ the exponent... but this fixes it!
    img32 = (img.view(dtype=np.single) * 1e39).astype(np.single)
    # RANGE LIMITS MUST MATCH CAMERA CLIPPING PLANES
    img32[img32 > MAX_RANGE] = np.Inf
    img32[img32 < MIN_RANGE] = np.NaN
    return img32


def read_rgb_images(msg):
    # for reading in rgb/compressed 
    arr = np.fromstring(msg.data, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_UNCHANGED)
    return img



def agents_in_range(msg, theta=1.0472, limit=15):
    '''
    theta (in radians) is the FOV/2, i.e. a theta of 1.0472 is 60 degrees from either side
        of the robot's forward vector.
    limit (in meters) is the range of the depth camera, though here we set it to 15m (<20m)
        to be conservative.
    required is the number of agents in range that is required for the function to return False

    Returns the number of agents in range (int)
    '''
    res = 0
    for agent in msg.poses:
        vec_forward_unit = np.array([1, 0])
        vec_person = np.array([agent.position.x, agent.position.y])
        dist = np.linalg.norm(vec_person)
        vec_person_unit = vec_person / dist
        angle = np.arccos(np.dot(vec_forward_unit, vec_person_unit))

        # check for the agent being directly in front of or behind the robot
        if(np.array_equal(vec_forward_unit, vec_person_unit) and dist < limit):
            res+=1
        elif(np.array_equal(vec_forward_unit, -1*vec_person_unit)):
            continue
        elif(angle < theta and dist < limit): # there is an agent in range
            res+=1

    return res


def make_image_dataset(args):
    ''' turns bags into torchvision compatible datasets 
        See: 
        https://github.com/pytorch/vision/blob/4ec38d496db69833eb0a6f144ebbd6f751cd3912/torchvision/datasets/folder.py#L57
    '''
    if os.path.isdir(args.path):
        args.path = os.path.join(args.path, '*.bag')
    bags = glob.iglob(args.path)
    metadata = {}
    metadata_scenes = {}
    metadata_classes = {}
    metadata_densities = {}
    for bagname in bags:
        bag = rosbag.Bag(bagname)
        print("processing bag for image dataset: %s" % bagname)
        ts, rest = bagname.split('/')[-1].split('.', 1)
        _, mode, scene, scenario, num_ppl = rest.split('.')[0].split('-')
        cls, density = scenario.split('_Density')
        cls = re.sub(r'\d$', '', cls) + "_" + density

        agent_count = 0
        if args.img_type == "rgb": 
            ## RGB Images  
            topics = ['/rgb/compressed']
        else: 
            ## Depth images 
            topics = ['/camera/depth/compressed', '/social_sim/transformed_agent_positions'] 
        
        # Looping through the topics 
        for topic, msg, t in bag.read_messages(topics=topics):
            ## for the image dataset 
            if topic == "/rgb/compressed": 
                np_arr = read_rgb_images(msg)
                output_path = os.path.join(args.output, cls)
                output_basename = str(t.to_nsec()) + ".png"
                output_file = os.path.join(output_path, output_basename)
                print(output_file)

            ## for the depth dataset 
            if topic == '/camera/depth/compressed':
                np_arr = read_raw_depth(msg)
                output_path = os.path.join(args.output, cls)
                output_basename = str(t.to_nsec()) + ".png"
                output_file = os.path.join(output_path, output_basename)
                
            if (topic =='/social_sim/transformed_agent_positions' and "empty" not in cls.lower()):
                agent_count = agents_in_range(msg)

            ## for all datasets, persisting metadata
            if scene not in metadata_scenes:
                metadata_scenes[scene] = 0
            metadata_scenes[scene] += 1
            if cls not in metadata_classes:
                metadata_classes[cls] = 0
            metadata_classes[cls] += 1
            if density not in metadata_densities:
                metadata_densities[density] = 0
            metadata_densities[density] += 1
            if cls not in metadata:
                metadata[cls] = {}
            metadata[cls][output_basename] = {
                'nanoseconds': t.to_nsec(),
                'basename': output_basename,
                'file': output_file,
                'scene': scene,
                'class': cls,
                'density': density,
                'num_ppl': num_ppl,
                'num_ppl_in_fov': agent_count
            }
            
            # WRITING OUT IMAGE 
            mkdirp(output_path)
            cv2.imwrite(output_file, np_arr)

        bag.close()

    metadata_file = os.path.join(args.output, 'metadata.json')
    with open(metadata_file, 'w') as f:
        json.dump({
            'scenes': metadata_scenes,
            'classes': metadata_classes,
            'densities': metadata_densities,
            'files': metadata
        }, f, ensure_ascii=False, indent=2)
    print("DONE! metadata: %s" % metadata_file)


""" CODE TO MAKE GROUND TRUTH PEOPLE DATASET (via API) """
def make_gt_people_dataset(args):
    ''' exctracts the ground truth people positions from bags
    '''
    if os.path.isdir(args.path):
        args.path = os.path.join(args.path, '*.bag')
    bags = glob.iglob(args.path)
    metadata = {}
    metadata_scenes = {}
    metadata_classes = {}
    metadata_densities = {}

    for bagname in bags:
        bag = rosbag.Bag(bagname)
        print("processing bag for people dataset: %s" % bagname)
        ts, rest = bagname.split('/')[-1].split('.', 1)
        _, mode, scene, scenario, num_ppl = rest.split('.')[0].split('-')
        cls, density = scenario.split('_Density')
        cls = re.sub(r'\d$', '', cls) + "_" + density

        tf_t = tf.TransformerROS(True, rospy.Duration(3600.0))
        for topic, msg, t in bag.read_messages(topics=['/tf']):
            for msg_tf in msg.transforms:
                msg_tf.header.stamp = t
                tf_t.setTransform(msg_tf)

        n = args.people_seq_len
        ns_gap = 1e9 / args.people_freq_hz
        last_ns = 0
        n_pose_arrs = []
        for topic, msg, t in bag.read_messages(topics=['/social_sim/agent_positions']):
            # drop the messages that are not transformable
            if not tf_t.canTransform('base_link', 'map', t):
                continue

            # limit frequency of data
            cur_ns = t.to_nsec()
            if cur_ns - last_ns < ns_gap:
                continue

            n_pose_arrs.append(msg.poses)
            last_ns = cur_ns
            # transform and save data when reaching n samples
            if len(n_pose_arrs) == n:
                timestamp = t.to_nsec()
                pose_arrs_seq = []
                for poses in n_pose_arrs:
                    pose_arr = []
                    for pose in poses:
                        # transform to robot's frame at last timestamp
                        msg.header.stamp = t
                        msg.header.frame_id = 'map'
                        temp_pose = PoseStamped(header=msg.header, pose=pose)
                        pose = tf_t.transformPose('base_link', temp_pose).pose
                        position = [pose.position.x,
                                    pose.position.y,
                                    pose.position.z]
                        orientation = [pose.orientation.x,
                                       pose.orientation.y,
                                       pose.orientation.z,
                                       pose.orientation.w]
                        pose_arr.append([position, orientation])
                    pose_arrs_seq.append(pose_arr)

                # calculate distances to robot at last timestamp
                distances = []
                for pos, rot in pose_arr:
                    x, y, z = pos
                    distances.append(math.sqrt(x ** 2 + y ** 2))

                # persist metadata
                if scene not in metadata_scenes:
                    metadata_scenes[scene] = 0
                metadata_scenes[scene] += 1
                if cls not in metadata_classes:
                    metadata_classes[cls] = 0
                metadata_classes[cls] += 1
                if density not in metadata_densities:
                    metadata_densities[density] = 0
                metadata_densities[density] += 1
                if cls not in metadata:
                    metadata[cls] = {}
                metadata[cls][timestamp] = {
                    'nanoseconds': timestamp,
                    'agent_positions_sequence': pose_arrs_seq,
                    'agent_distances': distances,
                    'scene': scene,
                    'class': cls,
                    'density': density,
                    'num_ppl': num_ppl
                }

                # reset array of agents positions sequence
                n_pose_arrs = []
        

        bag.close()

    metadata_file = os.path.join(args.output, 'metadata.json')
    with open(metadata_file, 'w') as f:
        json.dump({
            'scenes': metadata_scenes,
            'classes': metadata_classes,
            'densities': metadata_densities,
            'files': metadata
        }, f, ensure_ascii=False, indent=2)
    print("DONE! metadata: %s" % metadata_file)




""" CODE TO MAKE DEPTH ROBOT VEL DATASET (via python api)"""

def make_depth_robot_vel_dataset(args):
    # make cmd_vel jsons with same name as image
    # for use as the "label" for training a new robot controller via behavior cloning

    if os.path.isdir(args.path):
        args.path = os.path.join(args.path, '*.bag')
    bags = glob.iglob(args.path)
    metadata = {}
    metadata_scenes = {}
    metadata_classes = {}
    metadata_densities = {}
    metadata_densities = {}

    for bagname in bags:
        # reset variables
        last_cmd_vel = {
                'linear_x': 0,
                'linear_y': 0,
                'linear_z': 0,
                'angular_x': 0,
                'angular_y': 0,
                'angular_z': 0
            }
        cmd_vel_flag = False # set to true once we've seen a cmd_vel
        
        print("processing %s for depth_robot_vel dataset" % bagname)

        bag = rosbag.Bag(bagname)
        ts, rest = bagname.split('/')[-1].split('.', 1)
        _, mode, scene, scenario, num_ppl = rest.split('.')[0].split('-')
        cls, density = scenario.split('_Density')
        cls = re.sub(r'\d$', '', cls) + "_" + density

        output_path = os.path.join(args.output, cls)

        for topic, msg, t in bag.read_messages(topics=["/mobile_base_controller/cmd_vel", "/camera/depth/compressed"]):
            if(topic == "/mobile_base_controller/cmd_vel"):
                cmd_vel_flag = True
                last_cmd_vel['linear_x'] = msg.linear.x
                last_cmd_vel['linear_y'] = msg.linear.y
                last_cmd_vel['linear_z'] = msg.linear.z
                last_cmd_vel['angular_x'] = msg.angular.x
                last_cmd_vel['angular_y'] = msg.angular.y
                last_cmd_vel['angular_z'] = msg.angular.z

            elif(topic == "/camera/depth/compressed" and cmd_vel_flag):
                np_arr =  read_raw_depth(msg)

                output_basename = str(t.to_nsec()) # no prefix
                output_file_vel = os.path.join(output_path, output_basename+ ".json")
                output_file_image = os.path.join(output_path, output_basename + ".png")

                mkdirp(output_path)
                cv2.imwrite(output_file_image, np_arr)
                with open(output_file_vel, 'w') as f:
                    json.dump(last_cmd_vel, f, ensure_ascii=False, indent=2)

                # persist metadata
                if scene not in metadata_scenes:
                    metadata_scenes[scene] = 0
                metadata_scenes[scene] += 1
                if cls not in metadata_classes:
                    metadata_classes[cls] = 0
                metadata_classes[cls] += 1
                if density not in metadata_densities:
                    metadata_densities[density] = 0
                metadata_densities[density] += 1
                if cls not in metadata:
                    metadata[cls] = {}
                metadata[cls][output_basename] = {
                    'nanoseconds': t.to_nsec(),
                    'basename': output_basename,
                    'file': output_file_image,
                    'vel_file': output_file_vel,
                    'scene': scene,
                    'class': cls,
                    'density': density,
                    'num_ppl': num_ppl
                }
        bag.close()

    metadata_file = os.path.join(args.output, 'metadata.json')
    with open(metadata_file, 'w') as f:
        json.dump({
            'scenes': metadata_scenes,
            'classes': metadata_classes,
            'densities': metadata_densities,
            'files': metadata
        }, f, ensure_ascii=False, indent=2)
    print("DONE! metadata: %s" % metadata_file)

def make_social_situation_clips(args):
    """
    Given the input bags, extract 10s clips of each social situation from the bags
    and place them in the output directory.
    """

    if os.path.isdir(args.path):
        args.path = os.path.join(args.path, '*.bag')
    bags = glob.iglob(args.path)

    TOPIC_FOR_VIDEO = '/robot_thirdperson_rgb/compressed' # '/robot_firstperson_rgb/compressed'
    topics = [
        TOPIC_FOR_VIDEO,
        '/social_sim/scene_info',
        '/social_sim/situations/rule_based/cross_path',
        '/social_sim/situations/rule_based/down_path',
        '/social_sim/situations/rule_based/empty',
        '/social_sim/situations/rule_based/join_group',
        '/social_sim/situations/rule_based/leave_group',
    ] 

    social_situations = [
        'down_path',
        'join_group',
        'leave_group',
        'cross_path',
        'empty'
    ]

    for ss in social_situations: 
        output_path = os.path.join(args.output, ss)
        mkdirp(output_path)

    skipped = 0
    total = 0
    ss_idx = defaultdict(int)
    for bagname in bags:
        bag = rosbag.Bag(bagname)
        print()
        print("processing bag for social situation clips: %s" % bagname)
        # ts, rest = bagname.split('/')[-1].split('.', 1)

        # store "chains" images to be concatenated into clips, along with their timestamps and ss
        # each chain is made up of consecutive images from the same social situation
        chains = []

        # NOTE: We default to empty, even when empty is not triggered
        ss_dict = defaultdict(int)

        # Looping through the topics 
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic == TOPIC_FOR_VIDEO: 
                np_arr = read_rgb_images(msg)

                # NOTE: The case where two social situations are active at the same time is unhandled
                latest_ss = 'empty' # default to empty
                for ss in ss_dict:
                    if(ss_dict[ss] == 1):
                        latest_ss = ss

                # Case that we have no chains or can't add to an existing chain
                if(len(chains) == 0 or latest_ss != chains[-1].ss):
                    chains.append(ImageChain(latest_ss, [[t.to_nsec(), np_arr]]))
                # Case that we can add to an existing chain
                else:
                    chains[-1].add_image(t.to_nsec(), np_arr)
            
            elif topic == '/social_sim/situations/rule_based/cross_path':
                ss_dict['cross_path'] = msg.data
            elif topic == '/social_sim/situations/rule_based/down_path':
                ss_dict['down_path'] = msg.data
            elif topic == '/social_sim/situations/rule_based/join_group':
                ss_dict['join_group'] = msg.data
            elif topic == '/social_sim/situations/rule_based/leave_group':
                ss_dict['leave_group'] = msg.data

        bag.close()

        # Summarize the chains we extracted
        average_length = np.mean([chain.length for chain in chains])
        average_runtime = np.mean([chain.runtime for chain in chains])
        total_runtime = np.sum([chain.runtime for chain in chains])
        print(f"Before consolidation ({len(chains)} chains, {total_runtime:.2f}s total), the average chain length is {average_length:.2f} and runtime is {average_runtime:.2f}s")

        # Consolidate the chains
        temp = [] # finished chains
        p = 1
        while(len(chains) > 1):
            if(chains[0].join_with(chains[1])):
                # print("SUCCESS")
                chains.pop(1)
            else:
                temp.append(chains[0])
                chains.pop(0)

        temp =  temp + chains
        chains = temp

        average_length = np.mean([chain.length for chain in chains])
        average_runtime = np.mean([chain.runtime for chain in chains])
        total_runtime = np.sum([chain.runtime for chain in chains])
        print(f"After consolidation ({len(chains)} chains, {total_runtime:.2f}s total), the average chain length is {average_length:.2f} and runtime is {average_runtime:.2f}s")

        total += len(chains)
        for _, chain in enumerate(chains):

            if chain.runtime < 5 or chain.runtime > 15:
                skipped += 1
                continue

            # estimate the fps of the chain
            size = chain.imgs[0][1].shape
            fps = chain.length / chain.runtime
            out = cv2.VideoWriter(
                f'{args.output}/{chain.ss}/video-{ss_idx[chain.ss]}.mp4', 
                cv2.VideoWriter_fourcc(*'avc1'), 
                fps, 
                (size[1], size[0]), 
                isColor=True
            )
            ss_idx[chain.ss] += 1

            for _, img in chain.imgs:
                out.write(img)

            out.release()
            print(f"* Built chain ({chain.ss}) with runtime {chain.runtime}")

    print(f"Done. Built {total-skipped}/{total} chains.")


def main():
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('--path', dest='path', type=str,
                        help='path for bags', required=True)
    parser.add_argument('--output', dest='output', type=str,
                        help='output path', required=True)
    parser.add_argument('--type', dest='type', type=str,
                        choices=['image', 'people', 'vel', 'ss_clips'],
                        help='type of dataset', required=True)

    parser.add_argument('--img_type', dest='img_type', type=str,
                        choices=['depth', 'rgb'],
                        help='type of image', required=True)

    parser.add_argument('--people-seq-len', dest='people_seq_len', type=int,
                        help='length of people positions sequence', required=False)
    parser.add_argument('--people-freq-hz', dest='people_freq_hz', type=int,
                        help='desired frequency of people positions frames', required=False)
    args = parser.parse_args()

    if args.type == 'image':
        print("This function is deprecated, check/update topic names, bag names, and other formats!")
        make_image_dataset(args)
    elif args.type == 'people':
        print("This function is deprecated, check/update topic names, bag names, and other formats!")
        make_gt_people_dataset(args)
    elif args.type == 'vel':
        print("This function is deprecated, check/update topic names, bag names, and other formats!")
        make_depth_robot_vel_dataset(args)
    elif args.type == 'ss_clips':
        make_social_situation_clips(args)

if __name__ == '__main__':
    main()


# python src/social_sim_ros/social_sim_ros/scripts/bag_image_extractor.py --path '/data/1614403340.48-interactive-RobotControlSmallWarehouseScene-Empty1_Density0-0.bag' --output /data/tmp --type vel

"""
Some unit tests for no_agents_in_range:

test_msg = PoseArray(poses=[Pose(Point(1, 1, 0), 0)])
print("False", no_agents_in_range(test_msg))

test_msg = PoseArray(poses=[Pose(Point(1, 100, 0), 0)])
print("True", no_agents_in_range(test_msg))

test_msg = PoseArray(poses=[Pose(Point(1, 100, 0), 0), Pose(Point(1, 100, 0), 0), Pose(Point(1, 1, 0), 0)])
print("False test", no_agents_in_range(test_msg))

test_msg = PoseArray(poses=[Pose(Point(10, 0, 0), 0)])
print("False test (directly in front)", no_agents_in_range(test_msg))

test_msg = PoseArray(poses=[Pose(Point(-10, 0, 0), 0)])
print("True test (directly behind)", no_agents_in_range(test_msg))

test_msg = PoseArray(poses=[Pose(Point(15, 0, 0), 0)])
print("True test (out of range)", no_agents_in_range(test_msg))

"""
