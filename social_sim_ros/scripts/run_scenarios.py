#!/usr/bin/env python3
import rospy

import argparse
import os
import random
import time

from rosgraph_msgs.msg import Clock

import psutil
import subprocess
import shlex
import signal

SCENES = [
    "RobotControlSmallWarehouseScene",
    "RobotControlOutdoorScene",
    "RobotControlLabScene",
    # "AgentControlLabScene",
    # "AgentControlLabScene",
]

GRAPH_SCENES = [
    "RobotControlSmallWarehouseSceneGraph",
    "RobotControlOutdoorSceneGraph",
    "RobotControlLabSceneGraph"
]

scenarios = [
    "Empty1_Density0",
    "JoinGroup1_Density0",
    "JoinGroup2_Density0",
    "LeaveGroup1_Density0",
    "LeaveGroup2_Density0",
    "NavigateDownPathway1_Density1",
    "NavigateDownPathway1_Density2",
    "NavigateDownPathway2_Density1",
    "NavigateDownPathway2_Density2",
    "NavigateDownPathway3_Density1",
    "NavigateDownPathway3_Density2",
    "CrossPath1_Density1",
    "CrossPath1_Density2",
    #"CrossPathJoinGroup1_Density1",
    #"CrossPathJoinGroup1_Density2",
]

scenarios_short = {
    "Empty1_Density0",
    "JoinGroup1_Density0",
    "LeaveGroup1_Density0",
    "NavigateDownPathway1_Density1",
    "CrossPath1_Density1"
}

start_locations = [0,1,2,3,4]
end_locations = [0,1,2,3,4]

def signal_process_and_children(pid, signal_to_send, wait=False):
    process = psutil.Process(pid)
    for children in process.children(recursive=True):
        if signal_to_send == 'suspend':
            children.suspend()
        elif signal_to_send == 'resume':
            children.resume()
        else:
            children.send_signal(signal_to_send)
    if wait:
        process.wait()


class ScenarioRunner(object):
    def __init__(self, args, cmds):
        self.args = args
        self.last_ts = None
        self.process = None
        self.cmds = cmds
        rospy.init_node('scenario_runner')

        with open(self.args.log_file, 'w') as f:
            formatted_cmds = '\n  '.join(self.cmds)
            f.write(f"Queued \n{self.cmds}")

        rospy.Subscriber("/clock", Clock, self.clock_callback)

        # Recording is also stopped on node shutdown. This allows stopping to be done via service call or regular Ctrl-C
        rospy.on_shutdown(self.stop)

        print("ready")

        self.run_next()

        while not rospy.is_shutdown():
            self.check()
            # depends on the simulator clock running!
            #rospy.sleep(1.0)
            #rospy.wallsleep(1.0)


    def clock_callback(self, msg):
        self.last_ts = msg.clock
        #print(f"last_ts: {self.last_ts}")


    def run_next(self):
        if len(self.cmds) < 1:
            return False

        next_cmd = self.cmds.pop(0)
        self.run_one(next_cmd)

        with open(self.args.log_file, 'a') as f:
            formatted_cmds = '\n'.join(next_cmd)
            f.write(f"Running: {formatted_cmds}")


    def run_one(self, cmd_str):
        if self.last_ts is not None:
            cmd_str += f" --start-clock {self.last_ts.to_sec() * 1000}"
        self.cmd = shlex.split(cmd_str)
        self.process = subprocess.Popen(self.cmd)
        rospy.loginfo(f"Started {' '.join(self.cmd)}")


    def check(self):
        # blocking
        #print("Check")

        # process isnt running yet
        if self.process is None:
            return

        # p.subprocess is alive when poll is None
        if self.process.poll() is None:
            return

        # start the next one
        self.run_next()

        #try:
        #    outs, errs = self.process.communicate(timeout=1.0)
        #    print("  Success")
        #except subprocess.TimeoutExpired:
        #    print("  TimeoutExpired")
        #    return None


    def stop(self):
        if self.process is not None:
            signal_process_and_children(self.process.pid, signal.SIGINT, wait=True)
            self.pid = None
            rospy.loginfo(f"Stopped {' '.join(self.cmd)}")


def main():
    log_file = f"/data/run_scenarios-{time.time()}.log"
    parser = argparse.ArgumentParser(description='generate script to run all scenarios')
    parser.add_argument('--shuffle', action='store_true', help='shuffle the output')
    parser.add_argument('--short', action='store_true', help='only run 3 scenarios')
    parser.add_argument('--graph', action='store_true', help='use graph scenes')
    parser.add_argument('--interactive', action='store_true', help='run in interactive mode')
    parser.add_argument('--log-file', default=log_file, help='output file name')
    parser.add_argument('--time_limit', type=float, default=3.0, help='set time until scene times out')
    parser.add_argument('--dest_thresh', type=float, default=1.0, help='end the scenario once the robot comes w/in this far of the goal position')
    parser.add_argument('--repeat', type=int, default=1, help='run each command this many times')

    parser.add_argument('--only-warehouse', action='store_true', help='only use the warehouse scene')
    parser.add_argument('--only-outdoor', action='store_true', help='only use the outdoor scene')
    parser.add_argument('--only-lab', action='store_true', help='only use the lab scene')

    args = parser.parse_args()

    cmds = []
    cmds_run = []

    scenes = SCENES
    if args.graph:
         scenes = GRAPH_SCENES
    if args.only_warehouse or args.only_outdoor or args.only_lab:
        scenes = []
        if args.only_warehouse:
            if args.graph:
                scenes.append("RobotControlSmallWarehouseSceneGraph")
            else:
                scenes.append("RobotControlSmallWarehouseScene")
        if args.only_outdoor:
            if args.graph:
                scenes.append("RobotControlOutdoorSceneGraph")
            else:
                scenes.append("RobotControlOutdoorScene")
        if args.only_lab:
            if args.graph:
                scenes.append("RobotControlLabSceneGraph")
            else:
                scenes.append("RobotControlLabScene")

    if args.short:
        for scene in scenes:
            for scenario in scenarios_short:
                cmd = ' '.join([
                    os.path.expanduser("~/social_sim_unity/Build/SEAN.x86_64"),
                    "-scene {}".format(scene),
                    "-scenarios {}".format(scenario),
                    "-robot_locations {}".format(start_locations[0]),
                    "-target_locations {}".format(end_locations[0]),
                    "-interactive {}".format('true' if args.interactive else 'false'),
                    "-time_limit {}".format(args.time_limit),
                    "-dest_thresh {}".format(args.dest_thresh)
                ])
                cmds.append(cmd)
                

    else:
        for scene in scenes:
            for scenario in scenarios:
                for start in start_locations:
                    for end in end_locations:
                        cmd = ' '.join([
                            os.path.expanduser("~/social_sim_unity/Build/SEAN.x86_64"),
                            "-scene {}".format(scene),
                            "-scenarios {}".format(scenario),
                            "-robot_locations {}".format(start),
                            "-target_locations {}".format(end),
                            "-interactive {}".format('true' if args.interactive else 'false'),
                            "-time_limit {}".format(args.time_limit)
                        ])
                        cmds.append(cmd)



    if args.shuffle:
        random.shuffle(cmds)

    final_cmds = []
    for i in range(args.repeat):
        final_cmds = final_cmds + cmds

    ScenarioRunner(args, final_cmds)

if __name__ == "__main__":
    # execute only if run as a script
    main()

