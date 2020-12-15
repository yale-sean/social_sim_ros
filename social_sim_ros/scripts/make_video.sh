#!/bin/bash

if [ "$#" -ne 2 ]
then
  echo "Usage: make_video.sh [a784c] [agent|robot], where [a784c] is your run code"
  exit 1
fi

DATAPATH=$HOME/sim_ws/data/$1
if [[ "$2" = "agent" ]]; then
rosrun bag2video bag2video.py $DATAPATH/*.bag /thirdpersonview/compressed --outfile $DATAPATH/agent.mp4 --height 480 --width 640 --stop_topic /actor_goal_status
else
rosrun bag2video bag2video.py $DATAPATH/*.bag /rgb/compressed --outfile $DATAPATH/robot.mp4 --height 480 --width 640 --stop_topic /actor_goal_status
fi

  
