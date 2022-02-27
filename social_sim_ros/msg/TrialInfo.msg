# Message containing the information of running a single A-B navigation trial

Header                    header

# Information about the current interaction
time                      trial_start                                    # When did we start tracking metrics for this trial
float64                   timeout_time                                   # How long is allowed for the episode have to complete?
string                    trial_name                                     # Which trial name are we running
uint16                    trial_number                                   # Which trial number are we running
uint32                    num_actors                                     # How many people in the scene?

# Robot start / goal locations
geometry_msgs/Pose        robot_start
geometry_msgs/Pose        robot_goal

# Robot location / distance relative to start / goal
float64                   dist_to_target                                 # Current distance to target
float64                   min_dist_to_target                             # Closest difference to the target the robot has come
geometry_msgs/Pose[]      robot_poses                                    # Poses of the robot over the episode
time[]                    robot_poses_ts                                 # Timestamps of the robot poses over the episode

# Robot location relative to pedestrians
float64                   min_dist_to_ped                                # Minimum distance to any pedestrian throughout the trial

# Collisions between robots and people
uint32                    robot_on_person_intimate_dist_violations       # Robot passes w/in the intimate distance of a pedestrian
uint32                    person_on_robot_intimate_dist_violations       # A person passes w/in the intimate distance of the robot
uint32                    robot_on_person_personal_dist_violations       # Robot passes w/in the personal distance of a pedestrian
uint32                    person_on_robot_personal_dist_violations       # A person passes w/in the personal distance of the robot
uint32                    robot_on_person_collisions                     # Number of times that the robot collided with a person
uint32                    person_on_robot_collisions                     # Number of times that ap person collided with the robot

# Collisions w/ static objects
uint32                    obj_collisions                                 # Number of times that the robot collided with a static object

# Computed post-hoc
#bool                     completed                                      # was the robots finals distance to the goal within the desired distance?
#float64                  targ_dist_norm                                 # dist_to_target normalized by path length
#float64                  path_length                                    # Approximate distance traveled by robot
#float64                  mean_dist_to_target_not_moving                 # How far away from the target was the robot while not moving
#float64                  time_not_moving                                # Seconds the robot was not moving. Computed by checking if the robot moved more than 0.05m since a check the last 1s ago. If not, accumulate the time since the last check.
#bool                     episode_timed_out
#float64                  path_irregularity
#float64                  path_efficiency
