# Message containing the parameters to start an A-B navigation trial

Header                  header
string                  trial_name      # Which trial name are we running
uint16                  trial_number    # Which trial number are we running
geometry_msgs/Pose      spawn           # Robot spawn position
geometry_msgs/Pose      target          # Robot target position
geometry_msgs/PoseArray people          # People spawn positions
float64                 time_limit      # Time limit for the trial (in seconds)
