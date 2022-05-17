# Message defining an entry of a agent

Header      header         # Age of the track
uint64      track_id       # Unique ID for each agent

# Type of agent
string                  type

# Pose of the track
geometry_msgs/Pose      pose

# Velocity of the track
geometry_msgs/Twist     twist